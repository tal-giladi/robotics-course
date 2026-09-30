"""Triggers and the weapon state machine (lessons 21.05, 21.06, 21.07).

The state machine is the one from 21.01 Level 4:

    SAFE -> LOADED -> ARMED -> FIRED -> SAFE      (release confirmed)
                     ARMED -> SAFE                (e-stop / disarm)
                     FIRED -> JAMMED              (no confirmation in release_confirm_ms)
    JAMMED -> SAFE | LOADED                       (manual reset, payload verified)
    any state + e-stop -> SAFE                    (the hard reset you trust blindly)

Three trigger families sit in front of it, all with the same contract:
they return "fire requested" as events, and the pipeline decides (with the e-stop
interlock) whether a request becomes a shot.

* ButtonTrigger  — keypad / local button: edge + debounce (21.05 Level 2)
* RcTrigger      — 2.4 GHz frames: dead-man timeout + pulse-width threshold + edge (21.05 Level 3)
* VisionTrigger  — detection stream: dwell frames + confidence + distance gate (21.06)
* SoundTrigger   — acoustic envelope: hysteresis + double-clap code (21.07)

    py 21-war-machine/code/trigger_pipeline.py demo

All functions take explicit time arguments (no time.time()): tests run the pipeline
at 1000x speed without a single sleep.
"""

from __future__ import annotations

import argparse
import sys
from collections import deque
from dataclasses import dataclass, field
from enum import Enum


class State(str, Enum):
    SAFE = "SAFE"
    LOADED = "LOADED"
    ARMED = "ARMED"
    FIRED = "FIRED"
    JAMMED = "JAMMED"


# ==============================================================================================
#  The weapon state machine (21.01 Level 4)
# ==============================================================================================
@dataclass
class Weapon:
    """One weapon slot: a payload position + its releaser, modelled as an FSM.

    The e-stop is the only event trusted blindly: ``estop()`` works from ANY state and
    needs no sensor. Everything else is a sensor reading that can lie (21.09 FMEA row T5).
    """

    name: str
    release_confirm_ms: float = 2000.0
    state: State = State.SAFE
    fired_at: float | None = None
    log: list[tuple[float, str]] = field(default_factory=list)

    def _go(self, t: float, s: State) -> None:
        self.state = s
        self.log.append((t, s))

    # --- transitions ---------------------------------------------------------------------------
    def load(self, t: float) -> None:
        """A payload is verified in the slot (weight pin / optical check)."""
        if self.state not in (State.SAFE, State.FIRED):
            raise ValueError(f"{self.name}: cannot load in state {self.state}")
        self._go(t, State.LOADED)

    def arm(self, t: float, estop_out: bool) -> None:
        """Arm requires the e-stop to be OUT (the interlock of 21.01 Level 4, rule 1)."""
        if self.state != State.LOADED:
            raise ValueError(f"{self.name}: cannot arm in state {self.state}")
        if not estop_out:
            raise ValueError(f"{self.name}: e-stop is in; the weapon bus has no power")
        self._go(t, State.ARMED)

    def fire(self, t: float) -> bool:
        """Fire is only legal from ARMED — this one arrow is the double-fire guard."""
        if self.state != State.ARMED:
            return False
        self.fired_at = t
        self._go(t, State.FIRED)
        return True

    def confirm(self, t: float) -> bool:
        """Release confirmed (latch-open sensor / pin-out switch / payload-gone check)."""
        if self.state != State.FIRED:
            return False
        self._go(t, State.SAFE)
        return True

    def timeout(self, t: float) -> bool:
        """No confirmation within release_confirm_ms: the payload is somewhere (21.01 rule 3)."""
        if self.state != State.FIRED or self.fired_at is None:
            return False
        if (t - self.fired_at) * 1000.0 >= self.release_confirm_ms:
            self._go(t, State.JAMMED)
            return True
        return False

    def reset_jam(self, t: float, payload_present: bool) -> None:
        """Manual reset: look at the slot. Payload still there -> LOADED, slot empty -> SAFE."""
        if self.state != State.JAMMED:
            raise ValueError(f"{self.name}: nothing to reset in state {self.state}")
        self._go(t, State.LOADED if payload_present else State.SAFE)

    def estop(self, t: float) -> None:
        """Hard reset from any state. The copper did the work; the code just notes it."""
        self._go(t, State.SAFE)

    @property
    def can_fire(self) -> bool:
        return self.state == State.ARMED


# ==============================================================================================
#  Triggers
# ==============================================================================================
@dataclass(frozen=True)
class Detection:
    """One detected object in one camera frame (21.06): label, confidence, estimated range."""

    label: str
    confidence: float
    distance_m: float


class ButtonTrigger:
    """Local keypad key: a fire request on the RISING edge, debounced (21.05 Level 2).

    ``pressed`` is the raw switch state; the debounce eats contacts bouncing for
    ~20 ms, and the edge logic eats the human finger that stays on the key.
    """

    def __init__(self, name: str = "keypad", debounce_ms: float = 20.0) -> None:
        self.name = name
        self._debounce_ms = debounce_ms
        self._pressed = False
        self._last_edge_t: float | None = None

    def update(self, pressed: bool, t: float) -> bool:
        fire = False
        if pressed and not self._pressed:  # rising edge
            if self._last_edge_t is None or (t - self._last_edge_t) * 1000.0 >= self._debounce_ms:
                self._last_edge_t = t
                fire = True
        self._pressed = pressed
        return fire


class RcTrigger:
    """2.4 GHz channel with dead-man (21.05 Level 3).

    The receiver emits one frame per ~30 ms while the transmitter is alive. A frame carries
    the channel pulse width (1000..2000 us; >1500 us = "on"). Two guards:

    * **dead-man / relink**: if no frame arrives for ``deadman_ms``, the next frame is a
      RE-LINK: it resets the channel state but produces no fire — otherwise a transmitter
      that died with the switch held would fire the instant it restarted;
    * **edge**: one fire per press; holding the switch down is not a machine gun.

    The very first frame after power-on is also a re-link (baseline).
    """

    def __init__(self, name: str = "rc-ch1", fire_threshold_us: int = 1500, deadman_ms: float = 300.0) -> None:
        self.name = name
        self._threshold = fire_threshold_us
        self._deadman_ms = deadman_ms
        self._last_frame_t: float | None = None
        self._on = False

    def update(self, pulse_us: int | None, t: float) -> bool:
        """``pulse_us`` is the frame's channel width; ``None`` = the frame arrived with this
        channel unused. Returns a fire request."""
        fire = False
        relink = (self._last_frame_t is None
                  or (t - self._last_frame_t) * 1000.0 > self._deadman_ms)
        self._last_frame_t = t  # any frame proves the link is alive, used or not
        if pulse_us is not None:
            on = pulse_us >= self._threshold
            if not relink and on and not self._on:  # rising edge within a live link
                fire = True
            self._on = on
        elif self._on:
            self._on = False
        return fire

    def link_alive(self, t: float) -> bool:
        return self._last_frame_t is not None and (t - self._last_frame_t) * 1000.0 <= self._deadman_ms


class VisionTrigger:
    """Detection stream -> fire gate (21.06 Level 4).

    A single good frame is NOT a shot: the target must be present, confident, and in range for
    ``dwell_frames`` CONSECUTIVE frames (any miss resets the count), then the trigger fires
    once and cools down for ``cooldown_s``. Dwell is the whole argument: a false fire costs
    one payload and one state machine cycle, a false-negative costs nothing.
    """

    def __init__(
        self,
        name: str = "vision",
        target_label: str = "aruco",
        dwell_frames: int = 15,
        confidence_min: float = 0.5,
        d_min_m: float = 1.0,
        d_max_m: float = 5.0,
        cooldown_s: float = 2.0,
    ) -> None:
        self.name = name
        self.target_label = target_label
        self.dwell_frames = dwell_frames
        self.confidence_min = confidence_min
        self.d_min_m = d_min_m
        self.d_max_m = d_max_m
        self.cooldown_s = cooldown_s
        self._streak = 0
        self._last_fire_t: float | None = None

    def frame_ok(self, detections: list[Detection]) -> bool:
        """Is the target good in THIS frame?"""
        for d in detections:
            if (
                d.label == self.target_label
                and d.confidence >= self.confidence_min
                and self.d_min_m <= d.distance_m <= self.d_max_m
            ):
                return True
        return False

    def update(self, detections: list[Detection], t: float) -> bool:
        good = self.frame_ok(detections)
        self._streak = self._streak + 1 if good else 0
        if self._streak < self.dwell_frames:
            return False
        if self._last_fire_t is not None and t - self._last_fire_t < self.cooldown_s:
            return False
        self._streak = 0
        self._last_fire_t = t
        return True


class SoundTrigger:
    """Acoustic envelope -> clap events -> fire code (21.07).

    Input is the envelope value (e.g. dB above the noise floor, 2 ms RMS window). Hysteresis:
    a clap is registered when the envelope crosses ``high_db`` while it is below ``low_db``
    afterwards; oscillation around one threshold is the classic false-fire. The fire code is
    the DOUBLE CLAP: two claps 250..900 ms apart. One clap is a "you there?" — free of charge.
    """

    def __init__(
        self,
        name: str = "sound",
        high_db: float = 15.0,
        low_db: float = 8.0,
        min_gap_ms: float = 250.0,
        max_gap_ms: float = 900.0,
    ) -> None:
        self.name = name
        self.high_db = high_db
        self.low_db = low_db
        self.min_gap_ms = min_gap_ms
        self.max_gap_ms = max_gap_ms
        self._waiting = False          # envelope above high, waiting to cross low (clap armed)
        self._clap_t: float | None = None
        self._claps: deque[float] = deque()

    def update(self, env_db: float, t: float) -> bool:
        """Returns True exactly when the double-clap code completes."""
        if not self._waiting and env_db >= self.high_db:
            self._waiting = True
        elif self._waiting and env_db <= self.low_db:
            self._waiting = False
            if self._clap_t is None or (t - self._clap_t) * 1000.0 >= self.min_gap_ms:
                self._clap_t = t
                self._claps.append(t)
                if len(self._claps) == 2:
                    gap_ms = (self._claps[1] - self._claps[0]) * 1000.0
                    if gap_ms <= self.max_gap_ms:
                        self._claps.clear()
                        return True
                    self._claps.clear()  # too slow to be a code: both claps were noise
        return False


# ==============================================================================================
#  The pipeline: trigger request + weapon state + e-stop interlock
# ==============================================================================================
class FirePipeline:
    """Wires one trigger to one weapon and enforces the 21.01 rules.

    ``step_release`` is called when the release sensor reports (or on the confirm timeout);
    it moves FIRED to SAFE (confirmed) or JAMMED (timeout).
    """

    def __init__(self, weapon: Weapon, estop_out: bool = True) -> None:
        self.weapon = weapon
        self.estop_out = estop_out
        self.shots = 0

    def on_trigger(self, fired_requested: bool, t: float) -> bool:
        """A trigger asked to fire. Did it become a shot?"""
        if not fired_requested:
            return False
        if not self.weapon.can_fire:
            return False
        # The state machine requires ARMED, which requires the e-stop to have been out at
        # arm() time. Re-check NOW: the e-stop may have been pressed between arm and fire.
        if not self.estop_out:
            return False
        if self.weapon.fire(t):
            self.shots += 1
            return True
        return False

    def step_release(self, t: float, release_sensor_ok: bool | None = None) -> State:
        """Drive FIRED to its outcome. ``release_sensor_ok=None`` means "ask the timeout"."""
        w = self.weapon
        if w.state != State.FIRED:
            return w.state
        if release_sensor_ok is True:
            w.confirm(t)
        elif w.timeout(t):
            pass  # -> JAMMED
        return w.state

    def set_estop(self, out: bool, t: float) -> None:
        self.estop_out = out
        if not out:
            self.weapon.estop(t)  # hard reset, any state


# ==============================================================================================
#  demo: a scripted session you can read
# ==============================================================================================
def _demo() -> None:
    print("=== demo 1: normal shot (RC trigger) ===")
    p = FirePipeline(Weapon("cannon"))
    w = p.weapon
    w.load(0.0)
    w.arm(0.1, estop_out=p.estop_out)
    rc = RcTrigger()
    rc.update(1000, 0.4)  # link baseline frame (switch off)
    fired = p.on_trigger(rc.update(1900, 0.5), 0.5)
    print(f"  t=0.50  rc frame (1900 us) -> fired={fired}   state={w.state}")
    print(f"  t=0.70  release sensor OK  -> state={p.step_release(0.7, True)}")

    print()
    print("=== demo 2: double-fire guard ===")
    print(f"  t=0.80  second RC press, no reload -> fired={p.on_trigger(rc.update(1900, 0.8), 0.8)}")
    print(f"           state={w.state}  (a second shot needs a fresh LOADED)")

    print()
    print("=== demo 3: e-stop between arm and fire ===")
    w.load(1.0)
    w.arm(1.1, estop_out=True)
    p.set_estop(False, 1.2)
    print(f"  t=1.20  e-stop pressed -> state={w.state}")
    rc2 = RcTrigger()
    rc2.update(1000, 1.25)  # fresh link, switch off
    print(f"  t=1.30  RC fires anyway -> fired={p.on_trigger(rc2.update(1900, 1.3), 1.3)}")
    print(f"           the bus had no power; the state machine already said SAFE")
    p.set_estop(True, 1.9)  # e-stop released for the next demos

    print()
    print("=== demo 4: jam and reset ===")
    w.load(2.0)
    w.arm(2.1, estop_out=True)
    rc3 = RcTrigger()
    rc3.update(1000, 2.4)  # fresh link, switch off
    p.on_trigger(rc3.update(1900, 2.5), 2.5)
    print(f"  t=2.50  fired, release sensor stuck...")
    print(f"  t=4.60  no confirmation -> state={p.step_release(4.6)}")
    w.reset_jam(5.0, payload_present=True)
    print(f"  t=5.00  manual reset, payload still in slot -> state={w.state}")

    print()
    print("=== demo 5: vision dwell (15 frames at 30 fps = 0.5 s) ===")
    vt = VisionTrigger(target_label="aruco")
    dets = [Detection("aruco", 0.8, 2.5)]
    for i in range(20):
        f = vt.update(dets, i / 30.0)
        if f:
            print(f"  fired at frame {i} (t={i / 30.0:.2f} s) after {vt.dwell_frames} good frames")
            break

    print()
    print("=== demo 6: sound double-clap ===")
    st = SoundTrigger()
    seq = [(0.00, 3), (0.02, 18), (0.04, 2), (0.45, 3), (0.47, 19), (0.49, 2)]
    for t, env in seq:
        f = st.update(env, t)
        print(f"  t={t:5.2f}  env={env:2d} dB  -> {'DOUBLE-CLAP: FIRE' if f else '...'}")


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("command", choices=("demo",))
    args = ap.parse_args(argv)
    _demo()
    return 0


if __name__ == "__main__":
    sys.exit(main())
