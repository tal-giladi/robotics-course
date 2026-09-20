"""run_voice.py - the voice interface: wake word, endpointing, ASR, agent, TTS (19.11).

    py 19-llm-robot-agents/code/run_voice.py            # a scripted conversation, with a timeline
    py 19-llm-robot-agents/code/run_voice.py --vad      # endpointing on a synthetic recording
    py 19-llm-robot-agents/code/run_voice.py --budget   # where the seconds go, four stacks
    py 19-llm-robot-agents/code/run_voice.py --mishear  # "desk" heard as "deck"
    py 19-llm-robot-agents/code/run_voice.py --stranger # a voice the robot does not know

No microphone, no models, no API key: the audio is synthesised with numpy and the ASR/TTS are
latency models measured on a Raspberry Pi 5. The real packages are listed in ``REAL_STACK``.
"""

from __future__ import annotations

import argparse
import re
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path[:0] = [str(HERE), str(HERE.parents[1] / "labs" / "python")]
if hasattr(sys.stdout, "reconfigure"):
    sys.stdout.reconfigure(encoding="utf-8")

from robot_agent.planning import Goal, MockPlanner, plan_and_repair  # noqa: E402
from robot_agent.safety import SafetyLayer  # noqa: E402
from robot_agent.sim_world import HomeWorld  # noqa: E402
from robot_agent.skills import CancelToken, RobotSkills  # noqa: E402
from robot_agent.verify import ClosedLoopExecutor, LoopConfig, SearchReplanner  # noqa: E402
from robot_agent.voice import (  # noqa: E402
    REAL_STACK,
    ASRConfig,
    EnergyVAD,
    KeywordConfig,
    LatencyBudget,
    ScriptedASR,
    Speaker,
    SpokenInput,
    TTSConfig,
    VADConfig,
    VoiceConfig,
    VoiceLoop,
    announce,
    synth_speech,
)

TAL = Speaker("tal", 0.95, True)
STRANGER = Speaker("unknown", 0.0, False)


def vad_demo() -> int:
    cfg = VADConfig()
    audio = synth_speech(cfg, pattern=((0.5, 1.6), (2.6, 3.4)), noise_db=-48.0)
    vad = EnergyVAD(cfg)
    print(f"{len(audio) / cfg.sample_rate:.1f} s at {cfg.sample_rate} Hz, {cfg.frame_ms} ms frames, "
          f"room noise -48 dBFS, speech -22 dBFS")
    print(f"threshold {cfg.threshold_db} dBFS, hangover {cfg.hangover_ms} ms\n")
    print("spoken:   (0.50, 1.60)  (2.60, 3.40)")
    print("detected: " + "  ".join(f"({a:.2f}, {b:.2f})" for a, b in vad.segments(audio)))
    print("\nthreshold sweep — too low hears the room, too high clips the ends of words:")
    for db in (-55.0, -50.0, -44.0, -38.0, -30.0, -24.0):
        segs = EnergyVAD(VADConfig(threshold_db=db)).segments(audio)
        print(f"  {db:>6.0f} dBFS -> {len(segs)} segment(s): "
              + "  ".join(f"({a:.2f}, {b:.2f})" for a, b in segs))
    print("\nhangover sweep — the silence that means 'finished', and the delay it costs:")
    for hang in (150, 300, 400, 700, 1000):
        segs = EnergyVAD(VADConfig(hangover_ms=hang)).segments(audio)
        ends = "  ".join(f"speech ends {b:.2f}, decided at {b + hang / 1000:.2f}" for _, b in segs[:1])
        print(f"  {hang:>5} ms -> {len(segs)} segment(s); {ends}")
    return 0


def budget_demo() -> int:
    stacks = {
        "Pi 5, whisper tiny": (ASRConfig("faster-whisper tiny int8", 0.25, 0.25), 2.0, TTSConfig()),
        "Pi 5, whisper small": (ASRConfig(), 2.0, TTSConfig()),
        "Pi 5 + cloud model": (ASRConfig(), 4.5, TTSConfig()),
        "Jetson + cloud model": (ASRConfig("faster-whisper small on GPU", 0.15, 0.08), 4.5,
                                 TTSConfig("piper on GPU", 0.15)),
    }
    audio_s = 2.4
    print(f"one utterance of {audio_s:.1f} s, 400 ms hangover, a 9-word answer\n")
    print(f"{'stack':<24}{'endpoint':>10}{'ASR':>8}{'agent':>8}{'TTS':>8}{'to first word':>16}  worst")
    for name, (asr, agent_s, tts) in stacks.items():
        budget = LatencyBudget(0.4, asr.fixed_s + asr.real_time_factor * audio_s, agent_s,
                               tts.first_audio_s)
        print(f"{name:<24}{budget.endpoint_s:>10.2f}{budget.asr_s:>8.2f}{budget.agent_s:>8.2f}"
              f"{budget.tts_first_audio_s:>8.2f}{budget.to_first_word_s:>16.2f}  {budget.worst_component()}")
    print("\nthe stop word does not use any of this:")
    print(f"  keyword spotter -> CancelToken -> brakes: 0.02 s, and 0.30 m/s x 0.02 s = 6 mm of travel")
    print(f"  through ASR + agent: 0.40 + 1.43 + 2.00 = 3.83 s, and 3.83 s x 0.30 m/s = 1.15 m")
    return 0


def conversation(args: argparse.Namespace) -> int:
    home = HomeWorld(seed=0)
    base = RobotSkills(home)
    layer = SafetyLayer(base, "autonomous")
    layer.accept_task("water bottle", "kitchen_table")
    cancel = CancelToken()

    task_re = re.compile(r"(?:find|fetch|get|bring)\s+(?:the\s+)?(?P<obj>.+?)\s+and\s+"
                         r"(?:put|place)\s+it\s+(?:on|onto)\s+(?:the\s+)?(?P<target>[\w\s]+?)\s*\.?$", re.I)

    def agent(text: str, speaker: Speaker) -> tuple[str, float]:
        """The 19.06/19.07 pipeline behind the microphone."""
        match = task_re.search(text.strip())
        if match is None:
            return "I can fetch things. Try: find the red cup and put it on the sofa.", 0.9
        # A new spoken command from a recognised speaker clears an earlier stop. On the real
        # robot this is a deliberate decision, and it belongs to the operator, not to the model.
        cancel.__init__()
        home.estop = False
        goal = Goal(match.group("obj").strip().lower(),
                    match.group("target").strip().lower().replace(" ", "_"))
        layer.accept_task(goal.label, goal.surface)
        plan, _ = plan_and_repair(MockPlanner(obj=goal.label, target=goal.surface), layer,
                                  text, goal, max_attempts=1)
        if plan is None:
            return "I could not work out a plan for that.", 1.2
        report = ClosedLoopExecutor(layer, goal, LoopConfig(max_replans=3), SearchReplanner()).run(plan)
        return announce(report), 1.8

    cfg = VoiceConfig(keywords=KeywordConfig(), asr=ASRConfig())
    loop = VoiceLoop(agent, cfg, on_stop=lambda: (cancel.cancel(), home.base.stop()))
    if args.mishear:
        loop.asr = ScriptedASR(cfg.asr, {"desk": "deck", "kitchen": "chicken"})
    speaker = STRANGER if args.stranger else TAL
    script = [
        SpokenInput(0.0, "is the coffee ready", 1.4, speaker),
        SpokenInput(6.0, "hey karmel find the water bottle and put it on the kitchen table", 3.2, speaker),
        SpokenInput(14.0, "stop", 0.4, speaker),
        SpokenInput(20.0, "hey karmel find the red cup and put it on the sofa", 2.6, speaker),
    ]
    print(f"stack: {REAL_STACK['asr']['package']} / {REAL_STACK['tts']['package']} / "
          f"{REAL_STACK['wake_word']['package']} (modelled, not installed)\n")
    turns = loop.run(script)
    for turn in turns:
        for event in sorted(turn.events, key=lambda e: e.t):
            print(f"t={event.t:6.2f}  {event.stage:<9} {event.text[:52]:<52} {event.detail}")
        if turn.budget:
            print("           budget: " + ", ".join(f"{name} {s:.2f} s ({share:.0%})"
                                                    for name, s, share in turn.budget.breakdown())
                  + f"  -> first word at +{turn.budget.to_first_word_s:.2f} s")
        if turn.interrupted:
            print("           INTERRUPTED: the robot stopped talking mid-sentence")
        print()
    print(f"stop words heard: {loop.stops}   bottle on: {home.objects['bottle-1'].on}   "
          f"cup on: {home.objects['cup-1'].on}")
    return 0


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--vad", action="store_true")
    ap.add_argument("--budget", action="store_true")
    ap.add_argument("--mishear", action="store_true")
    ap.add_argument("--stranger", action="store_true")
    args = ap.parse_args()
    if args.vad:
        return vad_demo()
    if args.budget:
        return budget_demo()
    return conversation(args)


if __name__ == "__main__":
    raise SystemExit(main())
