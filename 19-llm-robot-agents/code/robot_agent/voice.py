"""Talking to the robot: wake word, endpointing, ASR, the agent, TTS (19.11).

Version-sensitive (verified 2026-09): the model names and packages in ``REAL_STACK`` move fast.
The *structure* below does not — it is the same five stages in every voice assistant since 2015,
and the robotics-specific parts are stages 0 and 5:

    0  wake word     always-on, tiny, on-device        openWakeWord            ~20 ms/frame
    1  endpointing   when did the person stop talking  energy/Silero VAD       300-700 ms of silence
    2  ASR           audio -> text                     faster-whisper small    0.8-2.5 s on a Pi 5
    3  the agent     text -> skill calls               modules 19.03-19.09     1-8 s
    4  TTS           text -> audio                     Piper                   0.3-0.8 s to first audio
    5  barge-in      the human interrupts              a keyword spotter       must not need stage 3

Two things here are safety, not user experience, and they are the reason this lesson exists:

* **"Stop" must never be a tool call.** It is recognised by the keyword spotter in stage 0 and
  routed straight to the ``CancelToken`` and the brakes. If stopping the robot requires the ASR to
  finish, the LLM to answer and a skill to be dispatched, then stopping takes six seconds, and
  six seconds is a metre and a half.
* **A microphone is an open channel.** Anyone in the room — or a television — can talk to your
  robot. ``Speaker`` carries who is talking and how sure we are, and an unrecognised speaker gets
  the read-only mode of [19.09](../../19.09-agent-safety-boundaries.md).

Everything runs offline: ``EnergyVAD`` is real signal processing over a numpy array, and the loop
is a discrete-event simulation with a measured latency model, so you can change one number and see
the end-to-end delay move without owning a microphone.
"""

from __future__ import annotations

import math
import re
from collections.abc import Callable, Iterable, Sequence
from dataclasses import dataclass, field, replace
from typing import Any

import numpy as np

# ----------------------------------------------------------------------------------------------
# The real stack (what you would actually install)
# ----------------------------------------------------------------------------------------------
#: Verified 2026-09. Every one of these runs on the Raspberry Pi 5 with no GPU and no API key.
REAL_STACK: dict[str, dict[str, str]] = {
    "wake_word": {"package": "openwakeword", "repo": "https://github.com/dscripka/openWakeWord",
                  "note": "Apache-2.0, ONNX/tflite models, ~20 ms per 80 ms frame on a Pi 5 CPU."},
    "vad": {"package": "silero-vad", "repo": "https://github.com/snakers4/silero-vad",
            "note": "A learned VAD; better than energy in a noisy kitchen. EnergyVAD below is the "
                    "thing it replaces, and it is worth writing once."},
    "asr": {"package": "faster-whisper", "repo": "https://github.com/SYSTRAN/faster-whisper",
            "note": "CTranslate2 Whisper. 'small' int8 is the usual Pi 5 choice; 'tiny' for latency."},
    "tts": {"package": "piper-tts", "repo": "https://github.com/OHF-Voice/piper1-gpl",
            "note": "Neural TTS, fast on CPU. Maintained by the Open Home Foundation (the original "
                    "rhasspy/piper repository is archived) — check it is still maintained."},
}


# ----------------------------------------------------------------------------------------------
# Stage 1: endpointing
# ----------------------------------------------------------------------------------------------
@dataclass(frozen=True)
class VADConfig:
    sample_rate: int = 16_000
    frame_ms: int = 20
    threshold_db: float = -38.0  # frame RMS above this counts as speech
    hangover_ms: int = 400  # silence this long ends the utterance: the endpointing delay
    min_speech_ms: int = 200  # shorter bursts are a door, a cough, a chair
    max_utterance_s: float = 12.0

    @property
    def frame_len(self) -> int:
        return int(self.sample_rate * self.frame_ms / 1000)


def rms_db(frame: np.ndarray) -> float:
    """Frame level in dBFS. -inf for digital silence."""
    rms = float(np.sqrt(np.mean(np.square(frame.astype(np.float64)))))
    return -math.inf if rms <= 0 else 20.0 * math.log10(rms)


class EnergyVAD:
    """Frame energy + hysteresis. Cheap, explainable, and wrong in a noisy room — which is the
    point: you measure how wrong before deciding whether you need a learned VAD."""

    def __init__(self, cfg: VADConfig = VADConfig()) -> None:
        self.cfg = cfg

    def frames(self, audio: np.ndarray) -> np.ndarray:
        n = self.cfg.frame_len
        usable = len(audio) - len(audio) % n
        return audio[:usable].reshape(-1, n)

    def speech_flags(self, audio: np.ndarray) -> np.ndarray:
        return np.array([rms_db(f) > self.cfg.threshold_db for f in self.frames(audio)])

    def segments(self, audio: np.ndarray) -> list[tuple[float, float]]:
        """Speech segments as (start_s, end_s), with the hangover applied to the end."""
        cfg = self.cfg
        flags = self.speech_flags(audio)
        hang = max(1, cfg.hangover_ms // cfg.frame_ms)
        out: list[tuple[float, float]] = []
        start: int | None = None
        silence = 0
        for i, is_speech in enumerate(flags):
            if is_speech:
                silence = 0
                if start is None:
                    start = i
            elif start is not None:
                silence += 1
                if silence >= hang:
                    out.append((start, i - hang + 1))
                    start = None
        if start is not None:
            out.append((start, len(flags)))
        ms = cfg.frame_ms / 1000.0
        return [(a * ms, b * ms) for a, b in out
                if (b - a) * cfg.frame_ms >= cfg.min_speech_ms]


def synth_speech(cfg: VADConfig = VADConfig(), seed: int = 0,
                 pattern: Sequence[tuple[float, float]] = ((0.5, 1.6), (2.6, 3.4)),
                 duration_s: float = 5.0, noise_db: float = -55.0, speech_db: float = -22.0) -> np.ndarray:
    """A synthetic recording: band-limited noise bursts where the person talks, hiss elsewhere.

    Not speech — but it has the property the VAD actually uses (level), so the endpointing
    arithmetic below is real arithmetic on a real signal.
    """
    rng = np.random.default_rng(seed)
    n = int(cfg.sample_rate * duration_s)
    audio = rng.normal(0, 10 ** (noise_db / 20), n)
    for start, end in pattern:
        a, b = int(start * cfg.sample_rate), int(end * cfg.sample_rate)
        envelope = np.hanning(b - a) * 0.5 + 0.5
        audio[a:b] += rng.normal(0, 10 ** (speech_db / 20), b - a) * envelope
    return audio


# ----------------------------------------------------------------------------------------------
# Stage 0 and 5: wake word and the stop word
# ----------------------------------------------------------------------------------------------
@dataclass(frozen=True)
class KeywordConfig:
    wake_phrase: str = "hey karmel"
    #: Words that must stop the robot without ever reaching the language model.
    stop_words: tuple[str, ...] = ("stop", "halt", "עצור")
    threshold: float = 0.6
    #: Per-hour false accepts at that threshold, measured on your own recordings. Publish it.
    false_accepts_per_hour: float = 0.5


#: How long the keyword spotter takes to fire, end to end. This is the number that decides
#: whether shouting "stop" is a safety control or a request.
WAKE_LATENCY_S = 0.02


class KeywordSpotter:
    """Stands in for openWakeWord: a score in [0, 1] per candidate phrase.

    The offline implementation scores text (so the lab is deterministic); the real one scores
    80 ms of audio. What matters for the lesson is identical in both: a **threshold**, a measured
    false-accept rate, and the fact that the stop word is handled here, before anything slow.
    """

    def __init__(self, cfg: KeywordConfig = KeywordConfig()) -> None:
        self.cfg = cfg

    def wake_score(self, heard: str) -> float:
        return _phrase_score(heard, self.cfg.wake_phrase)

    def is_wake(self, heard: str) -> bool:
        return self.wake_score(heard) >= self.cfg.threshold

    def is_stop(self, heard: str) -> bool:
        words = set(re.findall(r"\w+", heard.lower()))
        return bool(words & {w.lower() for w in self.cfg.stop_words})


def _phrase_score(heard: str, phrase: str) -> float:
    """Fraction of the phrase's words present, in order. Crude, deterministic, good enough."""
    target = phrase.lower().split()
    words = re.findall(r"\w+", heard.lower())
    i = 0
    for word in words:
        if i < len(target) and word == target[i]:
            i += 1
    return i / len(target) if target else 0.0


# ----------------------------------------------------------------------------------------------
# Stages 2 and 4: ASR and TTS behind interfaces
# ----------------------------------------------------------------------------------------------
@dataclass(frozen=True)
class Transcript:
    text: str
    confidence: float
    latency_s: float


@dataclass(frozen=True)
class ASRConfig:
    """Latency model for faster-whisper on a Raspberry Pi 5 (verified 2026-09, order of magnitude).

    ``latency ≈ fixed + rtf × audio_seconds``. The real-time factor is what you tune by choosing
    the model size; the fixed part is loading, resampling and the first decode step.
    """

    name: str = "faster-whisper small int8"
    fixed_s: float = 0.35
    real_time_factor: float = 0.45
    word_error_rate: float = 0.06


class ScriptedASR:
    """Replays a script, with a latency computed from the audio length and an optional error.

    Errors are what a voice interface is actually about: "put it on the *desk*" heard as "on the
    *deck*" is a skill call with a place name that is not on the map — which the skill API of
    19.02 rejects with an enum error, not a wrong action. That is the design working.
    """

    def __init__(self, cfg: ASRConfig = ASRConfig(), mishear: dict[str, str] | None = None) -> None:
        self.cfg = cfg
        self.mishear = mishear or {}

    def transcribe(self, said: str, audio_s: float) -> Transcript:
        text = said
        for wrong, right in self.mishear.items():
            text = re.sub(rf"\b{re.escape(wrong)}\b", right, text, flags=re.I)
        confidence = 0.95 if text == said else 0.62
        return Transcript(text, confidence, self.cfg.fixed_s + self.cfg.real_time_factor * audio_s)


@dataclass(frozen=True)
class TTSConfig:
    """Piper on a Pi 5. ``first_audio_s`` is the number the user feels; the rest streams."""

    name: str = "piper en_US-lessac-medium"
    first_audio_s: float = 0.45
    words_per_minute: float = 165.0

    def speak_s(self, text: str) -> float:
        return 60.0 * len(text.split()) / self.words_per_minute


# ----------------------------------------------------------------------------------------------
# The latency budget
# ----------------------------------------------------------------------------------------------
@dataclass(frozen=True)
class LatencyBudget:
    """Where the seconds go between "…on the kitchen table." and the robot answering."""

    endpoint_s: float
    asr_s: float
    agent_s: float
    tts_first_audio_s: float
    wake_s: float = 0.02

    @property
    def to_first_word_s(self) -> float:
        return self.endpoint_s + self.asr_s + self.agent_s + self.tts_first_audio_s

    def breakdown(self) -> list[tuple[str, float, float]]:
        parts = [("endpointing", self.endpoint_s), ("ASR", self.asr_s),
                 ("agent", self.agent_s), ("TTS first audio", self.tts_first_audio_s)]
        total = self.to_first_word_s
        return [(name, seconds, seconds / total) for name, seconds in parts]

    def worst_component(self) -> str:
        return max(self.breakdown(), key=lambda row: row[1])[0]


# ----------------------------------------------------------------------------------------------
# Who is talking
# ----------------------------------------------------------------------------------------------
@dataclass(frozen=True)
class Speaker:
    """A microphone has no authentication. This is the little that can be said about a voice."""

    name: str = "unknown"
    confidence: float = 0.0
    enrolled: bool = False

    @property
    def mode(self) -> str:
        """The operating mode of 19.09 this speaker may command."""
        if self.enrolled and self.confidence >= 0.8:
            return "autonomous"
        if self.enrolled:
            return "supervised"
        return "observe"  # a voice from the television gets to ask questions and nothing else


# ----------------------------------------------------------------------------------------------
# The loop
# ----------------------------------------------------------------------------------------------
@dataclass(frozen=True)
class SpokenInput:
    """Something said into the room at ``t`` seconds, taking ``audio_s`` to say."""

    t: float
    text: str
    audio_s: float = 1.6
    speaker: Speaker = Speaker("tal", 0.95, True)


@dataclass
class VoiceEvent:
    t: float
    stage: str  # "wake" | "stop" | "listen" | "asr" | "agent" | "speak" | "ignored" | "barge_in"
    text: str
    detail: str = ""


@dataclass
class VoiceTurn:
    heard: str
    transcript: Transcript | None
    reply: str
    budget: LatencyBudget | None
    interrupted: bool = False
    events: list[VoiceEvent] = field(default_factory=list)


@dataclass(frozen=True)
class VoiceConfig:
    vad: VADConfig = VADConfig()
    keywords: KeywordConfig = KeywordConfig()
    asr: ASRConfig = ASRConfig()
    tts: TTSConfig = TTSConfig()
    require_wake_word: bool = True
    confirm_low_confidence_below: float = 0.75


class VoiceLoop:
    """Wake → listen → transcribe → agent → speak, with barge-in and a hard stop path.

    ``agent`` is any callable ``(text, speaker) -> (reply, seconds)``: the agent loop of
    [19.03](../../19.03-tool-calling-sim-robot.md), the planner of 19.06, or a stub. ``on_stop`` is
    called the moment a stop word is spotted — synchronously, before anything else in the pipeline.
    """

    def __init__(self, agent: Callable[[str, Speaker], tuple[str, float]],
                 cfg: VoiceConfig = VoiceConfig(),
                 on_stop: Callable[[], None] | None = None) -> None:
        self.agent = agent
        self.cfg = cfg
        self.on_stop = on_stop or (lambda: None)
        self.spotter = KeywordSpotter(cfg.keywords)
        self.asr = ScriptedASR(cfg.asr)
        self.awake = not cfg.require_wake_word
        self.stops = 0

    def run(self, inputs: Iterable[SpokenInput]) -> list[VoiceTurn]:
        turns: list[VoiceTurn] = []
        speaking_until = 0.0
        for spoken in inputs:
            events: list[VoiceEvent] = []
            # Stage 0/5 first, always: the stop word never waits for ASR or for the model.
            if self.spotter.is_stop(spoken.text):
                self.stops += 1
                self.on_stop()
                events.append(VoiceEvent(spoken.t, "stop", spoken.text,
                                         f"cancelled {WAKE_LATENCY_S * 1000:.0f} ms after the word, "
                                         "without ASR and without the model"))
                if speaking_until > spoken.t:
                    events.append(VoiceEvent(spoken.t, "barge_in", spoken.text, "stopped speaking"))
                    speaking_until = spoken.t
                    if turns:
                        turns[-1].interrupted = True
                turns.append(VoiceTurn(spoken.text, None, "(stopping)", None, events=events))
                self.awake = False
                continue
            if self.cfg.require_wake_word and not self.awake:
                if not self.spotter.is_wake(spoken.text):
                    events.append(VoiceEvent(spoken.t, "ignored", spoken.text,
                                             f"wake score {self.spotter.wake_score(spoken.text):.2f} "
                                             f"< {self.cfg.keywords.threshold}"))
                    turns.append(VoiceTurn(spoken.text, None, "", None, events=events))
                    continue
                events.append(VoiceEvent(spoken.t, "wake", spoken.text, "woken"))
            if speaking_until > spoken.t:  # the human talked over the robot
                events.append(VoiceEvent(spoken.t, "barge_in", spoken.text, "stopped speaking to listen"))
                if turns:
                    turns[-1].interrupted = True
            command = _strip_wake(spoken.text, self.cfg.keywords.wake_phrase)
            endpoint_s = self.cfg.vad.hangover_ms / 1000.0
            transcript = self.asr.transcribe(command, spoken.audio_s)
            events.append(VoiceEvent(spoken.t + spoken.audio_s + endpoint_s, "asr", transcript.text,
                                     f"confidence {transcript.confidence:.2f}, {transcript.latency_s:.2f} s"))
            if transcript.confidence < self.cfg.confirm_low_confidence_below:
                reply = f"I heard '{transcript.text}'. Is that right?"
                agent_s = 0.0
                events.append(VoiceEvent(spoken.t, "agent", reply, "low confidence: confirm, do not act"))
            elif spoken.speaker.mode == "observe":
                reply = "I do not recognise your voice, so I can answer questions but not move."
                agent_s = 0.0
                events.append(VoiceEvent(spoken.t, "agent", reply, f"speaker mode {spoken.speaker.mode}"))
            else:
                reply, agent_s = self.agent(transcript.text, spoken.speaker)
                events.append(VoiceEvent(spoken.t, "agent", reply, f"{agent_s:.2f} s in the agent"))
            budget = LatencyBudget(endpoint_s, transcript.latency_s, agent_s, self.cfg.tts.first_audio_s)
            start_speaking = spoken.t + spoken.audio_s + budget.to_first_word_s
            speaking_until = start_speaking + self.cfg.tts.speak_s(reply)
            events.append(VoiceEvent(start_speaking, "speak", reply,
                                     f"first audio at +{budget.to_first_word_s:.2f} s after the user "
                                     f"stopped talking"))
            turns.append(VoiceTurn(spoken.text, transcript, reply, budget, events=events))
            self.awake = False  # one command per wake: an always-listening robot is a different product
        return turns


def _strip_wake(text: str, phrase: str) -> str:
    return re.sub(rf"^\s*{re.escape(phrase)}[,\s]*", "", text, flags=re.I).strip() or text


def announce(result: Any) -> str:
    """Turn a ``SkillResult``/``ExecutionReport``-like object into one sentence worth saying aloud.

    Speech is a low-bandwidth channel: no JSON, no ids, no coordinates, one clause of *what* and
    one of *what now*. This is the same problem as writing a tool result for a model, with a much
    tighter budget.
    """
    ok = bool(getattr(result, "ok", getattr(result, "outcome", "") == "succeeded"))
    if ok:
        return "Done."
    message = str(getattr(result, "message", "") or "it did not work")
    hint = getattr(result, "hint", None)
    first = re.split(r"[;.]", message)[0].strip()
    return f"I could not do that: {first}." + (f" {hint.split('.')[0]}." if hint else "")
