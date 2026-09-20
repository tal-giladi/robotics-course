"""19.11 — reference solution: endpointing, keyword spotting and the latency budget."""

from __future__ import annotations

import math
import re
from collections.abc import Sequence
from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class VADConfig:
    sample_rate: int = 16_000
    frame_ms: int = 20
    threshold_db: float = -38.0
    hangover_ms: int = 400
    min_speech_ms: int = 200

    @property
    def frame_len(self) -> int:
        return int(self.sample_rate * self.frame_ms / 1000)


def rms_db(frame: np.ndarray) -> float:
    rms = float(np.sqrt(np.mean(np.square(frame.astype(np.float64)))))
    return -math.inf if rms <= 0 else 20.0 * math.log10(rms)


def frames(audio: np.ndarray, cfg: VADConfig) -> np.ndarray:
    n = cfg.frame_len
    usable = len(audio) - len(audio) % n
    return audio[:usable].reshape(-1, n)


def speech_flags(audio: np.ndarray, cfg: VADConfig) -> np.ndarray:
    return np.array([rms_db(f) > cfg.threshold_db for f in frames(audio, cfg)])


def segments(audio: np.ndarray, cfg: VADConfig) -> list[tuple[float, float]]:
    flags = speech_flags(audio, cfg)
    hang = max(1, cfg.hangover_ms // cfg.frame_ms)
    out: list[tuple[int, int]] = []
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
    return [(a * ms, b * ms) for a, b in out if (b - a) * cfg.frame_ms >= cfg.min_speech_ms]


def phrase_score(heard: str, phrase: str) -> float:
    target = phrase.lower().split()
    words = re.findall(r"\w+", heard.lower())
    i = 0
    for word in words:
        if i < len(target) and word == target[i]:
            i += 1
    return i / len(target) if target else 0.0


def is_stop(heard: str, stop_words: Sequence[str] = ("stop", "halt", "עצור")) -> bool:
    words = set(re.findall(r"\w+", heard.lower(), flags=re.UNICODE))
    return bool(words & {w.lower() for w in stop_words})


@dataclass(frozen=True)
class LatencyBudget:
    endpoint_s: float
    asr_s: float
    agent_s: float
    tts_first_audio_s: float

    @property
    def to_first_word_s(self) -> float:
        return self.endpoint_s + self.asr_s + self.agent_s + self.tts_first_audio_s

    def breakdown(self) -> list[tuple[str, float, float]]:
        parts = [("endpointing", self.endpoint_s), ("ASR", self.asr_s),
                 ("agent", self.agent_s), ("TTS first audio", self.tts_first_audio_s)]
        total = self.to_first_word_s
        return [(name, seconds, seconds / total if total else 0.0) for name, seconds in parts]

    def worst_component(self) -> str:
        return max(self.breakdown(), key=lambda row: row[1])[0]


def asr_latency_s(audio_s: float, fixed_s: float, real_time_factor: float) -> float:
    return fixed_s + real_time_factor * audio_s


def travel_m(seconds: float, speed_m_s: float = 0.3) -> float:
    return seconds * speed_m_s


def route(heard: str, awake: bool, wake_phrase: str = "hey karmel",
          wake_threshold: float = 0.6) -> str:
    if is_stop(heard):
        return "stop"
    if awake:
        return "command"
    return "wake" if phrase_score(heard, wake_phrase) >= wake_threshold else "ignore"
