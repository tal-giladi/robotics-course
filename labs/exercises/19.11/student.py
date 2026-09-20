"""19.11 — the voice front end: when did they stop talking, what did they say, how long did it take.

Three pieces, in the order the audio meets them: endpointing (real signal processing on a numpy
array), keyword routing (the wake word, and the stop word that must never wait for the model), and
the latency budget that tells you which stage to spend money on.

Fill in every ``TODO(student)``; check with ``python course.py check 19.11``.
"""

from __future__ import annotations

import math
import re
from collections.abc import Sequence
from dataclasses import dataclass

import numpy as np


# ----------------------------------------------------------------------------------------------
# Given
# ----------------------------------------------------------------------------------------------
@dataclass(frozen=True)
class VADConfig:
    sample_rate: int = 16_000
    frame_ms: int = 20
    threshold_db: float = -38.0  # frame level above this counts as speech
    hangover_ms: int = 400  # silence this long ends the utterance
    min_speech_ms: int = 200  # shorter bursts are a door, a cough, a chair

    @property
    def frame_len(self) -> int:
        return int(self.sample_rate * self.frame_ms / 1000)


# ----------------------------------------------------------------------------------------------
# Yours: endpointing
# ----------------------------------------------------------------------------------------------
def rms_db(frame: np.ndarray) -> float:
    """Frame level in dBFS: ``20 log10(sqrt(mean(x^2)))``. Digital silence is ``-math.inf``."""
    # TODO(student): implement.
    raise NotImplementedError("rms_db")


def frames(audio: np.ndarray, cfg: VADConfig) -> np.ndarray:
    """Reshape into ``(n_frames, cfg.frame_len)``, dropping the incomplete tail."""
    # TODO(student): implement.
    raise NotImplementedError("frames")


def speech_flags(audio: np.ndarray, cfg: VADConfig) -> np.ndarray:
    """One boolean per frame: ``rms_db(frame) > cfg.threshold_db``."""
    # TODO(student): implement.
    raise NotImplementedError("speech_flags")


def segments(audio: np.ndarray, cfg: VADConfig) -> list[tuple[float, float]]:
    """Speech segments as ``(start_s, end_s)``, in seconds.

    Walk the flags with a hangover counter of ``hang = max(1, hangover_ms // frame_ms)`` frames:

    * a speech frame resets the counter and, if no segment is open, opens one at this frame;
    * a silent frame while a segment is open increments the counter, and when it reaches ``hang``
      the segment closes at frame ``i - hang + 1`` — the end of the *speech*, not the end of the
      silence that proved it was over;
    * a segment still open at the end of the audio closes at the last frame.

    Then convert frame indices to seconds (``× frame_ms / 1000``) and drop segments shorter than
    ``min_speech_ms``.

    The hangover is the delay the user feels before the robot starts thinking. It is not in the
    returned times, and it is in the latency budget below.
    """
    # TODO(student): implement.
    raise NotImplementedError("segments")


# ----------------------------------------------------------------------------------------------
# Yours: keywords
# ----------------------------------------------------------------------------------------------
def phrase_score(heard: str, phrase: str) -> float:
    """Fraction of ``phrase``'s words that appear in ``heard``, **in order**.

    Lower-case both, take ``re.findall(r"\\w+", ...)`` of ``heard``, and walk it advancing a
    pointer into the phrase's words on each match. An empty phrase scores 0.0.
    """
    # TODO(student): implement.
    raise NotImplementedError("phrase_score")


def is_stop(heard: str, stop_words: Sequence[str] = ("stop", "halt", "עצור")) -> bool:
    """Does any whole word of ``heard`` match a stop word (case-insensitively)?

    Whole words: "stopwatch" is not a stop. Use ``re.findall(r"\\w+", heard.lower())`` and set
    intersection, so the word is found anywhere in the sentence — a person shouting "no, stop
    that!" is not going to phrase it as a command.
    """
    # TODO(student): implement.
    raise NotImplementedError("is_stop")


def route(heard: str, awake: bool, wake_phrase: str = "hey karmel",
          wake_threshold: float = 0.6) -> str:
    """Where this utterance goes: ``"stop"``, ``"command"``, ``"wake"`` or ``"ignore"``.

    The order is the safety property. A stop word is routed **first**, whether or not the robot is
    awake and without waiting for ASR or the model. Then: if already awake it is a command;
    otherwise wake when ``phrase_score >= wake_threshold``, else ignore.
    """
    # TODO(student): implement.
    raise NotImplementedError("route")


# ----------------------------------------------------------------------------------------------
# Yours: latency
# ----------------------------------------------------------------------------------------------
@dataclass(frozen=True)
class LatencyBudget:
    endpoint_s: float
    asr_s: float
    agent_s: float
    tts_first_audio_s: float

    @property
    def to_first_word_s(self) -> float:
        """Seconds between the user stopping and the robot's first syllable: the four, summed."""
        # TODO(student): implement.
        raise NotImplementedError("to_first_word_s")

    def breakdown(self) -> list[tuple[str, float, float]]:
        """``[(name, seconds, share_of_total), ...]`` for "endpointing", "ASR", "agent",
        "TTS first audio", in that order. A zero total gives a share of 0.0, not a crash."""
        # TODO(student): implement.
        raise NotImplementedError("breakdown")

    def worst_component(self) -> str:
        """The name of the largest component — the only one worth optimising first."""
        # TODO(student): implement.
        raise NotImplementedError("worst_component")


def asr_latency_s(audio_s: float, fixed_s: float, real_time_factor: float) -> float:
    """``fixed + rtf × audio_seconds``: model loading and the first decode, then the transcription."""
    # TODO(student): implement.
    raise NotImplementedError("asr_latency_s")


def travel_m(seconds: float, speed_m_s: float = 0.3) -> float:
    """How far the robot goes while you are still deciding. This is why ``route`` is ordered."""
    # TODO(student): implement.
    raise NotImplementedError("travel_m")
