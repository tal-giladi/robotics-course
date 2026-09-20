"""Checker for 19.11 — endpointing, keyword routing and the latency budget. Milliseconds."""

from __future__ import annotations

import math

import numpy as np
import pytest


def synth(impl, pattern=((0.5, 1.6), (2.6, 3.4)), duration_s=5.0, noise_db=-48.0,
          speech_db=-22.0, seed=0, cfg=None):
    cfg = cfg or impl.VADConfig()
    rng = np.random.default_rng(seed)
    n = int(cfg.sample_rate * duration_s)
    audio = rng.normal(0, 10 ** (noise_db / 20), n)
    for start, end in pattern:
        a, b = int(start * cfg.sample_rate), int(end * cfg.sample_rate)
        envelope = np.hanning(b - a) * 0.5 + 0.5
        audio[a:b] += rng.normal(0, 10 ** (speech_db / 20), b - a) * envelope
    return audio


# ----------------------------------------------------------------------------------------------
# Levels and framing
# ----------------------------------------------------------------------------------------------
def test_silence_is_minus_infinity(impl) -> None:
    assert impl.rms_db(np.zeros(320)) == -math.inf


def test_rms_db_of_a_known_level(impl) -> None:
    assert impl.rms_db(np.full(320, 0.1)) == pytest.approx(-20.0, abs=0.01)


def test_frames_drop_the_incomplete_tail(impl) -> None:
    cfg = impl.VADConfig()
    out = impl.frames(np.zeros(cfg.frame_len * 3 + 7), cfg)
    assert out.shape == (3, cfg.frame_len)


def test_speech_flags_are_one_per_frame(impl) -> None:
    cfg = impl.VADConfig()
    audio = synth(impl, cfg=cfg)
    assert len(impl.speech_flags(audio, cfg)) == len(audio) // cfg.frame_len


# ----------------------------------------------------------------------------------------------
# Endpointing
# ----------------------------------------------------------------------------------------------
def test_the_synthesised_segments_are_found(impl) -> None:
    cfg = impl.VADConfig()
    found = impl.segments(synth(impl, cfg=cfg), cfg)
    assert len(found) == 2
    assert found[0][0] == pytest.approx(0.5, abs=0.06)
    assert found[0][1] == pytest.approx(1.6, abs=0.06)
    assert found[1][0] == pytest.approx(2.6, abs=0.06)


def test_a_threshold_below_the_room_noise_hears_the_room(impl) -> None:
    cfg = impl.VADConfig(threshold_db=-55.0)
    found = impl.segments(synth(impl, cfg=cfg), cfg)
    assert len(found) == 1 and found[0][1] - found[0][0] > 4.0


def test_a_long_hangover_merges_two_utterances_into_one(impl) -> None:
    cfg = impl.VADConfig(hangover_ms=1200)
    assert len(impl.segments(synth(impl, cfg=cfg), cfg)) == 1


def test_a_short_burst_is_not_an_utterance(impl) -> None:
    cfg = impl.VADConfig(min_speech_ms=200)
    audio = synth(impl, pattern=((1.0, 1.05),), noise_db=-60.0, cfg=cfg)
    assert impl.segments(audio, cfg) == []


def test_the_segment_ends_where_speech_ended_not_where_silence_was_proven(impl) -> None:
    cfg = impl.VADConfig(hangover_ms=700)
    audio = synth(impl, pattern=((0.5, 1.6),), duration_s=4.0, cfg=cfg)
    (_, end), = impl.segments(audio, cfg)
    assert end == pytest.approx(1.6, abs=0.08)


def test_speech_running_to_the_end_of_the_recording_still_closes(impl) -> None:
    cfg = impl.VADConfig()
    audio = synth(impl, pattern=((0.5, 2.0),), duration_s=2.0, cfg=cfg)
    assert len(impl.segments(audio, cfg)) == 1


# ----------------------------------------------------------------------------------------------
# Keywords
# ----------------------------------------------------------------------------------------------
def test_the_wake_phrase_scores_one_when_complete_and_in_order(impl) -> None:
    assert impl.phrase_score("hey karmel come here", "hey karmel") == 1.0
    assert impl.phrase_score("HEY Karmel!", "hey karmel") == 1.0


def test_out_of_order_words_do_not_score_a_full_match(impl) -> None:
    assert impl.phrase_score("karmel hey", "hey karmel") < 1.0


def test_unrelated_speech_scores_zero(impl) -> None:
    assert impl.phrase_score("is the coffee ready", "hey karmel") == 0.0


def test_an_empty_phrase_scores_zero(impl) -> None:
    assert impl.phrase_score("anything", "") == 0.0


def test_stop_is_recognised_anywhere_in_a_sentence(impl) -> None:
    assert impl.is_stop("stop")
    assert impl.is_stop("no no stop that")
    assert impl.is_stop("STOP")


def test_stop_matches_whole_words_only(impl) -> None:
    assert not impl.is_stop("stopwatch")
    assert not impl.is_stop("nonstop music")


def test_a_second_stop_word_can_be_configured(impl) -> None:
    assert impl.is_stop("bitte anhalten", stop_words=("anhalten",))


# ----------------------------------------------------------------------------------------------
# Routing
# ----------------------------------------------------------------------------------------------
def test_stop_is_routed_first_even_when_asleep(impl) -> None:
    assert impl.route("stop", awake=False) == "stop"


def test_stop_beats_a_sentence_that_also_contains_the_wake_word(impl) -> None:
    assert impl.route("hey karmel stop", awake=False) == "stop"


def test_the_wake_word_wakes(impl) -> None:
    assert impl.route("hey karmel go to the kitchen", awake=False) == "wake"


def test_speech_without_the_wake_word_is_ignored(impl) -> None:
    assert impl.route("go to the kitchen", awake=False) == "ignore"


def test_an_awake_robot_takes_the_sentence_as_a_command(impl) -> None:
    assert impl.route("go to the kitchen", awake=True) == "command"


# ----------------------------------------------------------------------------------------------
# Latency
# ----------------------------------------------------------------------------------------------
def test_the_budget_sums_its_four_parts(impl) -> None:
    assert impl.LatencyBudget(0.4, 1.43, 2.0, 0.45).to_first_word_s == pytest.approx(4.28)


def test_the_breakdown_shares_add_to_one(impl) -> None:
    budget = impl.LatencyBudget(0.4, 1.43, 2.0, 0.45)
    rows = budget.breakdown()
    assert [name for name, _, _ in rows] == ["endpointing", "ASR", "agent", "TTS first audio"]
    assert sum(share for _, _, share in rows) == pytest.approx(1.0)


def test_an_empty_budget_does_not_divide_by_zero(impl) -> None:
    assert impl.LatencyBudget(0, 0, 0, 0).breakdown()[0][2] == 0.0


def test_the_worst_component_is_what_you_optimise(impl) -> None:
    assert impl.LatencyBudget(0.4, 1.43, 2.0, 0.45).worst_component() == "agent"
    assert impl.LatencyBudget(0.4, 3.0, 0.5, 0.45).worst_component() == "ASR"


def test_asr_latency_grows_with_the_utterance(impl) -> None:
    assert impl.asr_latency_s(2.4, 0.35, 0.45) == pytest.approx(1.43)
    assert impl.asr_latency_s(5.0, 0.35, 0.45) > impl.asr_latency_s(1.0, 0.35, 0.45)


def test_stopping_through_the_pipeline_travels_more_than_a_metre(impl) -> None:
    through = impl.LatencyBudget(0.4, 1.43, 2.0, 0.45).to_first_word_s
    assert impl.travel_m(through) > 1.0
    assert impl.travel_m(0.02) < 0.01
