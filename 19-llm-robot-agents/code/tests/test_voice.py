"""Tests for 19.11 — endpointing, wake/stop words, the voice loop and the latency budget."""

from __future__ import annotations

import math

import pytest

from robot_agent.voice import (
    REAL_STACK,
    WAKE_LATENCY_S,
    ASRConfig,
    EnergyVAD,
    KeywordConfig,
    KeywordSpotter,
    LatencyBudget,
    ScriptedASR,
    Speaker,
    SpokenInput,
    TTSConfig,
    VADConfig,
    VoiceConfig,
    VoiceLoop,
    announce,
    rms_db,
    synth_speech,
)


def stub_agent(reply: str = "Done.", seconds: float = 1.5):
    def agent(text: str, speaker: Speaker) -> tuple[str, float]:
        return reply, seconds
    return agent


TAL = Speaker("tal", 0.95, True)


# ----------------------------------------------------------------------------------------------
# Endpointing
# ----------------------------------------------------------------------------------------------
def test_rms_db_of_silence_is_minus_infinity() -> None:
    import numpy as np

    assert rms_db(np.zeros(320)) == -math.inf


def test_the_vad_finds_the_segments_that_were_synthesised() -> None:
    cfg = VADConfig()
    audio = synth_speech(cfg, pattern=((0.5, 1.6), (2.6, 3.4)), noise_db=-48.0)
    segments = EnergyVAD(cfg).segments(audio)
    assert len(segments) == 2
    assert segments[0][0] == pytest.approx(0.5, abs=0.06)
    assert segments[0][1] == pytest.approx(1.6, abs=0.06)


def test_a_threshold_below_the_room_noise_hears_the_room() -> None:
    audio = synth_speech(noise_db=-48.0)
    segments = EnergyVAD(VADConfig(threshold_db=-55.0)).segments(audio)
    assert len(segments) == 1 and segments[0][1] - segments[0][0] > 4.0


def test_a_short_burst_is_not_an_utterance() -> None:
    cfg = VADConfig(min_speech_ms=200)
    audio = synth_speech(cfg, pattern=((1.0, 1.05),), noise_db=-60.0)  # a 50 ms door
    assert EnergyVAD(cfg).segments(audio) == []


def test_the_hangover_is_the_delay_the_user_feels() -> None:
    cfg = VADConfig(hangover_ms=700)
    audio = synth_speech(cfg, pattern=((0.5, 1.6),), noise_db=-48.0)
    (_, end), = EnergyVAD(cfg).segments(audio)
    assert end == pytest.approx(1.6, abs=0.08)  # the segment ends where speech ended...
    assert cfg.hangover_ms / 1000.0 == 0.7  # ... and the decision is 0.7 s later


# ----------------------------------------------------------------------------------------------
# Wake and stop
# ----------------------------------------------------------------------------------------------
def test_the_wake_word_needs_the_phrase_in_order() -> None:
    spotter = KeywordSpotter()
    assert spotter.is_wake("hey karmel come here")
    assert spotter.wake_score("karmel hey") < 1.0
    assert not spotter.is_wake("is the coffee ready")


def test_the_stop_word_is_recognised_in_any_sentence_and_in_hebrew() -> None:
    spotter = KeywordSpotter()
    assert spotter.is_stop("stop")
    assert spotter.is_stop("no no stop that")
    assert spotter.is_stop("עצור")
    assert not spotter.is_stop("stopwatch")


def test_stop_does_not_need_the_wake_word_or_the_model() -> None:
    stopped: list[int] = []
    loop = VoiceLoop(stub_agent(), VoiceConfig(), on_stop=lambda: stopped.append(1))
    turns = loop.run([SpokenInput(0.0, "stop", 0.3, TAL)])
    assert stopped == [1]
    assert turns[0].transcript is None  # ASR never ran
    assert turns[0].budget is None  # ... so there is no latency budget to speak of
    assert WAKE_LATENCY_S < 0.05


# ----------------------------------------------------------------------------------------------
# The loop
# ----------------------------------------------------------------------------------------------
def test_speech_without_the_wake_word_is_ignored() -> None:
    turns = VoiceLoop(stub_agent()).run([SpokenInput(0.0, "put it on the bed", 1.2, TAL)])
    assert turns[0].reply == "" and turns[0].events[0].stage == "ignored"


def test_one_wake_word_buys_one_command() -> None:
    loop = VoiceLoop(stub_agent())
    turns = loop.run([SpokenInput(0.0, "hey karmel go to the kitchen", 1.8, TAL),
                      SpokenInput(10.0, "and now the study", 1.2, TAL)])
    assert turns[0].reply == "Done." and turns[1].reply == ""


def test_the_wake_phrase_is_stripped_before_the_agent_sees_it() -> None:
    seen: list[str] = []

    def agent(text: str, speaker: Speaker) -> tuple[str, float]:
        seen.append(text)
        return "ok", 0.1

    VoiceLoop(agent).run([SpokenInput(0.0, "hey karmel go to the kitchen", 1.8, TAL)])
    assert seen == ["go to the kitchen"]


def test_a_low_confidence_transcript_is_confirmed_not_acted_on() -> None:
    cfg = VoiceConfig(asr=ASRConfig())
    loop = VoiceLoop(stub_agent(), cfg)
    loop.asr = ScriptedASR(cfg.asr, {"desk": "deck"})
    turn = loop.run([SpokenInput(0.0, "hey karmel put it on the desk", 1.6, TAL)])[0]
    assert turn.transcript is not None and turn.transcript.confidence < 0.75
    assert turn.reply.startswith("I heard") and turn.budget is not None and turn.budget.agent_s == 0.0


def test_an_unknown_voice_gets_the_read_only_mode() -> None:
    stranger = Speaker("unknown", 0.0, False)
    assert stranger.mode == "observe"
    assert Speaker("tal", 0.95, True).mode == "autonomous"
    assert Speaker("tal", 0.5, True).mode == "supervised"
    turn = VoiceLoop(stub_agent()).run(
        [SpokenInput(0.0, "hey karmel go to the kitchen", 1.8, stranger)])[0]
    assert "do not recognise" in turn.reply


def test_barge_in_marks_the_previous_turn_interrupted() -> None:
    loop = VoiceLoop(stub_agent("This is a long answer with many words in it", 1.0))
    turns = loop.run([SpokenInput(0.0, "hey karmel go to the kitchen", 1.8, TAL),
                      SpokenInput(5.0, "hey karmel never mind", 1.0, TAL)])
    assert turns[0].interrupted


# ----------------------------------------------------------------------------------------------
# Latency
# ----------------------------------------------------------------------------------------------
def test_the_budget_adds_up_and_names_its_worst_component() -> None:
    budget = LatencyBudget(0.4, 1.43, 2.0, 0.45)
    assert budget.to_first_word_s == pytest.approx(4.28)
    assert budget.worst_component() == "agent"
    assert sum(share for _, _, share in budget.breakdown()) == pytest.approx(1.0)


def test_asr_latency_grows_with_the_length_of_the_utterance() -> None:
    asr = ScriptedASR(ASRConfig())
    assert asr.transcribe("hello", 1.0).latency_s < asr.transcribe("hello", 5.0).latency_s


def test_tts_duration_is_words_over_speaking_rate() -> None:
    cfg = TTSConfig(words_per_minute=165.0)
    assert cfg.speak_s(" ".join(["word"] * 165)) == pytest.approx(60.0)


def test_stopping_through_the_agent_would_travel_more_than_a_metre() -> None:
    through_the_pipeline = LatencyBudget(0.4, 1.43, 2.0, 0.45)
    assert through_the_pipeline.to_first_word_s * 0.3 > 1.0  # metres at the course robot's top speed
    assert WAKE_LATENCY_S * 0.3 < 0.01


# ----------------------------------------------------------------------------------------------
# Speaking results
# ----------------------------------------------------------------------------------------------
def test_announce_is_one_sentence_a_person_can_hear() -> None:
    from robot_agent.skills import SkillResult

    assert announce(SkillResult("place", True, "SUCCEEDED", "placed 'bottle-1' on 'kitchen_table'")) == "Done."
    spoken = announce(SkillResult("pick", False, "OUT_OF_REACH", "'bottle-1' is 1.42 m away; the arm "
                                                                "reaches 0.75 m", hint="navigate_to it first."))
    assert spoken.startswith("I could not do that:") and "{" not in spoken and len(spoken.split()) < 25


def test_the_real_stack_entries_have_a_package_and_a_repo() -> None:
    for stage, info in REAL_STACK.items():
        assert info["package"] and info["repo"].startswith("https://")
        assert info["note"]
