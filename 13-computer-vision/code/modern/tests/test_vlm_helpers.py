"""Tests for the model-free helpers in vlm.py (parsing, verification, token estimate)."""
import pytest

from common import Detection
from vlm import claude_image_tokens, extract_json, verify


@pytest.mark.parametrize("w,h,edge,cap,tokens,size", [
    (200, 200, 1568, 1568, 64, (200, 200)),
    (1000, 1000, 1568, 1568, 1296, (1000, 1000)),
    (1920, 1080, 1568, 1568, 1560, (1456, 819)),
    (1920, 1080, 2576, 4784, 2691, (1920, 1080)),
    (3840, 2160, 2576, 4784, 4784, (2576, 1449)),
])
def test_token_estimate_matches_docs_table(w, h, edge, cap, tokens, size):
    t, sw, sh = claude_image_tokens(w, h, edge, cap)
    assert t == tokens and (sw, sh) == size


def test_extract_json_from_chatty_output():
    text = 'Sure! Here is the JSON:\n```json\n{"objects": [{"name": "mug", "category": "cup", "position": "left"}]}\n```'
    assert extract_json(text)["objects"][0]["name"] == "mug"
    assert extract_json('The answer {"objects": []} hope it helps')["objects"] == []
    with pytest.raises(ValueError):
        extract_json("I see a mug and a bottle.")
    # What SmolVLM2-256M actually returned: Python quotes outside, the template copied inside.
    copied = "{'objects': [{\"name\": \"...\", \"category\": \"...\", \"position\": \"left|center|right\"}]}"
    assert extract_json(copied)["name"] == "..."          # without a key check, a fragment looks like an answer
    with pytest.raises(ValueError):
        extract_json(copied, required_key="objects")


def test_verify_flags_hallucinations_and_wrong_positions():
    dets = [Detection("bottle", 0.9, (420, 10, 500, 190)), Detection("cup", 0.8, (20, 10, 80, 90))]
    scene = {"objects": [
        {"name": "oil bottle", "category": "bottle", "position": "right"},
        {"name": "glass", "category": "cup", "position": "right"},
        {"name": "tennis ball", "category": "sports ball", "position": "center"},
        {"name": "pizza", "category": "other", "position": "center"},
    ]}
    status = [v.status for v in verify(scene, dets, 640)]
    assert status == ["confirmed", "wrong position", "not found by detector", "not checkable"]
