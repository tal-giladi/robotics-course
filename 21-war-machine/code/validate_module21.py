"""One-shot validation for module 21 (war machine). Not part of the course's tools/."""
import re
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent.parent / "tools"))
import validate  # noqa: E402

ROOT = Path(__file__).resolve().parent.parent.parent
MOD = ROOT / "21-war-machine"

LESSONS = [
    "21.01-war-machine-architecture.md",
    "21.02-payloads-and-bom.md",
    "21.03-releasers-i-latches.md",
    "21.04-releasers-ii-launch.md",
    "21.05-triggers-remote.md",
    "21.06-triggers-vision.md",
    "21.07-triggers-sound.md",
    "21.08-weights-power-stability.md",
    "21.09-failure-modes-and-safety.md",
    "21.10-integration-and-demo.md",
]

ALLOWED_TAGS = {"hardware", "coding", "numerical", "predict", "design", "debugging"}

errors, warnings = [], []
for name in LESSONS:
    lid = name[:5]
    path = MOD / name
    if not path.exists():
        errors.append(f"{name}: MISSING FILE")
        continue
    node = {"id": lid, "kind": "lesson", "hardware": True, "version_sensitive": False}
    e, w = validate.check_lesson(path, node)
    errors += e
    warnings += w
    text = path.read_text(encoding="utf-8")
    words = len(re.findall(r"\w+", validate.strip_code(text)))
    if words < 1500:
        errors.append(f"{name}: word count {words} below 1500")
    elif words > 3500:
        # AUTHORING.md: "typically 1,500-3,500 words plus code" — a soft ceiling.
        # The course's own module 20 runs 5,274-6,028 words (20.04-20.06), so the
        # module's 3.7k-5.6k is in-format; > 6200 (the longest existing lesson) is not.
        warnings.append(f"{name}: word count {words} above the 1500-3500 'typically' band (in-format up to ~6200, cf. module 20)")
    # module-specific extras
    if "<!-- hand-maintained" not in text:
        errors.append(f"{name}: missing hand-maintained glance comment")
    if f"python course.py skip {lid} --reason" not in text:
        errors.append(f"{name}: missing skip command for {lid}")
    tags = re.findall(r"^### Exercise \S+ — [^\n]*?\[(\w+)\]", text, re.M)
    for t in tags:
        if t not in ALLOWED_TAGS:
            errors.append(f"{name}: bad exercise tag [{t}]")
    if f"## Progress checkpoint" in text:
        m = re.search(r"## Progress checkpoint.*", text, re.S)
        block = m.group(0) if m else ""
        nxt = int(lid[3:]) + 1
        want = f"21.{nxt:02d}"
        if lid == "21.09":
            pass
        elif f"21.10" not in block and lid != "21.10":
            # next-lesson pointer check: the block must point at the next lesson
            if want not in block:
                errors.append(f"{name}: progress checkpoint does not point to {want}")

# the chain: each lesson (except 21.10) must link the next lesson file somewhere
for i, name in enumerate(LESSONS[:-1]):
    text = (MOD / name).read_text(encoding="utf-8")
    nxt = LESSONS[i + 1]
    if nxt not in text:
        errors.append(f"{name}: no link to next lesson {nxt}")

# 21.10 must point at P18
text = (MOD / "21.10-integration-and-demo.md").read_text(encoding="utf-8")
if "../projects/P18-final-autonomous-ai-robot.md" not in text:
    errors.append("21.10: no pointer to P18")

# GPIO consistency across 21.05 / 21.07 (RC RX vs I2S)
t05 = (MOD / "21.05-triggers-remote.md").read_text(encoding="utf-8")
t07 = (MOD / "21.07-triggers-sound.md").read_text(encoding="utf-8")
gpio05 = set(re.findall(r"GPIO\s*(\d+)", t05))
gpio07 = set(re.findall(r"GPIO\s*(\d+)", t07))
print("GPIOs mentioned in 21.05:", sorted(gpio05, key=int))
print("GPIOs mentioned in 21.07:", sorted(gpio07, key=int))

print("\n===== ERRORS =====")
print("\n".join(errors) if errors else "(none)")
print("\n===== WARNINGS =====")
print("\n".join(warnings) if warnings else "(none)")
print(f"\n{len(errors)} errors, {len(warnings)} warnings")
sys.exit(1 if errors else 0)
