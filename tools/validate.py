#!/usr/bin/env python3
"""Validate course content.

  python tools/validate.py                      all lesson/project files that exist
  python tools/validate.py --paths a.md b.md    only these files
  python tools/validate.py --links              also check relative links in every markdown file
  python tools/validate.py --final              links to planned-but-unwritten lessons are errors
  python tools/validate.py --versions           list every Version-sensitive marker
  python tools/validate.py --external           HTTP-check external URLs (slow; needs network)
  python tools/validate.py --strict             also fail on warnings

Exit code 1 on errors.
"""
from __future__ import annotations

import argparse
import concurrent.futures as cf
import json
import re
import sys
import urllib.error
import urllib.request
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
GRAPH = ROOT / "curriculum" / "graph.json"

LESSON_HEADINGS = [
    "What you will learn",
    "Why it matters",
    "Prerequisites",
    "Concept",
    "Technical explanation",
    "Diagram",
    "Code",
    "Exercise",
    "Expected result",
    "Troubleshooting",
    "Common mistakes",
    "Knowledge check",
    "Practical challenge",
    "You can skip this if",
    "Go deeper",
    "Progress checkpoint",
]
PROJECT_HEADINGS = [
    "Goal",
    "Why this project",
    "Prerequisites",
    "Hardware and software",
    "Architecture",
    "Milestones",
    "Acceptance criteria",
    "Safety",
    "Troubleshooting",
    "Stretch goals",
    "Evidence to keep",
    "Progress checkpoint",
]

LINK_RE = re.compile(r"(?<!!)\[[^\]]*\]\(([^)\s]+)(?:\s+\"[^\"]*\")?\)")
IMG_RE = re.compile(r"!\[[^\]]*\]\(([^)\s]+)\)")
URL_RE = re.compile(r"https?://[^\s)>\]\"'`]+")
FENCE_RE = re.compile(r"^(```|~~~).*?^\1", re.S | re.M)
PLANNED: set[Path] = set()
FINAL = False


def strip_code(text: str) -> str:
    return FENCE_RE.sub("", text)


def load_graph() -> dict:
    return json.loads(GRAPH.read_text(encoding="utf-8"))


def section(body: str, name: str) -> str | None:
    m = re.search(rf"^## {re.escape(name)}.*?$(.*?)(?=^## |\Z)", body, re.S | re.M)
    return m.group(1) if m else None


def check_lesson(path: Path, node: dict) -> tuple[list[str], list[str]]:
    errors, warnings = [], []
    text = path.read_text(encoding="utf-8")
    body = strip_code(text)
    rel = path.relative_to(ROOT).as_posix()
    kind = node["kind"]

    h1 = re.search(r"^# (.+)$", body, re.M)
    if not h1:
        errors.append("missing H1 title")
    elif not h1.group(1).startswith(node["id"]):
        errors.append(f"H1 should start with the id {node['id']}: '{h1.group(1)}'")

    if "<!-- glance:start -->" not in text:
        errors.append("missing <!-- glance:start --> block (run tools/build.py)")
    if kind != "project" and "<!-- prereqs:start -->" not in text:
        errors.append("missing <!-- prereqs:start --><!-- prereqs:end --> markers after 'Why it matters'")

    headings = [h.strip() for h in re.findall(r"^## (.+)$", body, re.M)]
    expected = PROJECT_HEADINGS if kind == "project" else LESSON_HEADINGS
    pos = -1
    for exp in expected:
        idx = next((i for i, h in enumerate(headings) if h.startswith(exp)), None)
        if idx is None:
            errors.append(f"missing section '## {exp}'")
        elif idx < pos:
            errors.append(f"section '## {exp}' is out of order")
        else:
            pos = idx

    if kind != "project":
        exercises = re.findall(r"^### Exercise (\S+)", body, re.M)
        if not exercises:
            errors.append("no '### Exercise <id>-E<n> — title' headings")
        for ex in exercises:
            if not re.fullmatch(re.escape(node["id"]) + r"-E\d+", ex):
                errors.append(f"exercise id '{ex}' should look like {node['id']}-E1")
        details = len(re.findall(r"<details>", section(text, "Knowledge check") or ""))
        if details < 5:
            errors.append(f"knowledge check needs ≥5 questions with <details> answers (found {details})")
        deeper = section(body, "Go deeper")
        if deeper is not None and len(URL_RE.findall(deeper)) < 2:
            warnings.append("'Go deeper' has fewer than 2 external URLs")
        expected_result = section(body, "Expected result")
        if expected_result is not None:
            for ex in exercises:
                if ex not in expected_result:
                    warnings.append(f"Expected result does not mention {ex}")
        if "```mermaid" not in text and "```text" not in text and "![" not in text:
            errors.append("no diagram (mermaid, ```text or image)")
        if "Ask your teacher" not in text:
            warnings.append("no 'Ask your teacher' tip")
        if node.get("version_sensitive") and "Version-sensitive" not in text:
            errors.append("syllabus marks this version-sensitive but there is no '**Version-sensitive**' marker")
        if node["hardware"] and "[!CAUTION]" not in text and "[!WARNING]" not in text:
            warnings.append("uses hardware but has no safety callout ([!CAUTION] / [!WARNING])")

    for bad in ("TODO", "TBD", "lorem ipsum", "Search for ", "search for "):
        for line in body.splitlines():
            if bad in line and "TODO(student)" not in line:
                warnings.append(f"contains '{bad.strip()}': {line.strip()[:80]}")
                break

    words = len(re.findall(r"\w+", body))
    if words < 900 and kind != "project":
        warnings.append(f"short lesson ({words} words)")
    errors += check_links(path, text)
    return [f"{rel}: {e}" for e in errors], [f"{rel}: {w}" for w in warnings]


def check_links(path: Path, text: str) -> list[str]:
    errors = []
    body = strip_code(text)
    body = re.sub(r"`[^`\n]*`", "", body)
    for target in LINK_RE.findall(body) + IMG_RE.findall(body):
        if re.match(r"^(https?:|mailto:|#)", target):
            if target.startswith("http://") and "localhost" not in target:
                errors.append(f"use https: {target}")
            continue
        t = target.split("#")[0]
        if not t:
            continue
        resolved = (path.parent / t).resolve()
        if not resolved.exists():
            if not FINAL and resolved in PLANNED:
                continue  # planned lesson not written yet; --final makes this an error
            errors.append(f"broken link: {target}")
    return errors


def all_markdown() -> list[Path]:
    skip = {".git", "node_modules", "build", "install", "log", ".venv", "research"}
    return [p for p in ROOT.rglob("*.md") if not any(part in skip for part in p.relative_to(ROOT).parts)]


def check_external(paths: list[Path]) -> list[str]:
    urls: dict[str, list[str]] = {}
    for p in paths:
        for u in URL_RE.findall(p.read_text(encoding="utf-8")):
            u = u.rstrip(".,;:*_")
            if any(x in u for x in ("localhost", "127.0.0.1", "example.com", "<", "{", "0.0.0.0")):
                continue
            urls.setdefault(u, []).append(p.relative_to(ROOT).as_posix())

    def probe(u: str) -> tuple[str, str | None]:
        last = "unreachable"
        for method in ("HEAD", "GET"):
            req = urllib.request.Request(u, method=method, headers={"User-Agent": "Mozilla/5.0 (course link checker)"})
            try:
                with urllib.request.urlopen(req, timeout=25) as r:
                    if r.status < 400:
                        return u, None
            except urllib.error.HTTPError as e:
                if e.code in (401, 403, 405, 429, 999):
                    if method == "GET":
                        return u, None  # bot protection / auth wall: treat as alive
                    continue
                last = f"HTTP {e.code}"
            except Exception as e:  # noqa: BLE001
                last = type(e).__name__
        return u, last

    errors = []
    with cf.ThreadPoolExecutor(max_workers=16) as ex:
        for u, err in ex.map(probe, sorted(urls)):
            if err:
                errors.append(f"{err}: {u}  (in {', '.join(sorted(set(urls[u]))[:3])})")
    return errors


def main() -> int:
    global FINAL
    ap = argparse.ArgumentParser()
    ap.add_argument("--paths", nargs="*")
    ap.add_argument("--links", action="store_true")
    ap.add_argument("--versions", action="store_true")
    ap.add_argument("--external", action="store_true")
    ap.add_argument("--strict", action="store_true")
    ap.add_argument("--final", action="store_true")
    ap.add_argument("--quiet", action="store_true", help="hide warnings")
    args = ap.parse_args()
    FINAL = args.final
    if hasattr(sys.stdout, "reconfigure"):
        sys.stdout.reconfigure(encoding="utf-8")

    g = load_graph()
    by_path = {n["path"]: n for n in g["nodes"].values()}
    PLANNED.update((ROOT / n["path"]).resolve() for n in g["nodes"].values())
    errors: list[str] = []
    warnings: list[str] = []

    if args.versions:
        for p in sorted(all_markdown()):
            for i, line in enumerate(p.read_text(encoding="utf-8").splitlines(), 1):
                if "Version-sensitive" in line:
                    print(f"{p.relative_to(ROOT).as_posix()}:{i}: {line.strip()[:140]}")
        return 0

    if args.paths:
        targets = [(ROOT / x).resolve() if not Path(x).is_absolute() else Path(x) for x in args.paths]
    else:
        targets = [ROOT / n["path"] for n in g["nodes"].values() if (ROOT / n["path"]).exists()]

    for p in targets:
        rel = p.relative_to(ROOT).as_posix()
        if rel in by_path:
            e, w = check_lesson(p, by_path[rel])
            errors += e
            warnings += w
        else:
            errors += [f"{rel}: {x}" for x in check_links(p, p.read_text(encoding="utf-8"))]

    if args.links:
        lesson_paths = {ROOT / n["path"] for n in g["nodes"].values()}
        for p in all_markdown():
            if p not in lesson_paths:
                errors += [f"{p.relative_to(ROOT).as_posix()}: {x}" for x in check_links(p, p.read_text(encoding="utf-8"))]

    if args.external:
        errors += check_external(targets if args.paths else all_markdown())

    if not args.quiet:
        for w in warnings:
            print("warning: " + w)
    for e in errors:
        print("ERROR: " + e)
    print(f"{len(targets)} files checked, {len(errors)} errors, {len(warnings)} warnings")
    return 1 if errors or (args.strict and warnings) else 0


if __name__ == "__main__":
    sys.exit(main())
