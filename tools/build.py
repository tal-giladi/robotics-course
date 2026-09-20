#!/usr/bin/env python3
"""Build derived course files from curriculum/syllabus.yaml.

Generates:
  curriculum/graph.json            machine-readable course graph (read by course.py and AI agents)
  curriculum/dependency-graph.md   Mermaid dependency graphs + prerequisite tables
  curriculum/concept-index.md      concept -> lesson index
  _sidebar.md                      docsify navigation
  <module dir>/README.md           module index pages
  COURSE_MAP.md (generated part)   between <!-- lessons:start --> / <!-- lessons:end -->
  every lesson's <!-- glance --> and <!-- prereqs --> blocks

Usage:
  python tools/build.py                 write everything
  python tools/build.py --check         exit 1 if anything is stale or invalid (CI)
  python tools/build.py --only a.md     only inject the generated blocks into these lesson files

Requires PyYAML (maintainers only; course.py itself is stdlib-only).
"""
from __future__ import annotations

import argparse
import json
import os
import re
import sys
from collections import defaultdict
from pathlib import Path

try:
    import yaml
except ImportError:  # pragma: no cover
    sys.exit("PyYAML is required: pip install pyyaml")

ROOT = Path(__file__).resolve().parent.parent
SYLLABUS = ROOT / "curriculum" / "syllabus.yaml"
DIFFICULTY = ("beginner", "intermediate", "advanced")
EXERCISE_RE = re.compile(r"^###\s+Exercise\s+([0-9A-Z]+\.[0-9]+-E[0-9]+|P[0-9]+-E[0-9]+)\s*[—-]+\s*(.+?)\s*$", re.M)
TYPE_TAG_RE = re.compile(r"`\[(hardware|simulation|coding|numerical|debugging|predict|design)\]`")


class BuildError(Exception):
    pass


def load_syllabus() -> dict:
    with SYLLABUS.open(encoding="utf-8") as f:
        return yaml.safe_load(f)


def fmt_time(minutes: int) -> str:
    if minutes < 60:
        return f"{minutes} min"
    h, m = divmod(minutes, 60)
    return f"{h} h" if m == 0 else f"{h} h {m} min"


def build_graph(syl: dict) -> dict:
    nodes: dict[str, dict] = {}
    modules = []
    order = 0
    hardware = syl.get("hardware", {})
    errors: list[str] = []

    for mod in syl["modules"]:
        mod_entry = {
            "id": str(mod["id"]),
            "dir": mod["dir"],
            "track": mod["track"],
            "title": mod["title"],
            "summary": mod.get("summary", ""),
            "lessons": [],
        }
        for les in mod["lessons"]:
            lid = str(les["id"])
            if lid in nodes:
                errors.append(f"duplicate id {lid}")
            for key in ("slug", "title", "difficulty", "time"):
                if key not in les:
                    errors.append(f"{lid}: missing {key}")
            if les.get("difficulty") not in DIFFICULTY:
                errors.append(f"{lid}: bad difficulty {les.get('difficulty')}")
            path = f"{mod['dir']}/{lid}-{les['slug']}.md"
            nodes[lid] = {
                "id": lid,
                "kind": "lesson" if mod["track"] == "main" else "foundation",
                "module": mod_entry["id"],
                "title": les["title"],
                "path": path,
                "difficulty": les["difficulty"],
                "time": int(les["time"]),
                "requires": [str(x) for x in les.get("requires", [])],
                "optional": [str(x) for x in les.get("optional", [])],
                "hardware": list(les.get("hardware", [])),
                "software": list(les.get("software", [])),
                "teaches": list(les.get("teaches", [])),
                "skip_if": les.get("skip_if", ""),
                "version_sensitive": bool(les.get("version_sensitive", False)),
                "order": order,
                "exercises": [],
            }
            order += 1
            mod_entry["lessons"].append(lid)
        modules.append(mod_entry)

    for proj in syl.get("projects", []):
        pid = str(proj["id"])
        nodes[pid] = {
            "id": pid,
            "kind": "project",
            "module": "projects",
            "title": proj["title"],
            "path": f"projects/{pid}-{proj['slug']}.md",
            "difficulty": proj["difficulty"],
            "time": int(proj["time"]),
            "requires": [str(x) for x in proj.get("requires", [])],
            "optional": [],
            "hardware": list(proj.get("hardware", [])),
            "software": list(proj.get("software", [])),
            "teaches": [],
            "skip_if": "",
            "version_sensitive": False,
            "order": order,
            "exercises": [],
        }
        order += 1

    for n in nodes.values():
        for dep in n["requires"] + n["optional"]:
            if dep not in nodes:
                errors.append(f"{n['id']}: unknown prerequisite {dep}")
        for hw in n["hardware"]:
            if hw not in hardware:
                errors.append(f"{n['id']}: unknown hardware id {hw}")
        for dep in n["optional"]:
            if dep in nodes and nodes[dep]["kind"] != "foundation":
                errors.append(f"{n['id']}: optional prerequisite {dep} should be a foundation lesson")

    # cycles (hard + optional edges together must be acyclic)
    state: dict[str, int] = {}

    def visit(nid: str, stack: list[str]) -> None:
        state[nid] = 1
        for dep in nodes[nid]["requires"] + nodes[nid]["optional"]:
            if dep not in nodes:
                continue
            if state.get(dep) == 1:
                cyc = stack[stack.index(dep):] + [dep] if dep in stack else stack + [dep]
                errors.append("cycle: " + " -> ".join(cyc))
            elif state.get(dep) is None:
                visit(dep, stack + [dep])
        state[nid] = 2

    for nid in nodes:
        if state.get(nid) is None:
            visit(nid, [nid])

    concepts: dict[str, list[str]] = defaultdict(list)
    for n in nodes.values():
        for c in n["teaches"]:
            concepts[c].append(n["id"])

    for n in nodes.values():
        p = ROOT / n["path"]
        if p.exists():
            text = p.read_text(encoding="utf-8")
            for m in EXERCISE_RE.finditer(text):
                ex_id, title = m.group(1), m.group(2)
                tag = TYPE_TAG_RE.search(title)
                n["exercises"].append({
                    "id": ex_id,
                    "title": TYPE_TAG_RE.sub("", title).strip(),
                    "type": tag.group(1) if tag else "exercise",
                })
        if (ROOT / "labs" / "exercises" / n["id"]).is_dir():
            n["checker"] = f"labs/exercises/{n['id']}"

    unlocks: dict[str, list[str]] = defaultdict(list)
    for n in nodes.values():
        for dep in n["requires"]:
            unlocks[dep].append(n["id"])
    for n in nodes.values():
        n["unlocks"] = sorted(unlocks.get(n["id"], []), key=lambda x: nodes[x]["order"])

    if errors:
        raise BuildError("\n".join(errors))

    return {
        "generated_by": "tools/build.py — do not edit by hand; edit curriculum/syllabus.yaml",
        "course": syl["course"],
        "tracks": syl["tracks"],
        "modules": modules,
        "hardware": hardware,
        "nodes": nodes,
        "concepts": dict(sorted(concepts.items())),
    }


def rel(from_path: str, to_path: str) -> str:
    return os.path.relpath(ROOT / to_path, (ROOT / from_path).parent).replace(os.sep, "/")


def link(graph: dict, from_path: str, nid: str) -> str:
    n = graph["nodes"][nid]
    return f"[{nid} {n['title']}]({rel(from_path, n['path'])})"


def glance_block(graph: dict, n: dict) -> str:
    hw = graph["hardware"]
    hardware = ", ".join(f"{hw[h]['title']} (stage {hw[h]['stage']})" for h in n["hardware"]) or "none"
    software = ", ".join(n["software"]) or "none"
    kind = {"lesson": "Main path", "foundation": "Optional foundation", "project": "Project"}[n["kind"]]
    lines = [
        "<!-- glance:start -->",
        "<!-- generated by tools/build.py from curriculum/syllabus.yaml — edit the syllabus, not this block -->",
        "",
        "| | |",
        "|---|---|",
        f"| **Track** | {kind} |",
        f"| **Difficulty** | {n['difficulty'].capitalize()} |",
        f"| **Time** | {fmt_time(n['time'])} |",
        f"| **Hardware** | {hardware} |",
        f"| **Software** | {software} |",
    ]
    if n["requires"]:
        lines.append("| **Prerequisites** | " + "<br>".join(link(graph, n["path"], d) for d in n["requires"]) + " |")
    else:
        lines.append("| **Prerequisites** | none |")
    if n["optional"]:
        lines.append("| **Optional prerequisites** | " + "<br>".join(link(graph, n["path"], d) for d in n["optional"]) + " |")
    if n["version_sensitive"]:
        lines.append("| **Version-sensitive** | yes — see [curriculum/versions.yaml](" + rel(n["path"], "curriculum/versions.yaml") + ") |")
    if n["skip_if"]:
        lines.append(f"| **Skip if** | {n['skip_if']} |")
    lines += ["", "<!-- glance:end -->"]
    return "\n".join(lines)


def prereqs_block(graph: dict, n: dict) -> str:
    lines = ["<!-- prereqs:start -->", "## Prerequisites", ""]
    if n["requires"]:
        lines.append("You need these first:")
        lines.append("")
        for d in n["requires"]:
            lines.append(f"- {link(graph, n['path'], d)}")
    else:
        lines.append("None — you can start here.")
    lines += ["", "### Optional prerequisite links", ""]
    if n["optional"]:
        lines.append("New to any of these? Take the foundation lesson first; skip it if you already know it.")
        lines.append("")
        for d in n["optional"]:
            lines.append(f"- {link(graph, n['path'], d)}")
    else:
        lines.append("None.")
    lines += ["", f"Check your readiness: `python course.py why {n['id']}`", "<!-- prereqs:end -->"]
    return "\n".join(lines)


def replace_block(text: str, name: str, block: str) -> str:
    pattern = re.compile(rf"<!-- {name}:start -->.*?<!-- {name}:end -->", re.S)
    if pattern.search(text):
        return pattern.sub(lambda _: block, text, count=1)
    return text


def inject(graph: dict, check: bool, only: set[str] | None = None) -> list[str]:
    stale = []
    for n in graph["nodes"].values():
        if only is not None and n["path"] not in only:
            continue
        p = ROOT / n["path"]
        if not p.exists():
            continue
        text = p.read_text(encoding="utf-8")
        new = text
        if "<!-- glance:start -->" not in new:
            new = re.sub(r"^(# .+\n)", lambda m: m.group(1) + "\n<!-- glance:start -->\n<!-- glance:end -->\n", new, count=1, flags=re.M)
        new = replace_block(new, "glance", glance_block(graph, n))
        if n["kind"] != "project":
            new = replace_block(new, "prereqs", prereqs_block(graph, n))
        if new != text:
            stale.append(n["path"])
            if not check:
                p.write_text(new, encoding="utf-8", newline="\n")
    return stale


def sidebar(graph: dict) -> str:
    out = [
        "<!-- generated by tools/build.py -->",
        "- [Home](README.md)",
        "- [Course map](COURSE_MAP.md)",
        "- [Progress](PROGRESS.md)",
        "- [Hardware](HARDWARE.md)",
        "- [Safety](SAFETY.md)",
        "- [Glossary](references/glossary.md)",
        "",
        "- **Main path**",
    ]
    for mod in graph["modules"]:
        if mod["track"] != "main":
            continue
        out.append(f"  - [**{mod['id']} · {mod['title']}**]({mod['dir']}/README.md)")
        for lid in mod["lessons"]:
            n = graph["nodes"][lid]
            out.append(f"    - [{lid} {n['title']}]({n['path']})")
    out.append("- **Projects**")
    out.append("  - [Projects overview](projects/README.md)")
    for n in graph["nodes"].values():
        if n["kind"] == "project":
            out.append(f"  - [{n['title']}]({n['path']})")
    out.append("- **Optional foundations**")
    out.append("  - [Foundations overview](optional-foundations/README.md)")
    for mod in graph["modules"]:
        if mod["track"] != "foundations":
            continue
        out.append(f"  - [**{mod['title'].replace('Foundations: ', '')}**]({mod['dir']}/README.md)")
        for lid in mod["lessons"]:
            n = graph["nodes"][lid]
            out.append(f"    - [{lid} {n['title']}]({n['path']})")
    out += [
        "- **Reference**",
        "  - [Papers](references/papers.md)",
        "  - [Resources](references/resources.md)",
        "  - [Troubleshooting](references/troubleshooting/README.md)",
        "  - [Dependency graph](curriculum/dependency-graph.md)",
        "  - [Concept index](curriculum/concept-index.md)",
        "  - [Labs](labs/README.md)",
        "  - [Maintaining the course](MAINTAINING.md)",
    ]
    return "\n".join(out) + "\n"


def course_map_tables(graph: dict) -> str:
    out = ["<!-- lessons:start -->", "<!-- generated by tools/build.py -->", ""]
    for track, heading in (("main", "Main path"), ("foundations", "Optional foundations")):
        out.append(f"## {heading} — every lesson")
        out.append("")
        for mod in graph["modules"]:
            if mod["track"] != track:
                continue
            total = sum(graph["nodes"][l]["time"] for l in mod["lessons"])
            out.append(f"### {mod['id']} · {mod['title']}")
            out.append("")
            out.append(f"{mod['summary']} *({len(mod['lessons'])} lessons, ≈ {fmt_time(total)})*")
            out.append("")
            out.append("| Id | Lesson | Level | Time | Needs | Hardware |")
            out.append("|---|---|---|---|---|---|")
            for lid in mod["lessons"]:
                n = graph["nodes"][lid]
                needs = ", ".join(n["requires"]) or "—"
                hw = ", ".join(n["hardware"]) or "—"
                out.append(f"| {lid} | [{n['title']}]({n['path']}) | {n['difficulty']} | {fmt_time(n['time'])} | {needs} | {hw} |")
            out.append("")
    out.append("## Projects")
    out.append("")
    out.append("| Id | Project | Level | Time | Needs | Hardware |")
    out.append("|---|---|---|---|---|---|")
    for n in graph["nodes"].values():
        if n["kind"] == "project":
            out.append(f"| {n['id']} | [{n['title']}]({n['path']}) | {n['difficulty']} | {fmt_time(n['time'])} | {', '.join(n['requires'])} | {', '.join(n['hardware']) or '—'} |")

    def count(kind: str) -> int:
        return sum(1 for n in graph["nodes"].values() if n["kind"] == kind)

    def minutes(kind: str) -> int:
        return sum(n["time"] for n in graph["nodes"].values() if n["kind"] == kind)

    out += [
        "",
        "## Totals",
        "",
        f"- Main path: {count('lesson')} lessons, ≈ {minutes('lesson') // 60} hours",
        f"- Optional foundations: {count('foundation')} lessons, ≈ {minutes('foundation') // 60} hours (take only what you need)",
        f"- Projects: {count('project')}, ≈ {minutes('project') // 60} hours",
        "",
        "<!-- lessons:end -->",
    ]
    return "\n".join(out)


def mermaid_id(nid: str) -> str:
    return "n_" + nid.replace(".", "_")


def dependency_graph_md(graph: dict) -> str:
    nodes = graph["nodes"]
    out = [
        "# Dependency graph",
        "",
        "<!-- generated by tools/build.py from curriculum/syllabus.yaml — do not edit -->",
        "",
        "The course is a directed acyclic graph. **Solid arrows** are hard prerequisites; **dotted arrows** are optional foundation lessons that help if the topic is new.",
        "The same data is in [`graph.json`](graph.json) for tools and AI agents. From the command line:",
        "",
        "```bash",
        "python course.py why 11.07          # everything 11.07 depends on, with your status",
        "python course.py learn quaternions  # which lesson teaches a concept, and what it needs",
        "```",
        "",
        "## Module-level roadmap",
        "",
        "```mermaid",
        "flowchart TD",
    ]
    mod_of = {n["id"]: n["module"] for n in nodes.values()}
    edges = set()
    fedges = set()
    for n in nodes.values():
        if n["kind"] != "lesson":
            continue
        for d in n["requires"]:
            a, b = mod_of[d], n["module"]
            if a != b and nodes[d]["kind"] == "lesson":
                edges.add((a, b))
        for d in n["optional"]:
            fedges.add((mod_of[d], n["module"]))
    for m in graph["modules"]:
        label = m["title"].replace('"', "'")
        out.append(f'  M{m["id"]}["{m["id"]} · {label}"]')
    for a, b in sorted(edges):
        out.append(f"  M{a} --> M{b}")
    for a, b in sorted(fedges):
        out.append(f"  M{a} -.-> M{b}")
    out += ["```", ""]

    for mod in graph["modules"]:
        out.append(f"## {mod['id']} · {mod['title']}")
        out.append("")
        out.append("```mermaid")
        out.append("flowchart LR")
        ids_in = set(mod["lessons"])
        external = set()
        for lid in mod["lessons"]:
            title = nodes[lid]["title"].replace('"', "'")
            out.append(f'  {mermaid_id(lid)}["{lid} {title}"]')
        for lid in mod["lessons"]:
            n = nodes[lid]
            for d in n["requires"]:
                if d not in ids_in:
                    external.add(d)
                out.append(f"  {mermaid_id(d)} --> {mermaid_id(lid)}")
            for d in n["optional"]:
                if d not in ids_in:
                    external.add(d)
                out.append(f"  {mermaid_id(d)} -.-> {mermaid_id(lid)}")
        for d in sorted(external, key=lambda x: nodes[x]["order"]):
            title = nodes[d]["title"].replace('"', "'")
            out.append(f'  {mermaid_id(d)}(["{d} {title}"])')
        out.append("```")
        out.append("")
        out.append("| Lesson | Requires | Optional | Unlocks |")
        out.append("|---|---|---|---|")
        for lid in mod["lessons"]:
            n = nodes[lid]
            out.append(f"| {lid} {n['title']} | {', '.join(n['requires']) or '—'} | {', '.join(n['optional']) or '—'} | {', '.join(n['unlocks']) or '—'} |")
        out.append("")
    return "\n".join(out)


def concept_index_md(graph: dict) -> str:
    nodes = graph["nodes"]
    out = [
        "# Concept index",
        "",
        "<!-- generated by tools/build.py — do not edit -->",
        "",
        "Every concept the course teaches and the lesson that owns it. `python course.py learn <concept>` searches this.",
        "",
    ]
    by_letter: dict[str, list[str]] = defaultdict(list)
    for c in graph["concepts"]:
        by_letter[c[0].upper()].append(c)
    for letter in sorted(by_letter):
        out.append(f"## {letter}")
        out.append("")
        for c in sorted(by_letter[letter]):
            refs = ", ".join(f"[{nid}]({rel('curriculum/concept-index.md', nodes[nid]['path'])})" for nid in graph["concepts"][c])
            out.append(f"- **{c}** — {refs}")
        out.append("")
    return "\n".join(out)


def module_readme(graph: dict, mod: dict) -> str:
    nodes = graph["nodes"]
    total = sum(nodes[l]["time"] for l in mod["lessons"])
    up = "../" * (mod["dir"].count("/") + 1)
    out = [
        f"# {mod['id']} · {mod['title']}",
        "",
        "<!-- generated by tools/build.py from curriculum/syllabus.yaml — do not edit -->",
        "",
        mod["summary"],
        "",
        f"**{len(mod['lessons'])} lessons · ≈ {fmt_time(total)}** · "
        f"[Course map]({up}COURSE_MAP.md) · [Dependency graph]({up}curriculum/dependency-graph.md)",
        "",
        "| Id | Lesson | Level | Time | Prerequisites | Hardware |",
        "|---|---|---|---|---|---|",
    ]
    for lid in mod["lessons"]:
        n = nodes[lid]
        fname = n["path"].split("/")[-1]
        needs = ", ".join(n["requires"]) or "—"
        hw = ", ".join(n["hardware"]) or "—"
        out.append(f"| {lid} | [{n['title']}]({fname}) | {n['difficulty']} | {fmt_time(n['time'])} | {needs} | {hw} |")
    out += ["", f"Start with `python course.py show {mod['lessons'][0]}`."]
    return "\n".join(out) + "\n"


def write_or_check(path: Path, content: str, check: bool, stale: list[str]) -> None:
    old = path.read_text(encoding="utf-8") if path.exists() else None
    if old != content:
        stale.append(str(path.relative_to(ROOT)))
        if not check:
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_text(content, encoding="utf-8", newline="\n")


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--check", action="store_true", help="fail if generated files are stale")
    ap.add_argument("--only", nargs="+", help="only inject blocks into these lesson files (safe while others write)")
    args = ap.parse_args()
    if hasattr(sys.stdout, "reconfigure"):
        sys.stdout.reconfigure(encoding="utf-8")

    try:
        graph = build_graph(load_syllabus())
    except BuildError as e:
        print("syllabus errors:\n" + str(e), file=sys.stderr)
        return 1

    if args.only:
        only = {Path(x).resolve().relative_to(ROOT).as_posix() for x in args.only}
        unknown = only - {n["path"] for n in graph["nodes"].values()}
        if unknown:
            print("not in syllabus: " + ", ".join(sorted(unknown)), file=sys.stderr)
            return 1
        changed = inject(graph, False, only)
        print(f"injected blocks into {len(changed)} of {len(only)} files")
        return 0

    stale: list[str] = []
    stale += inject(graph, args.check)
    write_or_check(ROOT / "curriculum" / "graph.json", json.dumps(graph, indent=1, ensure_ascii=False) + "\n", args.check, stale)
    write_or_check(ROOT / "_sidebar.md", sidebar(graph), args.check, stale)
    for mod in graph["modules"]:
        write_or_check(ROOT / mod["dir"] / "README.md", module_readme(graph, mod), args.check, stale)
    write_or_check(ROOT / "curriculum" / "dependency-graph.md", dependency_graph_md(graph) + "\n", args.check, stale)
    write_or_check(ROOT / "curriculum" / "concept-index.md", concept_index_md(graph) + "\n", args.check, stale)

    cm = ROOT / "COURSE_MAP.md"
    if cm.exists():
        text = cm.read_text(encoding="utf-8")
        new = replace_block(text, "lessons", course_map_tables(graph))
        if new != text:
            stale.append("COURSE_MAP.md")
            if not args.check:
                cm.write_text(new, encoding="utf-8", newline="\n")

    missing = [n["path"] for n in graph["nodes"].values() if not (ROOT / n["path"]).exists()]
    total = len(graph["nodes"])
    print(f"{total} nodes, {total - len(missing)} files present, {len(missing)} missing")
    if args.check:
        if stale:
            print("stale generated files (run python tools/build.py):\n  " + "\n  ".join(stale), file=sys.stderr)
            return 1
        if missing:
            print("missing lesson files:\n  " + "\n  ".join(missing), file=sys.stderr)
            return 1
    elif stale:
        print(f"updated {len(stale)} files")
    return 0


if __name__ == "__main__":
    sys.exit(main())
