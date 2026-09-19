#!/usr/bin/env python3
"""course.py — track where you are in the robotics course and what to study next.

Standard library only. Reads curriculum/graph.json (generated from curriculum/syllabus.yaml)
and reads/writes progress/progress.json. Every change regenerates PROGRESS.md.

Quick reference
  python course.py status                 where am I?
  python course.py next                   what should I study next?
  python course.py start 04.3             mark a lesson in progress
  python course.py read 04.3              I read it (not yet practiced)
  python course.py complete 04.3          I did the exercises  ("I can do this")
  python course.py master 04.3            I did the practical challenge
  python course.py exercise 04.03-E2      mark one exercise done
  python course.py quiz 04.3 7/8          record a knowledge-check score
  python course.py skip FM.5 --reason "I know vectors"
  python course.py struggle 10.5 "covariance update makes no sense"
  python course.py resolve 10.5           the struggle is resolved
  python course.py why 11.7               prerequisite tree with your status
  python course.py learn quaternions      which lesson teaches a concept
  python course.py show 08.4              lesson details
  python course.py list [08|main|FM]      lessons with status
  python course.py project P04 start|done
  python course.py hw                     hardware status;  hw buy lidar / hw assemble lidar
  python course.py check 09.4             run the automated checker for an exercise lesson
  python course.py skills                 concepts you have practiced
  python course.py log                    recent activity
  python course.py reset 04.3             forget progress on a lesson
  python course.py render                 regenerate PROGRESS.md
"""
from __future__ import annotations

import argparse
import datetime as dt
import json
import os
import re
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent
GRAPH = ROOT / "curriculum" / "graph.json"
PROGRESS = Path(os.environ.get("COURSE_PROGRESS_FILE", ROOT / "progress" / "progress.json"))
PROGRESS_MD = Path(os.environ.get("COURSE_PROGRESS_MD", ROOT / "PROGRESS.md"))

STATUSES = ("not-started", "in-progress", "read", "practiced", "mastered", "skipped")
DONE = {"practiced", "mastered", "skipped"}          # satisfies a prerequisite
SOFT_DONE = DONE | {"read"}                           # satisfies it, with a warning
ICON = {
    "not-started": "⬜", "in-progress": "🟨", "read": "📖",
    "practiced": "✅", "mastered": "🏆", "skipped": "⏭️",
}
PROJECT_STATUSES = ("not-started", "in-progress", "done")


# --------------------------------------------------------------------------- data
def today() -> str:
    return dt.date.today().isoformat()


def load_graph() -> dict:
    if not GRAPH.exists():
        sys.exit("curriculum/graph.json is missing — run: python tools/build.py")
    return json.loads(GRAPH.read_text(encoding="utf-8"))


def empty_progress() -> dict:
    return {
        "schema": 1,
        "student": {"started": today()},
        "lessons": {},
        "exercises": {},
        "projects": {},
        "current_project": None,
        "hardware": {},
        "struggles": [],
        "log": [],
    }


def load_progress() -> dict:
    if PROGRESS.exists():
        data = json.loads(PROGRESS.read_text(encoding="utf-8"))
        base = empty_progress()
        base.update(data)
        return base
    return empty_progress()


def save_progress(p: dict, g: dict) -> None:
    PROGRESS.parent.mkdir(parents=True, exist_ok=True)
    PROGRESS.write_text(json.dumps(p, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    PROGRESS_MD.write_text(render_markdown(p, g), encoding="utf-8")


def log(p: dict, event: str) -> None:
    p["log"].append({"date": today(), "event": event})
    p["log"] = p["log"][-500:]


# --------------------------------------------------------------------------- ids
def normalize_id(raw: str, g: dict) -> str:
    s = raw.strip().upper()
    m = re.fullmatch(r"(\d{1,2})\.(\d{1,2})", s)
    if m:
        s = f"{int(m.group(1)):02d}.{int(m.group(2)):02d}"
    m = re.fullmatch(r"([A-Z3]+)\.(\d{1,2})", s)
    if m and not s[0].isdigit():
        s = f"{m.group(1)}.{int(m.group(2)):02d}"
    m = re.fullmatch(r"P(\d{1,2})", s)
    if m:
        s = f"P{int(m.group(1)):02d}"
    if s not in g["nodes"]:
        close = [k for k in g["nodes"] if k.startswith(s.split(".")[0])][:8]
        hint = f" Did you mean one of: {', '.join(close)}?" if close else ""
        sys.exit(f"Unknown lesson/project id '{raw}'.{hint}")
    return s


def normalize_exercise(raw: str, g: dict) -> tuple[str, str]:
    m = re.fullmatch(r"(.+?)-E(\d+)", raw.strip().upper())
    if not m:
        sys.exit(f"Exercise ids look like 04.03-E1, got '{raw}'")
    nid = normalize_id(m.group(1), g)
    return nid, f"{nid}-E{int(m.group(2))}"


# --------------------------------------------------------------------------- status helpers
def status(p: dict, nid: str) -> str:
    return p["lessons"].get(nid, {}).get("status", "not-started")


def node_done(p: dict, g: dict, nid: str, soft: bool = True) -> bool:
    n = g["nodes"][nid]
    if n["kind"] == "project":
        return p["projects"].get(nid, {}).get("status") == "done"
    return status(p, nid) in (SOFT_DONE if soft else DONE)


def set_status(p: dict, nid: str, new: str) -> None:
    entry = p["lessons"].setdefault(nid, {})
    entry["status"] = new
    entry.setdefault("history", []).append({"date": today(), "status": new})
    if new == "in-progress":
        entry.setdefault("started", today())
    else:
        entry[new] = today()


def link_path(g: dict, nid: str) -> str:
    return g["nodes"][nid]["path"]


def fmt_time(minutes: int) -> str:
    if minutes < 60:
        return f"{minutes} min"
    h, m = divmod(minutes, 60)
    return f"{h} h" if not m else f"{h} h {m} min"


def all_prereqs(g: dict, nid: str, include_optional: bool = False) -> list[str]:
    """Transitive hard prerequisites in a valid learning order."""
    seen: set[str] = set()
    order: list[str] = []

    def visit(x: str) -> None:
        n = g["nodes"][x]
        deps = n["requires"] + (n["optional"] if include_optional else [])
        for d in sorted(deps, key=lambda k: g["nodes"][k]["order"]):
            if d not in seen:
                seen.add(d)
                visit(d)
                order.append(d)

    visit(nid)
    return order


def open_struggles(p: dict) -> list[dict]:
    return [s for s in p["struggles"] if not s.get("resolved")]


def missing_hardware(p: dict, g: dict, nid: str) -> list[str]:
    return [h for h in g["nodes"][nid]["hardware"] if not p["hardware"].get(h, {}).get("purchased")]


# --------------------------------------------------------------------------- recommendations
def recommend(p: dict, g: dict, limit: int = 3) -> dict:
    nodes = g["nodes"]
    rec: dict = {"continue": [], "next": [], "projects": [], "foundations": [], "warnings": []}

    rec["continue"] = [nid for nid, e in p["lessons"].items() if e.get("status") == "in-progress"]

    main = sorted((n for n in nodes.values() if n["kind"] == "lesson"), key=lambda n: n["order"])
    for n in main:
        if node_done(p, g, n["id"]) or status(p, n["id"]) == "in-progress":
            continue
        if all(node_done(p, g, d) for d in n["requires"]):
            rec["next"].append(n["id"])
            if len(rec["next"]) >= limit:
                break

    for n in sorted((n for n in nodes.values() if n["kind"] == "project"), key=lambda n: n["order"]):
        st = p["projects"].get(n["id"], {}).get("status", "not-started")
        if st == "done":
            continue
        if st == "in-progress" or all(node_done(p, g, d) for d in n["requires"]):
            rec["projects"].append(n["id"])

    # optional foundations for the first recommended lesson, prioritising struggles
    targets = rec["continue"] + rec["next"][:1]
    struggling = {s["id"] for s in open_struggles(p)}
    for t in targets:
        chain = [t] + all_prereqs(g, t)
        for c in chain:
            for f in nodes[c]["optional"]:
                if status(p, f) in ("not-started", "in-progress", "read") and f not in rec["foundations"]:
                    if c == t or c in struggling:
                        rec["foundations"].append(f)
        for d in all_prereqs(g, t):
            if status(p, d) == "read":
                rec["warnings"].append(f"{t} builds on {d}, which you only marked as read — consider doing its exercises.")
        mh = missing_hardware(p, g, t)
        if mh:
            rec["warnings"].append(f"{t} uses hardware you have not marked as purchased: {', '.join(mh)} "
                                   f"(python course.py hw buy <id>, or look for the simulation alternative in the lesson).")

    for sid in dict.fromkeys(s["id"] for s in open_struggles(p)):
        same = [x for x in p["struggles"] if x["id"] == sid]
        if len(same) >= 2:
            fnds = [f for f in nodes[sid]["optional"] if status(p, f) not in DONE]
            if fnds:
                rec["warnings"].append(f"You struggled with {sid} {len(same)} times — study {', '.join(fnds)} first.")
    return rec


# --------------------------------------------------------------------------- commands
def cmd_status(p: dict, g: dict, _args) -> None:
    nodes = g["nodes"]
    lessons = [n for n in nodes.values() if n["kind"] == "lesson"]
    practiced = [n for n in lessons if status(p, n["id"]) in ("practiced", "mastered", "skipped")]
    minutes_done = sum(n["time"] for n in practiced)
    minutes_all = sum(n["time"] for n in lessons)
    print(f"Main path: {len(practiced)}/{len(lessons)} lessons ({100 * minutes_done // max(minutes_all, 1)}% by time)")
    for mod in g["modules"]:
        if mod["track"] != "main":
            continue
        ids = mod["lessons"]
        done = sum(1 for i in ids if status(p, i) in DONE)
        touched = any(status(p, i) != "not-started" for i in ids)
        if touched:
            bar = "█" * round(10 * done / len(ids)) + "░" * (10 - round(10 * done / len(ids)))
            print(f"  {mod['id']} {bar} {done}/{len(ids)}  {mod['title']}")
    found = [n for n in nodes.values() if n["kind"] == "foundation" and status(p, n["id"]) != "not-started"]
    if found:
        print(f"Foundations touched: {len(found)}  (skipped {sum(1 for n in found if status(p, n['id']) == 'skipped')})")
    cur = p.get("current_project")
    if cur:
        print(f"Current project: {cur} {nodes[cur]['title']}")
    done_projects = [k for k, v in p["projects"].items() if v.get("status") == "done"]
    print(f"Projects done: {len(done_projects)}/{sum(1 for n in nodes.values() if n['kind']=='project')}")
    struggles = open_struggles(p)
    if struggles:
        print("Open struggles: " + "; ".join(f"{s['id']} ({s['note']})" for s in struggles))
    print()
    cmd_next(p, g, _args)


def cmd_next(p: dict, g: dict, _args) -> None:
    nodes = g["nodes"]
    rec = recommend(p, g)
    if rec["continue"]:
        print("Continue:")
        for nid in rec["continue"]:
            print(f"  {nid} {nodes[nid]['title']}  →  {link_path(g, nid)}")
    if rec["next"]:
        print("Next lesson" + ("s" if len(rec["next"]) > 1 else "") + ":")
        for nid in rec["next"]:
            n = nodes[nid]
            print(f"  {nid} {n['title']}  [{n['difficulty']}, {fmt_time(n['time'])}]  →  {n['path']}")
    if rec["foundations"]:
        print("Optional foundations that help (skip any you know: python course.py skip <id>):")
        for f in rec["foundations"]:
            print(f"  {f} {nodes[f]['title']}  →  {link_path(g, f)}")
    if rec["projects"]:
        print("Projects available:")
        for pid in rec["projects"]:
            st = p["projects"].get(pid, {}).get("status", "not-started")
            print(f"  {pid} {nodes[pid]['title']} ({st})  →  {link_path(g, pid)}")
    for w in rec["warnings"]:
        print(f"! {w}")
    if not (rec["continue"] or rec["next"] or rec["projects"]):
        print("Everything on the main path is done. Build something new.")


def change_status(new: str):
    def run(p: dict, g: dict, args) -> None:
        nid = normalize_id(args.id, g)
        if g["nodes"][nid]["kind"] == "project":
            sys.exit("Projects use: python course.py project P04 start|done")
        set_status(p, nid, new)
        if new == "skipped":
            p["lessons"][nid]["reason"] = args.reason or ""
        if new in ("practiced", "mastered"):
            for s in p["struggles"]:
                if s["id"] == nid and not s.get("resolved") and new == "mastered":
                    s["resolved"] = today()
        log(p, f"{new} {nid}")
        save_progress(p, g)
        print(f"{ICON[new]} {nid} {g['nodes'][nid]['title']} → {new}")
        if new in DONE:
            unlocked = [u for u in g["nodes"][nid]["unlocks"]
                        if all(node_done(p, g, d) for d in g["nodes"][u]["requires"]) and not node_done(p, g, u)]
            if unlocked:
                print("Unlocked: " + ", ".join(f"{u} {g['nodes'][u]['title']}" for u in unlocked))
        if new == "practiced":
            n = g["nodes"][nid]
            undone = [e["id"] for e in n["exercises"] if e["id"] not in p["exercises"]]
            if undone:
                print(f"(Exercises not individually marked: {', '.join(undone)} — mark with: python course.py exercise <id>)")
    return run


def cmd_exercise(p: dict, g: dict, args) -> None:
    nid, ex = normalize_exercise(args.id, g)
    known = {e["id"] for e in g["nodes"][nid]["exercises"]}
    if known and ex not in known:
        sys.exit(f"{ex} not found. Exercises in {nid}: {', '.join(sorted(known))}")
    p["exercises"][ex] = today()
    if status(p, nid) == "not-started":
        set_status(p, nid, "in-progress")
    log(p, f"exercise {ex}")
    all_done = known and all(e in p["exercises"] for e in known)
    if all_done and status(p, nid) in ("not-started", "in-progress", "read"):
        set_status(p, nid, "practiced")
        print(f"All exercises of {nid} done → practiced")
    save_progress(p, g)
    print(f"✔ {ex}")


def cmd_quiz(p: dict, g: dict, args) -> None:
    nid = normalize_id(args.id, g)
    m = re.fullmatch(r"(\d+)/(\d+)", args.score)
    if not m:
        sys.exit("Score looks like 7/8")
    entry = p["lessons"].setdefault(nid, {"status": "not-started"})
    entry.setdefault("quiz", []).append({"date": today(), "score": args.score})
    log(p, f"quiz {nid} {args.score}")
    save_progress(p, g)
    right, total = int(m.group(1)), int(m.group(2))
    print(f"Recorded {nid} knowledge check {args.score}.")
    if total and right / total < 0.7:
        fnds = g["nodes"][nid]["optional"]
        print("Below 70% — reread the Concept section or ask your teacher to explain it differently."
              + (f" Foundations that may help: {', '.join(fnds)}" if fnds else ""))


def cmd_struggle(p: dict, g: dict, args) -> None:
    nid = normalize_id(args.id, g)
    p["struggles"].append({"id": nid, "note": args.note, "date": today(), "resolved": None})
    log(p, f"struggle {nid}: {args.note}")
    save_progress(p, g)
    count = sum(1 for s in p["struggles"] if s["id"] == nid)
    print(f"Noted. Struggles on {nid}: {count}.")
    n = g["nodes"][nid]
    fnds = [f for f in n["optional"] if status(p, f) not in DONE]
    weak = [d for d in all_prereqs(g, nid) if status(p, d) in ("read", "skipped", "not-started")]
    if fnds:
        print("Foundation lessons that target this: " + ", ".join(f"{f} {g['nodes'][f]['title']}" for f in fnds))
    if weak:
        print("Prerequisites you only read, skipped or never did: " + ", ".join(weak[-6:]))
    if count >= 2 and fnds:
        print(f"This is a repeated struggle — do {fnds[0]} before retrying {nid}.")


def cmd_resolve(p: dict, g: dict, args) -> None:
    nid = normalize_id(args.id, g)
    n = 0
    for s in p["struggles"]:
        if s["id"] == nid and not s.get("resolved"):
            s["resolved"] = today()
            n += 1
    log(p, f"resolve {nid}")
    save_progress(p, g)
    print(f"Resolved {n} struggle(s) on {nid}.")


def cmd_why(p: dict, g: dict, args) -> None:
    nid = normalize_id(args.id, g)
    nodes = g["nodes"]
    print(f"{nid} {nodes[nid]['title']}  ({status(p, nid) if nodes[nid]['kind'] != 'project' else p['projects'].get(nid, {}).get('status', 'not-started')})")

    def tree(x: str, depth: int, seen: set[str]) -> None:
        for d in nodes[x]["requires"]:
            st = status(p, d) if nodes[d]["kind"] != "project" else p["projects"].get(d, {}).get("status", "not-started")
            mark = "✓" if node_done(p, g, d) else "✗"
            already = " (see above)" if d in seen else ""
            print("  " * depth + f"{mark} {d} {nodes[d]['title']} [{st}]{already}")
            if d not in seen and depth < args.depth:
                seen.add(d)
                tree(d, depth + 1, seen)

    tree(nid, 1, set())
    missing = [d for d in all_prereqs(g, nid) if not node_done(p, g, d)]
    if missing:
        total = sum(nodes[d]["time"] for d in missing)
        print(f"\nStill to do before {nid} ({len(missing)} items, ≈ {fmt_time(total)}), in order:")
        for d in missing:
            print(f"  {d} {nodes[d]['title']}")
    else:
        print(f"\nAll hard prerequisites of {nid} are satisfied.")
    opt = [f for f in nodes[nid]["optional"]]
    if opt:
        print("Optional foundations: " + ", ".join(f"{f} [{status(p, f)}]" for f in opt))


def cmd_learn(p: dict, g: dict, args) -> None:
    q = args.concept.lower().replace(" ", "-")
    hits = [(c, ids) for c, ids in g["concepts"].items() if q in c]
    if not hits:
        words = q.split("-")
        hits = [(c, ids) for c, ids in g["concepts"].items() if all(w in c for w in words)]
    if not hits:
        titles = [n for n in g["nodes"].values() if args.concept.lower() in n["title"].lower()]
        if titles:
            for n in titles[:10]:
                print(f"{n['id']} {n['title']}  →  {n['path']}")
            return
        sys.exit(f"No concept matching '{args.concept}'. See curriculum/concept-index.md")
    for c, ids in hits[:15]:
        for nid in ids:
            n = g["nodes"][nid]
            missing = [d for d in all_prereqs(g, nid) if not node_done(p, g, d)]
            print(f"{c}: {nid} {n['title']} [{status(p, nid)}]  →  {n['path']}")
            if missing:
                print(f"    needs first ({len(missing)}): {', '.join(missing[:12])}{' …' if len(missing) > 12 else ''}")


def cmd_show(p: dict, g: dict, args) -> None:
    nid = normalize_id(args.id, g)
    n = g["nodes"][nid]
    print(f"{nid} {n['title']}")
    print(f"  file:        {n['path']}")
    print(f"  kind:        {n['kind']}   difficulty: {n['difficulty']}   time: {fmt_time(n['time'])}")
    print(f"  status:      {status(p, nid) if n['kind'] != 'project' else p['projects'].get(nid, {}).get('status', 'not-started')}")
    print(f"  requires:    {', '.join(n['requires']) or '—'}")
    print(f"  optional:    {', '.join(n['optional']) or '—'}")
    print(f"  unlocks:     {', '.join(n['unlocks']) or '—'}")
    print(f"  hardware:    {', '.join(n['hardware']) or 'none'}")
    print(f"  software:    {', '.join(n['software']) or 'none'}")
    print(f"  teaches:     {', '.join(n['teaches']) or '—'}")
    if n["skip_if"]:
        print(f"  skip if:     {n['skip_if']}")
    if n["exercises"]:
        print("  exercises:")
        for e in n["exercises"]:
            mark = "✓" if e["id"] in p["exercises"] else " "
            print(f"    [{mark}] {e['id']} {e['title']} ({e['type']})")
    if n.get("checker"):
        print(f"  checker:     python course.py check {nid}")
    if n["version_sensitive"]:
        print("  version-sensitive: verify current docs before following install/config steps")


def cmd_list(p: dict, g: dict, args) -> None:
    sel = (args.filter or "main").upper()
    for mod in g["modules"]:
        if sel == "ALL" or (sel == "MAIN" and mod["track"] == "main") or (sel in ("FOUNDATIONS", "F") and mod["track"] == "foundations") \
                or mod["id"] == sel or mod["id"] == sel.zfill(2):
            print(f"{mod['id']} · {mod['title']}")
            for lid in mod["lessons"]:
                n = g["nodes"][lid]
                print(f"  {ICON[status(p, lid)]} {lid} {n['title']} ({fmt_time(n['time'])})")
    if sel in ("PROJECTS", "P", "ALL"):
        for n in g["nodes"].values():
            if n["kind"] == "project":
                st = p["projects"].get(n["id"], {}).get("status", "not-started")
                print(f"  {st:12} {n['id']} {n['title']}")


def cmd_project(p: dict, g: dict, args) -> None:
    nid = normalize_id(args.id, g)
    if g["nodes"][nid]["kind"] != "project":
        sys.exit(f"{nid} is not a project")
    action = args.action
    entry = p["projects"].setdefault(nid, {"status": "not-started"})
    if action == "start":
        missing = [d for d in g["nodes"][nid]["requires"] if not node_done(p, g, d)]
        if missing and not args.force:
            sys.exit(f"{nid} needs: {', '.join(missing)} (use --force to start anyway)")
        entry["status"] = "in-progress"
        entry["started"] = today()
        p["current_project"] = nid
    elif action == "done":
        entry["status"] = "done"
        entry["done"] = today()
        if p.get("current_project") == nid:
            p["current_project"] = None
    elif action == "reset":
        p["projects"].pop(nid, None)
        if p.get("current_project") == nid:
            p["current_project"] = None
    if args.note:
        (p["projects"].get(nid) or {}).setdefault("notes", []).append({"date": today(), "note": args.note})
    log(p, f"project {nid} {action}")
    save_progress(p, g)
    print(f"{nid} {g['nodes'][nid]['title']} → {action}")


def cmd_hw(p: dict, g: dict, args) -> None:
    catalog = g["hardware"]
    if not args.action or args.action == "list":
        for hid, h in sorted(catalog.items(), key=lambda kv: kv[1]["stage"]):
            e = p["hardware"].get(hid, {})
            state = "assembled" if e.get("assembled") else "purchased" if e.get("purchased") else "—"
            users = sum(1 for n in g["nodes"].values() if hid in n["hardware"])
            print(f"  stage {h['stage']}  {hid:13} {state:10} used by {users:3} lessons  {h['title']}")
        print("Details, prices and Israeli suppliers: HARDWARE.md")
        return
    if not args.item or args.item not in catalog:
        sys.exit(f"Hardware ids: {', '.join(catalog)}")
    e = p["hardware"].setdefault(args.item, {})
    if args.action == "buy":
        e["purchased"] = today()
    elif args.action == "assemble":
        e.setdefault("purchased", today())
        e["assembled"] = today()
    elif args.action == "unbuy":
        p["hardware"].pop(args.item, None)
    if args.note:
        p["hardware"].setdefault(args.item, {}).setdefault("notes", []).append(args.note)
    log(p, f"hardware {args.action} {args.item}")
    save_progress(p, g)
    print(f"{args.item}: {args.action}")


def cmd_check(p: dict, g: dict, args) -> None:
    nid = normalize_id(args.id, g)
    n = g["nodes"][nid]
    if not n.get("checker"):
        sys.exit(f"{nid} has no automated checker. Its exercises are verified by the Expected result section.")
    target = ROOT / n["checker"]
    env = dict(os.environ)
    if args.solution:
        env["COURSE_USE_SOLUTION"] = "1"
    cmd = [sys.executable, "-m", "pytest", "-q", str(target)]
    print("$ " + " ".join(cmd))
    try:
        rc = subprocess.call(cmd, cwd=ROOT, env=env)
    except FileNotFoundError:
        sys.exit("pytest not found: pip install -r labs/requirements.txt")
    if rc == 0 and not args.solution:
        entry = p["lessons"].setdefault(nid, {"status": "not-started"})
        entry["checker_passed"] = today()
        for e in n["exercises"]:
            if e["type"] == "coding":
                p["exercises"].setdefault(e["id"], today())
        if status(p, nid) in ("not-started", "in-progress", "read"):
            set_status(p, nid, "practiced")
        log(p, f"checker passed {nid}")
        save_progress(p, g)
        print(f"✅ {nid} checker passed — marked practiced.")
    elif rc != 0:
        print("Not yet. Read the failing assertion; `python course.py check " + nid + " --solution` shows the reference passes.")
    sys.exit(rc)


def cmd_skills(p: dict, g: dict, _args) -> None:
    practiced, mastered = [], []
    for nid, e in p["lessons"].items():
        if e.get("status") == "mastered":
            mastered += g["nodes"][nid]["teaches"]
        elif e.get("status") == "practiced":
            practiced += g["nodes"][nid]["teaches"]
    print("Mastered: " + (", ".join(sorted(set(mastered))) or "—"))
    print("Practiced: " + (", ".join(sorted(set(practiced) - set(mastered))) or "—"))


def cmd_log(p: dict, _g: dict, args) -> None:
    for e in p["log"][-args.n:]:
        print(f"{e['date']}  {e['event']}")


def cmd_reset(p: dict, g: dict, args) -> None:
    nid = normalize_id(args.id, g)
    p["lessons"].pop(nid, None)
    for ex in [k for k in p["exercises"] if k.startswith(nid + "-")]:
        p["exercises"].pop(ex)
    log(p, f"reset {nid}")
    save_progress(p, g)
    print(f"Reset {nid}")


def cmd_render(p: dict, g: dict, _args) -> None:
    save_progress(p, g)
    print(f"Wrote {PROGRESS_MD.relative_to(ROOT) if PROGRESS_MD.is_relative_to(ROOT) else PROGRESS_MD}")


# --------------------------------------------------------------------------- PROGRESS.md
def render_markdown(p: dict, g: dict) -> str:
    nodes = g["nodes"]
    lessons = [n for n in nodes.values() if n["kind"] == "lesson"]
    done = [n for n in lessons if status(p, n["id"]) in DONE]
    rec = recommend(p, g)
    L: list[str] = [
        "# Progress",
        "",
        "<!-- generated by course.py from progress/progress.json — do not edit by hand; use the commands below -->",
        "",
        f"_Last updated {today()}. Started {p['student'].get('started', '?')}._",
        "",
        "## Where I am",
        "",
        f"- **Main path:** {len(done)}/{len(lessons)} lessons "
        f"({100 * sum(n['time'] for n in done) // max(1, sum(n['time'] for n in lessons))}% by time)",
        f"- **Mastered (challenge done):** {sum(1 for n in lessons if status(p, n['id']) == 'mastered')}",
        f"- **Projects done:** {sum(1 for v in p['projects'].values() if v.get('status') == 'done')}"
        f"/{sum(1 for n in nodes.values() if n['kind'] == 'project')}",
        f"- **Exercises done:** {len(p['exercises'])}",
        f"- **Current project:** " + (f"{p['current_project']} {nodes[p['current_project']]['title']}" if p.get("current_project") else "none"),
    ]
    if rec["continue"]:
        L.append("- **In progress:** " + ", ".join(f"[{i} {nodes[i]['title']}]({nodes[i]['path']})" for i in rec["continue"]))
    if rec["next"]:
        i = rec["next"][0]
        L.append(f"- **Next recommended lesson:** [{i} {nodes[i]['title']}]({nodes[i]['path']})")
    if rec["foundations"]:
        L.append("- **Optional foundations that would help:** " + ", ".join(f"[{i}]({nodes[i]['path']})" for i in rec["foundations"]))
    for w in rec["warnings"]:
        L.append(f"- ⚠️ {w}")
    L += [
        "",
        "Legend: ⬜ not started · 🟨 in progress · 📖 read (not practiced) · ✅ practiced (exercises done) · 🏆 mastered (challenge done) · ⏭️ skipped",
        "",
        "```bash",
        "python course.py next              # what now?",
        "python course.py complete 04.3     # exercises done",
        "python course.py struggle 10.5 \"note\"",
        "```",
        "",
        "## Main path",
        "",
    ]
    for mod in g["modules"]:
        if mod["track"] != "main":
            continue
        ids = mod["lessons"]
        d = sum(1 for i in ids if status(p, i) in DONE)
        L.append(f"<details{' open' if 0 < d < len(ids) or any(status(p, i) == 'in-progress' for i in ids) else ''}>")
        L.append(f"<summary><b>{mod['id']} · {mod['title']}</b> — {d}/{len(ids)}</summary>")
        L.append("")
        for i in ids:
            n = nodes[i]
            st = status(p, i)
            exs = n["exercises"]
            exs_done = sum(1 for e in exs if e["id"] in p["exercises"])
            extra = f" · exercises {exs_done}/{len(exs)}" if exs else ""
            q = p["lessons"].get(i, {}).get("quiz")
            if q:
                extra += f" · quiz {q[-1]['score']}"
            box = "x" if st in DONE else " "
            L.append(f"- [{box}] {ICON[st]} [{i} {n['title']}]({n['path']}){extra}")
        L += ["", "</details>", ""]

    L += ["## Projects", "", "| | Project | Status | Started | Done |", "|---|---|---|---|---|"]
    for n in nodes.values():
        if n["kind"] != "project":
            continue
        e = p["projects"].get(n["id"], {})
        st = e.get("status", "not-started")
        icon = {"not-started": "⬜", "in-progress": "🟨", "done": "✅"}[st]
        L.append(f"| {icon} | [{n['title']}]({n['path']}) | {st} | {e.get('started', '')} | {e.get('done', '')} |")

    L += ["", "## Optional foundations", ""]
    touched = [n for n in nodes.values() if n["kind"] == "foundation" and status(p, n["id"]) != "not-started"]
    if touched:
        for n in touched:
            reason = p["lessons"][n["id"]].get("reason")
            L.append(f"- {ICON[status(p, n['id'])]} [{n['id']} {n['title']}]({n['path']})" + (f" — skipped: {reason}" if reason else ""))
    else:
        L.append("None yet — take them when a main lesson points you there.")

    L += ["", "## Skipped lessons", ""]
    skipped = [(i, e) for i, e in p["lessons"].items() if e.get("status") == "skipped"]
    L += [f"- {i} {nodes[i]['title']} — {e.get('reason', '')}" for i, e in skipped] or ["None."]

    L += ["", "## Concepts I struggled with", ""]
    if p["struggles"]:
        L += ["| Lesson | Note | Date | Resolved |", "|---|---|---|---|"]
        for s in p["struggles"]:
            L.append(f"| {s['id']} {nodes[s['id']]['title']} | {s['note']} | {s['date']} | {s.get('resolved') or '—'} |")
    else:
        L.append("None recorded. Use `python course.py struggle <id> \"what was hard\"` — the tool uses it to recommend foundations.")

    L += ["", "## Hardware", "", "| Stage | Item | Purchased | Assembled |", "|---|---|---|---|"]
    for hid, h in sorted(g["hardware"].items(), key=lambda kv: kv[1]["stage"]):
        e = p["hardware"].get(hid, {})
        L.append(f"| {h['stage']} | {h['title']} (`{hid}`) | {e.get('purchased', '—')} | {e.get('assembled', '—')} |")

    L += ["", "## Skills", ""]
    mastered, practiced = set(), set()
    for i, e in p["lessons"].items():
        if e.get("status") == "mastered":
            mastered |= set(nodes[i]["teaches"])
        elif e.get("status") == "practiced":
            practiced |= set(nodes[i]["teaches"])
    L.append("**Mastered:** " + (", ".join(sorted(mastered)) or "—"))
    L.append("")
    L.append("**Practiced:** " + (", ".join(sorted(practiced - mastered)) or "—"))
    L += ["", "## Recent activity", ""]
    L += [f"- {e['date']} — {e['event']}" for e in p["log"][-15:][::-1]] or ["Nothing yet."]
    return "\n".join(L) + "\n"


# --------------------------------------------------------------------------- CLI
def main(argv: list[str] | None = None) -> None:
    ap = argparse.ArgumentParser(description="Robotics course progress tool", formatter_class=argparse.RawDescriptionHelpFormatter,
                                 epilog=__doc__)
    sub = ap.add_subparsers(dest="cmd", required=True)
    sub.add_parser("status")
    sub.add_parser("next")
    for name in ("start", "read", "complete", "master"):
        s = sub.add_parser(name)
        s.add_argument("id")
    s = sub.add_parser("skip")
    s.add_argument("id")
    s.add_argument("--reason", default="")
    s = sub.add_parser("exercise")
    s.add_argument("id")
    s = sub.add_parser("quiz")
    s.add_argument("id")
    s.add_argument("score")
    s = sub.add_parser("struggle")
    s.add_argument("id")
    s.add_argument("note")
    s = sub.add_parser("resolve")
    s.add_argument("id")
    s = sub.add_parser("why")
    s.add_argument("id")
    s.add_argument("--depth", type=int, default=2)
    s = sub.add_parser("learn")
    s.add_argument("concept")
    s = sub.add_parser("show")
    s.add_argument("id")
    s = sub.add_parser("list")
    s.add_argument("filter", nargs="?")
    s = sub.add_parser("project")
    s.add_argument("id")
    s.add_argument("action", choices=("start", "done", "reset"))
    s.add_argument("--note")
    s.add_argument("--force", action="store_true")
    s = sub.add_parser("hw")
    s.add_argument("action", nargs="?", choices=("list", "buy", "assemble", "unbuy"))
    s.add_argument("item", nargs="?")
    s.add_argument("--note")
    s = sub.add_parser("check")
    s.add_argument("id")
    s.add_argument("--solution", action="store_true", help="run the tests against the reference solution")
    sub.add_parser("skills")
    s = sub.add_parser("log")
    s.add_argument("-n", type=int, default=20)
    s = sub.add_parser("reset")
    s.add_argument("id")
    sub.add_parser("render")

    args = ap.parse_args(argv)
    if hasattr(sys.stdout, "reconfigure"):
        sys.stdout.reconfigure(encoding="utf-8")
    g = load_graph()
    p = load_progress()
    handlers = {
        "status": cmd_status, "next": cmd_next,
        "start": change_status("in-progress"), "read": change_status("read"),
        "complete": change_status("practiced"), "master": change_status("mastered"),
        "skip": change_status("skipped"), "exercise": cmd_exercise, "quiz": cmd_quiz,
        "struggle": cmd_struggle, "resolve": cmd_resolve, "why": cmd_why, "learn": cmd_learn,
        "show": cmd_show, "list": cmd_list, "project": cmd_project, "hw": cmd_hw,
        "check": cmd_check, "skills": cmd_skills, "log": cmd_log, "reset": cmd_reset, "render": cmd_render,
    }
    handlers[args.cmd](p, g, args)


if __name__ == "__main__":
    main()
