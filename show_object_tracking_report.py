#!/usr/bin/env python3
import argparse
import json
import time
from collections import Counter
from pathlib import Path

ROOT = Path(__file__).resolve().parent
EVENT_LOG = ROOT / "object_tracking_events.jsonl"
STATE_FILE = ROOT / "object_tracking_state.json"
DAEMON_STATE_FILE = ROOT / "object_tracking_daemon_state.json"

IMPORTANT = {
    "candidate_found",
    "tracking_started",
    "tracking",
    "lost_grace_start",
    "lost_object",
    "zoom_wide_pulse",
    "zoom_wide_skip_cooldown",
    "zoom_wide_skip_already_wide",
    "scan_step",
    "target_not_acquired_stop",
    "continuous_scan_finished"
}


def load_jsonl(path):
    if not path.exists():
        return []

    out = []

    for line in path.read_text(encoding="utf-8", errors="replace").splitlines():
        line = line.strip()
        if not line:
            continue

        try:
            out.append(json.loads(line))
        except Exception:
            pass

    return out


def fmt_ts(ts):
    try:
        return time.strftime("%Y-%m-%d %H:%M:%S", time.localtime(float(ts)))
    except Exception:
        return str(ts)


def compact_event(e):
    name = e.get("event")

    base = {
        "time": fmt_ts(e.get("ts")),
        "event": name
    }

    for k in [
        "preset",
        "track_id",
        "cls",
        "score",
        "center",
        "mode",
        "ptz",
        "cmd_pan",
        "cmd_tilt",
        "reason",
        "cmd",
        "hold_ms",
        "skipped"
    ]:
        if k in e:
            base[k] = e[k]

    return base


def load_json(path):
    try:
        if path.exists():
            return json.loads(path.read_text(encoding="utf-8", errors="replace"))
    except Exception:
        pass
    return None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--last", type=int, default=40)
    ap.add_argument("--all", action="store_true")
    ap.add_argument("--full", action="store_true")
    args = ap.parse_args()

    events = load_jsonl(EVENT_LOG)

    print("== files ==")
    print("events =", EVENT_LOG, "exists=", EVENT_LOG.exists(), "size=", EVENT_LOG.stat().st_size if EVENT_LOG.exists() else 0)
    print("state =", STATE_FILE, "exists=", STATE_FILE.exists())
    print("daemon_state =", DAEMON_STATE_FILE, "exists=", DAEMON_STATE_FILE.exists())

    print("")
    print("== counters ==")
    c = Counter(e.get("event") for e in events)
    for name, count in c.most_common():
        print(name, "=", count)

    state = load_json(STATE_FILE)
    if state is not None:
        print("")
        print("== current tracking state ==")
        print(json.dumps(state, indent=2, ensure_ascii=False))

    daemon_state = load_json(DAEMON_STATE_FILE)
    if daemon_state is not None:
        print("")
        print("== daemon state ==")
        print(json.dumps(daemon_state, indent=2, ensure_ascii=False))

    shown = events if args.all else events[-args.last:]

    print("")
    print("== events ==")

    for e in shown:
        if not args.full and e.get("event") not in IMPORTANT:
            continue

        obj = e if args.full else compact_event(e)
        print(json.dumps(obj, indent=2, ensure_ascii=False))


if __name__ == "__main__":
    main()
