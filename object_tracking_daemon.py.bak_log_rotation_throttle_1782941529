#!/usr/bin/env python3
import json
import os
import signal
import subprocess
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parent
SETTINGS_FILE = ROOT / "ui_settings.json"
PRESET_FILE = ROOT / "ptz_object_presets.json"
LOG_FILE = ROOT / "object_tracking_daemon.log"
STATE_FILE = ROOT / "object_tracking_daemon_state.json"
APPLIER = ROOT / "apply_ptz_object_preset.py"

POLL_SEC = 1.0
RESTART_DELAY_SEC = 2.0

child = None
child_preset = None


def log(event, **data):
    row = {
        "ts": round(time.time(), 3),
        "event": event,
        **data
    }

    line = json.dumps(row, ensure_ascii=False, sort_keys=True)

    try:
        with LOG_FILE.open("a", encoding="utf-8") as f:
            f.write(line + "\n")
    except Exception:
        pass

    print(line, flush=True)

    try:
        tmp = STATE_FILE.with_suffix(".json.tmp")
        tmp.write_text(json.dumps(row, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
        tmp.replace(STATE_FILE)
    except Exception:
        pass


def load_json(path, default):
    try:
        if path.exists():
            return json.loads(path.read_text(encoding="utf-8"))
    except Exception as e:
        log("json_read_error", path=str(path), error=str(e))
    return default


def save_json(path, data):
    tmp = path.with_suffix(path.suffix + ".tmp")
    tmp.write_text(json.dumps(data, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    tmp.replace(path)


def active_preset_name(settings, presets_root):
    last = settings.get("lastAppliedObjectPreset") or {}
    if isinstance(last, dict) and last.get("name"):
        return str(last["name"])

    if settings.get("activeObjectPreset"):
        return str(settings["activeObjectPreset"])

    if presets_root.get("active"):
        return str(presets_root["active"])

    return "person_single"


def preset_by_name(name, settings, presets_root):
    presets = dict(presets_root.get("presets") or {})

    custom = settings.get("objectPresetsCustom") or {}
    if isinstance(custom, dict):
        presets.update(custom)

    return presets.get(name)


def is_single_auto(preset):
    if not isinstance(preset, dict):
        return False
    return str(preset.get("tracking_mode") or "") == "single_auto"


def child_alive():
    global child
    return child is not None and child.poll() is None


def stop_child(reason):
    global child, child_preset

    if child is None:
        return

    if child.poll() is None:
        log("child_stop", reason=reason, preset=child_preset, pid=child.pid)

        try:
            child.terminate()
            child.wait(timeout=5)
        except Exception:
            try:
                child.kill()
            except Exception:
                pass

    child = None
    child_preset = None


def start_child(preset_name):
    global child, child_preset

    cmd = [
        sys.executable,
        str(APPLIER),
        "--preset",
        preset_name,
        "--arm",
        "force",
        "--strategy",
        "preset",
        "--watch"
    ]

    f = LOG_FILE.open("a", encoding="utf-8")

    child = subprocess.Popen(
        cmd,
        cwd=str(ROOT),
        stdout=f,
        stderr=f,
        start_new_session=True
    )

    child_preset = preset_name

    log("child_start", preset=preset_name, pid=child.pid, cmd=cmd)


def handle_signal(signum, frame):
    stop_child(f"signal_{signum}")
    log("daemon_stop", signal=signum)
    raise SystemExit(0)


def main():
    signal.signal(signal.SIGTERM, handle_signal)
    signal.signal(signal.SIGINT, handle_signal)

    log("daemon_start", pid=os.getpid())

    last_restart = 0

    while True:
        settings = load_json(SETTINGS_FILE, {})
        presets_root = load_json(PRESET_FILE, {"presets": {}, "active": "person_single"})

        armed = bool(settings.get("ptzArmed"))
        preset_name = active_preset_name(settings, presets_root)
        preset = preset_by_name(preset_name, settings, presets_root)
        single = is_single_auto(preset)

        if not armed:
            if child_alive():
                stop_child("ptzArmed_false")

            log("idle_disarmed", preset=preset_name, armed=armed, single_auto=single)
            time.sleep(POLL_SEC)
            continue

        if not single:
            if child_alive():
                stop_child("preset_not_single_auto")

            log("idle_not_single_auto", preset=preset_name, armed=armed, single_auto=single)
            time.sleep(POLL_SEC)
            continue

        if child_alive():
            if child_preset != preset_name:
                stop_child("preset_changed")
                time.sleep(0.5)
            else:
                log("armed_running", preset=preset_name, pid=child.pid)
                time.sleep(POLL_SEC)
                continue

        now = time.time()

        if now - last_restart < RESTART_DELAY_SEC:
            time.sleep(POLL_SEC)
            continue

        last_restart = now
        start_child(preset_name)

        time.sleep(POLL_SEC)


if __name__ == "__main__":
    main()
