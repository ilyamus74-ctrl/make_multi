#!/usr/bin/env python3
import json
import os
import signal
import subprocess
import time
import urllib.request
from pathlib import Path

ROOT = Path("/root/new_yolo8")
MJPEG = "http://127.0.0.1:8080"
PTZ = "http://127.0.0.1:8090"

def get(url):
    with urllib.request.urlopen(url, timeout=3) as r:
        return json.loads(r.read().decode("utf-8", errors="replace"))

def post(url, body):
    data = json.dumps(body).encode("utf-8")
    req = urllib.request.Request(
        url,
        data=data,
        headers={"Content-Type": "application/json"},
        method="POST"
    )
    with urllib.request.urlopen(req, timeout=3) as r:
        raw = r.read().decode("utf-8", errors="replace")
    return json.loads(raw) if raw.strip() else {}

def ps_lines():
    out = subprocess.run(
        ["ps", "ax", "-o", "pid=,args="],
        text=True,
        capture_output=True
    ).stdout
    return [x.strip() for x in out.splitlines() if x.strip()]

def find(pattern):
    out = []

    for line in ps_lines():
        parts = line.split(None, 1)

        if len(parts) != 2:
            continue

        pid = int(parts[0])
        args = parts[1]

        if pid == os.getpid():
            continue

        if pattern in args:
            out.append((pid, args))

    return out

def watch_count():
    return len([
        x
        for x in find("apply_ptz_object_preset.py")
        if "--watch" in x[1]
    ])

def section(name):
    print("")
    print("=" * 100)
    print(name)
    print("=" * 100)

def print_state(label):
    section(label)

    settings = get(f"{MJPEG}/api/settings")
    active = settings.get("activeObjectPreset")
    preset = (settings.get("objectPresetsCustom") or {}).get(active) or {}

    print("settings.activeObjectPreset =", settings.get("activeObjectPreset"))
    print("settings.activeSearchPreset =", settings.get("activeSearchPreset"))
    print("settings.ptzArmed =", settings.get("ptzArmed"))
    print("settings.controlMode =", settings.get("controlMode"))
    print("settings.objectPresetTrackingMode =", settings.get("objectPresetTrackingMode"))
    print("settings.objectPresetLossBehavior =", settings.get("objectPresetLossBehavior"))
    print("settings.lastAppliedObjectPreset =", json.dumps(settings.get("lastAppliedObjectPreset"), ensure_ascii=False))
    print("active preset =", json.dumps(preset, indent=2, ensure_ascii=False))

    print("")
    print("daemon processes:")
    for pid, args in find("object_tracking_daemon.py"):
        print(pid, args)

    print("")
    print("watch processes:")
    for pid, args in find("apply_ptz_object_preset.py"):
        print(pid, args)

    print("")
    print("watch_count =", watch_count())

def tail_file(path, lines=80):
    p = Path(path)

    if not p.exists():
        print("missing", path)
        return

    data = p.read_text(encoding="utf-8", errors="replace").splitlines()

    for line in data[-lines:]:
        print(line)

def main():
    print_state("BEFORE")

    section("SET SINGLE AUTO TEST SETTINGS")

    settings = get(f"{MJPEG}/api/settings")
    custom = settings.get("objectPresetsCustom") or {}
    car = custom.get("car_single") or {}

    settings["activeObjectPreset"] = "car_single"
    settings["activeSearchPreset"] = settings.get("activeSearchPreset") or "lost_step_wait"
    settings["ptzArmed"] = False
    settings["controlMode"] = "manual"
    settings["objectPresetTrackingMode"] = "single_auto"
    settings["objectPresetLossBehavior"] = "continuous_wide_scan_x"
    settings["lastAppliedObjectPreset"] = {
        "name": "car_single",
        "label": car.get("label") or "МАШИНА",
        "tracking_mode": "single_auto",
        "loss_behavior": "continuous_wide_scan_x",
        "ts": int(time.time())
    }

    post(f"{MJPEG}/api/settings", settings)
    post(f"{PTZ}/api/autopilot/stop", {})

    time.sleep(3)
    print_state("AFTER FORCE DISARM")

    section("ARM")
    settings = get(f"{MJPEG}/api/settings")
    settings["ptzArmed"] = True
    settings["controlMode"] = "ptz"
    post(f"{MJPEG}/api/settings", settings)

    for i in range(10):
        time.sleep(1)
        print("second", i + 1, "watch_count", watch_count())

    print_state("AFTER ARM WAIT")

    section("DAEMON RUNTIME LOGS")

    for path in [
        "/dev/shm/new_yolo8_object_tracking/object_tracking_daemon.log",
        "/dev/shm/new_yolo8_object_tracking/object_tracking_events.jsonl",
        "/root/new_yolo8/object_tracking_daemon.log",
        "/root/new_yolo8/object_tracking_events.jsonl"
    ]:
        print("")
        print("---", path, "---")
        tail_file(path, 60)

    section("DISARM BACK")
    settings = get(f"{MJPEG}/api/settings")
    settings["ptzArmed"] = False
    settings["controlMode"] = "manual"
    post(f"{MJPEG}/api/settings", settings)
    post(f"{PTZ}/api/autopilot/stop", {})

    time.sleep(4)
    print_state("AFTER FINAL DISARM")

if __name__ == "__main__":
    main()

