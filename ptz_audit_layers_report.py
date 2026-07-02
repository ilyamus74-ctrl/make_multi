from pathlib import Path
import json
import urllib.request
import subprocess
import re

ROOT = Path("/root/new_yolo8")
WEB = ROOT / "web/index.html"

URLS = [
    "http://127.0.0.1:8080/api/settings",
    "http://127.0.0.1:8080/api/detector/config",
    "http://127.0.0.1:8080/api/detection/limits",
    "http://127.0.0.1:8080/api/detection/throttle",
    "http://127.0.0.1:8080/api/detection/roi_config",
    "http://127.0.0.1:8080/api/tracker/state",
    "http://127.0.0.1:8090/api/autopilot/state"
]

MARKERS = [
    "OBJECT_PRESET_MANAGER_START",
    "OBJECT_PRESET_BUTTONS_ACTIVE_FIX_START",
    "OBJECT_TRACKING_LOGS_BUTTON_START",
    "EDIT_ACTIVE_OBJECT_PRESET_START",
    "SEARCH_PRESET_SELECTOR_START",
    "OBJECT_PRESET_AUTO_LEARN_OPERATOR_CONTROLS_START",
    "OPERATOR_CONTROLS_HARD_PERSIST_APPLY_START",
    "OBJECT_PRESET_NO_AUTO_ARM_DURABLE_SAVE_START",
    "PTZ_START_STOP_ACTIVE_STATE_FIX_START",
    "SINGLE_PRESET_AUTO_ARM",
    "SAVE_MODE_MODELS_LATE_FIX_START",
    "SAVE_MODE_MODELS_HARDFIX_START"
]

KEYWORDS = [
    "ptzArmed",
    "data-object-preset",
    "activeObjectPreset",
    "objectPresetsCustom",
    "lastAppliedObjectPreset",
    "api/autopilot/start",
    "api/autopilot/stop",
    "applyPreset(",
    "selectObjectPresetNoArm",
    "saveDetectionToActivePreset",
    "saveActiveObjectPresetHard",
    "SAVE DETECTION TO PRESET",
    "START PTZ"
]

def get_json(url):
    try:
        with urllib.request.urlopen(url, timeout=3) as r:
            raw = r.read().decode("utf-8", errors="replace")
        return json.loads(raw)
    except Exception as e:
        return {"__error__": str(e)}

def section(title):
    print("")
    print("=" * 80)
    print(title)
    print("=" * 80)

section("FILES")
for name in [
    "web/index.html",
    "ui_settings.json",
    "ptz_object_presets.json",
    "ptz_search_presets.json",
    "apply_ptz_object_preset.py",
    "object_tracking_daemon.py",
    "launcher.sh"
]:
    p = ROOT / name
    print(name, "exists=", p.exists(), "size=", p.stat().st_size if p.exists() else 0)

section("WEB PATCH MARKERS")
if WEB.exists():
    s = WEB.read_text(encoding="utf-8", errors="replace")
    for m in MARKERS:
        print(m, "=", s.count(m))
else:
    s = ""
    print("web/index.html not found")

section("WEB KEYWORD COUNTS")
for k in KEYWORDS:
    print(k, "=", s.count(k))

section("OBJECT PRESET CLICK HANDLER SNIPPETS")
lines = s.splitlines()
for idx, line in enumerate(lines, start=1):
    if "data-object-preset" in line or "objectPresetPanel" in line:
        a = max(1, idx - 5)
        b = min(len(lines), idx + 12)
        print("")
        print("--- lines", a, "to", b, "---")
        for n in range(a, b + 1):
            print(f"{n}: {lines[n - 1]}")

section("PTZ ARM / START SNIPPETS")
for idx, line in enumerate(lines, start=1):
    if "ptzArmed" in line or "/api/autopilot/start" in line:
        a = max(1, idx - 5)
        b = min(len(lines), idx + 10)
        print("")
        print("--- lines", a, "to", b, "---")
        for n in range(a, b + 1):
            print(f"{n}: {lines[n - 1]}")

section("API STATE")
for url in URLS:
    print("")
    print("==", url, "==")
    data = get_json(url)
    print(json.dumps(data, indent=2, ensure_ascii=False))

section("IMPORTANT SETTINGS SUMMARY")
settings = get_json("http://127.0.0.1:8080/api/settings")
active = settings.get("activeObjectPreset")
custom = settings.get("objectPresetsCustom") or {}
preset = custom.get(active) or {}

for k in [
    "ptzArmed",
    "activeObjectPreset",
    "activeSearchPreset",
    "operatorModel",
    "operatorDetectionLimit",
    "operatorDetectEvery",
    "operatorDetectionAreaMode",
    "objectPresetTrackingMode",
    "objectPresetLossBehavior",
    "lastAppliedObjectPreset",
    "lastEditedObjectPreset"
]:
    print(k, "=", json.dumps(settings.get(k), ensure_ascii=False))

print("")
print("custom keys =", sorted(custom.keys()))
print("active custom preset =", json.dumps(preset, indent=2, ensure_ascii=False))

section("PROCESSES")
try:
    out = subprocess.run(
        ["ps", "ax", "-o", "pid=,args="],
        text=True,
        capture_output=True
    ).stdout

    for line in out.splitlines():
        if (
            "object_tracking_daemon.py" in line
            or "apply_ptz_object_preset.py" in line
            or "ptz_autopilot" in line
            or "mjpeg" in line
        ):
            print(line)
except Exception as e:
    print("process check error:", e)
