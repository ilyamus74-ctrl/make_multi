#!/usr/bin/env python3
from pathlib import Path
import json
import time
import urllib.request

ROOT = Path(__file__).resolve().parent
SETTINGS_FILE = ROOT / "ui_settings.json"

MJPEG_BASE = "http://127.0.0.1:8080"

def get_json(url, timeout=3):
    with urllib.request.urlopen(url, timeout=timeout) as r:
        raw = r.read().decode("utf-8", errors="replace")
    return json.loads(raw) if raw.strip() else {}

def post_json(url, body, timeout=3):
    data = json.dumps(body).encode("utf-8")
    req = urllib.request.Request(
        url,
        data=data,
        headers={"Content-Type": "application/json"},
        method="POST"
    )
    with urllib.request.urlopen(req, timeout=timeout) as r:
        raw = r.read().decode("utf-8", errors="replace")
    return json.loads(raw) if raw.strip() else {}

def wait_api():
    last = None

    for _ in range(40):
        try:
            return get_json(f"{MJPEG_BASE}/api/settings", timeout=2)
        except Exception as e:
            last = e
            time.sleep(0.25)

    raise RuntimeError(f"/api/settings not ready: {last}")

def main():
    if not SETTINGS_FILE.exists():
        print("WARN: ui_settings.json not found")
        return 0

    file_settings = json.loads(SETTINGS_FILE.read_text(encoding="utf-8", errors="replace"))
    runtime_settings = wait_api()

    merged = dict(runtime_settings or {})
    merged.update(file_settings or {})

    merged["config_version"] = max(16, int(merged.get("config_version") or 16))

    if not merged.get("activeObjectPreset"):
        merged["activeObjectPreset"] = "person_single"

    if not merged.get("activeSearchPreset"):
        merged["activeSearchPreset"] = "lost_step_wait"

    if "ptzArmed" not in merged:
        merged["ptzArmed"] = False

    if not merged.get("controlMode"):
        merged["controlMode"] = "manual"

    # Safety: service start must not accidentally arm PTZ.
    if merged.get("ptzArmed") is not True:
        merged["ptzArmed"] = False
        merged["controlMode"] = "manual"

    res = post_json(f"{MJPEG_BASE}/api/settings", merged, timeout=3)

    print("OK hydrated /api/settings from ui_settings.json")
    print("activeObjectPreset =", merged.get("activeObjectPreset"))
    print("activeSearchPreset =", merged.get("activeSearchPreset"))
    print("ptzArmed =", merged.get("ptzArmed"))
    print("controlMode =", merged.get("controlMode"))
    print("custom presets =", sorted((merged.get("objectPresetsCustom") or {}).keys()))
    print("api =", res)

    return 0

if __name__ == "__main__":
    raise SystemExit(main())
