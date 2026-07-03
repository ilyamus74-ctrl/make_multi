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

def read_file_settings():
    if not SETTINGS_FILE.exists():
        return {}

    try:
        data = json.loads(SETTINGS_FILE.read_text(encoding="utf-8", errors="replace"))
        return data if isinstance(data, dict) else {}
    except Exception:
        return {}

def wait_api():
    last = None

    for _ in range(60):
        try:
            return get_json(f"{MJPEG_BASE}/api/settings", timeout=2)
        except Exception as e:
            last = e
            time.sleep(0.25)

    raise RuntimeError(f"/api/settings not ready: {last}")

def non_empty_dict(x):
    return isinstance(x, dict) and bool(x)

def main():
    file_settings = read_file_settings()
    runtime_settings = wait_api()

    merged = dict(runtime_settings or {})
    merged.update(file_settings or {})

    runtime_custom = runtime_settings.get("objectPresetsCustom")
    file_custom = file_settings.get("objectPresetsCustom")

    if non_empty_dict(file_custom):
        merged["objectPresetsCustom"] = file_custom
    elif non_empty_dict(runtime_custom):
        merged["objectPresetsCustom"] = runtime_custom
    else:
        merged.pop("objectPresetsCustom", None)

    merged["config_version"] = max(16, int(merged.get("config_version") or 16))
    merged["activeObjectPreset"] = merged.get("activeObjectPreset") or "person_single"
    merged["activeSearchPreset"] = merged.get("activeSearchPreset") or "lost_step_wait"

    # Безопасность: после рестарта не auto-arm.
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
