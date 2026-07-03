#!/usr/bin/env python3
from pathlib import Path
import json
import os
import signal
import time
import urllib.request

ROOT = Path(__file__).resolve().parent
SETTINGS_FILE = ROOT / "ui_settings.json"
RUNTIME_DIR = Path("/dev/shm/new_yolo8_object_tracking")
LOG_FILE = RUNTIME_DIR / "settings_persist_daemon.log"

MJPEG_BASE = "http://127.0.0.1:8080"

ALLOWED_PTZ = {
    "target_x",
    "target_y",
    "auto_zoom_enable",
    "auto_zoom_target_h",
    "auto_zoom_deadzone",
    "auto_zoom_cmd",
    "auto_zoom_sign",
    "auto_zoom_period_ms",
    "zoom_scale_enable",
    "zoom_scale_min",
    "zoom_scale_max",
    "zoom_scale_smoothing"
}

STOP = False

def on_signal(signum, frame):
    global STOP
    STOP = True

signal.signal(signal.SIGTERM, on_signal)
signal.signal(signal.SIGINT, on_signal)

def log(msg):
    RUNTIME_DIR.mkdir(parents=True, exist_ok=True)
    line = f"{time.strftime('%Y-%m-%d %H:%M:%S')} {msg}"
    print(line, flush=True)

    try:
        with LOG_FILE.open("a", encoding="utf-8") as f:
            f.write(line + "\n")
    except Exception:
        pass

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
    except Exception as e:
        log(f"read file error: {e}")
        return {}

def sanitize_custom(custom):
    out = {}

    for name, preset in dict(custom or {}).items():
        if not isinstance(preset, dict):
            continue

        p = dict(preset)

        if isinstance(p.get("ptz"), dict):
            p["ptz"] = {
                k: v
                for k, v in p["ptz"].items()
                if k in ALLOWED_PTZ
            }

        out[name] = p

    return out

def stable_json(data):
    return json.dumps(data, sort_keys=True, ensure_ascii=False, separators=(",", ":"))

def write_file_settings(data):
    old = read_file_settings()

    if stable_json(old) == stable_json(data):
        return False

    tmp = SETTINGS_FILE.with_suffix(".json.tmp")
    tmp.write_text(json.dumps(data, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    tmp.replace(SETTINGS_FILE)

    return True

def wait_api():
    last = None

    for _ in range(80):
        try:
            return get_json(f"{MJPEG_BASE}/api/settings", timeout=2)
        except Exception as e:
            last = e
            time.sleep(0.25)

    raise RuntimeError(f"api not ready: {last}")

def non_empty_dict(x):
    return isinstance(x, dict) and bool(x)

def sync_once():
    runtime = get_json(f"{MJPEG_BASE}/api/settings", timeout=3)
    file_data = read_file_settings()

    runtime_custom = runtime.get("objectPresetsCustom")
    file_custom = file_data.get("objectPresetsCustom")

    api_needs_repair = False

    # Если API потерял custom, но файл его имеет — восстановить API.
    if not non_empty_dict(runtime_custom) and non_empty_dict(file_custom):
        runtime["objectPresetsCustom"] = file_custom
        api_needs_repair = True

    custom = runtime.get("objectPresetsCustom")

    if non_empty_dict(custom):
        runtime["objectPresetsCustom"] = sanitize_custom(custom)

    runtime["config_version"] = max(16, int(runtime.get("config_version") or 16))
    runtime["activeObjectPreset"] = runtime.get("activeObjectPreset") or file_data.get("activeObjectPreset") or "person_single"
    runtime["activeSearchPreset"] = runtime.get("activeSearchPreset") or file_data.get("activeSearchPreset") or "lost_step_wait"

    # В runtime не вмешиваемся, но в файл auto-arm не сохраняем.
    file_out = dict(runtime)
    file_out["ptzArmed"] = False
    file_out["controlMode"] = "manual"

    wrote = write_file_settings(file_out)

    if api_needs_repair:
        post_json(f"{MJPEG_BASE}/api/settings", runtime, timeout=3)
        log("repaired API objectPresetsCustom from file")

    if wrote:
        keys = sorted((file_out.get("objectPresetsCustom") or {}).keys())
        log(
            "persisted settings "
            + f"active={file_out.get('activeObjectPreset')} "
            + f"search={file_out.get('activeSearchPreset')} "
            + f"custom={keys}"
        )

def main():
    log("settings persist daemon start")
    wait_api()

    while not STOP:
        try:
            sync_once()
        except Exception as e:
            log(f"sync error: {e}")

        time.sleep(2.0)

    log("settings persist daemon stop")

if __name__ == "__main__":
    main()
