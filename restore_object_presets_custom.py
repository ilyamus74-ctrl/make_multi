from pathlib import Path
import json
import time
import urllib.request

ROOT = Path("/root/new_yolo8")
SETTINGS_FILE = ROOT / "ui_settings.json"

MJPEG_BASE = "http://127.0.0.1:8080"

DEFAULTS = {
    "person_single": {
        "label": "ЧЕЛОВЕК",
        "classes": [0],
        "detection_mode": "full_frame",
        "max_detections": 5,
        "max_raw_candidates": 25,
        "detect_every_n_frames": 1,
        "tracking_mode": "single_auto",
        "loss_behavior": "continuous_wide_scan_x",
        "ptz": {
            "target_x": 0.50,
            "target_y": 0.46,
            "auto_zoom_enable": True,
            "auto_zoom_target_h": 0.68,
            "auto_zoom_deadzone": 0.08,
            "auto_zoom_cmd": 10,
            "auto_zoom_sign": 1,
            "auto_zoom_period_ms": 350
        }
    },
    "car_single": {
        "label": "МАШИНА",
        "classes": [2, 3, 5, 7],
        "detection_mode": "full_frame",
        "max_detections": 8,
        "max_raw_candidates": 40,
        "detect_every_n_frames": 1,
        "tracking_mode": "single_auto",
        "loss_behavior": "continuous_wide_scan_x",
        "ptz": {
            "target_x": 0.50,
            "target_y": 0.50,
            "auto_zoom_enable": True,
            "auto_zoom_target_h": 0.48,
            "auto_zoom_deadzone": 0.10,
            "auto_zoom_cmd": 8,
            "auto_zoom_sign": 1,
            "auto_zoom_period_ms": 450
        }
    }
}

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

def read_json_file(path):
    try:
        return json.loads(path.read_text(encoding="utf-8", errors="replace"))
    except Exception:
        return None

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
        else:
            p["ptz"] = {}

        if not p.get("tracking_mode"):
            p["tracking_mode"] = "multi_operator" if name in ["people", "cars", "airplanes", "birds"] else "single_auto"

        if not p.get("loss_behavior"):
            p["loss_behavior"] = "operator_select" if p.get("tracking_mode") == "multi_operator" else "continuous_wide_scan_x"

        out[name] = p

    return out

def find_custom_in_backups():
    candidates = []

    for path in sorted(ROOT.glob("ui_settings.json*")):
        data = read_json_file(path)

        if not isinstance(data, dict):
            continue

        custom = data.get("objectPresetsCustom")

        if isinstance(custom, dict) and custom:
            candidates.append((path, path.stat().st_mtime, sanitize_custom(custom)))

    if not candidates:
        return None, {}

    candidates.sort(key=lambda x: x[1], reverse=True)

    return candidates[0][0], candidates[0][2]

def build_from_runtime(settings):
    active = settings.get("activeObjectPreset") or "car_single"
    model = settings.get("operatorModel") or ""

    custom = dict(DEFAULTS)

    if active == "car_single":
        p = dict(custom["car_single"])
        p["model"] = model
        p["operatorModel"] = model
        p["max_detections"] = int(settings.get("operatorDetectionLimit") or p["max_detections"])
        p["detect_every_n_frames"] = int(settings.get("operatorDetectEvery") or p["detect_every_n_frames"])
        p["detection_mode"] = str(settings.get("operatorDetectionAreaMode") or p["detection_mode"])
        p["max_raw_candidates"] = max(20, int(p["max_detections"]) * 4)
        custom["car_single"] = p

    if "person_single" in custom:
        custom["person_single"]["model"] = model
        custom["person_single"]["operatorModel"] = model

    return sanitize_custom(custom)

def main():
    runtime = get_json(f"{MJPEG_BASE}/api/settings")
    file_data = read_json_file(SETTINGS_FILE) or {}

    backup_path, backup_custom = find_custom_in_backups()

    if backup_custom:
        custom = backup_custom
        source = str(backup_path)
    else:
        custom = build_from_runtime(runtime)
        source = "runtime/defaults"

    merged = dict(file_data)
    merged.update(runtime)

    merged["objectPresetsCustom"] = custom
    merged["activeObjectPreset"] = runtime.get("activeObjectPreset") or merged.get("activeObjectPreset") or "car_single"
    merged["activeSearchPreset"] = runtime.get("activeSearchPreset") or merged.get("activeSearchPreset") or "lost_step_wait"
    merged["ptzArmed"] = False
    merged["controlMode"] = "manual"

    active = merged["activeObjectPreset"]
    active_preset = custom.get(active) or custom.get("car_single") or {}

    if active_preset:
        merged["operatorModel"] = active_preset.get("model") or active_preset.get("operatorModel") or merged.get("operatorModel") or ""
        merged["operatorDetectionLimit"] = int(active_preset.get("max_detections") or merged.get("operatorDetectionLimit") or 10)
        merged["operatorDetectEvery"] = int(active_preset.get("detect_every_n_frames") or merged.get("operatorDetectEvery") or 1)
        merged["operatorDetectionAreaMode"] = str(active_preset.get("detection_mode") or merged.get("operatorDetectionAreaMode") or "full_frame")
        merged["detectorSelectedClasses"] = active_preset.get("classes") or merged.get("detectorSelectedClasses") or []

    bak = SETTINGS_FILE.with_suffix(SETTINGS_FILE.suffix + f".bak_restore_custom_{int(time.time())}")

    if SETTINGS_FILE.exists():
        bak.write_text(SETTINGS_FILE.read_text(encoding="utf-8", errors="replace"), encoding="utf-8")

    SETTINGS_FILE.write_text(json.dumps(merged, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")

    post_json(f"{MJPEG_BASE}/api/settings", merged)

    print("OK restored objectPresetsCustom")
    print("source =", source)
    print("backup =", bak)
    print("activeObjectPreset =", merged.get("activeObjectPreset"))
    print("activeSearchPreset =", merged.get("activeSearchPreset"))
    print("ptzArmed =", merged.get("ptzArmed"))
    print("controlMode =", merged.get("controlMode"))
    print("custom keys =", sorted(custom.keys()))
    print("active preset =", json.dumps(custom.get(active, {}), indent=2, ensure_ascii=False))

if __name__ == "__main__":
    main()
