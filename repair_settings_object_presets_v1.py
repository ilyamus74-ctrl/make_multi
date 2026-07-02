from pathlib import Path
import json
import time
import urllib.request

ROOT = Path("/root/new_yolo8")
SETTINGS_FILE = ROOT / "ui_settings.json"
MJPEG_BASE = "http://127.0.0.1:8080"

PRESETS = {
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
    "people": {
        "label": "ЛЮДИ",
        "classes": [0],
        "detection_mode": "full_frame",
        "max_detections": 10,
        "max_raw_candidates": 50,
        "detect_every_n_frames": 1,
        "tracking_mode": "multi_operator",
        "loss_behavior": "operator_select",
        "ptz": {
            "target_x": 0.50,
            "target_y": 0.47,
            "auto_zoom_enable": True,
            "auto_zoom_target_h": 0.58,
            "auto_zoom_deadzone": 0.10,
            "auto_zoom_cmd": 8,
            "auto_zoom_sign": 1,
            "auto_zoom_period_ms": 450
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
    },
    "cars": {
        "label": "МАШИНЫ",
        "classes": [2, 3, 5, 7],
        "detection_mode": "tiled",
        "max_detections": 15,
        "max_raw_candidates": 80,
        "detect_every_n_frames": 1,
        "tracking_mode": "multi_operator",
        "loss_behavior": "operator_select",
        "ptz": {
            "target_x": 0.50,
            "target_y": 0.50,
            "auto_zoom_enable": True,
            "auto_zoom_target_h": 0.42,
            "auto_zoom_deadzone": 0.12,
            "auto_zoom_cmd": 8,
            "auto_zoom_sign": 1,
            "auto_zoom_period_ms": 500
        }
    },
    "airplane_single": {
        "label": "САМОЛЁТ",
        "classes": [4],
        "detection_mode": "hybrid",
        "max_detections": 8,
        "max_raw_candidates": 60,
        "detect_every_n_frames": 1,
        "tracking_mode": "single_auto",
        "loss_behavior": "continuous_wide_scan_x",
        "ptz": {
            "target_x": 0.50,
            "target_y": 0.45,
            "auto_zoom_enable": True,
            "auto_zoom_target_h": 0.30,
            "auto_zoom_deadzone": 0.08,
            "auto_zoom_cmd": 10,
            "auto_zoom_sign": 1,
            "auto_zoom_period_ms": 350
        }
    },
    "airplanes": {
        "label": "САМОЛЁТЫ",
        "classes": [4],
        "detection_mode": "hybrid",
        "max_detections": 15,
        "max_raw_candidates": 100,
        "detect_every_n_frames": 1,
        "tracking_mode": "multi_operator",
        "loss_behavior": "operator_select",
        "ptz": {
            "target_x": 0.50,
            "target_y": 0.45,
            "auto_zoom_enable": True,
            "auto_zoom_target_h": 0.25,
            "auto_zoom_deadzone": 0.10,
            "auto_zoom_cmd": 10,
            "auto_zoom_sign": 1,
            "auto_zoom_period_ms": 400
        }
    },
    "bird_single": {
        "label": "ПТИЦА",
        "classes": [14],
        "detection_mode": "hybrid",
        "max_detections": 10,
        "max_raw_candidates": 80,
        "detect_every_n_frames": 1,
        "tracking_mode": "single_auto",
        "loss_behavior": "continuous_wide_scan_x",
        "ptz": {
            "target_x": 0.50,
            "target_y": 0.45,
            "auto_zoom_enable": True,
            "auto_zoom_target_h": 0.24,
            "auto_zoom_deadzone": 0.07,
            "auto_zoom_cmd": 12,
            "auto_zoom_sign": 1,
            "auto_zoom_period_ms": 300
        }
    },
    "birds": {
        "label": "ПТИЦЫ",
        "classes": [14],
        "detection_mode": "hybrid",
        "max_detections": 20,
        "max_raw_candidates": 120,
        "detect_every_n_frames": 1,
        "tracking_mode": "multi_operator",
        "loss_behavior": "operator_select",
        "ptz": {
            "target_x": 0.50,
            "target_y": 0.45,
            "auto_zoom_enable": True,
            "auto_zoom_target_h": 0.20,
            "auto_zoom_deadzone": 0.08,
            "auto_zoom_cmd": 12,
            "auto_zoom_sign": 1,
            "auto_zoom_period_ms": 300
        }
    }
}

def get_json(url):
    with urllib.request.urlopen(url, timeout=3) as r:
        raw = r.read().decode("utf-8", errors="replace")
    return json.loads(raw) if raw.strip() else {}

def post_json(url, body):
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

def read_file():
    if SETTINGS_FILE.exists():
        try:
            return json.loads(SETTINGS_FILE.read_text(encoding="utf-8", errors="replace"))
        except Exception:
            return {}
    return {}

def main():
    runtime = get_json(f"{MJPEG_BASE}/api/settings")
    file_data = read_file()

    model = runtime.get("operatorModel") or file_data.get("operatorModel") or "/root/new_yolo8/model_rknn/yolov8s_800x800_9out_fp32.rknn"

    custom = {}

    old_custom = {}
    old_custom.update(file_data.get("objectPresetsCustom") or {})
    old_custom.update(runtime.get("objectPresetsCustom") or {})

    for name, base in PRESETS.items():
        p = dict(base)

        if name in old_custom and isinstance(old_custom[name], dict):
            previous = dict(old_custom[name])
            previous.update({
                k: v
                for k, v in p.items()
                if k not in previous
            })
            p = previous

        p["model"] = p.get("model") or model
        p["operatorModel"] = p.get("operatorModel") or model

        custom[name] = p

    active = runtime.get("activeObjectPreset") or file_data.get("activeObjectPreset") or "car_single"

    if active not in custom:
        active = "car_single"

    active_preset = custom[active]

    merged = dict(file_data)
    merged.update(runtime)

    merged["objectPresetsCustom"] = custom
    merged["activeObjectPreset"] = active
    merged["activeSearchPreset"] = merged.get("activeSearchPreset") or "lost_step_wait"
    merged["ptzArmed"] = False
    merged["controlMode"] = "manual"

    merged["operatorModel"] = active_preset.get("model") or active_preset.get("operatorModel") or model
    merged["operatorDetectionLimit"] = int(active_preset.get("max_detections") or 10)
    merged["operatorDetectEvery"] = int(active_preset.get("detect_every_n_frames") or 1)
    merged["operatorDetectionAreaMode"] = str(active_preset.get("detection_mode") or "full_frame")
    merged["detectorSelectedClasses"] = active_preset.get("classes") or []

    merged["objectPresetTrackingMode"] = active_preset.get("tracking_mode") or "single_auto"
    merged["objectPresetLossBehavior"] = active_preset.get("loss_behavior") or "continuous_wide_scan_x"

    merged["lastAppliedObjectPreset"] = {
        "name": active,
        "label": active_preset.get("label") or active,
        "tracking_mode": merged["objectPresetTrackingMode"],
        "loss_behavior": merged["objectPresetLossBehavior"],
        "ts": int(time.time())
    }

    bak = SETTINGS_FILE.with_suffix(SETTINGS_FILE.suffix + f".bak_repair_presets_v1_{int(time.time())}")

    if SETTINGS_FILE.exists():
        bak.write_text(SETTINGS_FILE.read_text(encoding="utf-8", errors="replace"), encoding="utf-8")

    SETTINGS_FILE.write_text(json.dumps(merged, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")

    post_json(f"{MJPEG_BASE}/api/settings", merged)

    post_json(f"{MJPEG_BASE}/api/detector/config", {
        "detect_enabled": True,
        "selected_classes": active_preset.get("classes") or [],
        "current_model": merged["operatorModel"]
    })

    post_json(f"{MJPEG_BASE}/api/detection/limits", {
        "max_detections": merged["operatorDetectionLimit"],
        "max_raw_candidates": int(active_preset.get("max_raw_candidates") or max(20, merged["operatorDetectionLimit"] * 4))
    })

    post_json(f"{MJPEG_BASE}/api/detection/throttle", {
        "detect_every_n_frames": merged["operatorDetectEvery"]
    })

    print("OK repaired settings object presets")
    print("backup =", bak)
    print("activeObjectPreset =", active)
    print("custom keys =", sorted(custom.keys()))
    print("limit/every =", merged["operatorDetectionLimit"], merged["operatorDetectEvery"])
    print("area =", merged["operatorDetectionAreaMode"])

if __name__ == "__main__":
    main()
