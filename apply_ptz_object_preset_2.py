cd /root/new_yolo8

cp -a apply_ptz_object_preset.py apply_ptz_object_preset.py.bak_auto_arm_$(date +%s)

cat > apply_ptz_object_preset.py <<'PY'
#!/usr/bin/env python3
import argparse
import json
import sys
import time
import urllib.request
from pathlib import Path

ROOT = Path(__file__).resolve().parent
PRESET_FILE = ROOT / "ptz_object_presets.json"
SETTINGS_FILE = ROOT / "ui_settings.json"

MJPEG_BASE = "http://127.0.0.1:8080"
AUTOPILOT_BASE = "http://127.0.0.1:8090"


def http_json(method, url, payload=None, timeout=3):
    data = None
    headers = {}

    if payload is not None:
        data = json.dumps(payload).encode("utf-8")
        headers["Content-Type"] = "application/json"

    req = urllib.request.Request(url, data=data, headers=headers, method=method)

    with urllib.request.urlopen(req, timeout=timeout) as r:
        raw = r.read().decode("utf-8", errors="replace")

    if not raw.strip():
        return {}

    return json.loads(raw)


def get_json(url, timeout=3):
    return http_json("GET", url, None, timeout)


def post_json(url, payload, timeout=3):
    return http_json("POST", url, payload, timeout)


def load_json_file(path, default):
    try:
        if path.exists():
            return json.loads(path.read_text(encoding="utf-8"))
    except Exception as e:
        print(f"WARN: failed to read {path}: {e}", file=sys.stderr)
    return default


def save_json_file(path, data):
    tmp = path.with_suffix(path.suffix + ".tmp")
    tmp.write_text(json.dumps(data, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    tmp.replace(path)


def load_all_presets():
    base = load_json_file(PRESET_FILE, {"version": 1, "active": "person_single", "presets": {}})
    settings = load_json_file(SETTINGS_FILE, {})

    presets = dict(base.get("presets") or {})
    custom = settings.get("objectPresetsCustom") or {}

    if isinstance(custom, dict):
        presets.update(custom)

    active = (
        settings.get("lastAppliedObjectPreset", {}).get("name")
        or settings.get("activeObjectPreset")
        or base.get("active")
        or "person_single"
    )

    return base, settings, presets, active


def wait_api(url, attempts=30, delay=0.25):
    last = None

    for _ in range(attempts):
        try:
            return get_json(url, timeout=2)
        except Exception as e:
            last = e
            time.sleep(delay)

    raise RuntimeError(f"API not ready: {url}: {last}")


def frame_size():
    try:
        d = get_json(f"{MJPEG_BASE}/api/detections", timeout=2)
        w = int(d.get("width") or 1920)
        h = int(d.get("height") or 1080)
        return max(1, w), max(1, h)
    except Exception:
        return 1920, 1080


def sky_roi(w, h):
    return {
        "id": "sky_top",
        "enabled": True,
        "x": 0,
        "y": 0,
        "w": w,
        "h": max(1, round(h * 0.68)),
        "every_n_frames": 1,
        "classes": []
    }


def center_roi(w, h):
    rw = round(w * 0.60)
    rh = round(h * 0.55)
    return {
        "id": "center",
        "enabled": True,
        "x": max(0, round((w - rw) / 2)),
        "y": max(0, round((h - rh) / 2)),
        "w": max(1, rw),
        "h": max(1, rh),
        "every_n_frames": 1,
        "classes": []
    }


def full_roi(w, h):
    return {
        "id": "full_frame",
        "enabled": True,
        "x": 0,
        "y": 0,
        "w": w,
        "h": h,
        "every_n_frames": 1,
        "classes": []
    }


def roi_payload(mode):
    mode = str(mode or "full_frame")
    w, h = frame_size()

    if mode == "full_frame":
        return {"detection_mode": "full_frame", "rois": []}

    if mode == "tiled":
        return {"detection_mode": "tiled", "rois": []}

    if mode == "roi":
        return {"detection_mode": "roi", "rois": [sky_roi(w, h)]}

    if mode == "multi_roi":
        return {"detection_mode": "multi_roi", "rois": [sky_roi(w, h), center_roi(w, h)]}

    if mode == "hybrid":
        return {"detection_mode": "hybrid", "rois": [full_roi(w, h), sky_roi(w, h)]}

    return {"detection_mode": "full_frame", "rois": []}


def merge_settings(settings, preset_name, preset, ptz_armed=None):
    out = dict(settings or {})

    classes = list(preset.get("classes") or [])
    ptz = dict(preset.get("ptz") or {})

    out["config_version"] = max(16, int(out.get("config_version") or 16))
    out["activeObjectPreset"] = preset_name
    out["detectorEnabled"] = True
    out["detectorSelectedClasses"] = classes
    out["operatorDetectionLimit"] = int(preset.get("max_detections") or 10)
    out["operatorDetectEvery"] = int(preset.get("detect_every_n_frames") or 1)
    out["operatorDetectionAreaMode"] = str(preset.get("detection_mode") or "full_frame")
    out["objectPresetTrackingMode"] = str(preset.get("tracking_mode") or "manual_select")

    if ptz_armed is not None:
        out["ptzArmed"] = bool(ptz_armed)

    if ptz:
        out["ptzConfig"] = {
            "target_x": ptz.get("target_x"),
            "target_y": ptz.get("target_y"),
            "min_pan": ptz.get("min_pan"),
            "min_tilt": ptz.get("min_tilt")
        }
        out["ptzAutoZoom"] = ptz

    out["lastAppliedObjectPreset"] = {
        "name": preset_name,
        "label": preset.get("label") or preset_name,
        "tracking_mode": preset.get("tracking_mode") or "manual_select",
        "ts": int(time.time())
    }

    return out


def choose_best_detection(classes, attempts=30, delay=0.25):
    wanted = {int(x) for x in classes or []}
    last_count = 0

    for _ in range(max(1, attempts)):
        det = get_json(f"{MJPEG_BASE}/api/detections", timeout=3)
        items = det.get("items") if isinstance(det, dict) else []
        items = items if isinstance(items, list) else []
        last_count = len(items)

        candidates = []

        for item in items:
            try:
                cls = int(item.get("cls"))
            except Exception:
                cls = None

            if wanted and cls not in wanted:
                continue

            try:
                score = float(item.get("prop") or item.get("score") or item.get("conf") or 0)
            except Exception:
                score = 0.0

            try:
                tid = int(item.get("id"))
            except Exception:
                continue

            candidates.append((score, tid, item))

        if candidates:
            candidates.sort(key=lambda x: x[0], reverse=True)
            return candidates[0][2]

        time.sleep(delay)

    raise RuntimeError(f"no matching detections after attempts; total_last={last_count}; classes={sorted(wanted)}")


def select_target_and_start(classes, attempts=30, delay=0.25):
    target = choose_best_detection(classes, attempts=attempts, delay=delay)
    track_id = int(target["id"])

    post_json(f"{MJPEG_BASE}/api/tracker/clear", {})
    select_res = post_json(f"{MJPEG_BASE}/api/tracker/select", {"track_id": track_id})

    time.sleep(0.4)

    tr = get_json(f"{MJPEG_BASE}/api/tracker/state", timeout=3)

    if tr.get("mode") != "TRACKING" or tr.get("selected_box_valid") is not True:
        raise RuntimeError(f"tracker not ready after select: {tr}")

    start_res = post_json(f"{AUTOPILOT_BASE}/api/autopilot/start", {})

    time.sleep(0.2)

    ap = get_json(f"{AUTOPILOT_BASE}/api/autopilot/state", timeout=3)

    return {
        "target": target,
        "select": select_res,
        "tracker": tr,
        "start": start_res,
        "autopilot": ap
    }


def stop_ptz():
    try:
        post_json(f"{AUTOPILOT_BASE}/api/autopilot/stop", {})
    except Exception:
        pass

    try:
        post_json(f"{AUTOPILOT_BASE}/api/control/stop", {})
    except Exception:
        pass


def apply_preset(preset_name, preset, set_active=True, dry_run=False, arm="off", select_attempts=30, select_delay=0.25):
    classes = [int(x) for x in (preset.get("classes") or [])]
    mode = str(preset.get("detection_mode") or "full_frame")
    max_det = int(preset.get("max_detections") or 10)
    max_raw = int(preset.get("max_raw_candidates") or max(20, max_det * 5))
    every = int(preset.get("detect_every_n_frames") or 1)
    ptz = dict(preset.get("ptz") or {})
    tracking_mode = str(preset.get("tracking_mode") or "manual_select")

    base, settings, presets, active = load_all_presets()

    should_arm = False
    if arm == "force":
        should_arm = tracking_mode == "single_auto"
    elif arm == "auto":
        should_arm = bool(settings.get("ptzArmed")) and tracking_mode == "single_auto"

    report = {
        "preset": preset_name,
        "label": preset.get("label") or preset_name,
        "tracking_mode": tracking_mode,
        "classes": classes,
        "detection_mode": mode,
        "max_detections": max_det,
        "max_raw_candidates": max_raw,
        "detect_every_n_frames": every,
        "ptz": ptz,
        "arm": arm,
        "should_arm": should_arm
    }

    if dry_run:
        print(json.dumps({"ok": True, "dry_run": True, "would_apply": report}, indent=2, ensure_ascii=False))
        return report

    wait_api(f"{MJPEG_BASE}/api/ping", attempts=30, delay=0.25)
    wait_api(f"{AUTOPILOT_BASE}/api/autopilot/state", attempts=30, delay=0.25)

    det_res = post_json(f"{MJPEG_BASE}/api/detector/config", {
        "detect_enabled": True,
        "selected_classes": classes
    })

    lim_res = post_json(f"{MJPEG_BASE}/api/detection/limits", {
        "max_detections": max_det,
        "max_raw_candidates": max_raw
    })

    thr_res = post_json(f"{MJPEG_BASE}/api/detection/throttle", {
        "detect_every_n_frames": every
    })

    roi_res = post_json(f"{MJPEG_BASE}/api/detection/roi_config", roi_payload(mode))

    ptz_res = {}
    if ptz:
        ptz_res = post_json(f"{AUTOPILOT_BASE}/api/autopilot/config", ptz)

    arm_res = None
    if should_arm:
        arm_res = select_target_and_start(classes, attempts=select_attempts, delay=select_delay)

    if set_active:
        settings = merge_settings(settings, preset_name, preset, ptz_armed=should_arm if arm == "force" else None)
        if arm == "force":
            settings["controlMode"] = "ptz"

        save_json_file(SETTINGS_FILE, settings)

        base["active"] = preset_name
        save_json_file(PRESET_FILE, base)

        try:
            post_json(f"{MJPEG_BASE}/api/settings", settings)
        except Exception as e:
            print(f"WARN: /api/settings update failed: {e}", file=sys.stderr)

    result = {
        "ok": True,
        "applied": report,
        "detector": det_res,
        "limits": lim_res,
        "throttle": thr_res,
        "roi": roi_res,
        "autopilot": ptz_res,
        "arm_result": arm_res
    }

    print(json.dumps(result, indent=2, ensure_ascii=False))
    return result


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--preset", default="active", help="preset name or active")
    ap.add_argument("--list", action="store_true")
    ap.add_argument("--dry-run", action="store_true")
    ap.add_argument("--no-set-active", action="store_true")
    ap.add_argument("--arm", choices=["off", "auto", "force"], default="off")
    ap.add_argument("--disarm", action="store_true")
    ap.add_argument("--select-attempts", type=int, default=30)
    ap.add_argument("--select-delay", type=float, default=0.25)
    args = ap.parse_args()

    base, settings, presets, active = load_all_presets()

    if args.list:
        print("active =", active)
        print("ptzArmed =", bool(settings.get("ptzArmed")))
        for name, preset in presets.items():
            print(name, "=", preset.get("label") or name, "|", preset.get("tracking_mode") or "manual_select")
        return 0

    if args.disarm:
        stop_ptz()
        settings["ptzArmed"] = False
        settings["controlMode"] = settings.get("controlMode") or "ptz"
        save_json_file(SETTINGS_FILE, settings)
        try:
            post_json(f"{MJPEG_BASE}/api/settings", settings)
        except Exception:
            pass
        print(json.dumps({"ok": True, "disarmed": True}, indent=2, ensure_ascii=False))
        return 0

    name = active if args.preset == "active" else args.preset

    if name not in presets:
        print(f"ERROR: preset not found: {name}", file=sys.stderr)
        print("Available:", file=sys.stderr)
        for k in presets:
            print(f"  {k}", file=sys.stderr)
        return 2

    apply_preset(
        name,
        presets[name],
        set_active=not args.no_set_active,
        dry_run=args.dry_run,
        arm=args.arm,
        select_attempts=args.select_attempts,
        select_delay=args.select_delay,
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
PY

chmod +x apply_ptz_object_preset.py