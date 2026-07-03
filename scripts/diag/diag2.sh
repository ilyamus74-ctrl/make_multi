#!/usr/bin/env bash
set -u

cd /root/new_yolo8 || exit 1

TS="$(date +%Y%m%d_%H%M%S)"
DIAG="/root/new_yolo8/logs/motion_target_diag_$TS"
mkdir -p "$DIAG"

echo "DIAG=$DIAG"

echo
echo "===================================================================================================="
echo "0. OPTIONAL: STOP PTZ FOR STATIONARY MOTION MEASUREMENT"
echo "===================================================================================================="

echo "Current autopilot before stop:"
curl -sS http://127.0.0.1:8090/api/autopilot/state | tee "$DIAG/00_autopilot_before_stop.json" | python3 -m json.tool || true

echo
echo "Stopping autopilot for clean motion capture..."
curl -sS -X POST http://127.0.0.1:8090/api/autopilot/stop \
  -H 'Content-Type: application/json' \
  -d '{}' | tee "$DIAG/01_autopilot_stop_response.json" | python3 -m json.tool || true

sleep 1

echo
echo "Autopilot after stop:"
curl -sS http://127.0.0.1:8090/api/autopilot/state | tee "$DIAG/02_autopilot_after_stop.json" | python3 -m json.tool || true

echo
echo "===================================================================================================="
echo "1. ACTIVE SETTINGS"
echo "===================================================================================================="

curl -sS http://127.0.0.1:8080/api/settings > "$DIAG/settings.json"

python3 - "$DIAG/settings.json" <<'PY' | tee "$DIAG/settings_summary.txt"
import json, sys
path = sys.argv[1]
s = json.load(open(path, encoding="utf-8"))

active = s.get("activeObjectPreset")
search = s.get("activeSearchPreset")
preset = (s.get("objectPresetsCustom") or {}).get(active) or {}

print("activeObjectPreset =", active)
print("activeSearchPreset =", search)
print("ptzArmed =", s.get("ptzArmed"))
print("controlMode =", s.get("controlMode"))
print("operatorModel =", s.get("operatorModel"))
print("operatorDetectionLimit =", s.get("operatorDetectionLimit"))
print("operatorDetectEvery =", s.get("operatorDetectEvery"))
print("operatorDetectionAreaMode =", s.get("operatorDetectionAreaMode"))
print("detectorSelectedClasses =", s.get("detectorSelectedClasses"))
print()
print("active preset body:")
print(json.dumps(preset, indent=2, ensure_ascii=False))
PY

echo
echo "===================================================================================================="
echo "2. DETECTOR SNAPSHOT"
echo "===================================================================================================="

curl -sS http://127.0.0.1:8080/api/detector/config      | tee "$DIAG/detector_config.json"      | python3 -m json.tool || true
curl -sS http://127.0.0.1:8080/api/detection/limits     | tee "$DIAG/detection_limits.json"     | python3 -m json.tool || true
curl -sS http://127.0.0.1:8080/api/detection/throttle   | tee "$DIAG/detection_throttle.json"   | python3 -m json.tool || true
curl -sS http://127.0.0.1:8080/api/detection/roi_config | tee "$DIAG/detection_roi_config.json" | python3 -m json.tool || true

echo
echo "===================================================================================================="
echo "3. COLLECT DETECTIONS FOR MOTION ANALYSIS"
echo "===================================================================================================="

for i in $(seq 1 100); do
  curl -sS http://127.0.0.1:8080/api/detections > "$DIAG/detections_$i.json"
  sleep 0.12
done

echo
echo "===================================================================================================="
echo "4. TRACKER / AUTOPILOT STATE AFTER CAPTURE"
echo "===================================================================================================="

curl -sS http://127.0.0.1:8080/api/tracker/state   | tee "$DIAG/tracker_state.json"   | python3 -m json.tool || true
curl -sS http://127.0.0.1:8090/api/autopilot/state | tee "$DIAG/autopilot_state.json" | python3 -m json.tool || true

echo
echo "===================================================================================================="
echo "5. MOTION SCORE BY TRACK_ID"
echo "===================================================================================================="

python3 - "$DIAG" <<'PY' | tee "$DIAG/motion_score_summary.txt"
import json, math, glob, os, sys
from collections import defaultdict

diag = sys.argv[1]
tracks = defaultdict(list)

def get_items(d):
    if isinstance(d, dict):
        for key in ("items", "detections", "objects"):
            v = d.get(key)
            if isinstance(v, list):
                return v
    if isinstance(d, list):
        return d
    return []

def box_from_item(b):
    # current API style
    if all(k in b for k in ("left", "top", "right", "bottom")):
        return float(b["left"]), float(b["top"]), float(b["right"]), float(b["bottom"])

    # fallback possible styles
    box = b.get("box") or b.get("bbox")
    if isinstance(box, dict) and all(k in box for k in ("left", "top", "right", "bottom")):
        return float(box["left"]), float(box["top"]), float(box["right"]), float(box["bottom"])
    if isinstance(box, list) and len(box) >= 4:
        return float(box[0]), float(box[1]), float(box[2]), float(box[3])

    return None

def cls_from_item(b):
    for k in ("cls", "class", "class_id", "cls_id"):
        if k in b:
            try:
                return int(b[k])
            except Exception:
                return b[k]
    return None

def conf_from_item(b):
    for k in ("prop", "score", "confidence", "conf"):
        if k in b:
            try:
                return float(b[k])
            except Exception:
                pass
    return 0.0

def tid_from_item(b):
    for k in ("id", "track_id", "trackId"):
        if k in b:
            try:
                return int(b[k])
            except Exception:
                pass
    return None

paths = sorted(
    glob.glob(os.path.join(diag, "detections_*.json")),
    key=lambda p: int(os.path.basename(p).split("_")[1].split(".")[0])
)

for idx, path in enumerate(paths, 1):
    try:
        d = json.load(open(path, encoding="utf-8"))
    except Exception:
        continue

    w = float(d.get("width") or d.get("frame_width") or 1920) if isinstance(d, dict) else 1920
    h = float(d.get("height") or d.get("frame_height") or 1080) if isinstance(d, dict) else 1080
    frame = d.get("frameId") if isinstance(d, dict) else idx

    for b in get_items(d):
        tid = tid_from_item(b)
        if tid is None:
            continue

        box = box_from_item(b)
        if not box:
            continue

        l, t, r, bb = box
        cx = (l + r) * 0.5
        cy = (t + bb) * 0.5
        area = max(1.0, (r - l) * (bb - t))

        tracks[tid].append({
            "sample": idx,
            "frame": frame,
            "cls": cls_from_item(b),
            "cx": cx,
            "cy": cy,
            "cxn": cx / max(1.0, w),
            "cyn": cy / max(1.0, h),
            "area": area,
            "area_norm": area / max(1.0, w * h),
            "conf": conf_from_item(b),
            "box": [l, t, r, bb],
        })

print("diag =", diag)
print("samples =", len(paths))
print("track_count =", len(tracks))
print()

rows = []

for tid, arr in tracks.items():
    if len(arr) < 4:
        continue

    path_px = 0.0
    path_norm = 0.0

    for a, b in zip(arr, arr[1:]):
        path_px += math.hypot(b["cx"] - a["cx"], b["cy"] - a["cy"])
        path_norm += math.hypot(b["cxn"] - a["cxn"], b["cyn"] - a["cyn"])

    first = arr[0]
    last = arr[-1]

    direct_px = math.hypot(last["cx"] - first["cx"], last["cy"] - first["cy"])
    direct_norm = math.hypot(last["cxn"] - first["cxn"], last["cyn"] - first["cyn"])

    avg_conf = sum(x["conf"] for x in arr) / len(arr)
    avg_area = sum(x["area_norm"] for x in arr) / len(arr)

    motion_score = min(1.0, path_norm / 0.06)
    conf_score = max(0.0, min(1.0, avg_conf))
    size_score = min(1.0, avg_area / 0.02)

    center_dist = math.hypot(last["cxn"] - 0.5, last["cyn"] - 0.5)
    center_score = max(0.0, 1.0 - center_dist / 0.7)

    total = (
        0.45 * motion_score +
        0.25 * conf_score +
        0.15 * size_score +
        0.15 * center_score
    )

    rows.append({
        "tid": tid,
        "cls": first["cls"],
        "samples": len(arr),
        "path_px": path_px,
        "direct_px": direct_px,
        "path_norm": path_norm,
        "direct_norm": direct_norm,
        "avg_conf": avg_conf,
        "avg_area": avg_area,
        "last_center": [round(last["cx"], 1), round(last["cy"], 1)],
        "last_box": last["box"],
        "score": total,
    })

rows.sort(key=lambda x: x["score"], reverse=True)

for x in rows:
    print(
        f"id={x['tid']:<5}",
        f"cls={str(x['cls']):<3}",
        f"samples={x['samples']:<3}",
        f"path_px={x['path_px']:.1f}",
        f"direct_px={x['direct_px']:.1f}",
        f"path_norm={x['path_norm']:.4f}",
        f"direct_norm={x['direct_norm']:.4f}",
        f"avg_conf={x['avg_conf']:.3f}",
        f"avg_area={x['avg_area']:.4f}",
        f"score={x['score']:.3f}",
        f"last_center={x['last_center']}"
    )

print()
if rows:
    print("BEST_MOVING_CANDIDATE =", rows[0]["tid"])
PY

echo
echo "===================================================================================================="
echo "6. CODE MAP SHORT"
echo "===================================================================================================="

grep -nE 'choose_best_detection_once|try_acquire_once|start_tracking_target|selected_track_id|tracking_ok|lost_grace|run_active_search_preset_once' \
  apply_ptz_object_preset.py | tee "$DIAG/grep_apply.txt" || true

grep -nE 'SimpleTracker|selected_track_id|selected_box_valid|TRACKING|LOST|REACQUIRE|tracker/state|/api/tracker/state|iou|lost|reacquire' \
  mjpeg_gst_http.cpp | tee "$DIAG/grep_mjpeg.txt" || true

echo
echo "===================================================================================================="
echo "7. ARCHIVE"
echo "===================================================================================================="

tar -czf "$DIAG.tar.gz" -C "$(dirname "$DIAG")" "$(basename "$DIAG")"
ls -lah "$DIAG" "$DIAG.tar.gz"

echo
echo "DONE"
echo "Send:"
echo "cat $DIAG/motion_score_summary.txt"
echo "cat $DIAG/tracker_state.json"
echo "cat $DIAG/autopilot_state.json"
echo "or archive: $DIAG.tar.gz"
