!#/bin/bash

TS="$(date +%Y%m%d_%H%M%S)"
DIAG="/root/new_yolo8/logs/single_object_diag_$TS"
mkdir -p "$DIAG"

echo "DIAG=$DIAG"

echo
echo "===================================================================================================="
echo "0. BASELINE AUDIT"
echo "===================================================================================================="
python3 hydrate_runtime_settings.py > "$DIAG/00_hydrate_runtime_settings.log" 2>&1 || true
sleep 3
python3 ptz_contract_audit.py | tee "$DIAG/01_ptz_contract_audit.txt"

echo
echo "===================================================================================================="
echo "1. PROCESS SNAPSHOT"
echo "===================================================================================================="
pgrep -af 'mjpeg_rknn_http|ptz_autopilot|settings_persist_daemon.py|object_tracking_daemon.py|apply_ptz_object_preset.py' | tee "$DIAG/02_processes.txt"

echo
echo "===================================================================================================="
echo "2. SETTINGS / ACTIVE OBJECT PRESET"
echo "===================================================================================================="
curl -sS http://127.0.0.1:8080/api/settings > "$DIAG/10_api_settings.json"
python3 - <<'PY' | tee "$DIAG/11_active_object_preset.txt"
import json
from pathlib import Path

p = Path("/root/new_yolo8/logs")
latest = sorted(p.glob("single_object_diag_*"))[-1]
settings = json.loads((latest / "10_api_settings.json").read_text())

active = settings.get("activeObjectPreset")
search = settings.get("activeSearchPreset")
custom = settings.get("objectPresetsCustom") or {}
preset = custom.get(active) or {}

print("activeObjectPreset =", active)
print("activeSearchPreset =", search)
print("ptzArmed =", settings.get("ptzArmed"))
print("controlMode =", settings.get("controlMode"))
print("operatorModel =", settings.get("operatorModel"))
print("operatorDetectionLimit =", settings.get("operatorDetectionLimit"))
print("operatorDetectEvery =", settings.get("operatorDetectEvery"))
print("operatorDetectionAreaMode =", settings.get("operatorDetectionAreaMode"))
print("detectorSelectedClasses =", settings.get("detectorSelectedClasses"))
print()
print("active preset body:")
print(json.dumps(preset, indent=2, ensure_ascii=False))
PY

echo
echo "===================================================================================================="
echo "3. DETECTOR CONFIG"
echo "===================================================================================================="
curl -sS http://127.0.0.1:8080/api/detector/config       | tee "$DIAG/20_detector_config.json"       | python3 -m json.tool || true
curl -sS http://127.0.0.1:8080/api/detection/limits      | tee "$DIAG/21_detection_limits.json"      | python3 -m json.tool || true
curl -sS http://127.0.0.1:8080/api/detection/throttle    | tee "$DIAG/22_detection_throttle.json"    | python3 -m json.tool || true
curl -sS http://127.0.0.1:8080/api/detection/roi_config  | tee "$DIAG/23_detection_roi_config.json"  | python3 -m json.tool || true

echo
echo "===================================================================================================="
echo "4. DETECTIONS / TRACKER / AUTOPILOT SAMPLES"
echo "===================================================================================================="

for i in $(seq 1 12); do
  NOW="$(date --iso-8601=ns)"

  echo "===== sample $i $NOW detections =====" | tee -a "$DIAG/30_samples.txt"
  curl -sS http://127.0.0.1:8080/api/detections \
    | tee "$DIAG/30_detections_$i.json" \
    | python3 -m json.tool 2>/dev/null | tee -a "$DIAG/30_samples.txt" || true

  echo "===== sample $i $NOW tracker/state =====" | tee -a "$DIAG/30_samples.txt"
  curl -sS http://127.0.0.1:8080/api/tracker/state \
    | tee "$DIAG/31_tracker_state_$i.json" \
    | python3 -m json.tool 2>/dev/null | tee -a "$DIAG/30_samples.txt" || true

  echo "===== sample $i $NOW autopilot/state =====" | tee -a "$DIAG/30_samples.txt"
  curl -sS http://127.0.0.1:8090/api/autopilot/state \
    | tee "$DIAG/32_autopilot_state_$i.json" \
    | python3 -m json.tool 2>/dev/null | tee -a "$DIAG/30_samples.txt" || true

  sleep 0.5
done

echo
echo "===================================================================================================="
echo "5. OBJECT TRACKING EVENTS / DAEMON LOGS"
echo "===================================================================================================="

echo "--- find object tracking logs ---" | tee "$DIAG/40_log_paths.txt"
find /root/new_yolo8 /dev/shm -maxdepth 5 -type f \( \
  -name 'object_tracking_events.jsonl' -o \
  -name 'object_tracking_daemon.log' -o \
  -name '*object_tracking*log*' \
\) 2>/dev/null | sort | tee -a "$DIAG/40_log_paths.txt"

echo
echo "--- tail known event logs ---" | tee "$DIAG/41_object_tracking_events_tail.txt"

for f in \
  /dev/shm/new_yolo8_object_tracking/object_tracking_events.jsonl \
  /dev/shm/new_yolo8_object_tracking/object_tracking_daemon.log \
  /root/new_yolo8/object_tracking_events.jsonl \
  /root/new_yolo8/object_tracking_daemon.log
do
  if [ -f "$f" ]; then
    echo
    echo "===== $f ====="
    tail -n 200 "$f"
  fi
done | tee -a "$DIAG/41_object_tracking_events_tail.txt"

echo
echo "===================================================================================================="
echo "6. CODE MAP — mjpeg_gst_http.cpp tracker/select/acquire"
echo "===================================================================================================="

grep -nE \
'SimpleTracker|selected_track_id|selected_box_valid|TRACKING|LOST|REACQUIRE|tracker/state|/api/tracker/state|class Track|choose|best|iou|color|lost|reacquire' \
mjpeg_gst_http.cpp \
| tee "$DIAG/50_grep_mjpeg_tracker.txt"

echo
echo "===================================================================================================="
echo "7. CODE MAP — apply_ptz_object_preset.py acquire/search"
echo "===================================================================================================="

grep -nE \
'choose_best_detection_once|try_acquire_once|start_tracking_target|selected_track_id|run_continuous_wide_scan_x|tracking_ok|lost_grace|run_active_search_preset_once' \
apply_ptz_object_preset.py \
| tee "$DIAG/51_grep_apply_object_preset.txt"

echo
echo "===================================================================================================="
echo "8. COMPACT TRACKING SUMMARY"
echo "===================================================================================================="

python3 - <<'PY' | tee "$DIAG/60_compact_tracking_summary.txt"
import json
from pathlib import Path

diag = sorted(Path("/root/new_yolo8/logs").glob("single_object_diag_*"))[-1]
print("DIAG:", diag)

def load(path):
    try:
        return json.loads(path.read_text())
    except Exception as e:
        return {"__error__": str(e)}

settings = load(diag / "10_api_settings.json")
active = settings.get("activeObjectPreset")
preset = (settings.get("objectPresetsCustom") or {}).get(active) or {}

print()
print("ACTIVE")
print("  activeObjectPreset:", active)
print("  activeSearchPreset:", settings.get("activeSearchPreset"))
print("  classes:", preset.get("classes"))
print("  tracking_mode:", preset.get("tracking_mode"))
print("  loss_behavior:", preset.get("loss_behavior"))
print("  max_detections:", preset.get("max_detections"))
print("  detect_every_n_frames:", preset.get("detect_every_n_frames"))
print("  detection_mode:", preset.get("detection_mode"))

print()
print("SAMPLES")
for i in range(1, 13):
    det = load(diag / f"30_detections_{i}.json")
    trk = load(diag / f"31_tracker_state_{i}.json")
    ap = load(diag / f"32_autopilot_state_{i}.json")

    detections = det.get("detections")
    if detections is None:
        detections = det.get("objects")
    if detections is None and isinstance(det, list):
        detections = det
    det_count = len(detections or [])

    print(
        f"sample={i:02d}",
        "det_count=", det_count,
        "tracker_mode=", trk.get("mode"),
        "selected_track_id=", trk.get("selected_track_id"),
        "selected_box_valid=", trk.get("selected_box_valid"),
        "tracking_ok=", trk.get("tracking_ok"),
        "lost=", trk.get("lost"),
        "autopilot_enabled=", ap.get("enabled"),
        "autopilot_mode=", ap.get("mode"),
        "last_tracker_mode=", ap.get("last_tracker_mode"),
        "last_track_id=", ap.get("last_track_id"),
    )
PY

echo
echo "===================================================================================================="
echo "9. ARCHIVE"
echo "===================================================================================================="
tar -czf "$DIAG.tar.gz" -C "$(dirname "$DIAG")" "$(basename "$DIAG")"
ls -lah "$DIAG" "$DIAG.tar.gz"

echo
echo "DONE"
echo "Send back:"
echo "  $DIAG/60_compact_tracking_summary.txt"
echo "  $DIAG/41_object_tracking_events_tail.txt"
echo "  $DIAG/50_grep_mjpeg_tracker.txt"
echo "  $DIAG/51_grep_apply_object_preset.txt"
echo "  or archive: $DIAG.tar.gz"