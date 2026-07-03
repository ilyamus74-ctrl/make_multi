#!/usr/bin/env bash
set -u

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"

cd "$ROOT" || exit 1

cd /root/new_yolo8 || exit 1

TS="$(date +%Y%m%d_%H%M%S)"
DIAG="$ROOT/logs/tracking_live_diag_$TS"
mkdir -p "$DIAG"

echo "DIAG=$DIAG"

echo
echo "===== baseline ====="
curl -sS http://127.0.0.1:8080/api/settings > "$DIAG/settings_start.json" || true
curl -sS http://127.0.0.1:8080/api/detector/config > "$DIAG/detector_config_start.json" || true
curl -sS http://127.0.0.1:8080/api/zoom/state > "$DIAG/zoom_start.json" || true
curl -sS http://127.0.0.1:8090/api/autopilot/state > "$DIAG/autopilot_start.json" || true

: > "$DIAG/timeline.jsonl"
: > "$DIAG/timeline_compact.txt"

echo
echo "===== collect 90 sec ====="

for i in $(seq 1 90); do
  TS_NOW="$(date +%s.%N)"

  curl -sS http://127.0.0.1:8080/last.json > "$DIAG/last_$i.json" 2>/dev/null || echo '{}' > "$DIAG/last_$i.json"
  curl -sS http://127.0.0.1:8080/api/zoom/state > "$DIAG/zoom_$i.json" 2>/dev/null || echo '{}' > "$DIAG/zoom_$i.json"
  curl -sS http://127.0.0.1:8090/api/autopilot/state > "$DIAG/ap_$i.json" 2>/dev/null || echo '{}' > "$DIAG/ap_$i.json"

  python3 - "$DIAG" "$i" "$TS_NOW" <<'PY' >> "$DIAG/timeline_compact.txt"
import json, sys, pathlib

d = pathlib.Path(sys.argv[1])
i = int(sys.argv[2])
ts = sys.argv[3]

def load(name):
    try:
        return json.loads((d / name).read_text())
    except Exception:
        return {}

last = load(f"last_{i}.json")
zoom = load(f"zoom_{i}.json")
ap = load(f"ap_{i}.json")

items = last.get("items") or last.get("detections") or last.get("objects") or []
tracks = last.get("tracks") or []
tracker = last.get("tracker") or {}

det_count = len(items) if isinstance(items, list) else 0
track_count = len(tracks) if isinstance(tracks, list) else 0

mode = tracker.get("mode") or last.get("tracker_mode") or last.get("mode")
sel_id = tracker.get("selected_track_id") or last.get("selected_track_id") or last.get("track_id")
box_valid = tracker.get("selected_box_valid") or last.get("selected_box_valid")

print(
    f"sample={i:03d}",
    f"ts={ts}",
    f"det={det_count}",
    f"tracks={track_count}",
    f"trk_mode={mode}",
    f"sel_id={sel_id}",
    f"box_valid={box_valid}",
    f"zoom_sample={zoom.get('zoom_sample_idx')}",
    f"zoom_busy={zoom.get('zoom_move_busy')}",
    f"ap_enabled={ap.get('enabled')}",
    f"ap_mode={ap.get('mode')}",
    f"ap_last_tracker={ap.get('last_tracker_mode')}",
    f"ap_track={ap.get('last_track_id')}",
    f"cmd_pan={ap.get('cmd_pan')}",
    f"cmd_tilt={ap.get('cmd_tilt')}",
    f"err_x={ap.get('err_x')}",
    f"err_y={ap.get('err_y')}",
)
PY

  sleep 1
done

echo
echo "===== copy runtime logs ====="
cp -a /dev/shm/new_yolo8_object_tracking/object_tracking_events.jsonl "$DIAG/object_tracking_events.jsonl" 2>/dev/null || true
cp -a /dev/shm/new_yolo8_object_tracking/object_tracking_daemon.log "$DIAG/object_tracking_daemon.log" 2>/dev/null || true

echo
echo "===== summary ====="
tail -n 90 "$DIAG/timeline_compact.txt"

echo
echo "===== important events ====="
grep -E 'selected|TRACKING|LOST|REACQUIRE|target|acquire|search_preset|zoom_frame_move|manual_search_pulse|ptz_speed' \
  "$DIAG/object_tracking_events.jsonl" "$DIAG/object_tracking_daemon.log" 2>/dev/null | tail -160 \
  | tee "$DIAG/important_events_tail.txt" || true

tar -czf "$DIAG.tar.gz" -C "$(dirname "$DIAG")" "$(basename "$DIAG")"

echo
echo "DONE"
echo "Send:"
echo "$DIAG.tar.gz"
echo "and paste:"
echo "cat $DIAG/timeline_compact.txt | tail -90"
echo "cat $DIAG/important_events_tail.txt"