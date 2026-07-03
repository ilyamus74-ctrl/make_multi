#!/usr/bin/env bash
set -u
cd /root/new_yolo8 || exit 1

TS="$(date +%Y%m%d_%H%M%S)"
OUT="/root/new_yolo8/logs/target_policy_live_$TS.txt"
mkdir -p /root/new_yolo8/logs

echo "OUT=$OUT"

for i in $(seq 1 120); do
  python3 - <<'PY' | tee -a "$OUT"
import json, urllib.request, time

def get(url):
    try:
        with urllib.request.urlopen(url, timeout=1.5) as r:
            return json.loads(r.read().decode("utf-8", "replace"))
    except Exception as e:
        return {"_err": str(e)}

last = get("http://127.0.0.1:8080/last.json")
ap = get("http://127.0.0.1:8090/api/autopilot/state")
zoom = get("http://127.0.0.1:8080/api/zoom/state")

items = last.get("items") or last.get("detections") or last.get("objects") or []
tracks = last.get("tracks") or []

print(
    f"ts={time.time():.3f}",
    f"det={len(items) if isinstance(items,list) else 'na'}",
    f"tracks={len(tracks) if isinstance(tracks,list) else 'na'}",
    f"last_mode={last.get('mode')}",
    f"has_sel={last.get('has_selected_track')}",
    f"sel_valid={last.get('selected_box_valid')}",
    f"sel_id={last.get('selected_track_id')}",
    f"lost={last.get('lost_frames')}/{last.get('max_lost_frames')}",
    f"cx={last.get('selected_cx')}",
    f"cy={last.get('selected_cy')}",
    f"pcx={last.get('predicted_cx')}",
    f"pcy={last.get('predicted_cy')}",
    f"vx={last.get('target_vx')}",
    f"vy={last.get('target_vy')}",
    f"motion={last.get('target_motion_norm')}",
    f"score={last.get('target_selection_score')}",
    f"policy={last.get('target_policy_profile')}",
    f"switch={last.get('target_switch_reason')}",
    f"ap_en={ap.get('enabled')}",
    f"ap_mode={ap.get('mode')}",
    f"ap_trk={ap.get('last_tracker_mode')}",
    f"ap_id={ap.get('last_track_id')}",
    f"err=({ap.get('err_x')},{ap.get('err_y')})",
    f"cmd=({ap.get('cmd_pan')},{ap.get('cmd_tilt')})",
    f"zoom={zoom.get('zoom_sample_idx')}",
)
PY
  sleep 0.5
done

echo
echo "===== target events tail =====" | tee -a "$OUT"
grep -E 'target_selection|target_switch|target_prediction|target_lost|target_reacquired|lost_grace|auto_zoom|TRACKING|LOST' \
  /dev/shm/new_yolo8_object_tracking/object_tracking_events.jsonl 2>/dev/null | tail -200 | tee -a "$OUT"

echo "DONE $OUT"
