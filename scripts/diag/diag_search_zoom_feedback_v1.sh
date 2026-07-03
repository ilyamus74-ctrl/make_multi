#!/usr/bin/env bash
set -u

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"

cd "$ROOT" || exit 1

cd /root/new_yolo8 || exit 1

TS="$(date +%Y%m%d_%H%M%S)"
DIAG="$ROOT/logs/search_zoom_feedback_diag_$TS"
mkdir -p "$DIAG"

echo "DIAG=$DIAG"

echo
echo "===================================================================================================="
echo "0. BASELINE STATE"
echo "===================================================================================================="

curl -sS http://127.0.0.1:8080/api/settings > "$DIAG/settings_before.json"
curl -sS http://127.0.0.1:8080/api/zoom/state > "$DIAG/zoom_before.json"
curl -sS http://127.0.0.1:8090/api/autopilot/state > "$DIAG/autopilot_before.json"
curl -sS http://127.0.0.1:8080/api/tracker/state > "$DIAG/tracker_before.json"

python3 - "$DIAG" <<'PY' | tee "$DIAG/baseline_summary.txt"
import json, sys, pathlib
d=pathlib.Path(sys.argv[1])

def load(name):
    try:
        return json.loads((d/name).read_text())
    except Exception as e:
        return {"__error__": str(e)}

s=load("settings_before.json")
z=load("zoom_before.json")
a=load("autopilot_before.json")
t=load("tracker_before.json")

print("activeObjectPreset =", s.get("activeObjectPreset"))
print("activeSearchPreset =", s.get("activeSearchPreset"))
print("ptzArmed =", s.get("ptzArmed"))
print("controlMode =", s.get("controlMode"))
print("zoom_sample_idx =", z.get("zoom_sample_idx"))
print("zoom_ratio =", z.get("zoom_ratio"))
print("zoom_move_busy =", z.get("zoom_move_busy"))
print("ap.active_profile_idx =", a.get("active_profile_idx"))
print("ap.active_zoom_sample_idx =", a.get("active_zoom_sample_idx"))
print("ap.speed_profile_source =", a.get("speed_profile_source"))
print("tracker.mode =", t.get("mode"))
print("tracker.selected_track_id =", t.get("selected_track_id"))
print("tracker.selected_box_valid =", t.get("selected_box_valid"))
PY

echo
echo "===================================================================================================="
echo "1. START PTZ / WATCH SEARCH FEEDBACK"
echo "===================================================================================================="

python3 - <<'PY'
import json, urllib.request, time

def get_json(url):
    with urllib.request.urlopen(url, timeout=3) as r:
        raw=r.read().decode("utf-8", errors="replace")
    return json.loads(raw) if raw.strip() else {}

def post_json(url, body):
    data=json.dumps(body).encode()
    req=urllib.request.Request(url, data=data, headers={"Content-Type":"application/json"}, method="POST")
    with urllib.request.urlopen(req, timeout=3) as r:
        raw=r.read().decode("utf-8", errors="replace")
    return json.loads(raw) if raw.strip() else {}

s=get_json("http://127.0.0.1:8080/api/settings")
s["ptzArmed"]=True
s["controlMode"]="ptz"
post_json("http://127.0.0.1:8080/api/settings", s)
print("armed ptz=true")
PY

sleep 1

for i in $(seq 1 80); do
  NOW="$(date +%s.%N)"

  curl -sS http://127.0.0.1:8080/api/zoom/state > "$DIAG/zoom_$i.json"
  curl -sS http://127.0.0.1:8090/api/autopilot/state > "$DIAG/autopilot_$i.json"
  curl -sS http://127.0.0.1:8080/api/tracker/state > "$DIAG/tracker_$i.json"
  curl -sS http://127.0.0.1:8080/api/detections > "$DIAG/detections_$i.json"

  python3 - "$DIAG" "$i" "$NOW" <<'PY' | tee -a "$DIAG/live_summary.txt"
import json, sys, pathlib
d=pathlib.Path(sys.argv[1])
i=sys.argv[2]
now=sys.argv[3]

def load(name):
    try:
        return json.loads((d/name).read_text())
    except Exception:
        return {}

z=load(f"zoom_{i}.json")
a=load(f"autopilot_{i}.json")
t=load(f"tracker_{i}.json")
det=load(f"detections_{i}.json")

items = det.get("items")
if not isinstance(items, list):
    items = []

print(
    f"sample={int(i):02d}",
    "ts=", now,
    "zoom_sample=", z.get("zoom_sample_idx"),
    "zoom_ratio=", z.get("zoom_ratio"),
    "zoom_busy=", z.get("zoom_move_busy"),
    "zoom_source=", z.get("zoom_source"),
    "ap_profile=", a.get("active_profile_idx"),
    "ap_zoom_sample=", a.get("active_zoom_sample_idx"),
    "ap_speed_src=", a.get("speed_profile_source"),
    "ap_mode=", a.get("mode"),
    "trk_mode=", t.get("mode"),
    "trk_id=", t.get("selected_track_id"),
    "box_valid=", t.get("selected_box_valid"),
    "lost=", t.get("lost_frames"),
    "det_count=", len(items),
)
PY

  sleep 0.5
done

echo
echo "===================================================================================================="
echo "2. STOP PTZ"
echo "===================================================================================================="

python3 - <<'PY'
import json, urllib.request

def get_json(url):
    with urllib.request.urlopen(url, timeout=3) as r:
        raw=r.read().decode("utf-8", errors="replace")
    return json.loads(raw) if raw.strip() else {}

def post_json(url, body):
    data=json.dumps(body).encode()
    req=urllib.request.Request(url, data=data, headers={"Content-Type":"application/json"}, method="POST")
    with urllib.request.urlopen(req, timeout=3) as r:
        raw=r.read().decode("utf-8", errors="replace")
    return json.loads(raw) if raw.strip() else {}

s=get_json("http://127.0.0.1:8080/api/settings")
s["ptzArmed"]=False
s["controlMode"]="manual"
post_json("http://127.0.0.1:8080/api/settings", s)
post_json("http://127.0.0.1:8090/api/autopilot/stop", {})
print("stopped")
PY

sleep 2

echo
echo "===================================================================================================="
echo "3. EVENTS"
echo "===================================================================================================="

for f in \
  /dev/shm/new_yolo8_object_tracking/object_tracking_events.jsonl \
  /dev/shm/new_yolo8_object_tracking/object_tracking_daemon.log \
  /root/new_yolo8/object_tracking_events.jsonl \
  /root/new_yolo8/object_tracking_daemon.log
do
  if [ -f "$f" ]; then
    echo "===== $f =====" | tee -a "$DIAG/events_tail.txt"
    tail -n 250 "$f" | tee -a "$DIAG/events_tail.txt"
  fi
done

echo
echo "===================================================================================================="
echo "4. COMPACT RESULT"
echo "===================================================================================================="

python3 - "$DIAG" <<'PY' | tee "$DIAG/compact_search_zoom_feedback.txt"
import json, sys, pathlib, re

d=pathlib.Path(sys.argv[1])

print("diag =", d)
print()

live = (d/"live_summary.txt").read_text(errors="replace").splitlines() if (d/"live_summary.txt").exists() else []

changes = []
last = None

for line in live:
    m = re.search(r"zoom_sample=\s*([^ ]+).*ap_profile=\s*([^ ]+).*ap_zoom_sample=\s*([^ ]+).*trk_mode=\s*([^ ]+)", line)
    if not m:
        continue
    state = m.groups()
    if state != last:
        changes.append(line)
        last = state

print("STATE CHANGES")
for line in changes:
    print(line)

print()
print("EVENTS FILTER")
events_path = d/"events_tail.txt"
if events_path.exists():
    for line in events_path.read_text(errors="replace").splitlines():
        if any(x in line for x in [
            "zoom_frame_move",
            "zoom_frame_skip_edge",
            "zoom_frame_move_not_settled",
            "search_preset_step",
            "search_preset_skip_acquire_zoom_not_settled",
            "search_preset_acquired",
            "lost_object_frame_search",
            "tracking",
            "candidate_found"
        ]):
            print(line)
PY

tar -czf "$DIAG.tar.gz" -C "$(dirname "$DIAG")" "$(basename "$DIAG")"

echo
echo "DONE"
echo "Send:"
echo "cat $DIAG/compact_search_zoom_feedback.txt"
echo "$DIAG.tar.gz"