#!/usr/bin/env bash
set -u
cd /root/new_yolo8 || exit 1

TS="$(date +%Y%m%d_%H%M%S)"
OUT="/root/new_yolo8/ai_collect_$TS"
mkdir -p "$OUT"

echo "OUT=$OUT"

save_cmd() {
  local name="$1"
  shift
  echo "### $*" > "$OUT/$name.txt"
  "$@" >> "$OUT/$name.txt" 2>&1 || true
}

save_url() {
  local name="$1"
  local url="$2"
  curl -sS "$url" > "$OUT/$name.json" 2>"$OUT/$name.err" || true
  python3 -m json.tool "$OUT/$name.json" > "$OUT/${name}.pretty.json" 2>/dev/null || true
}

echo "===== basic files ====="
cp -a ui_settings.json "$OUT/ui_settings.json" 2>/dev/null || true
cp -a PTZ_MASTER_CONTRACT.md "$OUT/PTZ_MASTER_CONTRACT.md" 2>/dev/null || true
cp -a ptz_contract_audit_last.json "$OUT/ptz_contract_audit_last.json" 2>/dev/null || true

echo "===== api snapshots ====="
save_url settings              http://127.0.0.1:8080/api/settings
save_url detector_config       http://127.0.0.1:8080/api/detector/config
save_url tracker_state         http://127.0.0.1:8080/api/tracker/state
save_url last_json             http://127.0.0.1:8080/last.json
save_url zoom_state            http://127.0.0.1:8080/api/zoom/state
save_url autopilot_state       http://127.0.0.1:8090/api/autopilot/state

echo "===== process/system ====="
save_cmd ps_all ps aux
save_cmd service_status systemctl status maklertour-ptz.service --no-pager
save_cmd journal_recent journalctl -u maklertour-ptz.service --no-pager -n 300
save_cmd disk df -h
save_cmd memory free -h

echo "===== source snippets ====="
for f in \
  launcher.sh \
  hydrate_runtime_settings.py \
  settings_persist_daemon.py \
  object_tracking_daemon.py \
  apply_ptz_object_preset.py \
  mjpeg_gst_http.cpp \
  ptz_autopilot.cpp \
  web/index.html
do
  if [ -f "$f" ]; then
    grep -nE 'TARGET_POLICY|moving_road|moving_near|PREDICTIVE|target_selection|target_switch|target_prediction|auto_zoom|go_to_sample|apply_nearest|speed_profile|zoom_sample|active_profile|scene_home|safe_home|roi|ROI' "$f" \
      > "$OUT/source_${f//\//_}_grep.txt" 2>/dev/null || true
  fi
done

echo "===== runtime logs copy ====="
mkdir -p "$OUT/dev_shm_logs"
cp -a /dev/shm/new_yolo8_object_tracking/* "$OUT/dev_shm_logs/" 2>/dev/null || true

echo "===== live sample 60 sec ====="
LIVE="$OUT/live_zoom_tracker.tsv"
echo -e "ts\tzoom_sample\tactive_profile\tzoom_ratio\tprofile_src\teff_pan\teff_tilt\teff_accel\tlead_ms\tap_mode\tap_trk\tap_id\terr_x\terr_y\tcmd_pan\tcmd_tilt\tbase_pan\tbase_tilt\ttrk_mode\tsel_id\tsel_valid\tlost\tcx\tcy\tpcx\tpcy\tvx\tvy\tmotion\tscore\tpolicy\tswitch\tauto_zoom_box_h\tauto_zoom_target_h\tauto_zoom_cmd" > "$LIVE"

for i in $(seq 1 240); do
python3 - <<'PY' >> "$LIVE"
import json, urllib.request, time

def get(url):
    try:
        with urllib.request.urlopen(url, timeout=0.8) as r:
            return json.loads(r.read().decode("utf-8", "replace"))
    except Exception as e:
        return {}

ap = get("http://127.0.0.1:8090/api/autopilot/state")
tr = get("http://127.0.0.1:8080/api/tracker/state")
zs = get("http://127.0.0.1:8080/api/zoom/state")

row = [
    f"{time.time():.3f}",
    zs.get("zoom_sample_idx"),
    ap.get("active_profile_idx"),
    ap.get("zoom_ratio"),
    ap.get("speed_profile_source"),
    ap.get("effective_max_pan"),
    ap.get("effective_max_tilt"),
    ap.get("effective_max_accel"),
    ap.get("ptz_lead_ms"),
    ap.get("mode"),
    ap.get("last_tracker_mode"),
    ap.get("last_track_id"),
    ap.get("err_x"),
    ap.get("err_y"),
    ap.get("cmd_pan"),
    ap.get("cmd_tilt"),
    ap.get("base_cmd_pan"),
    ap.get("base_cmd_tilt"),
    tr.get("mode"),
    tr.get("selected_track_id"),
    tr.get("selected_box_valid"),
    f"{tr.get('lost_frames')}/{tr.get('max_lost_frames')}",
    tr.get("selected_cx"),
    tr.get("selected_cy"),
    tr.get("predicted_cx"),
    tr.get("predicted_cy"),
    tr.get("target_vx"),
    tr.get("target_vy"),
    tr.get("target_motion_norm"),
    tr.get("target_selection_score"),
    tr.get("target_policy_profile"),
    tr.get("target_switch_reason"),
    ap.get("auto_zoom_box_h"),
    ap.get("auto_zoom_target_h"),
    ap.get("auto_zoom_cmd"),
]
print("\t".join("" if v is None else str(v) for v in row))
PY
sleep 0.25
done

echo "===== quick analysis =====" > "$OUT/quick_analysis.txt"
python3 - "$LIVE" <<'PY' >> "$OUT/quick_analysis.txt"
import sys, csv, math
rows=list(csv.DictReader(open(sys.argv[1]), delimiter="\t"))

def f(v):
    try: return float(v)
    except: return None

mismatch=[]
big=[]
switches=[]
zoom_moves=[]
prev_id=None
prev_zoom=None
prev_cmd=None

for r in rows:
    if r["zoom_sample"] and r["active_profile"] and r["zoom_sample"] != r["active_profile"]:
        mismatch.append(r)

    z=f(r["zoom_sample"])
    cp=abs(f(r["cmd_pan"]) or 0)
    ct=abs(f(r["cmd_tilt"]) or 0)
    if z is not None and z >= 7 and (cp >= 5 or ct >= 5):
        big.append(r)

    sid=r["sel_id"]
    if sid and prev_id and sid != prev_id:
        switches.append((prev_id, sid, r))
    if sid:
        prev_id=sid

    if r["zoom_sample"] and prev_zoom and r["zoom_sample"] != prev_zoom:
        zoom_moves.append((prev_zoom, r["zoom_sample"], r))
    if r["zoom_sample"]:
        prev_zoom=r["zoom_sample"]

print("samples", len(rows))
print("zoom_profile_mismatch", len(mismatch))
print("big_cmd_zoom_ge_7", len(big))
print("target_switches", len(switches))
print("zoom_sample_changes", len(zoom_moves))

print("\n-- last 20 rows --")
for r in rows[-20:]:
    print(r)

print("\n-- big commands zoom>=7 first 20 --")
for r in big[:20]:
    print({
        "ts": r["ts"],
        "zoom": r["zoom_sample"],
        "profile": r["active_profile"],
        "err": (r["err_x"], r["err_y"]),
        "cmd": (r["cmd_pan"], r["cmd_tilt"]),
        "eff": (r["eff_pan"], r["eff_tilt"]),
        "track": r["sel_id"],
        "motion": r["motion"],
        "switch": r["switch"],
        "az_box": r["auto_zoom_box_h"],
        "az_target": r["auto_zoom_target_h"],
        "az_cmd": r["auto_zoom_cmd"],
    })

print("\n-- target switches first 20 --")
for a,b,r in switches[:20]:
    print({
        "ts": r["ts"],
        "from": a,
        "to": b,
        "reason": r["switch"],
        "motion": r["motion"],
        "score": r["score"],
    })

print("\n-- zoom changes first 20 --")
for a,b,r in zoom_moves[:20]:
    print({
        "ts": r["ts"],
        "from": a,
        "to": b,
        "track": r["sel_id"],
        "az_box": r["auto_zoom_box_h"],
        "az_target": r["auto_zoom_target_h"],
        "az_cmd": r["auto_zoom_cmd"],
    })
PY

echo "===== audit quick ====="
python3 ptz_contract_audit.py > "$OUT/ptz_contract_audit.txt" 2>&1 || true

echo "===== package ====="
tar -czf "$OUT.tar.gz" -C "$(dirname "$OUT")" "$(basename "$OUT")"

echo
echo "DONE"
echo "$OUT.tar.gz"
echo
echo "Quick:"
cat "$OUT/quick_analysis.txt"
