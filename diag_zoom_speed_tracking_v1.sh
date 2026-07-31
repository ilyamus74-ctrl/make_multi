#!/usr/bin/env bash
set -u
cd /root/new_yolo8 || exit 1

TS="$(date +%Y%m%d_%H%M%S)"
OUT="/root/new_yolo8/logs/zoom_speed_tracking_$TS.tsv"
mkdir -p /root/new_yolo8/logs

echo "OUT=$OUT"

echo -e "ts\tzoom_sample\tactive_profile\tprofile_src\tzoom_ratio\tmax_pan\tmax_tilt\tmax_accel\teff_pan\teff_tilt\teff_accel\tkp\tkd\tlead_ms\tap_track\ttrk_mode\tsel_id\tmotion\tcx\tcy\tpcx\tpcy\terr_x\terr_y\tcmd_pan\tcmd_tilt\tbase_pan\tbase_tilt\tswitch_reason" > "$OUT"

for i in $(seq 1 240); do
python3 - <<'PY' >> "$OUT"
import json, urllib.request, time

def get(url):
    try:
        with urllib.request.urlopen(url, timeout=1.0) as r:
            return json.loads(r.read().decode("utf-8", "replace"))
    except Exception as e:
        return {"_err": str(e)}

ap = get("http://127.0.0.1:8090/api/autopilot/state")
tr = get("http://127.0.0.1:8080/api/tracker/state")
zs = get("http://127.0.0.1:8080/api/zoom/state")

row = [
    f"{time.time():.3f}",
    zs.get("zoom_sample_idx"),
    ap.get("active_profile_idx"),
    ap.get("speed_profile_source"),
    ap.get("zoom_ratio"),
    ap.get("profile_max_pan"),
    ap.get("profile_max_tilt"),
    ap.get("profile_max_accel"),
    ap.get("effective_max_pan"),
    ap.get("effective_max_tilt"),
    ap.get("effective_max_accel"),
    ap.get("kp"),
    ap.get("kd"),
    ap.get("ptz_lead_ms"),
    ap.get("last_track_id"),
    ap.get("last_tracker_mode"),
    tr.get("selected_track_id"),
    tr.get("target_motion_norm"),
    tr.get("selected_cx"),
    tr.get("selected_cy"),
    tr.get("predicted_cx"),
    tr.get("predicted_cy"),
    ap.get("err_x"),
    ap.get("err_y"),
    ap.get("cmd_pan"),
    ap.get("cmd_tilt"),
    ap.get("base_cmd_pan"),
    ap.get("base_cmd_tilt"),
    tr.get("target_switch_reason"),
]
print("\t".join("" if v is None else str(v) for v in row))
PY
sleep 0.25
done

echo
echo "===== quick analysis ====="

python3 - "$OUT" <<'PY'
import sys, csv, math
p=sys.argv[1]
rows=list(csv.DictReader(open(p), delimiter="\t"))

def f(x):
    try: return float(x)
    except: return None

mismatch=[]
big_cmd=[]
switches=[]
prev_id=None

for r in rows:
    zs=r["zoom_sample"]
    ap=r["active_profile"]
    if zs and ap and zs != ap:
        mismatch.append(r)

    zoom=f(r["zoom_sample"])
    cp=abs(f(r["cmd_pan"]) or 0)
    ct=abs(f(r["cmd_tilt"]) or 0)
    if zoom is not None and zoom >= 7 and (cp >= 6 or ct >= 6):
        big_cmd.append(r)

    sid=r["sel_id"]
    if sid and prev_id and sid != prev_id:
        switches.append((prev_id, sid, r))
    if sid:
        prev_id=sid

print("samples =", len(rows))
print("zoom/profile mismatches =", len(mismatch))
print("big commands on zoom>=7 =", len(big_cmd))
print("selected target switches =", len(switches))

if mismatch[:5]:
    print("\n-- mismatches sample --")
    for r in mismatch[:5]:
        print(r)

if big_cmd[:10]:
    print("\n-- big commands zoom>=7 sample --")
    for r in big_cmd[:10]:
        print(
            "ts",r["ts"],
            "zoom",r["zoom_sample"],
            "profile",r["active_profile"],
            "err",r["err_x"],r["err_y"],
            "cmd",r["cmd_pan"],r["cmd_tilt"],
            "eff",r["eff_pan"],r["eff_tilt"],
            "track",r["sel_id"],
            "motion",r["motion"],
            "switch",r["switch_reason"],
        )

if switches[:10]:
    print("\n-- switches sample --")
    for a,b,r in switches[:10]:
        print("ts",r["ts"], "from",a,"to",b,"reason",r["switch_reason"],"motion",r["motion"])
PY

echo
echo "DONE $OUT"
