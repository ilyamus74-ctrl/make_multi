#!/usr/bin/env bash
set -u

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"

cd "$ROOT" || exit 1

cd /root/new_yolo8 || exit 1

TS="$(date +%Y%m%d_%H%M%S)"
DIAG="$ROOT/logs/speed_profile_apply_diag_$TS"
mkdir -p "$DIAG"

echo "DIAG=$DIAG"

echo
echo "===================================================================================================="
echo "1. CURRENT SPEED PROFILE TABLE"
echo "===================================================================================================="

curl -sS http://127.0.0.1:8090/api/autopilot/speed_profile \
  | tee "$DIAG/speed_profile_table.json" \
  | python3 -m json.tool | tee "$DIAG/speed_profile_table.pretty.txt" || true

echo
echo "===================================================================================================="
echo "2. CURRENT AUTOPILOT STATE"
echo "===================================================================================================="

curl -sS http://127.0.0.1:8090/api/autopilot/state \
  | tee "$DIAG/autopilot_before.json" \
  | python3 -m json.tool | tee "$DIAG/autopilot_before.pretty.txt" || true

echo
echo "===================================================================================================="
echo "3. APPLY_NEAREST BY profile_idx ONLY — 5 ROUNDS"
echo "===================================================================================================="

for round in 1 2 3 4 5; do
  for idx in 0 1 2 3 4 5 6 7 8 9; do
    echo
    echo "===== round=$round profile_idx=$idx =====" | tee -a "$DIAG/raw_apply.log"

    curl -sS -X POST http://127.0.0.1:8090/api/autopilot/speed_profile/apply_nearest \
      -H 'Content-Type: application/json' \
      -d "{\"profile_idx\":$idx}" \
      | tee "$DIAG/apply_r${round}_idx${idx}.json" \
      | python3 -m json.tool | tee -a "$DIAG/raw_apply.log" || true

    curl -sS http://127.0.0.1:8090/api/autopilot/state > "$DIAG/state_r${round}_idx${idx}.json"

    sleep 0.15
  done
done

echo
echo "===================================================================================================="
echo "4. COMPACT SUMMARY"
echo "===================================================================================================="

python3 - "$DIAG" <<'PY' | tee "$DIAG/compact_speed_profile_apply_summary.txt"
import json, pathlib, sys

d = pathlib.Path(sys.argv[1])

print("diag =", d)
print()

fails = []

for round_no in range(1, 6):
    for idx in range(10):
        p = d / f"apply_r{round_no}_idx{idx}.json"
        s = d / f"state_r{round_no}_idx{idx}.json"

        try:
            res = json.loads(p.read_text())
        except Exception as e:
            print(f"round={round_no} req={idx} APPLY_READ_ERROR {e}")
            fails.append((round_no, idx, "read_error"))
            continue

        try:
            st = json.loads(s.read_text())
        except Exception:
            st = {}

        point = res.get("point") or res.get("applied_point") or res.get("config") or {}
        got = point.get("profile_idx")
        src = res.get("source") or point.get("source")
        ok = res.get("ok")
        applied = res.get("applied")

        ap_prof = st.get("active_profile_idx")
        ap_zoom = st.get("active_zoom_sample_idx")
        ap_src = st.get("speed_profile_source")

        status = "OK" if got == idx and ap_prof == idx else "BAD"

        print(
            f"{status}",
            f"round={round_no}",
            f"req={idx}",
            f"res_ok={ok}",
            f"applied={applied}",
            f"point_idx={got}",
            f"source={src}",
            f"state_profile={ap_prof}",
            f"state_zoom_sample={ap_zoom}",
            f"state_source={ap_src}",
        )

        if status != "OK":
            fails.append((round_no, idx, got, ap_prof, src))

print()
print("FAIL_COUNT =", len(fails))
for f in fails[:50]:
    print("FAIL_ITEM =", f)
PY

tar -czf "$DIAG.tar.gz" -C "$(dirname "$DIAG")" "$(basename "$DIAG")"

echo
echo "DONE"
echo "Send:"
echo "cat $DIAG/compact_speed_profile_apply_summary.txt"
echo "$DIAG.tar.gz"