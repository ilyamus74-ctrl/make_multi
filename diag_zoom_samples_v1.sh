#!/usr/bin/env bash
set -u

cd /root/new_yolo8 || exit 1

TS="$(date +%Y%m%d_%H%M%S)"
DIAG="/root/new_yolo8/logs/zoom_samples_diag_$TS"
mkdir -p "$DIAG"

echo "DIAG=$DIAG"

api_json() {
  local name="$1"
  local url="$2"
  echo
  echo "===== GET $url =====" | tee -a "$DIAG/summary.txt"
  curl -sS "$url" | tee "$DIAG/$name.json" | python3 -m json.tool 2>/dev/null | tee -a "$DIAG/summary.txt" || true
}

post_json() {
  local name="$1"
  local url="$2"
  local body="$3"
  echo
  echo "===== POST $url $body =====" | tee -a "$DIAG/summary.txt"
  curl -sS -X POST "$url" \
    -H 'Content-Type: application/json' \
    -d "$body" \
    | tee "$DIAG/$name.json" \
    | python3 -m json.tool 2>/dev/null | tee -a "$DIAG/summary.txt" || true
}

echo
echo "===================================================================================================="
echo "0. STOP PTZ / FREEZE SEARCH"
echo "===================================================================================================="

post_json "00_autopilot_stop" "http://127.0.0.1:8090/api/autopilot/stop" '{}'
sleep 1

api_json "01_autopilot_state" "http://127.0.0.1:8090/api/autopilot/state"
api_json "02_zoom_state" "http://127.0.0.1:8080/api/zoom/state"

echo
echo "===================================================================================================="
echo "1. WALK ZOOM SAMPLES 0..9"
echo "===================================================================================================="

for idx in 0 1 2 3 4 5 6 7 8 9; do
  echo
  echo "----------------------------------------------------------------------------------------------------"
  echo "SAMPLE $idx"
  echo "----------------------------------------------------------------------------------------------------"

  api_json "zoom_before_$idx" "http://127.0.0.1:8080/api/zoom/state"

  post_json "go_to_sample_$idx" "http://127.0.0.1:8080/api/zoom/go_to_sample" "{\"profile_idx\":$idx,\"mode\":\"sample\",\"source\":\"diag_zoom_samples_v1\"}"

  sleep 2

  api_json "zoom_after_$idx" "http://127.0.0.1:8080/api/zoom/state"

  post_json "speed_profile_$idx" "http://127.0.0.1:8090/api/autopilot/speed_profile/apply_nearest" "{\"profile_idx\":$idx}"

  api_json "autopilot_after_speed_$idx" "http://127.0.0.1:8090/api/autopilot/state"

  # frame snapshot for visual compare
  curl -sS "http://127.0.0.1:8080/frame.jpg?ts=$(date +%s%N)" -o "$DIAG/frame_sample_$idx.jpg" || true

  sleep 1
done

echo
echo "===================================================================================================="
echo "2. WALK BACK 9..0"
echo "===================================================================================================="

for idx in 9 8 7 6 5 4 3 2 1 0; do
  echo
  echo "----------------------------------------------------------------------------------------------------"
  echo "SAMPLE_BACK $idx"
  echo "----------------------------------------------------------------------------------------------------"

  post_json "back_go_to_sample_$idx" "http://127.0.0.1:8080/api/zoom/go_to_sample" "{\"profile_idx\":$idx,\"mode\":\"sample\",\"source\":\"diag_zoom_samples_v1_back\"}"

  sleep 2

  api_json "back_zoom_after_$idx" "http://127.0.0.1:8080/api/zoom/state"

  curl -sS "http://127.0.0.1:8080/frame.jpg?ts=$(date +%s%N)" -o "$DIAG/frame_back_sample_$idx.jpg" || true

  sleep 1
done

echo
echo "===================================================================================================="
echo "3. COMPACT SUMMARY"
echo "===================================================================================================="

python3 - "$DIAG" <<'PY' | tee "$DIAG/compact_zoom_summary.txt"
import json, sys, pathlib

diag = pathlib.Path(sys.argv[1])

print("diag =", diag)
print()

def load(name):
    try:
        return json.loads((diag / name).read_text())
    except Exception as e:
        return {"__error__": str(e)}

print("FORWARD")
for idx in range(10):
    before = load(f"zoom_before_{idx}.json")
    resp = load(f"go_to_sample_{idx}.json")
    after = load(f"zoom_after_{idx}.json")
    ap = load(f"autopilot_after_speed_{idx}.json")

    print(
        f"idx={idx}",
        "before_sample=", before.get("zoom_sample_idx"),
        "before_ratio=", before.get("zoom_ratio"),
        "resp_ok=", resp.get("ok"),
        "resp=", {k: resp.get(k) for k in ["ok", "profile_idx", "zoom_sample_idx", "zoom_ratio", "error"]},
        "after_sample=", after.get("zoom_sample_idx"),
        "after_ratio=", after.get("zoom_ratio"),
        "ap_sample=", ap.get("active_zoom_sample_idx"),
        "ap_ratio=", ap.get("zoom_ratio"),
        "speed_source=", ap.get("speed_profile_source"),
    )

print()
print("BACK")
for idx in range(9, -1, -1):
    resp = load(f"back_go_to_sample_{idx}.json")
    after = load(f"back_zoom_after_{idx}.json")

    print(
        f"idx={idx}",
        "resp_ok=", resp.get("ok"),
        "resp=", {k: resp.get(k) for k in ["ok", "profile_idx", "zoom_sample_idx", "zoom_ratio", "error"]},
        "after_sample=", after.get("zoom_sample_idx"),
        "after_ratio=", after.get("zoom_ratio"),
    )
PY

echo
echo "===================================================================================================="
echo "4. ARCHIVE"
echo "===================================================================================================="

tar -czf "$DIAG.tar.gz" -C "$(dirname "$DIAG")" "$(basename "$DIAG")"
ls -lah "$DIAG" "$DIAG.tar.gz"

echo
echo "DONE"
echo "Send:"
echo "cat $DIAG/compact_zoom_summary.txt"
echo "archive: $DIAG.tar.gz"
