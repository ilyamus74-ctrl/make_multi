#!/usr/bin/env bash
set -u

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"

cd "$ROOT" || exit 1

cd /root/new_yolo8 || exit 1

TS="$(date +%Y%m%d_%H%M%S)"
DIAG="$ROOT/logs/zoom_transaction_diag_$TS"
mkdir -p "$DIAG"

echo "DIAG=$DIAG"

get_zoom_state() {
  curl -sS http://127.0.0.1:8080/api/zoom/state
}

wait_zoom_idle() {
  local label="$1"
  local timeout="${2:-20}"
  local start
  start="$(date +%s)"

  echo "wait_zoom_idle: $label timeout=$timeout" | tee -a "$DIAG/summary.txt"

  while true; do
    local now age
    now="$(date +%s)"
    age=$((now - start))

    get_zoom_state > "$DIAG/zoom_wait_${label}_last.json"

    python3 - "$DIAG/zoom_wait_${label}_last.json" <<'PY'
import json, sys
s=json.load(open(sys.argv[1]))
print("sample=", s.get("zoom_sample_idx"), "ratio=", s.get("zoom_ratio"), "busy=", s.get("zoom_move_busy"), "source=", s.get("zoom_source"))
raise SystemExit(0 if s.get("zoom_move_busy") is False else 1)
PY
    rc=$?

    if [ "$rc" = "0" ]; then
      return 0
    fi

    if [ "$age" -ge "$timeout" ]; then
      echo "wait_zoom_idle timeout: $label" | tee -a "$DIAG/summary.txt"
      return 1
    fi

    sleep 0.4
  done
}

move_sample_tx() {
  local idx="$1"

  echo
  echo "====================================================================================================" | tee -a "$DIAG/summary.txt"
  echo "MOVE TX sample=$idx" | tee -a "$DIAG/summary.txt"
  echo "====================================================================================================" | tee -a "$DIAG/summary.txt"

  wait_zoom_idle "before_$idx" 25 || true

  get_zoom_state | tee "$DIAG/before_$idx.json" | python3 -m json.tool | tee -a "$DIAG/summary.txt" || true

  echo "POST go_to_sample $idx" | tee -a "$DIAG/summary.txt"
  curl -sS -X POST http://127.0.0.1:8080/api/zoom/go_to_sample \
    -H 'Content-Type: application/json' \
    -d "{\"profile_idx\":$idx,\"mode\":\"sample\",\"source\":\"diag_zoom_transaction_v1\"}" \
    | tee "$DIAG/post_$idx.json" | python3 -m json.tool | tee -a "$DIAG/summary.txt" || true

  echo "poll until target or idle..." | tee -a "$DIAG/summary.txt"

  for n in $(seq 1 40); do
    get_zoom_state > "$DIAG/poll_${idx}_${n}.json"

    python3 - "$DIAG/poll_${idx}_${n}.json" "$idx" "$n" <<'PY' | tee -a "$DIAG/summary.txt"
import json, sys
s=json.load(open(sys.argv[1]))
target=int(sys.argv[2])
n=int(sys.argv[3])
sample=s.get("zoom_sample_idx")
busy=s.get("zoom_move_busy")
ratio=s.get("zoom_ratio")
ok = (sample == target and busy is False)
print(f"poll={n:02d} target={target} sample={sample} ratio={ratio} busy={busy} ok={ok}")
raise SystemExit(0 if ok else 1)
PY
    rc=$?

    if [ "$rc" = "0" ]; then
      break
    fi

    sleep 0.4
  done

  get_zoom_state | tee "$DIAG/after_$idx.json" | python3 -m json.tool | tee -a "$DIAG/summary.txt" || true

  echo "apply speed profile $idx" | tee -a "$DIAG/summary.txt"
  curl -sS -X POST http://127.0.0.1:8090/api/autopilot/speed_profile/apply_nearest \
    -H 'Content-Type: application/json' \
    -d "{\"profile_idx\":$idx}" \
    | tee "$DIAG/speed_$idx.json" | python3 -m json.tool | tee -a "$DIAG/summary.txt" || true
}

echo
echo "===================================================================================================="
echo "0. FREEZE RUNTIME"
echo "===================================================================================================="

python3 - <<'PY'
import json, urllib.request

def get_json(url):
    with urllib.request.urlopen(url, timeout=3) as r:
        raw = r.read().decode("utf-8", errors="replace")
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
print("runtime frozen: ptzArmed=false controlMode=manual autopilot stopped")
PY

sleep 2

pgrep -af 'apply_ptz_object_preset.py --watch' | tee "$DIAG/watch_processes_before.txt" || true

echo
echo "===================================================================================================="
echo "1. ZOOM TRANSACTIONS"
echo "===================================================================================================="

for idx in 0 3 6 9 6 3 0; do
  move_sample_tx "$idx"
  sleep 1
done

echo
echo "===================================================================================================="
echo "2. COMPACT SUMMARY"
echo "===================================================================================================="

python3 - "$DIAG" <<'PY' | tee "$DIAG/compact_zoom_transaction_summary.txt"
import json, pathlib, sys

diag=pathlib.Path(sys.argv[1])
seq=[0,3,6,9,6,3,0]

print("diag =", diag)
print()

for idx in seq:
    def load(name):
        try:
            return json.loads((diag/name).read_text())
        except Exception as e:
            return {"__error__": str(e)}

    before=load(f"before_{idx}.json")
    post=load(f"post_{idx}.json")
    after=load(f"after_{idx}.json")
    speed=load(f"speed_{idx}.json")

    print(
        f"target={idx}",
        "before=", before.get("zoom_sample_idx"),
        "before_busy=", before.get("zoom_move_busy"),
        "post_ok=", post.get("ok"),
        "post_error=", post.get("error"),
        "after=", after.get("zoom_sample_idx"),
        "after_ratio=", after.get("zoom_ratio"),
        "after_busy=", after.get("zoom_move_busy"),
        "speed_ok=", speed.get("ok"),
        "speed_source=", speed.get("speed_profile_source") or speed.get("source"),
    )
PY

tar -czf "$DIAG.tar.gz" -C "$(dirname "$DIAG")" "$(basename "$DIAG")"

echo
echo "DONE"
echo "Send:"
echo "cat $DIAG/compact_zoom_transaction_summary.txt"
echo "$DIAG.tar.gz"