#!/usr/bin/env bash
set -u

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
TS="$(date '+%Y%m%d_%H%M%S')"
OUT_DIR="$ROOT_DIR/debug_ptz_$TS"
mkdir -p "$OUT_DIR"

MJPEG_URL="${MJPEG_URL:-http://127.0.0.1:8080}"
AUTOPILOT_URL="${AUTOPILOT_URL:-http://127.0.0.1:8090}"
WS_PORT="${WS_PORT:-8765}"

LOG="$OUT_DIR/summary.log"

run() {
  local name="$1"
  shift
  {
    echo
    echo "===== $name ====="
    echo "\$ $*"
    "$@"
    echo "exit_code=$?"
  } >> "$LOG" 2>&1
}

curl_json() {
  local name="$1"
  local url="$2"
  local out="$OUT_DIR/$name.json"

  echo "GET $url" >> "$LOG"
  curl -sS --max-time 2 "$url" > "$out.raw" 2>> "$LOG"

  if python3 -m json.tool "$out.raw" > "$out" 2>> "$LOG"; then
    rm -f "$out.raw"
  else
    mv "$out.raw" "$out"
  fi
}

echo "PTZ debug collection: $TS" | tee "$LOG"
echo "ROOT_DIR=$ROOT_DIR" >> "$LOG"
echo "MJPEG_URL=$MJPEG_URL" >> "$LOG"
echo "AUTOPILOT_URL=$AUTOPILOT_URL" >> "$LOG"

# Basic system info
run "date" date
run "uname" uname -a
run "uptime" uptime
run "df" df -h
run "free" free -h

# Process / ports
run "processes" bash -lc "ps aux | grep -E 'mjpeg_rknn_http|ws_uart_bridge|ptz_autopilot|launcher.sh' | grep -v grep || true"
run "ports" bash -lc "ss -ltnp | grep -E ':8080|:8090|:$WS_PORT' || true"

# Binary presence
run "binaries" bash -lc "ls -lah .build/mjpeg_rknn_http .build/ptz_autopilot web/ws_uart_bridge 2>/dev/null || true"
run "git status" bash -lc "git rev-parse --short HEAD 2>/dev/null; git status --short 2>/dev/null || true"

# Current configs/files
{
  echo
  echo "===== config files ====="
  ls -lah launcher.cfg detection_roi_config.json detector_state.cfg ui_settings.json 2>/dev/null || true
  echo
  echo "----- launcher.cfg -----"
  cat launcher.cfg 2>/dev/null || true
  echo
  echo "----- detection_roi_config.json -----"
  cat detection_roi_config.json 2>/dev/null || true
} >> "$LOG" 2>&1

# API snapshots
curl_json "api_detection_roi_config" "$MJPEG_URL/api/detection/roi_config"
curl_json "api_detections" "$MJPEG_URL/api/detections"
curl_json "api_tracker_state" "$MJPEG_URL/api/tracker/state"
curl_json "api_tracking_confidence" "$MJPEG_URL/api/tracking/confidence"
curl_json "api_command_limits" "$MJPEG_URL/api/command_limits"
curl_json "api_autopilot_state" "$AUTOPILOT_URL/api/autopilot/state"

# Repeated samples
SAMPLES="${SAMPLES:-20}"
SLEEP_SEC="${SLEEP_SEC:-0.25}"

echo "Collecting $SAMPLES state samples..." | tee -a "$LOG"

for i in $(seq 1 "$SAMPLES"); do
  {
    echo
    echo "===== sample $i / $SAMPLES $(date '+%H:%M:%S.%3N') ====="
    echo "--- tracker ---"
    curl -sS --max-time 1 "$MJPEG_URL/api/tracker/state" || true
    echo
    echo "--- detections ---"
    curl -sS --max-time 1 "$MJPEG_URL/api/detections" || true
    echo
    echo "--- autopilot ---"
    curl -sS --max-time 1 "$AUTOPILOT_URL/api/autopilot/state" || true
    echo
  } >> "$OUT_DIR/samples.log" 2>&1
  sleep "$SLEEP_SEC"
done

# Logs
mkdir -p "$OUT_DIR/runtime_logs"

for f in \
  "$ROOT_DIR/.logs/mjpeg.log" \
  "$ROOT_DIR/.logs/bridge.log" \
  "$ROOT_DIR/.logs/autopilot.log"
do
  if [[ -f "$f" ]]; then
    cp "$f" "$OUT_DIR/runtime_logs/$(basename "$f")"
    tail -n 300 "$f" > "$OUT_DIR/runtime_logs/$(basename "$f").tail300"
  fi
done

# Optional safe stop command capture
# This sends only STOP to autopilot if it is running.
if [[ "${SEND_AUTOPILOT_STOP:-0}" == "1" ]]; then
  {
    echo
    echo "===== optional autopilot stop ====="
    curl -sS --max-time 2 -X POST "$AUTOPILOT_URL/api/autopilot/stop" || true
    echo
  } >> "$LOG" 2>&1
fi

# Pack
ARCHIVE="$ROOT_DIR/debug_ptz_$TS.tar.gz"
tar -czf "$ARCHIVE" -C "$ROOT_DIR" "$(basename "$OUT_DIR")"

echo
echo "Done."
echo "Directory: $OUT_DIR"
echo "Archive:   $ARCHIVE"
echo
echo "Send me this:"
echo "  $ARCHIVE"
