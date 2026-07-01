#!/usr/bin/env bash
set -Eeuo pipefail

# launcher.sh — single entrypoint to run MJPEG server + WS<->UART bridge together.
#
# Designed as "Option B" (quick start) with migration path to systemd "Option A":
# - explicit env vars
# - foreground process model
# - proper signal handling
# - independent log files

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
LOG_DIR="${LOG_DIR:-$ROOT_DIR/.logs}"
mkdir -p "$LOG_DIR"

# Optional runtime config (KEY=VALUE), useful to avoid long ENV prefixes.
# Can be overridden with LAUNCHER_CFG=/path/to/file.
LAUNCHER_CFG="${LAUNCHER_CFG:-$ROOT_DIR/launcher.cfg}"
if [[ -f "$LAUNCHER_CFG" ]]; then
  # shellcheck disable=SC1090
  source "$LAUNCHER_CFG"
fi

MJPEG_BIN="${MJPEG_BIN:-$ROOT_DIR/.build/mjpeg_rknn_http}"
BRIDGE_BIN="${BRIDGE_BIN:-$ROOT_DIR/web/ws_uart_bridge}"

# MJPEG defaults
VIDEO_DEV="${VIDEO_DEV:-/dev/video0}"
MJPEG_PORT="${MJPEG_PORT:-8080}"
WIDTH="${WIDTH:-1920}"
HEIGHT="${HEIGHT:-1080}"
FPS="${FPS:-25}"
JPEG_QUALITY="${JPEG_QUALITY:-20}"
MJPEG_NICE="${MJPEG_NICE:-5}"
MJPEG_CPUSET="${MJPEG_CPUSET:-4-7}"
MJPEG_RENICE_WATCHDOG="${MJPEG_RENICE_WATCHDOG:-1}"
MJPEG_RENICE_INTERVAL="${MJPEG_RENICE_INTERVAL:-2}"
DEINTERLACE="${DEINTERLACE:-0}"            # 1 => --deinterlace, 0 => disabled
LABELS="${LABELS:-$ROOT_DIR/models/coco_80_labels_list.txt}"
CMD_MAX_PAN="${CMD_MAX_PAN:-20}"
CMD_MAX_TILT="${CMD_MAX_TILT:-20}"
CMD_MAX_ZOOM="${CMD_MAX_ZOOM:-12}"
MAX_DETECTIONS="${MAX_DETECTIONS:-10}"
MAX_RAW_CANDIDATES="${MAX_RAW_CANDIDATES:-50}"
DETECT_EVERY_N_FRAMES="${DETECT_EVERY_N_FRAMES:-1}"
MJPEG_RESTART="${MJPEG_RESTART:-1}"        # 1 => auto-restart MJPEG on exit
MJPEG_RESTART_DELAY="${MJPEG_RESTART_DELAY:-2}"
MJPEG_MAX_RESTARTS="${MJPEG_MAX_RESTARTS:-0}"  # 0 => unlimited
MODEL_DIR="${MODEL_DIR:-$ROOT_DIR/models}"

# WS bridge defaults
UART_DEV="${UART_DEV:-/dev/ttyUSB0}"
##UART_DEV="${UART_DEV:-/dev/ttyS4}"
UART_BAUD="${UART_BAUD:-115200}"
WS_PORT="${WS_PORT:-8765}"
ZOOM_CALIB_ENABLE="${ZOOM_CALIB_ENABLE:-1}"
ZOOM_CALIB_UART_DEV="${ZOOM_CALIB_UART_DEV:-$UART_DEV}"
ZOOM_CALIB_UART_BAUD="${ZOOM_CALIB_UART_BAUD:-$UART_BAUD}"

MJPEG_LOG="${MJPEG_LOG:-$LOG_DIR/mjpeg.log}"
BRIDGE_LOG="${BRIDGE_LOG:-$LOG_DIR/bridge.log}"

AUTOPILOT_ENABLE="${AUTOPILOT_ENABLE:-0}"
AUTOPILOT_REQUIRED="${AUTOPILOT_REQUIRED:-0}"
AUTOPILOT_BIN="${AUTOPILOT_BIN:-$ROOT_DIR/.build/ptz_autopilot}"
KILL_STALE_AUTOPILOT="${KILL_STALE_AUTOPILOT:-1}"
AUTOPILOT_LOG="${AUTOPILOT_LOG:-$LOG_DIR/autopilot.log}"
AUTOPILOT_PORT="${AUTOPILOT_PORT:-8090}"
AUTOPILOT_HZ="${AUTOPILOT_HZ:-20}"
AUTOPILOT_KP="${AUTOPILOT_KP:-20}"
AUTOPILOT_KI="${AUTOPILOT_KI:-0}"
AUTOPILOT_KD="${AUTOPILOT_KD:-3}"
AUTOPILOT_DEADZONE="${AUTOPILOT_DEADZONE:-0.05}"
AUTOPILOT_MAX_ACCEL="${AUTOPILOT_MAX_ACCEL:-4}"
AUTOPILOT_INVERT_TILT="${AUTOPILOT_INVERT_TILT:-1}"

AUTOPILOT_INVERT_PAN="${AUTOPILOT_INVERT_PAN:-0}"
AUTOPILOT_TARGET_X="${AUTOPILOT_TARGET_X:-0.5}"
AUTOPILOT_TARGET_Y="${AUTOPILOT_TARGET_Y:-0.5}"
AUTOPILOT_MIN_PAN="${AUTOPILOT_MIN_PAN:-4}"
AUTOPILOT_MIN_TILT="${AUTOPILOT_MIN_TILT:-3}"

MJPEG_PID=""
BRIDGE_PID=""
AUTOPILOT_PID=""
MJPEG_RENICE_WATCHDOG_PID=""

log() { printf '[%s] %s\n' "$(date '+%Y-%m-%d %H:%M:%S')" "$*"; }

print_last_log_lines() {
  local file="$1"
  local title="$2"
  if [[ -s "$file" ]]; then
    log "Last lines from $title ($file):"
    tail -n 30 "$file" | sed 's/^/  | /'
  else
    log "No log output yet in $file"
  fi
}

check_bin() {
  local path="$1"
  if [[ ! -x "$path" ]]; then
    log "ERROR: executable not found: $path"
    exit 1
  fi
}


renice_mjpeg_threads_once() {
  local pid="${MJPEG_PID:-}"
  [[ -n "$pid" ]] || return 0
  [[ -d "/proc/$pid/task" ]] || return 0

  for t in /proc/"$pid"/task/*; do
    [[ -e "$t" ]] || continue
    local tid="${t##*/}"
    renice -n "$MJPEG_NICE" -p "$tid" >/dev/null 2>&1 || true
  done
}

start_mjpeg_renice_watchdog() {
  if [[ "${MJPEG_RENICE_WATCHDOG:-1}" != "1" ]]; then
    return 0
  fi

  (
    while true; do
      renice_mjpeg_threads_once
      sleep "${MJPEG_RENICE_INTERVAL:-2}"
    done
  ) &
  MJPEG_RENICE_WATCHDOG_PID=$!
  log "MJPEG renice watchdog pid=$MJPEG_RENICE_WATCHDOG_PID nice=$MJPEG_NICE interval=${MJPEG_RENICE_INTERVAL:-2}s"
}


cleanup() {
  local rc=$?
  trap - EXIT INT TERM

  log "Stopping child processes..."

  if [[ -n "${MJPEG_RENICE_WATCHDOG_PID:-}" ]] && kill -0 "$MJPEG_RENICE_WATCHDOG_PID" 2>/dev/null; then
    kill "$MJPEG_RENICE_WATCHDOG_PID" 2>/dev/null || true
  fi

  if [[ -n "${BRIDGE_PID:-}" ]] && kill -0 "$BRIDGE_PID" 2>/dev/null; then
    kill "$BRIDGE_PID" 2>/dev/null || true
  fi
  if [[ -n "${MJPEG_PID:-}" ]] && kill -0 "$MJPEG_PID" 2>/dev/null; then
    kill "$MJPEG_PID" 2>/dev/null || true
  fi
  if [[ -n "${AUTOPILOT_PID:-}" ]] && kill -0 "$AUTOPILOT_PID" 2>/dev/null; then
    kill "$AUTOPILOT_PID" 2>/dev/null || true
  fi

  wait "$BRIDGE_PID" 2>/dev/null || true
  wait "$MJPEG_PID" 2>/dev/null || true
  wait "$AUTOPILOT_PID" 2>/dev/null || true

  log "Exited with code $rc"
  exit "$rc"
}

trap cleanup EXIT INT TERM

check_bin "$MJPEG_BIN"
check_bin "$BRIDGE_BIN"
if [[ "$AUTOPILOT_ENABLE" == "1" ]]; then check_bin "$AUTOPILOT_BIN"; fi

MJPEG_ARGS=(
  --dev "$VIDEO_DEV"
  --port "$MJPEG_PORT"
  --width "$WIDTH"
  --height "$HEIGHT"
  --fps "$FPS"
  --jpeg "$JPEG_QUALITY"
  --labels "$LABELS"
  --model-dir "$MODEL_DIR"
  --cmd-max-pan "$CMD_MAX_PAN"
  --cmd-max-tilt "$CMD_MAX_TILT"
  --cmd-max-zoom "$CMD_MAX_ZOOM"
  --max-detections "$MAX_DETECTIONS"
  --max-raw-candidates "$MAX_RAW_CANDIDATES"
  --detect-every-n-frames "$DETECT_EVERY_N_FRAMES"
)
if [[ "$ZOOM_CALIB_ENABLE" == "1" ]]; then
  MJPEG_ARGS+=(--zoom-calib-enable --zoom-calib-uart "$ZOOM_CALIB_UART_DEV" --zoom-calib-baud "$ZOOM_CALIB_UART_BAUD")
else
  MJPEG_ARGS+=(--no-zoom-calib)
fi
if [[ "$DEINTERLACE" == "1" ]]; then
  MJPEG_ARGS+=(--deinterlace)
fi

start_mjpeg() {
  log "Starting MJPEG server: $MJPEG_BIN"
  nice -n "$MJPEG_NICE" taskset -c "$MJPEG_CPUSET" "$MJPEG_BIN" "${MJPEG_ARGS[@]}" >>"$MJPEG_LOG" 2>&1 &
  MJPEG_PID=$!
  log "MJPEG pid=$MJPEG_PID (log: $MJPEG_LOG)"
  sleep 0.2
  renice_mjpeg_threads_once
  log "MJPEG nice set to $MJPEG_NICE, cpuset=$MJPEG_CPUSET for pid=$MJPEG_PID"
  start_mjpeg_renice_watchdog
  sleep 0.3
  if ! kill -0 "$MJPEG_PID" 2>/dev/null; then
    log "ERROR: MJPEG failed to start (see $MJPEG_LOG)"
    print_last_log_lines "$MJPEG_LOG" "MJPEG"
    return 1
  fi
  return 0
}

start_autopilot() {
  if [[ "${KILL_STALE_AUTOPILOT:-1}" == "1" ]]; then
    pkill -9 -f "$ROOT_DIR/.build/ptz_autopilot" 2>/dev/null || true
  fi
  log "Starting autopilot: $AUTOPILOT_BIN"
  "$AUTOPILOT_BIN" \
    --mjpeg-url "http://127.0.0.1:$MJPEG_PORT" \
    --bridge-host "127.0.0.1" \
    --bridge-port "$WS_PORT" \
    --control-port "$AUTOPILOT_PORT" \
    --width "$WIDTH" \
    --height "$HEIGHT" \
    --hz "$AUTOPILOT_HZ" \
    --kp "$AUTOPILOT_KP" \
    --ki "$AUTOPILOT_KI" \
    --kd "$AUTOPILOT_KD" \
    --deadzone "$AUTOPILOT_DEADZONE" \
    --max-pan "$CMD_MAX_PAN" \
    --max-tilt "$CMD_MAX_TILT" \
    --max-accel "$AUTOPILOT_MAX_ACCEL" \
    --invert-pan "$AUTOPILOT_INVERT_PAN" \
    --invert-tilt "$AUTOPILOT_INVERT_TILT" \
    --target-x "$AUTOPILOT_TARGET_X" \
    --target-y "$AUTOPILOT_TARGET_Y" \
    --min-pan "$AUTOPILOT_MIN_PAN" \
    --min-tilt "$AUTOPILOT_MIN_TILT" \
  --zoom-scale-enable "${AUTOPILOT_ZOOM_SCALE_ENABLE:-1}" \
  --zoom-scale-min "${AUTOPILOT_ZOOM_SCALE_MIN:-0.12}" \
  --zoom-scale-max "${AUTOPILOT_ZOOM_SCALE_MAX:-1.0}" \
  --zoom-scale-smoothing "${AUTOPILOT_ZOOM_SCALE_SMOOTHING:-0.25}" \
    --enable 0 \
    >>"$AUTOPILOT_LOG" 2>&1 &
  AUTOPILOT_PID=$!
  log "Autopilot pid=$AUTOPILOT_PID (log: $AUTOPILOT_LOG)"
}

start_bridge() {
  log "Starting bridge: $BRIDGE_BIN $UART_DEV $UART_BAUD $WS_PORT"
  "$BRIDGE_BIN" "$UART_DEV" "$UART_BAUD" "$WS_PORT" >>"$BRIDGE_LOG" 2>&1 &
  BRIDGE_PID=$!
  log "Bridge pid=$BRIDGE_PID (log: $BRIDGE_LOG)"

  sleep 0.3

  if [[ -n "${AUTOPILOT_PID:-}" ]] && ! kill -0 "$AUTOPILOT_PID" 2>/dev/null; then
    log "ERROR: autopilot process exited"
    print_last_log_lines "$AUTOPILOT_LOG" "autopilot"
    if [[ "$AUTOPILOT_REQUIRED" == "1" ]]; then
      exit 1
    fi
    AUTOPILOT_PID=""
  fi

  if ! kill -0 "$BRIDGE_PID" 2>/dev/null; then
    log "ERROR: bridge failed to start (see $BRIDGE_LOG)"
    print_last_log_lines "$BRIDGE_LOG" "bridge"
    return 1
  fi
  return 0
}

if ! start_mjpeg; then
  exit 1
fi
if ! start_bridge; then
  exit 1
fi
if [[ "$AUTOPILOT_ENABLE" == "1" ]]; then
  start_autopilot
fi

log "Services are up"
log "  MJPEG:  http://0.0.0.0:${MJPEG_PORT}/"
log "  Bridge: ws://0.0.0.0:${WS_PORT}/ws"
log "  Command limits: pan=±${CMD_MAX_PAN}, tilt=±${CMD_MAX_TILT}, zoom=±${CMD_MAX_ZOOM}"
log "  Auto-restart MJPEG: ${MJPEG_RESTART} (delay=${MJPEG_RESTART_DELAY}s, max=${MJPEG_MAX_RESTARTS})"
log "Press Ctrl+C to stop both"

mjpeg_restarts=0

# Watch both children. If bridge dies => exit. If MJPEG dies => optional auto-restart.
while true; do
  if ! kill -0 "$MJPEG_PID" 2>/dev/null; then
    log "ERROR: MJPEG process exited unexpectedly"
    print_last_log_lines "$MJPEG_LOG" "MJPEG"

    if [[ "$MJPEG_RESTART" != "1" ]]; then
      exit 1
    fi

    mjpeg_restarts=$((mjpeg_restarts + 1))
    if [[ "$MJPEG_MAX_RESTARTS" != "0" ]] && (( mjpeg_restarts > MJPEG_MAX_RESTARTS )); then
      log "ERROR: MJPEG restart limit reached (${MJPEG_MAX_RESTARTS})"
      exit 1
    fi

    log "Restarting MJPEG in ${MJPEG_RESTART_DELAY}s (attempt ${mjpeg_restarts})..."
    sleep "$MJPEG_RESTART_DELAY"
    if ! start_mjpeg; then
      log "ERROR: MJPEG restart failed"
      sleep "$MJPEG_RESTART_DELAY"
    fi
    continue
  fi

  if [[ -n "${AUTOPILOT_PID:-}" ]] && ! kill -0 "$AUTOPILOT_PID" 2>/dev/null; then
    log "ERROR: autopilot process exited"
    print_last_log_lines "$AUTOPILOT_LOG" "autopilot"
    if [[ "$AUTOPILOT_REQUIRED" == "1" ]]; then
      exit 1
    fi
    AUTOPILOT_PID=""
  fi

  if ! kill -0 "$BRIDGE_PID" 2>/dev/null; then
    log "ERROR: bridge process exited unexpectedly"
    print_last_log_lines "$BRIDGE_LOG" "bridge"
    exit 1
  fi
  sleep 1
 done
