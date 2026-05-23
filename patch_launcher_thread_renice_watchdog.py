from pathlib import Path
import shutil

p = Path("launcher.sh")
s = p.read_text()

backup = p.with_suffix(".sh.bak_thread_renice_watchdog")
if not backup.exists():
    shutil.copy2(p, backup)
    print(f"backup: {backup}")

# 1. Defaults
s = s.replace(
    'MJPEG_CPUSET="${MJPEG_CPUSET:-4-7}"\nDEINTERLACE=',
    'MJPEG_CPUSET="${MJPEG_CPUSET:-4-7}"\nMJPEG_RENICE_WATCHDOG="${MJPEG_RENICE_WATCHDOG:-1}"\nMJPEG_RENICE_INTERVAL="${MJPEG_RENICE_INTERVAL:-2}"\nDEINTERLACE='
)

# 2. PID variable
s = s.replace(
    'MJPEG_PID=""\nBRIDGE_PID=""\nAUTOPILOT_PID=""',
    'MJPEG_PID=""\nBRIDGE_PID=""\nAUTOPILOT_PID=""\nMJPEG_RENICE_WATCHDOG_PID=""'
)

# 3. Insert functions before cleanup()
marker = 'cleanup() {\n'
helper = r'''
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

'''
if helper.strip() not in s:
    s = s.replace(marker, helper + "\n" + marker, 1)

# 4. Kill watchdog in cleanup
s = s.replace(
    '  if [[ -n "${BRIDGE_PID:-}" ]] && kill -0 "$BRIDGE_PID" 2>/dev/null; then\n    kill "$BRIDGE_PID" 2>/dev/null || true\n  fi',
    '  if [[ -n "${MJPEG_RENICE_WATCHDOG_PID:-}" ]] && kill -0 "$MJPEG_RENICE_WATCHDOG_PID" 2>/dev/null; then\n    kill "$MJPEG_RENICE_WATCHDOG_PID" 2>/dev/null || true\n  fi\n\n  if [[ -n "${BRIDGE_PID:-}" ]] && kill -0 "$BRIDGE_PID" 2>/dev/null; then\n    kill "$BRIDGE_PID" 2>/dev/null || true\n  fi'
)

# 5. Replace one-shot renice block with thread renice + watchdog
old = '''  sleep 0.2
  renice -n "$MJPEG_NICE" -p "$MJPEG_PID" >/dev/null 2>&1 || true
  log "MJPEG nice set to $MJPEG_NICE, cpuset=$MJPEG_CPUSET for pid=$MJPEG_PID"
  sleep 0.3'''

new = '''  sleep 0.2
  renice_mjpeg_threads_once
  log "MJPEG nice set to $MJPEG_NICE, cpuset=$MJPEG_CPUSET for pid=$MJPEG_PID"
  start_mjpeg_renice_watchdog
  sleep 0.3'''

if old not in s:
    raise SystemExit("renice block not found")

s = s.replace(old, new, 1)

p.write_text(s)
print("patched launcher.sh")
