from pathlib import Path
import time

p = Path("launcher.sh")
s = p.read_text(encoding="utf-8", errors="ignore")

bak = p.with_suffix(p.suffix + f".bak_object_log_server_{int(time.time())}")
bak.write_text(s, encoding="utf-8")

changed = False

env_anchor = 'OBJECT_TRACKING_DAEMON_LOG="${OBJECT_TRACKING_DAEMON_LOG:-$ROOT_DIR/object_tracking_daemon.log}"\n'
env_insert = '''OBJECT_TRACKING_DAEMON_LOG="${OBJECT_TRACKING_DAEMON_LOG:-/dev/shm/new_yolo8_object_tracking/object_tracking_daemon.log}"
OBJECT_LOG_SERVER_ENABLE="${OBJECT_LOG_SERVER_ENABLE:-1}"
OBJECT_LOG_SERVER="${OBJECT_LOG_SERVER:-$ROOT_DIR/object_tracking_log_server.py}"
OBJECT_LOG_SERVER_PORT="${OBJECT_LOG_SERVER_PORT:-8091}"
OBJECT_LOG_SERVER_LOG="${OBJECT_LOG_SERVER_LOG:-/dev/shm/new_yolo8_object_tracking/object_tracking_log_server.log}"
'''

if "OBJECT_LOG_SERVER_ENABLE" not in s:
    if env_anchor in s:
        s = s.replace(env_anchor, env_insert, 1)
        changed = True
    else:
        fallback = 'OBJECT_TRACKING_DAEMON="${OBJECT_TRACKING_DAEMON:-$ROOT_DIR/object_tracking_daemon.py}"\n'
        if fallback in s:
            s = s.replace(fallback, fallback + env_insert, 1)
            changed = True
        else:
            print("WARN: env anchor not found")

func_anchor = 'start_bridge() {\n'
func = r'''
start_object_log_server() {
  if [[ "${OBJECT_LOG_SERVER_ENABLE:-1}" != "1" ]]; then
    log "Object log server disabled"
    return 0
  fi

  if [[ ! -f "$OBJECT_LOG_SERVER" ]]; then
    log "Object log server not found: $OBJECT_LOG_SERVER"
    return 0
  fi

  mkdir -p /dev/shm/new_yolo8_object_tracking

  log "Starting object log server on port ${OBJECT_LOG_SERVER_PORT:-8091}"
  python3 "$OBJECT_LOG_SERVER" --port "${OBJECT_LOG_SERVER_PORT:-8091}" >>"$OBJECT_LOG_SERVER_LOG" 2>&1 &
  OBJECT_LOG_SERVER_PID=$!
  log "Object log server PID: $OBJECT_LOG_SERVER_PID"
}

'''

if "start_object_log_server()" not in s:
    if func_anchor in s:
        s = s.replace(func_anchor, func + func_anchor, 1)
        changed = True
    else:
        print("WARN: start_bridge anchor not found")

call_anchor = '''if [[ "$AUTOPILOT_ENABLE" == "1" ]]; then
  start_autopilot
  apply_object_preset_on_start
  start_object_tracking_daemon
fi
'''

call_new = '''if [[ "$AUTOPILOT_ENABLE" == "1" ]]; then
  start_autopilot
  apply_object_preset_on_start
  start_object_log_server
  start_object_tracking_daemon
fi
'''

if "start_object_log_server" in s and call_anchor in s:
    s = s.replace(call_anchor, call_new, 1)
    changed = True
elif "start_object_log_server" in s and "start_object_log_server\n  start_object_tracking_daemon" in s:
    print("OK log server call already present")
else:
    print("WARN: autopilot block anchor not found")

if changed:
    p.write_text(s, encoding="utf-8")
    print("OK patched launcher.sh")
    print("Backup:", bak)
else:
    print("No changes made")
