from pathlib import Path
import time

p = Path("launcher.sh")
s = p.read_text(encoding="utf-8", errors="ignore")

bak = p.with_suffix(p.suffix + f".bak_object_tracking_daemon_{int(time.time())}")
bak.write_text(s, encoding="utf-8")

changed = False

env_anchor = 'OBJECT_PRESET_APPLIER="${OBJECT_PRESET_APPLIER:-$ROOT_DIR/apply_ptz_object_preset.py}"\n'
env_insert = '''OBJECT_PRESET_APPLIER="${OBJECT_PRESET_APPLIER:-$ROOT_DIR/apply_ptz_object_preset.py}"
OBJECT_TRACKING_DAEMON_ENABLE="${OBJECT_TRACKING_DAEMON_ENABLE:-1}"
OBJECT_TRACKING_DAEMON="${OBJECT_TRACKING_DAEMON:-$ROOT_DIR/object_tracking_daemon.py}"
OBJECT_TRACKING_DAEMON_LOG="${OBJECT_TRACKING_DAEMON_LOG:-$ROOT_DIR/object_tracking_daemon.log}"
'''

if "OBJECT_TRACKING_DAEMON_ENABLE" not in s:
    if env_anchor in s:
        s = s.replace(env_anchor, env_insert, 1)
        changed = True
    else:
        insert_after = 'OBJECT_PRESET_NAME="${OBJECT_PRESET_NAME:-active}"\n'
        if insert_after in s:
            s = s.replace(insert_after, insert_after + env_insert, 1)
            changed = True
        else:
            print("WARN: env anchor not found")

func_anchor = 'start_bridge() {\n'
func = r'''
start_object_tracking_daemon() {
  if [[ "${OBJECT_TRACKING_DAEMON_ENABLE:-1}" != "1" ]]; then
    log "Object tracking daemon disabled"
    return 0
  fi

  if [[ ! -f "$OBJECT_TRACKING_DAEMON" ]]; then
    log "Object tracking daemon not found: $OBJECT_TRACKING_DAEMON"
    return 0
  fi

  log "Starting object tracking daemon: $OBJECT_TRACKING_DAEMON"
  python3 "$OBJECT_TRACKING_DAEMON" >>"$OBJECT_TRACKING_DAEMON_LOG" 2>&1 &
  OBJECT_TRACKING_DAEMON_PID=$!
  log "Object tracking daemon PID: $OBJECT_TRACKING_DAEMON_PID"
}

'''

if "start_object_tracking_daemon()" not in s:
    if func_anchor in s:
        s = s.replace(func_anchor, func + func_anchor, 1)
        changed = True
    else:
        print("WARN: start_bridge anchor not found")

call_anchor = '''if [[ "$AUTOPILOT_ENABLE" == "1" ]]; then
  start_autopilot
  apply_object_preset_on_start
fi
'''

call_new = '''if [[ "$AUTOPILOT_ENABLE" == "1" ]]; then
  start_autopilot
  apply_object_preset_on_start
  start_object_tracking_daemon
fi
'''

if "start_object_tracking_daemon" in s and "start_object_tracking_daemon\nfi" not in s:
    if call_anchor in s:
        s = s.replace(call_anchor, call_new, 1)
        changed = True
    else:
        print("WARN: autopilot start block anchor not found")

if changed:
    p.write_text(s, encoding="utf-8")
    print("OK patched launcher.sh")
    print("Backup:", bak)
else:
    print("No changes made")
