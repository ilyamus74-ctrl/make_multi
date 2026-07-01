from pathlib import Path
import time

p = Path("launcher.sh")
s = p.read_text(encoding="utf-8")

bak = p.with_suffix(p.suffix + f".bak_apply_object_preset_{int(time.time())}")
bak.write_text(s, encoding="utf-8")

changed = False

anchor = '''AUTOPILOT_MIN_TILT="${AUTOPILOT_MIN_TILT:-3}"

'''
insert = '''AUTOPILOT_MIN_TILT="${AUTOPILOT_MIN_TILT:-3}"

OBJECT_PRESET_APPLY_ON_START="${OBJECT_PRESET_APPLY_ON_START:-1}"
OBJECT_PRESET_NAME="${OBJECT_PRESET_NAME:-active}"
OBJECT_PRESET_APPLIER="${OBJECT_PRESET_APPLIER:-$ROOT_DIR/apply_ptz_object_preset.py}"

'''

if "OBJECT_PRESET_APPLY_ON_START" not in s:
    if anchor not in s:
        raise SystemExit("ERROR: launcher env anchor not found")
    s = s.replace(anchor, insert, 1)
    changed = True

anchor = '''start_bridge() {
'''
func = r'''
apply_object_preset_on_start() {
  if [[ "${OBJECT_PRESET_APPLY_ON_START:-1}" != "1" ]]; then
    log "Object preset apply on start disabled"
    return 0
  fi

  if [[ ! -f "$OBJECT_PRESET_APPLIER" ]]; then
    log "Object preset applier not found: $OBJECT_PRESET_APPLIER"
    return 0
  fi

  (
    sleep 1
    log "Applying object preset: ${OBJECT_PRESET_NAME:-active}"
    if python3 "$OBJECT_PRESET_APPLIER" --preset "${OBJECT_PRESET_NAME:-active}" >>"$AUTOPILOT_LOG" 2>&1; then
      log "Object preset applied: ${OBJECT_PRESET_NAME:-active}"
    else
      log "WARN: object preset apply failed: ${OBJECT_PRESET_NAME:-active}. See $AUTOPILOT_LOG"
    fi
  ) &
}

'''
if "apply_object_preset_on_start()" not in s:
    if anchor not in s:
        raise SystemExit("ERROR: start_bridge anchor not found")
    s = s.replace(anchor, func + anchor, 1)
    changed = True

old = '''if [[ "$AUTOPILOT_ENABLE" == "1" ]]; then
  start_autopilot
fi

log "Services are up"
'''
new = '''if [[ "$AUTOPILOT_ENABLE" == "1" ]]; then
  start_autopilot
  apply_object_preset_on_start
fi

log "Services are up"
'''

if old in s:
    s = s.replace(old, new, 1)
    changed = True

if changed:
    p.write_text(s, encoding="utf-8")
    print(f"OK patched launcher.sh, backup={bak}")
else:
    print("OK launcher already patched")

