from pathlib import Path
import time

p = Path("launcher.sh")
s = p.read_text(encoding="utf-8")
bak = p.with_suffix(p.suffix + f".bak_manual_default_{int(time.time())}")
bak.write_text(s, encoding="utf-8")

changed = False

# Autopilot process must run because manual backend endpoints are inside ptz_autopilot.
for old in [
    'AUTOPILOT_ENABLE="${AUTOPILOT_ENABLE:-0}"',
    'AUTOPILOT_ENABLE="${AUTOPILOT_ENABLE:-1}"',
]:
    if old in s:
        s = s.replace(old, 'AUTOPILOT_ENABLE="${AUTOPILOT_ENABLE:-1}"', 1)
        changed = True
        break

# But actual tracking must start disabled by default.
if 'AUTOPILOT_START_ENABLED=' not in s:
    s = s.replace(
        'AUTOPILOT_ENABLE="${AUTOPILOT_ENABLE:-1}"\n',
        'AUTOPILOT_ENABLE="${AUTOPILOT_ENABLE:-1}"\nAUTOPILOT_START_ENABLED="${AUTOPILOT_START_ENABLED:-0}"\n',
        1
    )
    changed = True
else:
    s2 = s.replace(
        'AUTOPILOT_START_ENABLED="${AUTOPILOT_START_ENABLED:-1}"',
        'AUTOPILOT_START_ENABLED="${AUTOPILOT_START_ENABLED:-0}"'
    )
    if s2 != s:
        s = s2
        changed = True

if '    --enable 0 \\' in s:
    s = s.replace('    --enable 0 \\\n', '    --enable "$AUTOPILOT_START_ENABLED" \\\n', 1)
    changed = True

# Make MODEL_DIR robust if models are in model_rknn instead of models.
old = 'MODEL_DIR="${MODEL_DIR:-$ROOT_DIR/models}"'
new = '''if [[ -z "${MODEL_DIR:-}" ]]; then
  MODEL_DIR=""
  for d in "$ROOT_DIR/models" "$ROOT_DIR/model_rknn" "$ROOT_DIR/new_yolo8/models" "$ROOT_DIR/new_yolo8/model_rknn"; do
    if [[ -d "$d" ]] && find "$d" -maxdepth 1 -type f -iname '*.rknn' | grep -q .; then
      MODEL_DIR="$d"
      break
    fi
  done
  MODEL_DIR="${MODEL_DIR:-$ROOT_DIR/models}"
fi'''
if old in s:
    s = s.replace(old, new, 1)
    changed = True

if changed:
    p.write_text(s, encoding="utf-8")
    print(f"OK launcher patched, backup={bak}")
else:
    print("OK launcher already patched")
