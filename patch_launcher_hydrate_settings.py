from pathlib import Path
import time

p = Path("launcher.sh")
s = p.read_text(encoding="utf-8", errors="ignore")

bak = p.with_suffix(p.suffix + f".bak_hydrate_settings_{int(time.time())}")
bak.write_text(s, encoding="utf-8")

changed = False

if 'HYDRATE_RUNTIME_SETTINGS="${HYDRATE_RUNTIME_SETTINGS:-$ROOT_DIR/hydrate_runtime_settings.py}"' not in s:
    anchor = 'OBJECT_TRACKING_DAEMON="${OBJECT_TRACKING_DAEMON:-$ROOT_DIR/object_tracking_daemon.py}"\n'

    if anchor in s:
        s = s.replace(
            anchor,
            anchor + 'HYDRATE_RUNTIME_SETTINGS="${HYDRATE_RUNTIME_SETTINGS:-$ROOT_DIR/hydrate_runtime_settings.py}"\n',
            1
        )
        changed = True
    else:
        print("WARN: env anchor not found")

func = r'''
hydrate_runtime_settings() {
  if [ -f "$HYDRATE_RUNTIME_SETTINGS" ]; then
    echo "[launcher] hydrate runtime settings"
    python3 "$HYDRATE_RUNTIME_SETTINGS" || true
  fi
}

'''

if "hydrate_runtime_settings()" not in s:
    anchor = "start_object_tracking_daemon()"

    pos = s.find(anchor)

    if pos >= 0:
        s = s[:pos] + func + s[pos:]
        changed = True
    else:
        print("WARN: function insertion anchor not found")

# Call hydration after mjpeg starts and before object preset/daemon.
if "hydrate_runtime_settings\n" not in s:
    # Best effort: before apply_object_preset_on_start if exists.
    if "apply_object_preset_on_start\n" in s:
        s = s.replace(
            "apply_object_preset_on_start\n",
            "hydrate_runtime_settings\napply_object_preset_on_start\n",
            1
        )
        changed = True
    elif "start_object_tracking_daemon\n" in s:
        s = s.replace(
            "start_object_tracking_daemon\n",
            "hydrate_runtime_settings\nstart_object_tracking_daemon\n",
            1
        )
        changed = True
    else:
        print("WARN: call insertion anchor not found")

if changed:
    p.write_text(s, encoding="utf-8")
    print("OK patched launcher.sh")
else:
    print("NO CHANGES")

print("Backup:", bak)
