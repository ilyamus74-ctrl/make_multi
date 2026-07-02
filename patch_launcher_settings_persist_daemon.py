from pathlib import Path
import time

p = Path("launcher.sh")
s = p.read_text(encoding="utf-8", errors="ignore")

bak = p.with_suffix(p.suffix + f".bak_settings_persist_daemon_{int(time.time())}")
bak.write_text(s, encoding="utf-8")

changed = False

if 'SETTINGS_PERSIST_DAEMON="${SETTINGS_PERSIST_DAEMON:-$ROOT_DIR/settings_persist_daemon.py}"' not in s:
    anchor = 'HYDRATE_RUNTIME_SETTINGS="${HYDRATE_RUNTIME_SETTINGS:-$ROOT_DIR/hydrate_runtime_settings.py}"\n'

    if anchor in s:
        s = s.replace(
            anchor,
            anchor + 'SETTINGS_PERSIST_DAEMON="${SETTINGS_PERSIST_DAEMON:-$ROOT_DIR/settings_persist_daemon.py}"\n',
            1
        )
        changed = True
    else:
        print("WARN: env anchor not found")

func = r'''
start_settings_persist_daemon() {
  if [ -f "$SETTINGS_PERSIST_DAEMON" ]; then
    echo "[launcher] start settings persist daemon"
    python3 "$SETTINGS_PERSIST_DAEMON" &
    SETTINGS_PERSIST_DAEMON_PID=$!
    echo "[launcher] settings persist daemon pid=$SETTINGS_PERSIST_DAEMON_PID"
  fi
}

'''

if "start_settings_persist_daemon()" not in s:
    marker = "start_object_tracking_daemon()"

    pos = s.find(marker)

    if pos >= 0:
        s = s[:pos] + func + s[pos:]
        changed = True
    else:
        print("WARN: function insertion anchor not found")

if "start_settings_persist_daemon\n" not in s:
    if "hydrate_runtime_settings\n" in s:
        s = s.replace(
            "hydrate_runtime_settings\n",
            "hydrate_runtime_settings\nstart_settings_persist_daemon\n",
            1
        )
        changed = True
    elif "start_object_tracking_daemon\n" in s:
        s = s.replace(
            "start_object_tracking_daemon\n",
            "start_settings_persist_daemon\nstart_object_tracking_daemon\n",
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
