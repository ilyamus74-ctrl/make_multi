from pathlib import Path
import time

p = Path("object_tracking_daemon.py")
s = p.read_text(encoding="utf-8", errors="ignore")

bak = p.with_suffix(p.suffix + f".bak_tmpfs_logs_{int(time.time())}")
bak.write_text(s, encoding="utf-8")

if "from object_tracking_log_paths import" not in s:
    anchor = "from pathlib import Path\n"
    s = s.replace(
        anchor,
        anchor + "from object_tracking_log_paths import runtime_path, archive_current_logs, ensure_log_dirs\n",
        1
    )

s = s.replace(
    'LOG_FILE = ROOT / "object_tracking_daemon.log"',
    'LOG_FILE = runtime_path("object_tracking_daemon.log")'
)

s = s.replace(
    'STATE_FILE = ROOT / "object_tracking_daemon_state.json"',
    'STATE_FILE = runtime_path("object_tracking_daemon_state.json")'
)

if "ensure_log_dirs()" not in s:
    anchor = "POLL_SEC = 1.0\n"
    if anchor in s:
        s = s.replace(anchor, "ensure_log_dirs()\n\n" + anchor, 1)

old = '''    child = None
    child_preset = None
'''

new = '''    child = None
    child_preset = None

    try:
        archived = archive_current_logs(reason=reason)
        if archived:
            log("logs_archived", reason=reason, archived=archived)
    except Exception as e:
        log("logs_archive_failed", reason=reason, error=str(e))
'''

if old in s and "logs_archived" not in s:
    s = s.replace(old, new, 1)

old = '''    stop_child(f"signal_{signum}")
    log("daemon_stop", signal=signum)
'''

new = '''    stop_child(f"signal_{signum}")

    try:
        archived = archive_current_logs(reason=f"daemon_signal_{signum}")
        if archived:
            log("logs_archived", reason=f"daemon_signal_{signum}", archived=archived)
    except Exception as e:
        log("logs_archive_failed", reason=f"daemon_signal_{signum}", error=str(e))

    log("daemon_stop", signal=signum)
'''

if old in s:
    s = s.replace(old, new, 1)

p.write_text(s, encoding="utf-8")

print("OK patched object_tracking_daemon.py")
print("Backup:", bak)
