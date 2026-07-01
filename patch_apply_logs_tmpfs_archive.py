from pathlib import Path
import time

p = Path("apply_ptz_object_preset.py")
s = p.read_text(encoding="utf-8", errors="ignore")

bak = p.with_suffix(p.suffix + f".bak_tmpfs_logs_{int(time.time())}")
bak.write_text(s, encoding="utf-8")

if "import atexit" not in s:
    s = s.replace("import argparse\n", "import argparse\nimport atexit\nimport signal\n", 1)

if "from object_tracking_log_paths import" not in s:
    anchor = "from pathlib import Path\n"
    s = s.replace(
        anchor,
        anchor + "from object_tracking_log_paths import runtime_path, archive_current_logs, ensure_log_dirs\n",
        1
    )

s = s.replace(
    'EVENT_LOG_FILE = ROOT / "object_tracking_events.jsonl"',
    'EVENT_LOG_FILE = runtime_path("object_tracking_events.jsonl")'
)

s = s.replace(
    'STATE_FILE = ROOT / "object_tracking_state.json"',
    'STATE_FILE = runtime_path("object_tracking_state.json")'
)

if "ensure_log_dirs()" not in s:
    anchor = 'AUTOPILOT_BASE = "http://127.0.0.1:8090"\n'
    if anchor in s:
        s = s.replace(anchor, anchor + "\nensure_log_dirs()\n", 1)

exit_hook = r'''

# tmpfs logs are archived to disk on normal process exit / SIGTERM.
_ARCHIVE_DONE = False

def _archive_logs_once(reason="exit"):
    global _ARCHIVE_DONE
    if _ARCHIVE_DONE:
        return
    _ARCHIVE_DONE = True

    try:
        archive_current_logs(reason=reason)
    except Exception:
        pass


def _signal_archive_handler(signum, frame):
    try:
        stopper = globals().get("stop_ptz")
        if callable(stopper):
            stopper()
    except Exception:
        pass

    _archive_logs_once(reason=f"signal_{signum}")
    raise SystemExit(0)


atexit.register(lambda: _archive_logs_once(reason="atexit"))

try:
    signal.signal(signal.SIGTERM, _signal_archive_handler)
    signal.signal(signal.SIGINT, _signal_archive_handler)
except Exception:
    pass

'''

if "_archive_logs_once" not in s:
    marker = '\nif __name__ == "__main__":\n'
    if marker not in s:
        raise SystemExit("ERROR: __main__ marker not found")
    s = s.replace(marker, exit_hook + marker, 1)

p.write_text(s, encoding="utf-8")

print("OK patched apply_ptz_object_preset.py")
print("Backup:", bak)
