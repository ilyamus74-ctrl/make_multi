from pathlib import Path
import time

p = Path("object_tracking_daemon.py")
s = p.read_text(encoding="utf-8")

bak = p.with_suffix(p.suffix + f".bak_log_rotation_throttle_{int(time.time())}")
bak.write_text(s, encoding="utf-8")

anchor = 'STATE_FILE = ROOT / "object_tracking_daemon_state.json"\n'
insert = '''STATE_FILE = ROOT / "object_tracking_daemon_state.json"

MAX_DAEMON_LOG_BYTES = int(os.environ.get("OBJECT_TRACKING_DAEMON_LOG_MAX_BYTES", str(1024 * 1024)))
DAEMON_LOG_KEEP_FILES = int(os.environ.get("OBJECT_TRACKING_DAEMON_LOG_KEEP_FILES", "5"))

_last_log_ts = {}
_last_log_sig = {}
'''

if "MAX_DAEMON_LOG_BYTES" not in s:
    if anchor not in s:
        raise SystemExit("ERROR: STATE_FILE anchor not found")
    s = s.replace(anchor, insert, 1)

func_anchor = "\ndef log(event, **data):\n"
helper = r'''
def rotate_file_by_size(path, max_bytes, keep_files):
    try:
        path = Path(path)

        if max_bytes <= 0:
            return

        if not path.exists():
            return

        if path.stat().st_size < max_bytes:
            return

        keep_files = max(1, int(keep_files))

        oldest = path.with_name(path.name + f".{keep_files}")
        if oldest.exists():
            oldest.unlink()

        for idx in range(keep_files - 1, 0, -1):
            src = path.with_name(path.name + f".{idx}")
            dst = path.with_name(path.name + f".{idx + 1}")
            if src.exists():
                src.replace(dst)

        path.replace(path.with_name(path.name + ".1"))

    except Exception:
        pass


def should_write_daemon_log(row):
    event = row.get("event")
    now = float(row.get("ts") or time.time())

    # armed_running каждую секунду не нужен. Пишем максимум раз в 15 сек
    # или при смене pid/preset.
    if event == "armed_running":
        sig = f"{row.get('preset')}:{row.get('pid')}"
        prev_sig = _last_log_sig.get(event)
        prev_ts = float(_last_log_ts.get(event) or 0)

        if sig == prev_sig and now - prev_ts < 15:
            return False

        _last_log_sig[event] = sig
        _last_log_ts[event] = now
        return True

    return True


'''
if "def rotate_file_by_size(" not in s:
    if func_anchor not in s:
        raise SystemExit("ERROR: log anchor not found")
    s = s.replace(func_anchor, "\n" + helper + func_anchor, 1)

old = '''    try:
        with LOG_FILE.open("a", encoding="utf-8") as f:
            f.write(line + "\\n")
    except Exception:
        pass

    print(line, flush=True)
'''

new = '''    if should_write_daemon_log(row):
        try:
            rotate_file_by_size(LOG_FILE, MAX_DAEMON_LOG_BYTES, DAEMON_LOG_KEEP_FILES)
            with LOG_FILE.open("a", encoding="utf-8") as f:
                f.write(line + "\\n")
        except Exception:
            pass

        print(line, flush=True)
'''

if old not in s:
    raise SystemExit("ERROR: daemon log write block not found")

s = s.replace(old, new, 1)

p.write_text(s, encoding="utf-8")

print("OK patched daemon log rotation/throttle")
print("Backup:", bak)
