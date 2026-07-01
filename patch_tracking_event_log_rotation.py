from pathlib import Path
import time

p = Path("apply_ptz_object_preset.py")
s = p.read_text(encoding="utf-8")

bak = p.with_suffix(p.suffix + f".bak_event_log_rotation_{int(time.time())}")
bak.write_text(s, encoding="utf-8")

if "import os" not in s:
    s = s.replace("import json\n", "import json\nimport os\n", 1)

anchor = 'STATE_FILE = ROOT / "object_tracking_state.json"\n'
insert = '''STATE_FILE = ROOT / "object_tracking_state.json"

MAX_EVENT_LOG_BYTES = int(os.environ.get("OBJECT_TRACKING_EVENT_LOG_MAX_BYTES", str(2 * 1024 * 1024)))
EVENT_LOG_KEEP_FILES = int(os.environ.get("OBJECT_TRACKING_EVENT_LOG_KEEP_FILES", "5"))
'''

if "MAX_EVENT_LOG_BYTES" not in s:
    if anchor not in s:
        raise SystemExit("ERROR: STATE_FILE anchor not found")
    s = s.replace(anchor, insert, 1)

func_anchor = "\ndef event_log(event, **payload):\n"
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


'''
if "def rotate_file_by_size(" not in s:
    if func_anchor not in s:
        raise SystemExit("ERROR: event_log anchor not found")
    s = s.replace(func_anchor, "\n" + helper + func_anchor, 1)

old = '''    try:
        with EVENT_LOG_FILE.open("a", encoding="utf-8") as f:
            f.write(json.dumps(row, ensure_ascii=False, sort_keys=True) + "\\n")
    except Exception:
        pass
'''

new = '''    try:
        rotate_file_by_size(EVENT_LOG_FILE, MAX_EVENT_LOG_BYTES, EVENT_LOG_KEEP_FILES)
        with EVENT_LOG_FILE.open("a", encoding="utf-8") as f:
            f.write(json.dumps(row, ensure_ascii=False, sort_keys=True) + "\\n")
    except Exception:
        pass
'''

if old not in s:
    raise SystemExit("ERROR: event_log write block not found")

s = s.replace(old, new, 1)

p.write_text(s, encoding="utf-8")

print("OK patched event log rotation")
print("Backup:", bak)
