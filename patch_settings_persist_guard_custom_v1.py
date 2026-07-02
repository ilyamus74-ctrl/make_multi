from pathlib import Path
import time

p = Path("settings_persist_daemon.py")
s = p.read_text(encoding="utf-8", errors="ignore")

bak = p.with_suffix(p.suffix + f".bak_guard_custom_{int(time.time())}")
bak.write_text(s, encoding="utf-8")

old = '''    file_out = dict(runtime)
    file_out["ptzArmed"] = False
    file_out["controlMode"] = "manual"

    wrote = write_file_settings(file_out)
'''

new = '''    file_out = dict(runtime)
    file_out["ptzArmed"] = False
    file_out["controlMode"] = "manual"

    # Safety contract:
    # Never persist settings without objectPresetsCustom.
    # A short-lived runtime/default state must not destroy persistent presets.
    if not non_empty_dict(file_out.get("objectPresetsCustom")):
        log("skip persist: objectPresetsCustom missing or empty")
        return

    wrote = write_file_settings(file_out)
'''

if old not in s:
    raise SystemExit("anchor not found")

s = s.replace(old, new, 1)
p.write_text(s, encoding="utf-8")

print("OK patched settings_persist_daemon.py custom guard")
print("Backup:", bak)
