from pathlib import Path
import time

p = Path("ptz_contract_audit.py")
s = p.read_text(encoding="utf-8", errors="ignore")

bak = p.with_suffix(p.suffix + f".bak_speed_source_v2_{int(time.time())}")
bak.write_text(s, encoding="utf-8")

old = '''        add("PTZ speed profile source user", ap.get("speed_profile_source") == "user", f"source={ap.get('speed_profile_source')}")'''

new = '''        src = ap.get("speed_profile_source")
        add(
            "PTZ speed profile source not runtime override",
            src not in [None, "", "fallback", "runtime_override"],
            f"source={src}"
        )'''

if old not in s:
    raise SystemExit("anchor not found")

s = s.replace(old, new, 1)

p.write_text(s, encoding="utf-8")

print("OK patched ptz_contract_audit.py speed source rule")
print("Backup:", bak)
