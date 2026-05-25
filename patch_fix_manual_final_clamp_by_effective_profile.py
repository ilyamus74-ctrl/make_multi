from pathlib import Path
import shutil
import time

p = Path("ptz_autopilot.cpp")
s = p.read_text(encoding="utf-8")

backup = p.with_suffix(f".cpp.bak_fix_manual_final_clamp_{int(time.time())}")
shutil.copy2(p, backup)
print(f"backup: {backup}")

old = (
    "fp=clampi(fp,-cfg.max_pan,cfg.max_pan); "
    "ft=clampi(ft,-cfg.max_tilt,cfg.max_tilt); "
    "r.final_pan=fp; r.final_tilt=ft; return r;"
)

new = (
    "fp=clampi(fp,-r.effective_max_pan,r.effective_max_pan); "
    "ft=clampi(ft,-r.effective_max_tilt,r.effective_max_tilt); "
    "r.final_pan=fp; r.final_tilt=ft; return r;"
)

if old not in s:
    raise SystemExit("ERROR: final cfg.max_* clamp block not found")

s = s.replace(old, new, 1)
p.write_text(s, encoding="utf-8")

print("OK: manual_drive final clamp now uses effective profile limits")
