from pathlib import Path
import shutil
import time

p = Path("ptz_autopilot.cpp")
s = p.read_text(encoding="utf-8")

backup = p.with_suffix(f".cpp.bak_fix_manual_invert_accel_order_{int(time.time())}")
shutil.copy2(p, backup)
print(f"backup: {backup}")

old = (
    "fp=clampi(fp,lastp-r.effective_max_accel,lastp+r.effective_max_accel); "
    "ft=clampi(ft,lastt-r.effective_max_accel,lastt+r.effective_max_accel); "
    "if(cfg.invert_pan) fp=-fp; if(cfg.invert_tilt) ft=-ft; "
    "fp=clampi(fp,-cfg.max_pan,cfg.max_pan); ft=clampi(ft,-cfg.max_tilt,cfg.max_tilt); "
    "r.final_pan=fp; r.final_tilt=ft; return r;"
)

new = (
    "if(cfg.invert_pan) fp=-fp; if(cfg.invert_tilt) ft=-ft; "
    "fp=clampi(fp,lastp-r.effective_max_accel,lastp+r.effective_max_accel); "
    "ft=clampi(ft,lastt-r.effective_max_accel,lastt+r.effective_max_accel); "
    "fp=clampi(fp,-cfg.max_pan,cfg.max_pan); ft=clampi(ft,-cfg.max_tilt,cfg.max_tilt); "
    "r.final_pan=fp; r.final_tilt=ft; return r;"
)

if old not in s:
    raise SystemExit("ERROR: target manual invert/accel block not found")

s = s.replace(old, new, 1)
p.write_text(s, encoding="utf-8")

print("OK: manual_drive now applies inversion before accel limiting")
