from pathlib import Path
import shutil
import time

p = Path("ptz_autopilot.cpp")
s = p.read_text(encoding="utf-8")

backup = p.with_suffix(f".cpp.bak_manual_direction_change_and_floor_{int(time.time())}")
shutil.copy2(p, backup)
print(f"backup: {backup}")

old = (
    "if(cfg.invert_pan) fp=-fp; if(cfg.invert_tilt) ft=-ft; "
    "fp=clampi(fp,lastp-r.effective_max_accel,lastp+r.effective_max_accel); "
    "ft=clampi(ft,lastt-r.effective_max_accel,lastt+r.effective_max_accel); "
    "fp=clampi(fp,-r.effective_max_pan,r.effective_max_pan); "
    "ft=clampi(ft,-r.effective_max_tilt,r.effective_max_tilt); "
    "r.final_pan=fp; r.final_tilt=ft; return r;"
)

new = (
    "if(cfg.invert_pan) fp=-fp; if(cfg.invert_tilt) ft=-ft; "

    "// If operator changes direction, do not ramp through the old opposite command. "
    "if(fp!=0 && lastp!=0 && ((fp>0)!=(lastp>0))) lastp=0; "
    "if(ft!=0 && lastt!=0 && ((ft>0)!=(lastt>0))) lastt=0; "

    "fp=clampi(fp,lastp-r.effective_max_accel,lastp+r.effective_max_accel); "
    "ft=clampi(ft,lastt-r.effective_max_accel,lastt+r.effective_max_accel); "

    "// Manual physical startup floor after accel limiting. "
    "if(pn!=0.0 && minp>0 && std::abs(fp)<minp) fp=(fp<0 || pn<0)?-minp:minp; "
    "if(tn!=0.0 && mint>0 && std::abs(ft)<mint) ft=(ft<0 || tn<0)?-mint:mint; "

    "fp=clampi(fp,-r.effective_max_pan,r.effective_max_pan); "
    "ft=clampi(ft,-r.effective_max_tilt,r.effective_max_tilt); "
    "r.final_pan=fp; r.final_tilt=ft; return r;"
)

if old not in s:
    raise SystemExit("ERROR: target accel/final clamp block not found")

s = s.replace(old, new, 1)
p.write_text(s, encoding="utf-8")

print("OK: fixed manual direction change ramp and post-accel min floor")
