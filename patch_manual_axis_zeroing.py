from pathlib import Path
import shutil
import time

p = Path("ptz_autopilot.cpp")
s = p.read_text(encoding="utf-8")

backup = p.with_suffix(f".cpp.bak_manual_axis_zeroing_{int(time.time())}")
shutil.copy2(p, backup)
print(f"backup: {backup}")

func = "static MovementResult build_scaled_movement_command"
a = s.index(func)

block_start = s.index("if(cfg.invert_pan) fp=-fp;", a)
ret = s.index("return r;", block_start)
block_end = s.index("\n}", ret) + 2

new_block = """if(cfg.invert_pan) fp=-fp;
 if(cfg.invert_tilt) ft=-ft;

 /*
  * Manual axis rule:
  * - if operator does not request an axis, that axis must be zero immediately;
  * - do not let old pan/tilt decay through accel into a new one-axis command.
  */
 if(pn==0.0){ fp=0; lastp=0; }
 if(tn==0.0){ ft=0; lastt=0; }

 const int targetp = fp;
 const int targett = ft;

 /* If operator changes direction, do not ramp through the old opposite command. */
 if(targetp!=0 && lastp!=0 && ((targetp>0)!=(lastp>0))) lastp=0;
 if(targett!=0 && lastt!=0 && ((targett>0)!=(lastt>0))) lastt=0;

 fp=clampi(targetp,lastp-r.effective_max_accel,lastp+r.effective_max_accel);
 ft=clampi(targett,lastt-r.effective_max_accel,lastt+r.effective_max_accel);

 /* Manual physical startup floor after accel limiting. */
 if(targetp!=0 && minp>0 && std::abs(fp)<minp) fp=(targetp<0)?-minp:minp;
 if(targett!=0 && mint>0 && std::abs(ft)<mint) ft=(targett<0)?-mint:mint;

 fp=clampi(fp,-r.effective_max_pan,r.effective_max_pan);
 ft=clampi(ft,-r.effective_max_tilt,r.effective_max_tilt);
 r.final_pan=fp;
 r.final_tilt=ft;
 return r;
}"""

s = s[:block_start] + new_block + s[block_end:]
p.write_text(s, encoding="utf-8")

print("OK: manual inactive axis zeroing applied")
