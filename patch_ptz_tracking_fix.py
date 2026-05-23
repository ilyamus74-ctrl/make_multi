from pathlib import Path
import shutil

# 1. Fix web/index.html: apply PTZ config before START
p = Path("web/index.html")
s = p.read_text(encoding="utf-8")

backup = p.with_suffix(".html.bak_ptz_start_apply_config")
if not backup.exists():
    shutil.copy2(p, backup)
    print(f"backup: {backup}")

old = '''  disconnectWs();

  const tr = await apiGetJson('/api/tracker/state');'''

new = '''  disconnectWs();

  // Apply current server-side PTZ settings before starting autopilot.
  await ptzApplyConfig();

  const tr = await apiGetJson('/api/tracker/state');'''

if old not in s:
    print("WARN: ptzStart insert pattern not found or already patched")
else:
    s = s.replace(old, new, 1)

p.write_text(s, encoding="utf-8")
print("patched web/index.html")


# 2. Fix ptz_autopilot.cpp: min command must work even when PID rounded to 0
p = Path("ptz_autopilot.cpp")
s = p.read_text(encoding="utf-8")

backup = p.with_suffix(".cpp.bak_min_command_fix")
if not backup.exists():
    shutil.copy2(p, backup)
    print(f"backup: {backup}")

old = '''if(ex!=0.0 && cfg.min_pan>0 && pan!=0 && std::abs(pan)<cfg.min_pan) pan=(pan>0)?cfg.min_pan:-cfg.min_pan; if(ey!=0.0 && cfg.min_tilt>0 && tilt!=0 && std::abs(tilt)<cfg.min_tilt) tilt=(tilt>0)?cfg.min_tilt:-cfg.min_tilt; if(cfg.invert_pan) pan=-pan; if(cfg.invert_tilt) tilt=-tilt;'''

new = '''if(ex!=0.0 && cfg.min_pan>0 && std::abs(pan)<cfg.min_pan) pan=(ex>0)?cfg.min_pan:-cfg.min_pan; if(ey!=0.0 && cfg.min_tilt>0 && std::abs(tilt)<cfg.min_tilt) tilt=(ey>0)?cfg.min_tilt:-cfg.min_tilt; if(cfg.invert_pan) pan=-pan; if(cfg.invert_tilt) tilt=-tilt;'''

if old not in s:
    raise SystemExit("min command block not found")

s = s.replace(old, new, 1)

p.write_text(s, encoding="utf-8")
print("patched ptz_autopilot.cpp")
