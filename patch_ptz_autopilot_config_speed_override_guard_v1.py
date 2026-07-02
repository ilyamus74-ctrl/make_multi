from pathlib import Path
import time

p = Path("ptz_autopilot.cpp")
s = p.read_text(encoding="utf-8", errors="ignore")

bak = p.with_suffix(p.suffix + f".bak_speed_override_guard_{int(time.time())}")
bak.write_text(s, encoding="utf-8")

needle = 'clamp_config(); save_ptz_config(); set_runtime_speed_override_for_current_sample(); out=state_json();'

replacement = '''const bool speed_keys_present =
    body.find("\\"kp\\"")!=std::string::npos ||
    body.find("\\"ki\\"")!=std::string::npos ||
    body.find("\\"kd\\"")!=std::string::npos ||
    body.find("\\"deadzone\\"")!=std::string::npos ||
    body.find("\\"max_pan\\"")!=std::string::npos ||
    body.find("\\"max_tilt\\"")!=std::string::npos ||
    body.find("\\"max_accel\\"")!=std::string::npos ||
    body.find("\\"min_pan\\"")!=std::string::npos ||
    body.find("\\"min_tilt\\"")!=std::string::npos ||
    body.find("\\"hz\\"")!=std::string::npos;
  clamp_config();
  save_ptz_config();
  if(speed_keys_present){
    set_runtime_speed_override_for_current_sample();
  }
  out=state_json();'''

if needle not in s:
    raise SystemExit("anchor not found")

s = s.replace(needle, replacement, 1)

p.write_text(s, encoding="utf-8")

print("OK patched ptz_autopilot.cpp speed override guard")
print("Backup:", bak)
