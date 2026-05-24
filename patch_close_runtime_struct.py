from pathlib import Path
import shutil

p = Path("ptz_autopilot.cpp")
s = p.read_text(encoding="utf-8")

backup = p.with_suffix(".cpp.bak_close_runtime_struct")
if not backup.exists():
    shutil.copy2(p, backup)
    print(f"backup: {backup}")

old = '''int base_cmd_pan=0,base_cmd_tilt=0,scaled_cmd_pan=0,scaled_cmd_tilt=0,effective_max_pan=0,effective_max_tilt=0,effective_max_accel=0; std::string zoom_source,zoom_error;
static Runtime g; static Config cfg;
static std::string g_ptzConfigFile = "ptz_autopilot_config.json";
'''

new = '''int base_cmd_pan=0,base_cmd_tilt=0,scaled_cmd_pan=0,scaled_cmd_tilt=0,effective_max_pan=0,effective_max_tilt=0,effective_max_accel=0; std::string zoom_source,zoom_error;
};
static Runtime g; static Config cfg;
static std::string g_ptzConfigFile = "ptz_autopilot_config.json";
'''

if old not in s:
    raise SystemExit("target Runtime tail block not found")

s = s.replace(old, new, 1)

p.write_text(s, encoding="utf-8")
print("patched ptz_autopilot.cpp: closed Runtime struct")
