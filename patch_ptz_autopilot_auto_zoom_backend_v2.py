from pathlib import Path
import re
import time

p = Path("ptz_autopilot.cpp")
s = p.read_text(encoding="utf-8", errors="ignore")

bak = p.with_suffix(p.suffix + f".bak_auto_zoom_backend_v2_{int(time.time())}")
bak.write_text(s, encoding="utf-8")

if "static void auto_zoom_loop()" in s and "auto_zoom_enable" in s and "send_bridge_z" in s:
    print("OK auto zoom already present")
    raise SystemExit(0)

changed = False

def must_replace(old, new, label, count=1):
    global s, changed
    if old not in s:
        raise SystemExit(f"ERROR: anchor not found: {label}")
    s = s.replace(old, new, count)
    changed = True

# 1) Config struct
if "bool auto_zoom_enable" not in s:
    old = '''struct Config {std::string mjpeg_url="http://127.0.0.1:8080"; std::string tracker_host="127.0.0.1"; int tracker_port=8080; std::string bridge_host="127.0.0.1"; int bridge_port=8765; int control_port=8090; int width=1920; int height=1080; double hz=20, kp=20, ki=0, kd=3, deadzone=0.05; int max_pan=20,max_tilt=20,max_accel=4; bool invert_pan=false; bool invert_tilt=true; double target_x=0.5,target_y=0.5; int min_pan=0,min_tilt=0; bool enabled=false; bool zoom_scale_enable=true; double zoom_scale_min=0.12,zoom_scale_max=1.0,zoom_scale_smoothing=0.25;};'''
    new = '''struct Config {std::string mjpeg_url="http://127.0.0.1:8080"; std::string tracker_host="127.0.0.1"; int tracker_port=8080; std::string bridge_host="127.0.0.1"; int bridge_port=8765; int control_port=8090; int width=1920; int height=1080; double hz=20, kp=20, ki=0, kd=3, deadzone=0.05; int max_pan=20,max_tilt=20,max_accel=4; bool invert_pan=false; bool invert_tilt=true; double target_x=0.5,target_y=0.5; int min_pan=0,min_tilt=0; bool enabled=false; bool zoom_scale_enable=true; double zoom_scale_min=0.12,zoom_scale_max=1.0,zoom_scale_smoothing=0.25; bool auto_zoom_enable=false; double auto_zoom_target_h=0.68,auto_zoom_deadzone=0.08; int auto_zoom_cmd=10,auto_zoom_sign=1,auto_zoom_period_ms=350;};'''
    must_replace(old, new, "Config struct")

# 2) Runtime fields
if "auto_zoom_box_h" not in s:
    old = '''std::string speed_profile_source="fallback"; int speed_profile_left_idx=-1,speed_profile_right_idx=-1; double speed_profile_t=0.0;'''
    new = '''std::string speed_profile_source="fallback"; int speed_profile_left_idx=-1,speed_profile_right_idx=-1; double speed_profile_t=0.0; bool auto_zoom_enable=false; int auto_zoom_cmd=0,auto_zoom_sign=1; double auto_zoom_box_h=0.0,auto_zoom_target_h=0.68,auto_zoom_deadzone=0.08;'''
    must_replace(old, new, "Runtime speed_profile tail")

# 3) clamp_config
if "cfg.auto_zoom_target_h = std::max" not in s:
    old = '''  cfg.zoom_scale_smoothing = std::max(0.0, std::min(1.0, cfg.zoom_scale_smoothing));
}'''
    new = '''  cfg.zoom_scale_smoothing = std::max(0.0, std::min(1.0, cfg.zoom_scale_smoothing));
  cfg.auto_zoom_target_h = std::max(0.15, std::min(0.95, cfg.auto_zoom_target_h));
  cfg.auto_zoom_deadzone = std::max(0.02, std::min(0.30, cfg.auto_zoom_deadzone));
  cfg.auto_zoom_cmd = std::max(1, std::min(60, cfg.auto_zoom_cmd));
  cfg.auto_zoom_sign = cfg.auto_zoom_sign < 0 ? -1 : 1;
  cfg.auto_zoom_period_ms = std::max(150, std::min(1500, cfg.auto_zoom_period_ms));
}'''
    must_replace(old, new, "clamp_config tail")

# 4) save_ptz_config
if '"auto_zoom_enable"' not in s:
    old = '''    << "  \\"zoom_scale_max\\": " << cfg.zoom_scale_max << ",\\n"
    << "  \\"zoom_scale_smoothing\\": " << cfg.zoom_scale_smoothing << "\\n"
    << "}\\n";'''
    new = '''    << "  \\"zoom_scale_max\\": " << cfg.zoom_scale_max << ",\\n"
    << "  \\"zoom_scale_smoothing\\": " << cfg.zoom_scale_smoothing << ",\\n"
    << "  \\"auto_zoom_enable\\": " << (cfg.auto_zoom_enable ? "true" : "false") << ",\\n"
    << "  \\"auto_zoom_target_h\\": " << cfg.auto_zoom_target_h << ",\\n"
    << "  \\"auto_zoom_deadzone\\": " << cfg.auto_zoom_deadzone << ",\\n"
    << "  \\"auto_zoom_cmd\\": " << cfg.auto_zoom_cmd << ",\\n"
    << "  \\"auto_zoom_sign\\": " << cfg.auto_zoom_sign << ",\\n"
    << "  \\"auto_zoom_period_ms\\": " << cfg.auto_zoom_period_ms << "\\n"
    << "}\\n";'''
    must_replace(old, new, "save_ptz_config zoom_scale_smoothing")

# 5) load/API parsers: add auto_zoom parsing after zoom_scale_smoothing parser.
parser_anchor = 'json_num(body,"zoom_scale_smoothing",cfg.zoom_scale_smoothing);'
parser_add = ' bool aze=false; if(json_bool(body,"auto_zoom_enable",aze)) cfg.auto_zoom_enable=aze; json_num(body,"auto_zoom_target_h",cfg.auto_zoom_target_h); json_num(body,"auto_zoom_deadzone",cfg.auto_zoom_deadzone); json_int(body,"auto_zoom_cmd",cfg.auto_zoom_cmd); json_int(body,"auto_zoom_sign",cfg.auto_zoom_sign); json_int(body,"auto_zoom_period_ms",cfg.auto_zoom_period_ms);'

if 'json_num(body,"auto_zoom_target_h",cfg.auto_zoom_target_h)' not in s:
    if parser_anchor not in s:
        raise SystemExit("ERROR: parser anchor not found")
    s = s.replace(parser_anchor, parser_anchor + parser_add)
    changed = True

# 6) send_bridge_z
if "static bool send_bridge_z" not in s:
    old = '''static bool send_bridge_j(int pan,int tilt){
  int seq=g.seq.fetch_add(1);
  std::ostringstream cmd;
  cmd<<"J "<<seq<<" "<<pan<<" "<<tilt;
  return send_bridge_line(cmd.str());
}'''
    new = '''static bool send_bridge_j(int pan,int tilt){
  int seq=g.seq.fetch_add(1);
  std::ostringstream cmd;
  cmd<<"J "<<seq<<" "<<pan<<" "<<tilt;
  return send_bridge_line(cmd.str());
}

static bool send_bridge_z(int zoom){
  std::ostringstream cmd;
  cmd<<"Z "<<zoom;
  return send_bridge_line(cmd.str());
}'''
    must_replace(old, new, "send_bridge_j")

# 7) auto_zoom_loop
if "static void auto_zoom_loop()" not in s:
    auto_zoom_loop = r'''
static void auto_zoom_loop(){
  int last_cmd = 0;
  bool was_zooming = false;
  long long last_send_ms = 0;

  while(g.run){
    int cmd = 0;
    double box_h = 0.0;

    const bool active = g.enabled.load() && cfg.auto_zoom_enable;

    TrackerState ts;
    std::string body;
    bool ok = http_get(cfg.tracker_host,cfg.tracker_port,"/api/tracker/state",body) && parse_tracker(body,ts);

    if(active && ok && ts.mode=="TRACKING" && ts.valid){
      box_h = double(ts.b - ts.t) / double(std::max(1,cfg.height));

      const double low = cfg.auto_zoom_target_h - cfg.auto_zoom_deadzone;
      const double high = cfg.auto_zoom_target_h + cfg.auto_zoom_deadzone;

      if(box_h < low) cmd = cfg.auto_zoom_sign * cfg.auto_zoom_cmd;
      else if(box_h > high) cmd = -cfg.auto_zoom_sign * cfg.auto_zoom_cmd;
      else cmd = 0;
    }

    const long long now = now_ms();
    const bool periodic = cmd != 0 && (now - last_send_ms) >= cfg.auto_zoom_period_ms;
    const bool changed_cmd = cmd != last_cmd;

    if(changed_cmd || periodic || (!active && was_zooming)){
      if(send_bridge_z(cmd)){
        last_cmd = cmd;
        last_send_ms = now;
        was_zooming = (cmd != 0);
      }
    }

    {
      std::lock_guard<std::mutex> lk(g.m);
      g.auto_zoom_enable = cfg.auto_zoom_enable;
      g.auto_zoom_cmd = cmd;
      g.auto_zoom_box_h = box_h;
      g.auto_zoom_target_h = cfg.auto_zoom_target_h;
      g.auto_zoom_deadzone = cfg.auto_zoom_deadzone;
      g.auto_zoom_sign = cfg.auto_zoom_sign;
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  send_bridge_z(0);
}

'''
    anchor = '''static std::string state_json(){'''
    if anchor not in s:
        raise SystemExit("ERROR: state_json function anchor not found")
    s = s.replace(anchor, auto_zoom_loop + anchor, 1)
    changed = True

# 8) state_json: robust block patch
state_start = s.find("static std::string state_json(){")
if state_start < 0:
    raise SystemExit("ERROR: state_json start not found")

state_end = s.find("static void control_server(){", state_start)
if state_end < 0:
    raise SystemExit("ERROR: state_json end marker not found")

block = s[state_start:state_end]

if '\\"auto_zoom_enable\\"' not in block and '"auto_zoom_enable"' not in block:
    matches = list(re.finditer(r'<<\s*"\}"\s*;\s*return\s+os\.str\(\)\s*;\s*\}', block))
    if not matches:
        raise SystemExit("ERROR: state_json close marker not found")

    m = matches[-1]
    tail = '<<",\\"auto_zoom_enable\\":"<<(g.auto_zoom_enable?"true":"false")<<",\\"auto_zoom_cmd\\":"<<g.auto_zoom_cmd<<",\\"auto_zoom_box_h\\":"<<g.auto_zoom_box_h<<",\\"auto_zoom_target_h\\":"<<g.auto_zoom_target_h<<",\\"auto_zoom_deadzone\\":"<<g.auto_zoom_deadzone<<",\\"auto_zoom_sign\\":"<<g.auto_zoom_sign<<"}"; return os.str();}'
    block2 = block[:m.start()] + tail + block[m.end():]
    s = s[:state_start] + block2 + s[state_end:]
    changed = True

# 9) main thread: add auto_zoom_loop thread
if "t3(auto_zoom_loop)" not in s:
    old = '''std::thread t1(autopilot_loop), t2(control_server); t1.join(); t2.join(); return 0;}'''
    new = '''std::thread t1(autopilot_loop), t2(control_server), t3(auto_zoom_loop); t1.join(); t2.join(); t3.join(); return 0;}'''
    must_replace(old, new, "main thread")

if changed:
    p.write_text(s, encoding="utf-8")
    print(f"OK patched {p}")
    print(f"Backup: {bak}")
else:
    print("OK no changes needed")
