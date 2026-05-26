from pathlib import Path
import shutil
import time
import re

p = Path("ptz_autopilot.cpp")
s = p.read_text(encoding="utf-8")

backup = p.with_suffix(f".cpp.bak_manual_j_pulse_10_120_{int(time.time())}")
shutil.copy2(p, backup)
print("backup:", backup)

# --- add send_bridge_line if missing ---
old_send_j = '''static bool send_bridge_j(int pan,int tilt){ static WsClient ws; if(!ws.connect_ws(cfg.bridge_host,cfg.bridge_port)) return false; int seq=g.seq.fetch_add(1); std::ostringstream cmd; cmd<<"J "<<seq<<" "<<pan<<" "<<tilt; if(!ws.send_text(cmd.str())){ ws.close_ws(); return false; } return true; }
'''

new_send = '''static bool send_bridge_line(const std::string& line){
  static WsClient ws;
  if(!ws.connect_ws(cfg.bridge_host,cfg.bridge_port)) return false;
  if(!ws.send_text(line)){
    ws.close_ws();
    return false;
  }
  return true;
}

static bool send_bridge_j(int pan,int tilt){
  int seq=g.seq.fetch_add(1);
  std::ostringstream cmd;
  cmd<<"J "<<seq<<" "<<pan<<" "<<tilt;
  return send_bridge_line(cmd.str());
}
'''

if 'static bool send_bridge_line(' not in s:
    if old_send_j not in s:
        raise SystemExit("ERROR: original send_bridge_j block not found")
    s = s.replace(old_send_j, new_send, 1)
    print("OK: added send_bridge_line")
else:
    print("SKIP: send_bridge_line already exists")

# --- add pulse mutex near now_ms ---
anchor = 'static long long now_ms(){ return std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now().time_since_epoch()).count(); }\n'
if 'static std::mutex g_manualPulseMtx;' not in s:
    if anchor not in s:
        raise SystemExit("ERROR: now_ms anchor not found")
    s = s.replace(anchor, anchor + 'static std::mutex g_manualPulseMtx;\n', 1)
    print("OK: added g_manualPulseMtx")

# --- if endpoint already exists, only normalize clamp range ---
if '/api/control/manual_j_pulse' in s:
    s = re.sub(r'pulse\s*=\s*clampi\(pulse\s*,\s*\d+\s*,\s*\d+\s*\)\s*;', 'pulse=clampi(pulse,10,120);', s)
    print("OK: existing manual_j_pulse clamp changed to 10..120")
else:
    anchor = ' else if(method=="POST" && path=="/api/control/manual_drive"){\n'
    endpoint = ''' else if(method=="POST" && path=="/api/control/manual_j_pulse"){
  if(g.enabled.load() && g.mode=="ACTIVE"){
    out="{\\"ok\\":false,\\"error\\":\\"autopilot_active\\"}";
  } else {
    std::unique_lock<std::mutex> pulseLock(g_manualPulseMtx, std::try_to_lock);
    if(!pulseLock.owns_lock()){
      out="{\\"ok\\":false,\\"error\\":\\"pulse_busy\\"}";
    } else {
      MovementRequest mr;
      json_num(body,"pan",mr.pan_norm);
      json_num(body,"tilt",mr.tilt_norm);
      mr.source=json_str(body,"source");
      if(mr.source.empty()) mr.source="manual_j_pulse";

      int pulse=70;
      json_int(body,"pulse_ms",pulse);
      pulse=clampi(pulse,10,120);

      auto res=build_scaled_movement_command(mr);

      /*
       * Direct joystick pulse:
       * - bypass accel ramp final_pan/final_tilt;
       * - use base command from current sample/runtime profile;
       * - send neutral immediately after a short pulse.
       */
      int jp=0;
      int jt=0;

      if(std::abs(mr.pan_norm) >= 0.001){
        jp = res.base_pan;
        if(cfg.invert_pan) jp = -jp;
      }

      if(std::abs(mr.tilt_norm) >= 0.001){
        jt = res.base_tilt;
        if(cfg.invert_tilt) jt = -jt;
      }

      jp=clampi(jp,-res.effective_max_pan,res.effective_max_pan);
      jt=clampi(jt,-res.effective_max_tilt,res.effective_max_tilt);

      bool ok=true;
      ok = send_bridge_j(jp,jt) && ok;
      std::this_thread::sleep_for(std::chrono::milliseconds(pulse));
      ok = send_bridge_j(0,0) && ok;
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
      send_bridge_j(0,0);

      {
        std::lock_guard<std::mutex> lk(g.m);
        g.manual_active=false;
        g.manual_last_source=mr.source;
        g.manual_cmd_pan=0;
        g.manual_cmd_tilt=0;
        g.manual_profile_idx=res.profile_idx;
        g.manual_zoom_sample_idx=res.zoom_sample_idx;
        g.manual_zoom_ratio=res.zoom_ratio;
        g.manual_expires_ms=0;
        g.active_profile_idx=res.zoom_sample_idx;
        g.active_zoom_sample_idx=res.zoom_sample_idx;
        g.profile_max_pan=res.effective_max_pan;
        g.profile_max_tilt=res.effective_max_tilt;
        g.profile_max_accel=res.effective_max_accel;
        g.speed_profile_source=res.speed_profile_source;
        g.speed_profile_left_idx=res.left_profile_idx;
        g.speed_profile_right_idx=res.right_profile_idx;
        g.speed_profile_t=res.interpolation_t;
      }

      std::ostringstream os;
      os<<"{\\"ok\\":"<<(ok?"true":"false")
        <<",\\"mode\\":\\"manual_j_pulse\\""
        <<",\\"profile_idx\\":"<<res.zoom_sample_idx
        <<",\\"zoom_ratio\\":"<<res.zoom_ratio
        <<",\\"speed_profile_source\\":\\""<<res.speed_profile_source<<"\\""
        <<",\\"base_pan\\":"<<res.base_pan
        <<",\\"base_tilt\\":"<<res.base_tilt
        <<",\\"j_pan\\":"<<jp
        <<",\\"j_tilt\\":"<<jt
        <<",\\"pulse_ms\\":"<<pulse
        <<"}";
      out=os.str();
    }
  }
 }
'''
    if anchor not in s:
        raise SystemExit("ERROR: manual_drive anchor not found")
    s = s.replace(anchor, endpoint + "\n" + anchor, 1)
    print("OK: inserted /api/control/manual_j_pulse")

p.write_text(s, encoding="utf-8")
print("DONE")
