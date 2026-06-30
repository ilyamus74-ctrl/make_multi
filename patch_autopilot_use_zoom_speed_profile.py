from pathlib import Path
import shutil, time

p = Path("ptz_autopilot.cpp")
s = p.read_text(encoding="utf-8")

backup = p.with_suffix(f".cpp.bak_autopilot_use_speed_profile_{int(time.time())}")
shutil.copy2(p, backup)
print("backup:", backup)

# 1. ZoomState: add zoom_sample_idx
old = 'struct ZoomState { bool ok=false; bool profile_loaded=false; double zoom_ratio=0.0,focal_px=0.0,focal_min_px=0.0,focal_max_px=0.0,speed_scale=1.0; std::string source; std::string error; };'
new = 'struct ZoomState { bool ok=false; bool profile_loaded=false; int zoom_sample_idx=-1; double zoom_ratio=0.0,focal_px=0.0,focal_min_px=0.0,focal_max_px=0.0,speed_scale=1.0; std::string source; std::string error; };'

if old in s:
    s = s.replace(old, new, 1)
    print("OK: ZoomState.zoom_sample_idx added")
elif "zoom_sample_idx=-1" in s:
    print("SKIP: ZoomState already has zoom_sample_idx")
else:
    raise SystemExit("ERROR: ZoomState struct anchor not found")

# 2. fetch_zoom_state: parse zoom_sample_idx
old = 'json_bool(b,"profile_loaded",zs.profile_loaded); json_num(b,"zoom_ratio",zs.zoom_ratio); json_num(b,"focal_px",zs.focal_px);'
new = 'json_bool(b,"profile_loaded",zs.profile_loaded); json_int(b,"zoom_sample_idx",zs.zoom_sample_idx); json_num(b,"zoom_ratio",zs.zoom_ratio); json_num(b,"focal_px",zs.focal_px);'

if old in s:
    s = s.replace(old, new, 1)
    print("OK: fetch_zoom_state reads zoom_sample_idx")
elif 'json_int(b,"zoom_sample_idx",zs.zoom_sample_idx)' in s:
    print("SKIP: fetch_zoom_state already reads zoom_sample_idx")
else:
    raise SystemExit("ERROR: fetch_zoom_state anchor not found")

# 3. Insert helper for resolving active profile
anchor = '''static bool send_bridge_j(int pan,int tilt){
  int seq=g.seq.fetch_add(1);
  std::ostringstream cmd;
  cmd<<"J "<<seq<<" "<<pan<<" "<<tilt;
  return send_bridge_line(cmd.str());
}
'''

helper = r'''
static SpeedPointResolveResult resolve_active_speed_point_for_zoom(const ZoomState& zs){
  SpeedPointResolveResult sr = resolve_speed_point_for_sample(zs.zoom_sample_idx, zs.zoom_ratio, zs.focal_px);

  {
    std::lock_guard<std::mutex> lk(g_runtimeSpeedOverrideMtx);
    if(g_runtimeSpeedOverrideActive && g_runtimeSpeedOverride.profile_idx == zs.zoom_sample_idx){
      sr.point = g_runtimeSpeedOverride;
      sr.source = "runtime_override";
      sr.left_profile_idx = -1;
      sr.right_profile_idx = -1;
      sr.t = 0.0;
    }
  }

  clamp_speed_point(sr.point);
  return sr;
}
'''

if "resolve_active_speed_point_for_zoom" not in s:
    if anchor not in s:
        raise SystemExit("ERROR: send_bridge_j anchor not found")
    s = s.replace(anchor, anchor + helper, 1)
    print("OK: added resolve_active_speed_point_for_zoom()")
else:
    print("SKIP: helper already exists")

# 4. Patch autopilot_loop zoom/profile section
old = '''ZoomState zs; fetch_zoom_state(zs); const double rawScale=compute_zoom_speed_scale(zs); const double speedScale=(cfg.zoom_scale_smoothing<=0.0)?rawScale:(smoothScale=smoothScale*(1.0-cfg.zoom_scale_smoothing)+rawScale*cfg.zoom_scale_smoothing); bool active=g.enabled.load();'''

new = '''ZoomState zs; fetch_zoom_state(zs); auto sr=resolve_active_speed_point_for_zoom(zs); SpeedPoint sp=sr.point; const bool hasSpeedProfile=(sr.source!="fallback"); const double rawScale=hasSpeedProfile?1.0:compute_zoom_speed_scale(zs); const double speedScale=(cfg.zoom_scale_smoothing<=0.0)?rawScale:(smoothScale=smoothScale*(1.0-cfg.zoom_scale_smoothing)+rawScale*cfg.zoom_scale_smoothing); bool active=g.enabled.load();'''

if old in s:
    s = s.replace(old, new, 1)
    print("OK: autopilot_loop resolves speed profile")
elif "hasSpeedProfile=(sr.source!=\"fallback\")" in s:
    print("SKIP: autopilot_loop already resolves speed profile")
else:
    raise SystemExit("ERROR: autopilot_loop zoom/profile anchor not found")

# 5. Patch effective limits to use SpeedPoint instead of cfg
old = '''int effectiveMaxPan=std::max(1,(int)std::lround(cfg.max_pan*speedScale)); int effectiveMaxTilt=std::max(1,(int)std::lround(cfg.max_tilt*speedScale)); int effectiveMaxAccel=std::max(1,(int)std::lround(cfg.max_accel*speedScale)); const int effectiveMinPan=cfg.min_pan>0?std::max(1,(int)std::lround(cfg.min_pan*speedScale)):0; const int effectiveMinTilt=cfg.min_tilt>0?std::max(1,(int)std::lround(cfg.min_tilt*speedScale)):0;'''

new = '''int effectiveMaxPan=std::max(1,(int)std::lround((sp.max_pan>0?sp.max_pan:cfg.max_pan)*speedScale)); int effectiveMaxTilt=std::max(1,(int)std::lround((sp.max_tilt>0?sp.max_tilt:cfg.max_tilt)*speedScale)); int effectiveMaxAccel=std::max(1,(int)std::lround((sp.max_accel>0?sp.max_accel:cfg.max_accel)*speedScale)); const int effectiveMinPan=sp.min_pan>0?std::max(1,(int)std::lround(sp.min_pan*speedScale)):0; const int effectiveMinTilt=sp.min_tilt>0?std::max(1,(int)std::lround(sp.min_tilt*speedScale)):0;'''

if old in s:
    s = s.replace(old, new, 1)
    print("OK: autopilot effective limits use profile")
elif "(sp.max_pan>0?sp.max_pan:cfg.max_pan)" in s:
    print("SKIP: effective limits already use profile")
else:
    raise SystemExit("ERROR: effective limits anchor not found")

# 6. Patch PID params to use SpeedPoint
old = '''if(std::abs(ex)<cfg.deadzone) ex=0; if(std::abs(ey)<cfg.deadzone) ey=0;'''
new = '''const double activeDeadzone=sp.deadzone>0?sp.deadzone:cfg.deadzone; if(std::abs(ex)<activeDeadzone) ex=0; if(std::abs(ey)<activeDeadzone) ey=0;'''

if old in s:
    s = s.replace(old, new, 1)
    print("OK: autopilot deadzone uses profile")
elif "activeDeadzone=sp.deadzone" in s:
    print("SKIP: deadzone already uses profile")
else:
    raise SystemExit("ERROR: deadzone anchor not found")

old = '''basePan=int(std::lround(cfg.kp*ex+cfg.ki*ix+cfg.kd*dx)); baseTilt=int(std::lround(cfg.kp*ey+cfg.ki*iy+cfg.kd*dy));'''
new = '''basePan=int(std::lround(sp.kp*ex+sp.ki*ix+sp.kd*dx)); baseTilt=int(std::lround(sp.kp*ey+sp.ki*iy+sp.kd*dy));'''

if old in s:
    s = s.replace(old, new, 1)
    print("OK: autopilot PID uses profile kp/ki/kd")
elif "sp.kp*ex+sp.ki*ix+sp.kd*dx" in s:
    print("SKIP: PID already uses profile")
else:
    raise SystemExit("ERROR: PID anchor not found")

# 7. Patch runtime state update to expose active profile
old = '''g.effective_max_accel=effectiveMaxAccel; g.zoom_source=zs.source; g.zoom_error=zs.error; if(!ok) g.last_error="tracker_unreachable"; else g.last_error.clear();'''

new = '''g.effective_max_accel=effectiveMaxAccel; g.active_profile_idx=sp.profile_idx; g.active_zoom_sample_idx=zs.zoom_sample_idx; g.profile_max_pan=sp.max_pan; g.profile_max_tilt=sp.max_tilt; g.profile_max_accel=sp.max_accel; g.speed_profile_source=sr.source; g.speed_profile_left_idx=sr.left_profile_idx; g.speed_profile_right_idx=sr.right_profile_idx; g.speed_profile_t=sr.t; g.zoom_source=zs.source; g.zoom_error=zs.error; if(!ok) g.last_error="tracker_unreachable"; else g.last_error.clear();'''

if old in s:
    s = s.replace(old, new, 1)
    print("OK: runtime state exposes active speed profile")
elif "g.speed_profile_source=sr.source" in s:
    print("SKIP: runtime state already exposes profile")
else:
    raise SystemExit("ERROR: runtime state update anchor not found")

p.write_text(s, encoding="utf-8")
print("DONE")
