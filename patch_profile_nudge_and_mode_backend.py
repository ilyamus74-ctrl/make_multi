from pathlib import Path
import shutil, time

p = Path("ptz_autopilot.cpp")
s = p.read_text(encoding="utf-8")

backup = p.with_suffix(f".cpp.bak_profile_nudge_mode_{int(time.time())}")
shutil.copy2(p, backup)
print("backup:", backup)

# 1. SpeedPoint add nudge/manual_mode
old = 'struct SpeedPoint { int profile_idx=-1; int step=-1; double zoom_ratio=0.0,focal_px=0.0,kp=0,ki=0,kd=0,deadzone=0; int max_pan=0,max_tilt=0,max_accel=0,min_pan=0,min_tilt=0,j_pulse_ms=70; };'
new = 'struct SpeedPoint { int profile_idx=-1; int step=-1; double zoom_ratio=0.0,focal_px=0.0,kp=0,ki=0,kd=0,deadzone=0; int max_pan=0,max_tilt=0,max_accel=0,min_pan=0,min_tilt=0,j_pulse_ms=70,nudge_pan=0,nudge_tilt=0; std::string manual_mode="tap_hold"; };'
if old in s:
    s = s.replace(old, new, 1)
    print("OK: SpeedPoint extended")
elif 'nudge_pan=0,nudge_tilt=0' in s:
    print("SKIP: SpeedPoint already extended")
else:
    raise SystemExit("ERROR: SpeedPoint anchor not found")

# 2. MovementResult add nudge/manual_mode
old = 'int effective_max_pan=0; int effective_max_tilt=0; int effective_max_accel=0; int j_pulse_ms=70; std::string source;'
new = 'int effective_max_pan=0; int effective_max_tilt=0; int effective_max_accel=0; int j_pulse_ms=70; int nudge_pan=0,nudge_tilt=0; std::string manual_mode="tap_hold"; std::string source;'
if old in s:
    s = s.replace(old, new, 1)
    print("OK: MovementResult extended")
elif 'int nudge_pan=0,nudge_tilt=0' in s:
    print("SKIP: MovementResult already extended")
else:
    raise SystemExit("ERROR: MovementResult anchor not found")

# 3. speed_point_to_json add nudge/manual_mode
old = '<<",\\"min_tilt\\":"<<p.min_tilt<<",\\"j_pulse_ms\\":"<<p.j_pulse_ms<<",\\"source\\":\\"user\\"}"'
new = '<<",\\"min_tilt\\":"<<p.min_tilt<<",\\"j_pulse_ms\\":"<<p.j_pulse_ms<<",\\"nudge_pan\\":"<<p.nudge_pan<<",\\"nudge_tilt\\":"<<p.nudge_tilt<<",\\"manual_mode\\":\\""<<p.manual_mode<<"\\",\\"source\\":\\"user\\"}"'
if old in s:
    s = s.replace(old, new, 1)
    print("OK: speed_point_to_json writes nudge/manual_mode")
elif '\\"nudge_pan\\":"<<p.nudge_pan' in s:
    print("SKIP: speed_point_to_json already extended")
else:
    raise SystemExit("ERROR: speed_point_to_json anchor not found")

# 4. loader parse nudge/manual_mode
old = 'json_int(obj,"min_tilt",p.min_tilt); json_int(obj,"j_pulse_ms",p.j_pulse_ms); p.j_pulse_ms=std::max(1,std::min(120,p.j_pulse_ms)); pts.push_back(p);'
new = 'json_int(obj,"min_tilt",p.min_tilt); json_int(obj,"j_pulse_ms",p.j_pulse_ms); json_int(obj,"nudge_pan",p.nudge_pan); json_int(obj,"nudge_tilt",p.nudge_tilt); p.manual_mode=json_str(obj,"manual_mode"); if(p.manual_mode!="pulse"&&p.manual_mode!="hold"&&p.manual_mode!="tap_hold") p.manual_mode="tap_hold"; p.j_pulse_ms=std::max(1,std::min(120,p.j_pulse_ms)); pts.push_back(p);'
if old in s:
    s = s.replace(old, new, 1)
    print("OK: loader reads nudge/manual_mode")
elif 'json_int(obj,"nudge_pan",p.nudge_pan)' in s:
    print("SKIP: loader already extended")
else:
    raise SystemExit("ERROR: loader anchor not found")

# 5. clamp speed point
old = 'p.min_tilt=std::max(0,std::min(p.max_tilt,p.min_tilt)); p.j_pulse_ms=std::max(1,std::min(120,p.j_pulse_ms)); }'
new = 'p.min_tilt=std::max(0,std::min(p.max_tilt,p.min_tilt)); p.j_pulse_ms=std::max(1,std::min(120,p.j_pulse_ms)); p.nudge_pan=std::max(0,std::min(100,p.nudge_pan)); p.nudge_tilt=std::max(0,std::min(100,p.nudge_tilt)); if(p.manual_mode!="pulse"&&p.manual_mode!="hold"&&p.manual_mode!="tap_hold") p.manual_mode="tap_hold"; }'
if old in s:
    s = s.replace(old, new, 1)
    print("OK: clamp_speed_point extended")
elif 'p.nudge_pan=std::max(0,std::min(100,p.nudge_pan));' in s:
    print("SKIP: clamp already extended")
else:
    raise SystemExit("ERROR: clamp anchor not found")

# 6. interpolation add nudge/pulse/manual mode
old = 'r.point.min_tilt=(int)std::lround(L.min_tilt+(R.min_tilt-L.min_tilt)*t); r.point.j_pulse_ms=(int)std::lround(L.j_pulse_ms+(R.j_pulse_ms-L.j_pulse_ms)*t); clamp_speed_point(r.point);'
new = 'r.point.min_tilt=(int)std::lround(L.min_tilt+(R.min_tilt-L.min_tilt)*t); r.point.j_pulse_ms=(int)std::lround(L.j_pulse_ms+(R.j_pulse_ms-L.j_pulse_ms)*t); r.point.nudge_pan=(int)std::lround(L.nudge_pan+(R.nudge_pan-L.nudge_pan)*t); r.point.nudge_tilt=(int)std::lround(L.nudge_tilt+(R.nudge_tilt-L.nudge_tilt)*t); r.point.manual_mode=(t<0.5?L.manual_mode:R.manual_mode); clamp_speed_point(r.point);'
if old in s:
    s = s.replace(old, new, 1)
    print("OK: interpolation extended")
elif 'R.nudge_pan-L.nudge_pan' in s:
    print("SKIP: interpolation already extended")
else:
    raise SystemExit("ERROR: interpolation anchor not found")

# 7. fallback defaults
old = 'r.point.min_tilt=cfg.min_tilt; r.point.j_pulse_ms=70; r.source="fallback"; return r;'
new = 'r.point.min_tilt=cfg.min_tilt; r.point.j_pulse_ms=70; r.point.nudge_pan=cfg.max_pan; r.point.nudge_tilt=cfg.max_tilt; r.point.manual_mode="tap_hold"; r.source="fallback"; return r;'
if old in s:
    s = s.replace(old, new, 1)
    print("OK: fallback extended")
elif 'r.point.manual_mode="tap_hold"; r.source="fallback"' in s:
    print("SKIP: fallback already extended")
else:
    raise SystemExit("ERROR: fallback anchor not found")

# 8. build_scaled_movement_command expose nudge/manual_mode
old = 'r.speed_profile_source=sr.source; r.left_profile_idx=sr.left_profile_idx; r.right_profile_idx=sr.right_profile_idx; r.interpolation_t=sr.t; r.j_pulse_ms=std::max(1,std::min(120,sp.j_pulse_ms));'
new = 'r.speed_profile_source=sr.source; r.left_profile_idx=sr.left_profile_idx; r.right_profile_idx=sr.right_profile_idx; r.interpolation_t=sr.t; r.j_pulse_ms=std::max(1,std::min(120,sp.j_pulse_ms)); r.nudge_pan=(sp.nudge_pan>0?sp.nudge_pan:sp.max_pan); r.nudge_tilt=(sp.nudge_tilt>0?sp.nudge_tilt:sp.max_tilt); r.manual_mode=sp.manual_mode;'
if old in s:
    s = s.replace(old, new, 1)
    print("OK: movement result exposes nudge/manual_mode")
elif 'r.nudge_pan=(sp.nudge_pan>0?sp.nudge_pan:sp.max_pan);' in s:
    print("SKIP: movement result already extended")
else:
    raise SystemExit("ERROR: movement result anchor not found")

# 9. save_point reads nudge/manual_mode
old = 'np.min_pan=cfg.min_pan; np.min_tilt=cfg.min_tilt; json_int(body,"j_pulse_ms",np.j_pulse_ms); np.j_pulse_ms=std::max(1,std::min(120,np.j_pulse_ms)); bool replaced=false;'
new = 'np.min_pan=cfg.min_pan; np.min_tilt=cfg.min_tilt; json_int(body,"j_pulse_ms",np.j_pulse_ms); json_int(body,"nudge_pan",np.nudge_pan); json_int(body,"nudge_tilt",np.nudge_tilt); np.manual_mode=json_str(body,"manual_mode"); if(np.manual_mode!="pulse"&&np.manual_mode!="hold"&&np.manual_mode!="tap_hold") np.manual_mode="tap_hold"; if(np.nudge_pan<=0) np.nudge_pan=cfg.max_pan; if(np.nudge_tilt<=0) np.nudge_tilt=cfg.max_tilt; np.j_pulse_ms=std::max(1,std::min(120,np.j_pulse_ms)); bool replaced=false;'
if old in s:
    s = s.replace(old, new, 1)
    print("OK: save_point saves nudge/manual_mode")
elif 'json_int(body,"nudge_pan",np.nudge_pan)' in s:
    print("SKIP: save_point already extended")
else:
    raise SystemExit("ERROR: save_point anchor not found")

# 10. manual_j_pulse use nudge instead of base
old = '''      if(std::abs(mr.pan_norm) >= 0.001){
        jp = res.base_pan;
        if(cfg.invert_pan) jp = -jp;
      }

      if(std::abs(mr.tilt_norm) >= 0.001){
        jt = res.base_tilt;
        if(cfg.invert_tilt) jt = -jt;
      }
'''
new = '''      if(std::abs(mr.pan_norm) >= 0.001){
        jp = (mr.pan_norm > 0.0) ? res.nudge_pan : -res.nudge_pan;
        if(cfg.invert_pan) jp = -jp;
      }

      if(std::abs(mr.tilt_norm) >= 0.001){
        jt = (mr.tilt_norm > 0.0) ? res.nudge_tilt : -res.nudge_tilt;
        if(cfg.invert_tilt) jt = -jt;
      }
'''
if old in s:
    s = s.replace(old, new, 1)
    print("OK: manual_j_pulse uses nudge_pan/nudge_tilt")
elif 'res.nudge_pan' in s and 'res.nudge_tilt' in s:
    print("SKIP: manual_j_pulse already uses nudge")
else:
    raise SystemExit("ERROR: manual_j_pulse base block not found")

# 11. manual_j_pulse response add nudge/mode
old = '<<",\\"j_tilt\\":"<<jt\n        <<",\\"pulse_ms\\":"<<pulse'
new = '<<",\\"j_tilt\\":"<<jt\n        <<",\\"nudge_pan\\":"<<res.nudge_pan\n        <<",\\"nudge_tilt\\":"<<res.nudge_tilt\n        <<",\\"manual_mode\\":\\""<<res.manual_mode<<"\\""\n        <<",\\"pulse_ms\\":"<<pulse'
if old in s:
    s = s.replace(old, new, 1)
    print("OK: manual_j_pulse response extended")
elif '\\"nudge_pan\\":"<<res.nudge_pan' in s:
    print("SKIP: response already extended")
else:
    print("WARN: response anchor not found")

p.write_text(s, encoding="utf-8")
print("DONE")
