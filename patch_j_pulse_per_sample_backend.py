from pathlib import Path
import shutil, time, re

p = Path("ptz_autopilot.cpp")
s = p.read_text(encoding="utf-8")

backup = p.with_suffix(f".cpp.bak_j_pulse_per_sample_{int(time.time())}")
shutil.copy2(p, backup)
print("backup:", backup)

# 1) SpeedPoint: add j_pulse_ms
old = 'struct SpeedPoint { int profile_idx=-1; int step=-1; double zoom_ratio=0.0,focal_px=0.0,kp=0,ki=0,kd=0,deadzone=0; int max_pan=0,max_tilt=0,max_accel=0,min_pan=0,min_tilt=0; };'
new = 'struct SpeedPoint { int profile_idx=-1; int step=-1; double zoom_ratio=0.0,focal_px=0.0,kp=0,ki=0,kd=0,deadzone=0; int max_pan=0,max_tilt=0,max_accel=0,min_pan=0,min_tilt=0,j_pulse_ms=70; };'
if old in s:
    s = s.replace(old, new, 1)
    print("OK: SpeedPoint.j_pulse_ms added")
elif "j_pulse_ms=70" in s:
    print("SKIP: SpeedPoint already has j_pulse_ms")
else:
    raise SystemExit("ERROR: SpeedPoint struct not found")

# 2) MovementResult: add j_pulse_ms
old = 'int effective_max_pan=0; int effective_max_tilt=0; int effective_max_accel=0; std::string source;'
new = 'int effective_max_pan=0; int effective_max_tilt=0; int effective_max_accel=0; int j_pulse_ms=70; std::string source;'
if old in s:
    s = s.replace(old, new, 1)
    print("OK: MovementResult.j_pulse_ms added")
elif "int j_pulse_ms=70;" in s:
    print("SKIP: MovementResult already has j_pulse_ms")
else:
    raise SystemExit("ERROR: MovementResult anchor not found")

# 3) speed_point_to_json: include j_pulse_ms before source
old = ',\\"min_tilt\\":"<<p.min_tilt<<",\\"source\\":\\"user\\"}"'
new = ',\\"min_tilt\\":"<<p.min_tilt<<",\\"j_pulse_ms\\":"<<p.j_pulse_ms<<",\\"source\\":\\"user\\"}"'
if old in s:
    s = s.replace(old, new, 1)
    print("OK: speed_point_to_json writes j_pulse_ms")
elif '\\"j_pulse_ms\\":"<<p.j_pulse_ms' in s:
    print("SKIP: speed_point_to_json already writes j_pulse_ms")
else:
    raise SystemExit("ERROR: speed_point_to_json anchor not found")

# 4) load_speed_profile_points: parse j_pulse_ms
old = 'json_int(obj,"min_tilt",p.min_tilt); pts.push_back(p);'
new = 'json_int(obj,"min_tilt",p.min_tilt); json_int(obj,"j_pulse_ms",p.j_pulse_ms); p.j_pulse_ms=std::max(1,std::min(120,p.j_pulse_ms)); pts.push_back(p);'
if old in s:
    s = s.replace(old, new, 1)
    print("OK: speed profile loader reads j_pulse_ms")
elif 'json_int(obj,"j_pulse_ms",p.j_pulse_ms)' in s:
    print("SKIP: loader already reads j_pulse_ms")
else:
    raise SystemExit("ERROR: loader anchor not found")

# 5) clamp_speed_point: clamp j_pulse_ms
old = 'p.min_tilt=std::max(0,std::min(p.max_tilt,p.min_tilt)); }'
new = 'p.min_tilt=std::max(0,std::min(p.max_tilt,p.min_tilt)); p.j_pulse_ms=std::max(1,std::min(120,p.j_pulse_ms)); }'
if old in s:
    s = s.replace(old, new, 1)
    print("OK: clamp_speed_point clamps j_pulse_ms")
elif 'p.j_pulse_ms=std::max(1,std::min(120,p.j_pulse_ms));' in s:
    print("SKIP: clamp already has j_pulse_ms")
else:
    raise SystemExit("ERROR: clamp anchor not found")

# 6) interpolation: interpolate j_pulse_ms
old = 'r.point.min_tilt=(int)std::lround(L.min_tilt+(R.min_tilt-L.min_tilt)*t); clamp_speed_point(r.point);'
new = 'r.point.min_tilt=(int)std::lround(L.min_tilt+(R.min_tilt-L.min_tilt)*t); r.point.j_pulse_ms=(int)std::lround(L.j_pulse_ms+(R.j_pulse_ms-L.j_pulse_ms)*t); clamp_speed_point(r.point);'
if old in s:
    s = s.replace(old, new, 1)
    print("OK: interpolation includes j_pulse_ms")
elif 'R.j_pulse_ms-L.j_pulse_ms' in s:
    print("SKIP: interpolation already has j_pulse_ms")
else:
    raise SystemExit("ERROR: interpolation anchor not found")

# 7) fallback: default j_pulse_ms
old = 'r.point.min_tilt=cfg.min_tilt; r.source="fallback"; return r;'
new = 'r.point.min_tilt=cfg.min_tilt; r.point.j_pulse_ms=70; r.source="fallback"; return r;'
if old in s:
    s = s.replace(old, new, 1)
    print("OK: fallback j_pulse_ms=70")
elif 'r.point.j_pulse_ms=70; r.source="fallback"' in s:
    print("SKIP: fallback already has j_pulse_ms")
else:
    raise SystemExit("ERROR: fallback anchor not found")

# 8) build_scaled_movement_command: expose selected j_pulse_ms in MovementResult
old = 'r.speed_profile_source=sr.source; r.left_profile_idx=sr.left_profile_idx; r.right_profile_idx=sr.right_profile_idx; r.interpolation_t=sr.t;'
new = 'r.speed_profile_source=sr.source; r.left_profile_idx=sr.left_profile_idx; r.right_profile_idx=sr.right_profile_idx; r.interpolation_t=sr.t; r.j_pulse_ms=std::max(1,std::min(120,sp.j_pulse_ms));'
if old in s:
    s = s.replace(old, new, 1)
    print("OK: MovementResult gets j_pulse_ms")
elif 'r.j_pulse_ms=std::max(1,std::min(120,sp.j_pulse_ms));' in s:
    print("SKIP: MovementResult already gets j_pulse_ms")
else:
    raise SystemExit("ERROR: movement result anchor not found")

# 9) save_point: read j_pulse_ms from request
old = 'np.min_pan=cfg.min_pan; np.min_tilt=cfg.min_tilt; bool replaced=false;'
new = 'np.min_pan=cfg.min_pan; np.min_tilt=cfg.min_tilt; json_int(body,"j_pulse_ms",np.j_pulse_ms); np.j_pulse_ms=std::max(1,std::min(120,np.j_pulse_ms)); bool replaced=false;'
if old in s:
    s = s.replace(old, new, 1)
    print("OK: save_point saves j_pulse_ms from body")
elif 'json_int(body,"j_pulse_ms",np.j_pulse_ms)' in s:
    print("SKIP: save_point already saves j_pulse_ms")
else:
    raise SystemExit("ERROR: save_point anchor not found")

# 10) manual_j_pulse: default pulse from selected sample if caller does not send pulse_ms
old = '''      int pulse=70;
      json_int(body,"pulse_ms",pulse);
      pulse=clampi(pulse,1,120);

      auto res=build_scaled_movement_command(mr);
'''
new = '''      int pulse=-1;
      const bool pulseProvided=json_int(body,"pulse_ms",pulse);

      auto res=build_scaled_movement_command(mr);
      if(!pulseProvided) pulse=res.j_pulse_ms;
      pulse=clampi(pulse,1,120);
'''
if old in s:
    s = s.replace(old, new, 1)
    print("OK: manual_j_pulse defaults to sample j_pulse_ms")
elif 'if(!pulseProvided) pulse=res.j_pulse_ms;' in s:
    print("SKIP: manual_j_pulse already defaults to sample j_pulse_ms")
else:
    print("WARN: manual_j_pulse pulse block not found; check manually")

# 11) manual_j_pulse response: expose sample j_pulse_ms
old = '<<",\\"pulse_ms\\":"<<pulse'
new = '<<",\\"pulse_ms\\":"<<pulse\n      <<",\\"sample_j_pulse_ms\\":"<<res.j_pulse_ms'
if old in s and 'sample_j_pulse_ms' not in s:
    s = s.replace(old, new, 1)
    print("OK: manual_j_pulse response includes sample_j_pulse_ms")
elif 'sample_j_pulse_ms' in s:
    print("SKIP: response already includes sample_j_pulse_ms")
else:
    print("WARN: response pulse_ms anchor not found")

p.write_text(s, encoding="utf-8")
print("DONE")
