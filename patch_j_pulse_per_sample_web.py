from pathlib import Path
import shutil, time

p = Path("web/index.html")
s = p.read_text(encoding="utf-8")

backup = p.with_suffix(f".html.bak_j_pulse_per_sample_{int(time.time())}")
shutil.copy2(p, backup)
print("backup:", backup)

# 1) SAVE SPEED POINT payload: add j_pulse_ms
old = '''      focal_px: focalPx
    });
'''
new = '''      focal_px: focalPx,
      j_pulse_ms: getJPulseMs()
    });
'''
if old in s:
    s = s.replace(old, new, 1)
    print("OK: SAVE SPEED POINT sends j_pulse_ms")
elif 'j_pulse_ms: getJPulseMs()' in s:
    print("SKIP: save already sends j_pulse_ms")
else:
    raise SystemExit("ERROR: save payload anchor not found")

# 2) ptzTuneLoadSpeedForSample: sync slider from loaded point
old = '''async function ptzTuneLoadSpeedForSample(sampleIdx){ const res=await apiPostJson(`${autopilotBaseUrl()}/api/autopilot/speed_profile/apply_nearest`, {profile_idx:Number(sampleIdx)}); if(res&&res.ok){ const point=res.point||res.applied_point||res.config||res; fillPtzTuneFieldsFromConfig(point); const st=await apiGetJson(`${autopilotBaseUrl()}/api/autopilot/state`); fillPtzTuneFieldsFromConfig(st);} return res;}'''
new = '''async function ptzTuneLoadSpeedForSample(sampleIdx){ const res=await apiPostJson(`${autopilotBaseUrl()}/api/autopilot/speed_profile/apply_nearest`, {profile_idx:Number(sampleIdx)}); if(res&&res.ok){ const point=res.point||res.applied_point||res.config||res; fillPtzTuneFieldsFromConfig(point); if(point && point.j_pulse_ms != null) syncJPulseInputs(point.j_pulse_ms); const st=await apiGetJson(`${autopilotBaseUrl()}/api/autopilot/state`); fillPtzTuneFieldsFromConfig(st);} return res;}'''
if old in s:
    s = s.replace(old, new, 1)
    print("OK: ptzTuneLoadSpeedForSample loads j_pulse_ms into slider")
elif 'syncJPulseInputs(point.j_pulse_ms)' in s:
    print("SKIP: ptzTuneLoadSpeedForSample already syncs j_pulse_ms")
else:
    raise SystemExit("ERROR: ptzTuneLoadSpeedForSample one-line anchor not found")

# 3) ptzTuneApplyNearestSavedSpeed: sync slider when loading nearest saved speed
old = '''      const point = res.point || res.applied_point || res.config || res;
      fillPtzTuneFieldsFromConfig(point);
'''
new = '''      const point = res.point || res.applied_point || res.config || res;
      fillPtzTuneFieldsFromConfig(point);
      if (point && point.j_pulse_ms != null) syncJPulseInputs(point.j_pulse_ms);
'''
if old in s:
    s = s.replace(old, new, 1)
    print("OK: apply_nearest syncs j_pulse_ms")
elif 'syncJPulseInputs(point.j_pulse_ms);' in s:
    print("SKIP: apply_nearest already syncs j_pulse_ms")
else:
    print("WARN: apply_nearest anchor not found")

# 4) ptzTuneLoadPointForCurrentStep: sync slider from point
old = '''    fillPtzTuneFieldsFromConfig(point);
    const res = await apiPostJson(`${autopilotBaseUrl()}/api/autopilot/config`, {
'''
new = '''    fillPtzTuneFieldsFromConfig(point);
    if (point && point.j_pulse_ms != null) syncJPulseInputs(point.j_pulse_ms);
    const res = await apiPostJson(`${autopilotBaseUrl()}/api/autopilot/config`, {
'''
if old in s:
    s = s.replace(old, new, 1)
    print("OK: load current step syncs j_pulse_ms")
elif 'load current step syncs j_pulse_ms' in s:
    print("SKIP")
else:
    print("WARN: load current step anchor not found")

# 5) After SAVE, reload saved sample so UI shows exactly what was written
old = '''    await refreshPtzTuneSpeedProfile();
    tuneStatusSet(zoomRatio, focalPx);
    updatePtzTuneSampleButtonUi?.();
'''
new = '''    await refreshPtzTuneSpeedProfile();
    await ptzTuneLoadSpeedForSample(sampleIdx).catch(() => {});
    tuneStatusSet(zoomRatio, focalPx);
    updatePtzTuneSampleButtonUi?.();
'''
if old in s:
    s = s.replace(old, new, 1)
    print("OK: after save reloads saved sample")
elif 'await ptzTuneLoadSpeedForSample(sampleIdx).catch(() => {});' in s:
    print("SKIP: after save already reloads sample")
else:
    print("WARN: after-save anchor not found")

p.write_text(s, encoding="utf-8")
print("DONE")
