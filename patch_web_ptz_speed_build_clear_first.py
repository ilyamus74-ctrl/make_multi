from pathlib import Path
import shutil, time

p = Path("web/index.html")
s = p.read_text(encoding="utf-8")

backup = p.with_suffix(f".html.bak_ptz_speed_build_clear_first_{int(time.time())}")
shutil.copy2(p, backup)
print("Backup:", backup)

old = '''  async function buildPtzSpeedProfileFromZoomMaster() {
    const existing = await apiGetJson(`${autopilotBaseUrl()}/api/autopilot/speed_profile`).catch(() => ({ points: [] }));
    const existingPoints = Array.isArray(existing.points) ? existing.points : [];
    if (existingPoints.length && !window.confirm(`PTZ speed profile already has ${existingPoints.length} points. Overwrite matching samples?`)) {
      return { ok: false, cancelled: true };
    }
    const master = await apiGetJson('/api/zoom/master_profile');
'''

new = '''  async function buildPtzSpeedProfileFromZoomMaster() {
    const existing = await apiGetJson(`${autopilotBaseUrl()}/api/autopilot/speed_profile`).catch(() => ({ points: [] }));
    const existingPoints = Array.isArray(existing.points) ? existing.points : [];

    if ($('ptzTuneStatus')) {
      $('ptzTuneStatus').textContent = `clearing old PTZ speed profile: old=${existingPoints.length}`;
    }

    const clearRes = await apiPostJson(`${autopilotBaseUrl()}/api/autopilot/speed_profile/clear`, {
      reason: 'build_from_zoom_master',
      old_points: existingPoints.length
    }).catch(e => ({ ok:false, error:String(e.message || e) }));

    ptzLog('PTZ SPEED PROFILE clear before build', clearRes);

    const master = await apiGetJson('/api/zoom/master_profile');
'''

if old in s:
    s = s.replace(old, new, 1)
    print("OK: buildPtzSpeedProfileFromZoomMaster now clears old speed profile first")
elif "PTZ SPEED PROFILE clear before build" in s:
    print("SKIP: already patched")
else:
    raise SystemExit("ERROR: buildPtzSpeedProfileFromZoomMaster old block not found")

p.write_text(s, encoding="utf-8")
print("DONE")
