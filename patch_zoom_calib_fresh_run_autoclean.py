from pathlib import Path
import shutil, time

p = Path("web/index.html")
s = p.read_text(encoding="utf-8")

backup = p.with_suffix(f".html.bak_zoom_calib_fresh_run_autoclean_{int(time.time())}")
shutil.copy2(p, backup)
print("Backup:", backup)

# 1. Insert helper near getJson/postJson in ZoomCalib controller.
anchor = '''  async function getJson(url) { const r = await fetch(url, { cache: 'no-store' }); if (!r.ok) throw new Error(`${r.status} ${url}`); return r.json(); }
  async function postJson(url, body) { const r = await fetch(url, { method:'POST', headers:{'Content-Type':'application/json'}, body:JSON.stringify(body || {}) }); if (!r.ok) throw new Error(`${r.status} ${url}`); return r.json().catch(() => ({ ok:true })); }

'''

helper = r'''  async function requestFreshZoomCalibSession() {
    /*
     * Backend route is optional. If it exists, it archives old
     * zoom_apriltag_profile*.json before a new calibration run.
     * If old backend is running, do not block calibration.
     */
    try {
      const res = await postJson('/api/zoom_calibration/clear_profiles', {
        archive: true,
        reason: 'fresh_run_from_ui',
        ts: Date.now()
      });
      console.log('[zoom-calib] clear old profiles', res);
      return res;
    } catch (e) {
      console.warn('[zoom-calib] clear old profiles skipped', e);
      return { ok: false, skipped: true, error: String(e.message || e) };
    }
  }

'''

if "requestFreshZoomCalibSession" not in s:
    if anchor not in s:
        raise SystemExit("ERROR: getJson/postJson anchor not found")
    s = s.replace(anchor, anchor + helper, 1)
    print("OK: inserted requestFreshZoomCalibSession()")
else:
    print("SKIP: requestFreshZoomCalibSession already exists")

# 2. Patch runActive() so normal RUN button starts fresh session.
old = '''  async function runActive() {
    const a = activeAnchor();
    status(`starting calibration for ${activeLabel} id=${a.tag_id}...`);
    await save();
    const res = await postJson('/api/zoom_calibration', payload());
'''

new = '''  async function runActive() {
    const a = activeAnchor();
    status(`starting fresh calibration for ${activeLabel} id=${a.tag_id}...`);
    await save();
    await requestFreshZoomCalibSession();
    const res = await postJson('/api/zoom_calibration', payload());
'''

if old in s:
    s = s.replace(old, new, 1)
    print("OK: runActive now clears old profiles before calibration")
elif "starting fresh calibration" in s:
    print("SKIP: runActive already patched")
else:
    raise SystemExit("ERROR: runActive block not found")

p.write_text(s, encoding="utf-8")
print("DONE")
