from pathlib import Path
import shutil, time

p = Path("web/index.html")
s = p.read_text(encoding="utf-8")

backup = p.with_suffix(f".html.bak_ptz_speed_master_points_dedupe_{int(time.time())}")
shutil.copy2(p, backup)
print("Backup:", backup)

helper_anchor = '''  function makeFallbackZoomSamples(count) {
'''

helper = r'''  function normalizeZoomMasterPoints(master) {
    const src = Array.isArray(master?.profile_points)
      ? master.profile_points
      : (Array.isArray(master?.points) ? master.points : []);

    const byIdx = new Map();

    src.forEach((p, i) => {
      const idx = Number(p.profile_idx ?? p.step ?? i);
      if (!Number.isFinite(idx)) return;

      const prev = byIdx.get(idx);
      const item = {
        profile_idx: idx,
        step: idx,
        zoom_ratio: Number(p.zoom_ratio ?? 0),
        focal_px: Number(p.focal_px ?? 0),
        fallback: false
      };

      /*
       * Prefer point with focal_px/zoom_ratio, but keep first if both equal.
       */
      if (!prev || (!Number(prev.focal_px || 0) && Number(item.focal_px || 0))) {
        byIdx.set(idx, item);
      }
    });

    return Array.from(byIdx.values())
      .sort((a, b) => Number(a.profile_idx) - Number(b.profile_idx));
  }

'''

if "function normalizeZoomMasterPoints(master)" not in s:
    if helper_anchor not in s:
        raise SystemExit("ERROR: makeFallbackZoomSamples anchor not found")
    s = s.replace(helper_anchor, helper + helper_anchor, 1)
    print("OK: inserted normalizeZoomMasterPoints")
else:
    print("SKIP: normalizeZoomMasterPoints already exists")

old_init = '''      const mp = await apiGetJson('/api/zoom/master_profile');
      if (Array.isArray(mp?.profile_points)) {
        points = mp.profile_points
          .map((p, i) => ({
            profile_idx: Number(p.profile_idx ?? i),
            zoom_ratio: Number(p.zoom_ratio ?? 0),
            focal_px: Number(p.focal_px ?? 0),
            fallback: false
          }))
          .filter(p => Number.isFinite(p.profile_idx))
          .sort((a, b) => Number(a.profile_idx) - Number(b.profile_idx));
      }
'''

new_init = '''      const mp = await apiGetJson('/api/zoom/master_profile');
      points = normalizeZoomMasterPoints(mp);
'''

if old_init in s:
    s = s.replace(old_init, new_init, 1)
    print("OK: initPtzSpeedTuneSamples now uses profile_points OR points + dedupe")
elif "points = normalizeZoomMasterPoints(mp);" in s:
    print("SKIP: init already patched")
else:
    raise SystemExit("ERROR: initPtzSpeedTuneSamples master block not found")

old_build = '''    const master = await apiGetJson('/api/zoom/master_profile');
    const points = Array.isArray(master.profile_points) ? master.profile_points : (Array.isArray(master.points) ? master.points : []);
    let saved = 0;
'''

new_build = '''    const master = await apiGetJson('/api/zoom/master_profile');
    const points = normalizeZoomMasterPoints(master);

    if (!points.length) {
      if ($('ptzTuneStatus')) $('ptzTuneStatus').textContent = 'No zoom master points found';
      ptzLog('PTZ SPEED FROM ZOOM MASTER no points', { master });
      return { ok: false, error: 'no_zoom_master_points' };
    }

    let saved = 0;
'''

if old_build in s:
    s = s.replace(old_build, new_build, 1)
    print("OK: buildPtzSpeedProfileFromZoomMaster uses normalized points")
elif "no_zoom_master_points" in s:
    print("SKIP: build already patched")
else:
    raise SystemExit("ERROR: buildPtzSpeedProfileFromZoomMaster master block not found")

# Improve status after save
old_status = '''    if ($('ptzTuneStatus')) $('ptzTuneStatus').textContent = `PTZ speed profile created: points=${saved}`;
    ptzLog(`PTZ speed profile created: points=${saved}`, { ok: true, points: saved });
    return { ok: true, points: saved };
'''

new_status = '''    if ($('ptzTuneStatus')) $('ptzTuneStatus').textContent = `PTZ speed profile created: saved=${saved} master=${points.length}`;
    ptzLog(`PTZ speed profile created: saved=${saved} master=${points.length}`, { ok: true, points: saved, master_points: points.length });
    return { ok: true, points: saved, master_points: points.length };
'''

if old_status in s:
    s = s.replace(old_status, new_status, 1)
    print("OK: improved build status")
elif "master_points: points.length" in s:
    print("SKIP: status already patched")
else:
    print("WARN: status block not found; skipped")

p.write_text(s, encoding="utf-8")
print("DONE")
