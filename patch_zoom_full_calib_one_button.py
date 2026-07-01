from pathlib import Path
import shutil, time

p = Path("web/index.html")
s = p.read_text(encoding="utf-8")

backup = p.with_suffix(f".html.bak_zoom_full_calib_one_button_{int(time.time())}")
shutil.copy2(p, backup)
print("Backup:", backup)

# 1. Add button into ZOOM CALIB menu after BUILD MASTER PROFILE.
old_btns = '''        <button id="zoomRunAprilTagCalibBtn" class="small-btn">RUN APRILTAG ZOOM CALIB</button>
        <button id="zoomBuildMasterProfileBtn" class="small-btn">BUILD MASTER PROFILE</button>
        <button id="apriltagTestBtn" class="small-btn">TEST APRILTAG</button>
'''

new_btns = '''        <button id="zoomRunFullCalibBtn" class="small-btn" title="Clear old profiles, run all enabled anchors, build master profile, create PTZ speed profile">RUN FULL CALIB</button>
        <button id="zoomRunAprilTagCalibBtn" class="small-btn">RUN APRILTAG ZOOM CALIB</button>
        <button id="zoomBuildMasterProfileBtn" class="small-btn">BUILD MASTER PROFILE</button>
        <button id="apriltagTestBtn" class="small-btn">TEST APRILTAG</button>
'''

if 'id="zoomRunFullCalibBtn"' not in s:
    if old_btns not in s:
        raise SystemExit("ERROR: ZOOM CALIB button block not found")
    s = s.replace(old_btns, new_btns, 1)
    print("OK: added RUN FULL CALIB button")
else:
    print("SKIP: RUN FULL CALIB button already exists")

# 2. Add standalone controller at end.
block = r'''
<script>
// ZOOM_FULL_CALIB_ONE_BUTTON_START
(function () {
  if (window.__zoomFullCalibOneButtonInstalled) return;
  window.__zoomFullCalibOneButtonInstalled = true;

  const $ = (id) => document.getElementById(id);

  const anchorDefs = [
    { label: 'near', prefix: 'Near', defaultDistanceM: 1, defaultTagId: 7 },
    { label: 'mid',  prefix: 'Mid',  defaultDistanceM: 5, defaultTagId: -1 },
    { label: 'far',  prefix: 'Far',  defaultDistanceM: 10, defaultTagId: 12 }
  ];

  let fullCalibRunning = false;

  function setZoomStatus(txt) {
    const el = $('zoomAprilTagCalibStatus');
    if (el) el.textContent = txt;
    console.log('[zoom-full-calib]', txt);
  }

  function setPtzTuneStatus(txt) {
    const el = $('ptzTuneStatus');
    if (el) el.textContent = txt;
  }

  function sleep(ms) {
    return new Promise(resolve => setTimeout(resolve, ms));
  }

  async function getJson(url) {
    const r = await fetch(url + (url.includes('?') ? '&' : '?') + 'ts=' + Date.now(), { cache: 'no-store' });
    if (!r.ok) throw new Error(`GET ${url} HTTP ${r.status}`);
    return await r.json();
  }

  async function postJson(url, payload) {
    const r = await fetch(url, {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify(payload || {})
    });
    if (!r.ok) throw new Error(`POST ${url} HTTP ${r.status}`);
    return await r.json().catch(() => ({ ok: true }));
  }

  function num(id, fallback) {
    const v = Number($(id)?.value);
    return Number.isFinite(v) ? v : fallback;
  }

  function anchorInputId(def, field) {
    return `zoomAnchor${def.prefix}${field}`;
  }

  function readAnchor(def) {
    return {
      label: def.label,
      enabled: !!$(anchorInputId(def, 'Enabled'))?.checked,
      tag_id: Math.round(num(anchorInputId(def, 'TagId'), def.defaultTagId)),
      distance_mm: num(anchorInputId(def, 'DistanceM'), def.defaultDistanceM) * 1000,
      tag_size_mm: num(anchorInputId(def, 'SizeMm'), 160)
    };
  }

  function readAnchors() {
    return anchorDefs.map(readAnchor);
  }

  function getAnchor(label) {
    return readAnchors().find(a => a.label === label);
  }

  function enabledAnchors() {
    return readAnchors().filter(a =>
      a.enabled &&
      Number.isFinite(Number(a.tag_id)) &&
      Number(a.tag_id) >= 0 &&
      Number(a.distance_mm) > 0 &&
      Number(a.tag_size_mm) > 0
    );
  }

  function setActiveAnchorLabel(label) {
    const summary = $('zoomActiveAnchorSummary') || $('zoomCalibMenuSummary');
    if (summary) {
      const a = getAnchor(label);
      if (a) {
        summary.textContent = `active=${label} id=${a.tag_id} distance=${(a.distance_mm / 1000).toFixed(2)}m size=${a.tag_size_mm}mm`;
      }
    }

    document.querySelectorAll('.zoom-anchor-row').forEach(row => row.classList.remove('active'));
    const row = document.querySelector(`.zoom-anchor-row[data-anchor-label="${label}"]`) ||
      document.querySelector(`#zoomAnchor${label[0].toUpperCase() + label.slice(1)}TagId`)?.closest?.('.zoom-anchor-row');
    row?.classList.add('active');
  }

  function payloadForAnchor(anchor) {
    const anchors = readAnchors();

    return {
      mode: 'apriltag_zoom_table',

      tag_size_mm: Number(anchor.tag_size_mm || 160),
      anchor_tag_id: Number(anchor.tag_id),
      anchor_distance_mm: Number(anchor.distance_mm),

      samples: num('zoomSamples', 10),
      impulse_ms: num('zoomImpulseMs', 170),
      settle_ms: num('zoomSettleMs', 190),
      cmd_abs: num('zoomCmdAbs', 34),
      wide_cmd_sign: num('zoomWideSign', -1),
      wide_hold_ms: num('zoomWideHoldMs', 1500),
      calibration_direction: $('zoomCalibDirection')?.value || 'wide_to_tele',

      active_anchor_label: anchor.label,
      anchors
    };
  }

  async function saveSettingsForAnchor(anchor) {
    const payload = payloadForAnchor(anchor);
    await postJson('/api/zoom_calibration/settings', payload);
    return payload;
  }

  async function clearOldProfilesOnce() {
    setZoomStatus('clearing old calibration profiles...');
    try {
      const res = await postJson('/api/zoom_calibration/clear_profiles', {
        archive: true,
        reason: 'run_full_calib',
        ts: Date.now()
      });
      console.log('[zoom-full-calib] clear result', res);
      return res;
    } catch (e) {
      console.warn('[zoom-full-calib] clear_profiles failed/skipped', e);
      setZoomStatus('clear old profiles skipped; continuing...');
      return { ok: false, skipped: true, error: String(e.message || e) };
    }
  }

  async function waitCalibrationDone(label, timeoutMs = 180000) {
    const t0 = Date.now();
    let last = null;

    while (Date.now() - t0 < timeoutMs) {
      await sleep(1000);

      let st = null;
      try {
        st = await getJson('/api/zoom_calibration');
        last = st;
      } catch (e) {
        console.warn('[zoom-full-calib] status poll failed', e);
        continue;
      }

      const inProgress = !!(st.in_progress ?? st.running ?? st.busy);
      const idx = st.progress_idx ?? st.idx ?? st.current_idx ?? st.progress?.idx ?? '?';
      const total = st.progress_samples ?? st.samples ?? st.progress?.samples ?? '?';
      const tags = st.tags_found ?? st.progress_tags_found ?? st.progress?.tags_found ?? '?';
      const msg = st.message || st.last_message || st.status || '';

      setZoomStatus(`RUN ${label}: ${inProgress ? 'running' : 'done'} idx=${idx}/${total} tags=${tags} ${msg}`);

      if (window.ZoomCalibUi?.testAprilTag) {
        try { await window.ZoomCalibUi.testAprilTag(); } catch (_) {}
      } else if (window.zoomCalibHardTestAprilTag) {
        try { await window.zoomCalibHardTestAprilTag(); } catch (_) {}
      }

      if (!inProgress) return st;
    }

    throw new Error(`calibration timeout for ${label}; last=${JSON.stringify(last)}`);
  }

  async function runOneAnchor(anchor) {
    setActiveAnchorLabel(anchor.label);

    setZoomStatus(`RUN ${anchor.label}: saving settings id=${anchor.tag_id} distance=${(anchor.distance_mm / 1000).toFixed(2)}m...`);
    const payload = await saveSettingsForAnchor(anchor);

    setZoomStatus(`RUN ${anchor.label}: starting zoom calibration...`);
    const res = await postJson('/api/zoom_calibration', payload);
    console.log('[zoom-full-calib] run result', anchor.label, res);

    if (res.ok === false || res.started === false) {
      throw new Error(`RUN ${anchor.label} failed: ${JSON.stringify(res)}`);
    }

    return await waitCalibrationDone(anchor.label);
  }

  async function buildMasterProfile() {
    setZoomStatus('BUILD MASTER PROFILE...');
    const res = await postJson('/api/zoom_calibration/build_master_profile', {});
    console.log('[zoom-full-calib] build master result', res);

    const raw = res.points_total_raw ?? res.raw ?? '?';
    const clean = res.points_total_clean ?? res.clean ?? res.points ?? '?';

    if (res.ok === false) {
      setZoomStatus(`MASTER failed: ${res.error || JSON.stringify(res)}`);
      throw new Error(`MASTER failed: ${JSON.stringify(res)}`);
    }

    setZoomStatus(`MASTER ok raw=${raw} clean=${clean}`);

    if (window.ZoomCalibUi?.load) {
      try { await window.ZoomCalibUi.load(); } catch (_) {}
    }

    return res;
  }

  async function buildPtzSpeedFromZoomMaster() {
    setZoomStatus('BUILD PTZ SPEED FROM ZOOM MASTER...');
    setPtzTuneStatus('building PTZ speed from zoom master...');

    let res;
    if (typeof window.buildPtzSpeedProfileFromZoomMaster === 'function') {
      res = await window.buildPtzSpeedProfileFromZoomMaster();
    } else {
      throw new Error('buildPtzSpeedProfileFromZoomMaster is not available');
    }

    console.log('[zoom-full-calib] PTZ speed result', res);

    if (window.refreshPtzTuneSpeedProfile) {
      try { await window.refreshPtzTuneSpeedProfile(); } catch (_) {}
    }

    if (window.initPtzSpeedTuneSamples) {
      try { await window.initPtzSpeedTuneSamples(); } catch (_) {}
    }

    if (window.updatePtzTuneSampleButtonUi) {
      try { window.updatePtzTuneSampleButtonUi(); } catch (_) {}
    }

    const n = res?.points ?? res?.saved ?? res?.master_points ?? '?';
    setPtzTuneStatus(`PTZ speed profile created: points=${n}`);
    setZoomStatus(`FULL CALIB DONE: master built, PTZ speed points=${n}`);

    return res;
  }

  function setFullCalibButtonsDisabled(disabled) {
    [
      'zoomRunFullCalibBtn',
      'zoomRunAprilTagCalibBtn',
      'zoomBuildMasterProfileBtn',
      'apriltagTestBtn',
      'zoomRunNearBtn',
      'zoomRunMidBtn',
      'zoomRunFarBtn',
      'zoomSelectNearBtn',
      'zoomSelectMidBtn',
      'zoomSelectFarBtn',
      'ptzTuneBuildFromZoomMasterBtn'
    ].forEach(id => {
      const el = $(id);
      if (el) el.disabled = !!disabled;
    });
  }

  async function runFullCalibration() {
    if (fullCalibRunning) return;
    fullCalibRunning = true;
    setFullCalibButtonsDisabled(true);

    try {
      const anchors = enabledAnchors();

      if (!anchors.length) {
        throw new Error('No enabled anchors with valid tag_id/distance');
      }

      const names = anchors.map(a => `${a.label}:id=${a.tag_id}@${(a.distance_mm / 1000).toFixed(2)}m`).join(', ');
      setZoomStatus(`FULL CALIB: anchors=${names}`);

      await clearOldProfilesOnce();

      for (const anchor of anchors) {
        await runOneAnchor(anchor);
        await sleep(500);
      }

      await buildMasterProfile();
      await buildPtzSpeedFromZoomMaster();

      setZoomStatus(`FULL CALIB DONE: ${anchors.length} anchors → master → PTZ speed profile`);
    } catch (e) {
      console.error('[zoom-full-calib] failed', e);
      setZoomStatus(`FULL CALIB ERROR: ${String(e.message || e)}`);
    } finally {
      fullCalibRunning = false;
      setFullCalibButtonsDisabled(false);
    }
  }

  function install() {
    const btn = $('zoomRunFullCalibBtn');
    if (!btn || btn.__zoomFullCalibBound) return;

    btn.__zoomFullCalibBound = true;
    btn.addEventListener('click', (e) => {
      e.preventDefault();
      e.stopPropagation();
      e.stopImmediatePropagation();

      const anchors = enabledAnchors();
      const msg = [
        `Run full calibration for ${anchors.length} anchors?`,
        '',
        ...anchors.map(a => `${a.label}: id=${a.tag_id}, distance=${(a.distance_mm / 1000).toFixed(2)}m`),
        '',
        'This will archive old zoom calibration profiles, build master profile, and generate PTZ speed profile.'
      ].join('\n');

      if (!window.confirm(msg)) return;
      runFullCalibration();
    }, true);

    window.runZoomFullCalibration = runFullCalibration;
    console.log('[zoom-full-calib] installed');
  }

  if (document.readyState === 'loading') {
    document.addEventListener('DOMContentLoaded', install, { once: true });
  } else {
    install();
  }
})();
// ZOOM_FULL_CALIB_ONE_BUTTON_END
</script>
'''

if "ZOOM_FULL_CALIB_ONE_BUTTON_START" not in s:
    if "</body>" in s:
        s = s.replace("</body>", block + "\n</body>", 1)
    elif "</html>" in s:
        s = s.replace("</html>", block + "\n</html>", 1)
    else:
        s += "\n" + block + "\n"
    print("OK: inserted full calib controller")
else:
    print("SKIP: full calib controller already exists")

p.write_text(s, encoding="utf-8")
print("DONE")
