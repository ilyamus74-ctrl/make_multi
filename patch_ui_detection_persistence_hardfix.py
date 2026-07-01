from pathlib import Path
import time

p = Path("web/index.html")
s = p.read_text(encoding="utf-8")

bak = p.with_suffix(p.suffix + f".bak_detection_persistence_hardfix_{int(time.time())}")
bak.write_text(s, encoding="utf-8")

late_script = r'''
<script>
// DETECTION_PERSISTENCE_HARDFIX_START
(function () {
  const $ = (id) => document.getElementById(id);

  async function getJson(url) {
    const r = await fetch(url, { cache: 'no-store' });
    if (!r.ok) throw new Error(`${r.status} ${url}`);
    return await r.json();
  }

  async function postJson(url, body) {
    const r = await fetch(url, {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify(body || {})
    });
    if (!r.ok) throw new Error(`${r.status} ${url}`);
    return await r.json().catch(() => ({ ok: true }));
  }

  async function readSettings() {
    try {
      const cfg = await getJson('/api/settings');
      return cfg && typeof cfg === 'object' ? cfg : {};
    } catch (_) {
      return {};
    }
  }

  async function saveSettingsMerge(patch) {
    const cfg = await readSettings();
    const next = Object.assign({}, cfg, patch || {});
    next.config_version = Math.max(16, Number(next.config_version || 16));

    try {
      localStorage.setItem('pantilt_ui_settings_v2', JSON.stringify(next));
    } catch (_) {}

    await postJson('/api/settings', next);
    return next;
  }

  function shortModelName(v) {
    return String(v || '').split('/').filter(Boolean).pop() || String(v || '');
  }

  function readSelectedClassesFromUi() {
    return Array.from(document.querySelectorAll('#detectionClassGrid input[type="checkbox"]:checked'))
      .map(cb => Number(cb.dataset.classId || cb.value))
      .filter(Number.isFinite);
  }

  function setSelectedClassesInUi(ids) {
    const wanted = new Set((ids || []).map(Number));
    document.querySelectorAll('#detectionClassGrid input[type="checkbox"]').forEach(cb => {
      cb.checked = wanted.has(Number(cb.dataset.classId || cb.value));
    });
  }

  function classText(ids) {
    const arr = Array.isArray(ids) ? ids.map(Number).filter(Number.isFinite) : [];
    if (!arr.length) return 'all';
    return arr.join(',');
  }

  function renderDetectEnabled(enabled) {
    enabled = !!enabled;

    const btn = $('detectionToggleBtn');
    if (btn) {
      btn.textContent = enabled ? 'DETECTION: ON' : 'DETECTION: OFF';
      btn.classList.toggle('detect-on', enabled);
      btn.classList.toggle('detect-off', !enabled);
    }

    if ($('detectionEnabledState')) {
      $('detectionEnabledState').textContent = enabled ? 'detect: ON' : 'detect: OFF';
    }

    if ($('detectionMenuSummary')) {
      $('detectionMenuSummary').textContent = enabled ? 'DETECTION ON' : 'DETECTION OFF | stream only';
    }
  }

  function renderLimit(lim) {
    const maxDet = Number(lim?.max_detections || lim?.maxDetections || 10);
    const maxRaw = Number(lim?.max_raw_candidates || lim?.maxRawCandidates || Math.max(20, maxDet * 5));

    if ($('operatorDetectionLimit')) {
      $('operatorDetectionLimit').value = String(maxDet);
    }

    if ($('operatorLimitState')) {
      $('operatorLimitState').textContent = `limit: ${maxDet}/${maxRaw}`;
    }
  }

  function renderThrottle(th) {
    const n = Number(th?.detect_every_n_frames || th?.detectEveryNFrames || 1);

    if ($('operatorDetectEvery')) {
      $('operatorDetectEvery').value = String(n);
    }

    if ($('operatorDetectEveryState')) {
      $('operatorDetectEveryState').textContent = `detect: 1/${n}`;
    }
  }

  function renderArea(roi) {
    const mode = String(roi?.detection_mode || roi?.mode || 'full_frame');

    if ($('operatorDetectionAreaMode')) {
      $('operatorDetectionAreaMode').value = mode;
    }

    const count = Array.isArray(roi?.rois) ? roi.rois.length : 0;

    if ($('operatorDetectionAreaState')) {
      $('operatorDetectionAreaState').textContent = `area: ${mode} rois=${count}`;
    }
  }

  function renderClasses(ids) {
    const txt = classText(ids);

    if ($('detectionClassesState')) {
      $('detectionClassesState').textContent = `classes: ${txt}`;
    }
  }

  function renderModel(cfg) {
    const current = String(cfg?.current_model || cfg?.current || '').trim();

    if ($('operatorModelState')) {
      $('operatorModelState').textContent = current ? `model: ${shortModelName(current)}` : 'model: --';
    }
  }

  async function refreshDetectionUiHard() {
    let det = {};
    let lim = {};
    let th = {};
    let roi = {};

    try { det = await getJson('/api/detector/config'); } catch (e) { console.warn('[det-hardfix] detector config failed', e); }
    try { lim = await getJson('/api/detection/limits'); } catch (e) { console.warn('[det-hardfix] limits failed', e); }
    try { th = await getJson('/api/detection/throttle'); } catch (e) { console.warn('[det-hardfix] throttle failed', e); }
    try { roi = await getJson('/api/detection/roi_config'); } catch (e) { console.warn('[det-hardfix] roi failed', e); }

    const enabled = !!det.detect_enabled;
    const selected = Array.isArray(det.selected_classes)
      ? det.selected_classes.map(Number).filter(Number.isFinite)
      : [];

    renderDetectEnabled(enabled);
    renderModel(det);
    renderLimit(lim);
    renderThrottle(th);
    renderArea(roi);
    setSelectedClassesInUi(selected);
    renderClasses(selected);

    await saveSettingsMerge({
      detectorEnabled: enabled,
      detectorSelectedClasses: selected,
      operatorModel: String(det.current_model || det.current || ''),
      operatorDetectionLimit: Number(lim.max_detections || 10),
      operatorDetectEvery: Number(th.detect_every_n_frames || 1),
      operatorDetectionAreaMode: String(roi.detection_mode || 'full_frame')
    }).catch(() => {});

    return { det, lim, th, roi };
  }

  async function frameSize() {
    try {
      const d = await getJson('/api/detections');
      return {
        w: Math.max(1, Number(d.width || 1920)),
        h: Math.max(1, Number(d.height || 1080))
      };
    } catch (_) {
      return { w: 1920, h: 1080 };
    }
  }

  function skyRoi(w, h) {
    return {
      id: 'sky_top',
      enabled: true,
      x: 0,
      y: 0,
      w: w,
      h: Math.max(1, Math.round(h * 0.68)),
      every_n_frames: 1,
      classes: []
    };
  }

  function centerRoi(w, h) {
    const rw = Math.round(w * 0.60);
    const rh = Math.round(h * 0.55);
    return {
      id: 'center',
      enabled: true,
      x: Math.max(0, Math.round((w - rw) / 2)),
      y: Math.max(0, Math.round((h - rh) / 2)),
      w: Math.max(1, rw),
      h: Math.max(1, rh),
      every_n_frames: 1,
      classes: []
    };
  }

  function fullRoi(w, h) {
    return {
      id: 'full_frame',
      enabled: true,
      x: 0,
      y: 0,
      w: w,
      h: h,
      every_n_frames: 1,
      classes: []
    };
  }

  async function buildAreaPayload(mode) {
    const sz = await frameSize();
    const w = sz.w;
    const h = sz.h;

    mode = String(mode || 'full_frame');

    if (mode === 'full_frame') {
      return { detection_mode: 'full_frame', rois: [] };
    }

    if (mode === 'tiled') {
      return { detection_mode: 'tiled', rois: [] };
    }

    if (mode === 'roi') {
      return { detection_mode: 'roi', rois: [skyRoi(w, h)] };
    }

    if (mode === 'multi_roi') {
      return { detection_mode: 'multi_roi', rois: [skyRoi(w, h), centerRoi(w, h)] };
    }

    if (mode === 'hybrid') {
      return { detection_mode: 'hybrid', rois: [fullRoi(w, h), skyRoi(w, h)] };
    }

    return { detection_mode: 'full_frame', rois: [] };
  }

  async function applyLimitHard() {
    const maxDet = Number($('operatorDetectionLimit')?.value || 10);
    const maxRaw = Math.max(20, maxDet * 5);

    const res = await postJson('/api/detection/limits', {
      max_detections: maxDet,
      max_raw_candidates: maxRaw
    });

    renderLimit(res);

    await saveSettingsMerge({
      operatorDetectionLimit: Number(res.max_detections || maxDet)
    });

    return res;
  }

  async function applyThrottleHard() {
    const n = Number($('operatorDetectEvery')?.value || 1);

    const res = await postJson('/api/detection/throttle', {
      detect_every_n_frames: n
    });

    renderThrottle(res);

    await saveSettingsMerge({
      operatorDetectEvery: Number(res.detect_every_n_frames || n)
    });

    return res;
  }

  async function applyAreaHard() {
    const mode = String($('operatorDetectionAreaMode')?.value || 'full_frame');
    const payload = await buildAreaPayload(mode);

    const res = await postJson('/api/detection/roi_config', payload);

    renderArea(res.detection_mode ? res : payload);

    await saveSettingsMerge({
      operatorDetectionAreaMode: payload.detection_mode
    });

    return res;
  }

  async function toggleDetectionHard() {
    const cfg = await getJson('/api/detector/config');
    const selected = readSelectedClassesFromUi();

    const res = await postJson('/api/detector/config', {
      detect_enabled: !Boolean(cfg.detect_enabled),
      selected_classes: selected
    });

    const enabled = !!res.detect_enabled;
    renderDetectEnabled(enabled);
    setSelectedClassesInUi(Array.isArray(res.selected_classes) ? res.selected_classes : selected);
    renderClasses(Array.isArray(res.selected_classes) ? res.selected_classes : selected);

    await saveSettingsMerge({
      detectorEnabled: enabled,
      detectorSelectedClasses: Array.isArray(res.selected_classes) ? res.selected_classes : selected
    });

    return res;
  }

  async function applyClassesHard(selected) {
    const cfg = await getJson('/api/detector/config');

    if (!Array.isArray(selected)) {
      selected = readSelectedClassesFromUi();
    }

    const res = await postJson('/api/detector/config', {
      detect_enabled: Boolean(cfg.detect_enabled),
      selected_classes: selected
    });

    setSelectedClassesInUi(Array.isArray(res.selected_classes) ? res.selected_classes : selected);
    renderClasses(Array.isArray(res.selected_classes) ? res.selected_classes : selected);

    await saveSettingsMerge({
      detectorEnabled: Boolean(res.detect_enabled),
      detectorSelectedClasses: Array.isArray(res.selected_classes) ? res.selected_classes : selected
    });

    return res;
  }

  function bindClick(id, fn) {
    const el = $(id);
    if (!el || el.__detHardBound) return;

    el.__detHardBound = true;
    el.addEventListener('click', async (e) => {
      e.preventDefault();
      e.stopPropagation();
      e.stopImmediatePropagation();

      try {
        const res = await fn();
        console.log('[det-hardfix]', id, res);
        await refreshDetectionUiHard();
      } catch (err) {
        console.error('[det-hardfix]', id, err);
        if ($('detectionEnabledState')) {
          $('detectionEnabledState').textContent = `detect error: ${String(err.message || err)}`;
        }
      }
    }, true);
  }

  function bindChangeSave(id) {
    const el = $(id);
    if (!el || el.__detHardChangeBound) return;

    el.__detHardChangeBound = true;
    el.addEventListener('change', async () => {
      try {
        if (id === 'operatorDetectionLimit') await applyLimitHard();
        else if (id === 'operatorDetectEvery') await applyThrottleHard();
        else if (id === 'operatorDetectionAreaMode') await applyAreaHard();
        else await refreshDetectionUiHard();
      } catch (err) {
        console.error('[det-hardfix-change]', id, err);
      }
    }, true);
  }

  function installDetectionHardfix() {
    bindClick('operatorLimitApplyBtn', applyLimitHard);
    bindClick('operatorDetectEveryApplyBtn', applyThrottleHard);
    bindClick('operatorDetectionAreaApplyBtn', applyAreaHard);
    bindClick('detectionToggleBtn', toggleDetectionHard);
    bindClick('detectionApplyClassesBtn', () => applyClassesHard());
    bindClick('detectionPresetBirdPlaneBtn', () => {
      setSelectedClassesInUi([4, 14]);
      return applyClassesHard([4, 14]);
    });
    bindClick('detectionAllClassesBtn', () => {
      setSelectedClassesInUi([]);
      return applyClassesHard([]);
    });

    bindChangeSave('operatorDetectionLimit');
    bindChangeSave('operatorDetectEvery');
    bindChangeSave('operatorDetectionAreaMode');

    document.querySelectorAll('#detectionClassGrid input[type="checkbox"]').forEach(cb => {
      if (cb.__detHardClassBound) return;
      cb.__detHardClassBound = true;
      cb.addEventListener('change', () => {
        renderClasses(readSelectedClassesFromUi());
        saveSettingsMerge({
          detectorSelectedClasses: readSelectedClassesFromUi()
        }).catch(() => {});
      }, true);
    });
  }

  async function boot() {
    installDetectionHardfix();
    await refreshDetectionUiHard();
  }

  if (document.readyState === 'loading') {
    document.addEventListener('DOMContentLoaded', boot, { once: true });
  } else {
    boot();
  }

  setTimeout(boot, 500);
  setTimeout(boot, 1500);
  setTimeout(refreshDetectionUiHard, 3000);

  window.refreshDetectionUiHard = refreshDetectionUiHard;
})();
// DETECTION_PERSISTENCE_HARDFIX_END
</script>
'''

if "DETECTION_PERSISTENCE_HARDFIX_START" not in s:
    if "</body>" not in s:
        raise SystemExit("ERROR: </body> not found")
    s = s.replace("</body>", late_script + "\n</body>", 1)
    p.write_text(s, encoding="utf-8")
    print(f"OK patched {p}, backup={bak}")
else:
    print("OK detection persistence hardfix already installed")
