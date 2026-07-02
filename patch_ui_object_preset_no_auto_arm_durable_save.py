from pathlib import Path
import time

p = Path("web/index.html")
s = p.read_text(encoding="utf-8", errors="ignore")

bak = p.with_suffix(p.suffix + f".bak_no_auto_arm_durable_preset_{int(time.time())}")
bak.write_text(s, encoding="utf-8")

script = r'''
<script>
// OBJECT_PRESET_NO_AUTO_ARM_DURABLE_SAVE_START
(function () {
  const $ = (id) => document.getElementById(id);

  const LOCAL_BACKUP_KEY = 'ptz_object_presets_custom_v2';

  const BUILTIN = {
    person_single: {
      label: 'ЧЕЛОВЕК',
      classes: [0],
      detection_mode: 'full_frame',
      max_detections: 5,
      max_raw_candidates: 25,
      detect_every_n_frames: 1,
      tracking_mode: 'single_auto',
      loss_behavior: 'continuous_wide_scan_x',
      ptz: {
        target_x: 0.50,
        target_y: 0.46,
        auto_zoom_enable: true,
        auto_zoom_target_h: 0.68,
        auto_zoom_deadzone: 0.08,
        auto_zoom_cmd: 10,
        auto_zoom_sign: 1,
        auto_zoom_period_ms: 350
      }
    },
    people: {
      label: 'ЛЮДИ',
      classes: [0],
      detection_mode: 'full_frame',
      max_detections: 10,
      max_raw_candidates: 50,
      detect_every_n_frames: 1,
      tracking_mode: 'multi_operator',
      loss_behavior: 'operator_select'
    },
    car_single: {
      label: 'МАШИНА',
      classes: [2, 3, 5, 7],
      detection_mode: 'full_frame',
      max_detections: 8,
      max_raw_candidates: 40,
      detect_every_n_frames: 1,
      tracking_mode: 'single_auto',
      loss_behavior: 'continuous_wide_scan_x',
      ptz: {
        target_x: 0.50,
        target_y: 0.50,
        auto_zoom_enable: true,
        auto_zoom_target_h: 0.48,
        auto_zoom_deadzone: 0.10,
        auto_zoom_cmd: 8,
        auto_zoom_sign: 1,
        auto_zoom_period_ms: 450
      }
    },
    cars: {
      label: 'МАШИНЫ',
      classes: [2, 3, 5, 7],
      detection_mode: 'tiled',
      max_detections: 15,
      max_raw_candidates: 80,
      detect_every_n_frames: 1,
      tracking_mode: 'multi_operator',
      loss_behavior: 'operator_select'
    },
    airplane_single: {
      label: 'САМОЛЁТ',
      classes: [4],
      detection_mode: 'hybrid',
      max_detections: 8,
      max_raw_candidates: 60,
      detect_every_n_frames: 1,
      tracking_mode: 'single_auto',
      loss_behavior: 'continuous_wide_scan_x'
    },
    airplanes: {
      label: 'САМОЛЁТЫ',
      classes: [4],
      detection_mode: 'hybrid',
      max_detections: 15,
      max_raw_candidates: 100,
      detect_every_n_frames: 1,
      tracking_mode: 'multi_operator',
      loss_behavior: 'operator_select'
    },
    bird_single: {
      label: 'ПТИЦА',
      classes: [14],
      detection_mode: 'hybrid',
      max_detections: 10,
      max_raw_candidates: 80,
      detect_every_n_frames: 1,
      tracking_mode: 'single_auto',
      loss_behavior: 'continuous_wide_scan_x'
    },
    birds: {
      label: 'ПТИЦЫ',
      classes: [14],
      detection_mode: 'hybrid',
      max_detections: 20,
      max_raw_candidates: 120,
      detect_every_n_frames: 1,
      tracking_mode: 'multi_operator',
      loss_behavior: 'operator_select'
    }
  };

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

  function autopilotBaseUrl() {
    return `${location.protocol}//${location.hostname}:8090`;
  }

  async function readSettings() {
    try {
      const x = await getJson('/api/settings');
      return x && typeof x === 'object' ? x : {};
    } catch (_) {
      return {};
    }
  }

  async function writeSettings(x) {
    const next = Object.assign({}, x || {});
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

  function activePresetName(settings) {
    return String(
      settings?.lastAppliedObjectPreset?.name ||
      settings?.activeObjectPreset ||
      'person_single'
    );
  }

  function loadLocalCustomBackup() {
    try {
      const raw = localStorage.getItem(LOCAL_BACKUP_KEY);
      if (!raw) return {};
      const obj = JSON.parse(raw);
      return obj && typeof obj === 'object' ? obj : {};
    } catch (_) {
      return {};
    }
  }

  function saveLocalCustomBackup(custom) {
    try {
      localStorage.setItem(LOCAL_BACKUP_KEY, JSON.stringify(custom || {}));
    } catch (_) {}
  }

  async function restoreCustomFromLocalIfMissing() {
    const settings = await readSettings();

    if (settings.objectPresetsCustom && Object.keys(settings.objectPresetsCustom).length) {
      saveLocalCustomBackup(settings.objectPresetsCustom);
      return settings;
    }

    const backup = loadLocalCustomBackup();

    if (!backup || !Object.keys(backup).length) {
      return settings;
    }

    settings.objectPresetsCustom = backup;
    await writeSettings(settings);

    return settings;
  }

  function allPresets(settings) {
    const custom = settings.objectPresetsCustom || {};
    return Object.assign({}, BUILTIN, custom);
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

  async function roiPayload(mode) {
    const sz = await frameSize();
    const w = sz.w;
    const h = sz.h;

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

  function uiLimit(settings, preset) {
    return Number(
      $('operatorDetectionLimit')?.value ||
      settings.operatorDetectionLimit ||
      preset.max_detections ||
      10
    );
  }

  function uiEvery(settings, preset) {
    return Number(
      $('operatorDetectEvery')?.value ||
      settings.operatorDetectEvery ||
      preset.detect_every_n_frames ||
      1
    );
  }

  function uiArea(settings, preset) {
    return String(
      $('operatorDetectionAreaMode')?.value ||
      settings.operatorDetectionAreaMode ||
      preset.detection_mode ||
      'full_frame'
    );
  }

  function uiModel(settings, preset) {
    return String(
      $('operatorModelSelect')?.value ||
      settings.operatorModel ||
      preset.model ||
      preset.operatorModel ||
      ''
    ).trim();
  }

  function currentPresetFromUi(settings, active) {
    const presets = allPresets(settings);
    const old = Object.assign({}, BUILTIN[active] || {}, presets[active] || {});
    const maxDet = uiLimit(settings, old);
    const every = uiEvery(settings, old);
    const area = uiArea(settings, old);
    const model = uiModel(settings, old);

    const ptz = Object.assign(
      {},
      old.ptz || {},
      settings.ptzAutoZoom || {},
      settings.ptzConfig || {}
    );

    return Object.assign({}, old, {
      label: old.label || active,
      model: model,
      operatorModel: model,
      classes: Array.isArray(old.classes) ? old.classes.map(Number).filter(Number.isFinite) : [],
      detection_mode: area,
      max_detections: maxDet,
      max_raw_candidates: Math.max(20, Number(old.max_raw_candidates || maxDet * 5)),
      detect_every_n_frames: every,
      tracking_mode: old.tracking_mode || settings.objectPresetTrackingMode || 'single_auto',
      loss_behavior: old.loss_behavior || settings.objectPresetLossBehavior || 'continuous_wide_scan_x',
      ptz: ptz
    });
  }

  async function applyDetectionBackend(preset) {
    const model = String(preset.model || preset.operatorModel || '').trim();
    const classes = Array.isArray(preset.classes) ? preset.classes.map(Number).filter(Number.isFinite) : [];
    const maxDet = Number(preset.max_detections || 10);
    const maxRaw = Number(preset.max_raw_candidates || Math.max(20, maxDet * 5));
    const every = Number(preset.detect_every_n_frames || 1);
    const area = String(preset.detection_mode || 'full_frame');

    const detPayload = {
      detect_enabled: true,
      selected_classes: classes
    };

    if (model) {
      detPayload.current_model = model;
    }

    const det = await postJson('/api/detector/config', detPayload);
    const limits = await postJson('/api/detection/limits', {
      max_detections: maxDet,
      max_raw_candidates: maxRaw
    });
    const throttle = await postJson('/api/detection/throttle', {
      detect_every_n_frames: every
    });
    const roi = await postJson('/api/detection/roi_config', await roiPayload(area));

    return {
      detector: det,
      limits: limits,
      throttle: throttle,
      roi: roi
    };
  }

  async function applyPtzFramingOnly(preset) {
    const ptz = Object.assign({}, preset.ptz || {});
    const allowed = [
      'target_x',
      'target_y',
      'auto_zoom_enable',
      'auto_zoom_target_h',
      'auto_zoom_deadzone',
      'auto_zoom_cmd',
      'auto_zoom_sign',
      'auto_zoom_period_ms'
    ];

    const body = {};

    for (const key of allowed) {
      if (ptz[key] !== undefined) body[key] = ptz[key];
    }

    if (!Object.keys(body).length) return {};

    try {
      return await postJson(`${autopilotBaseUrl()}/api/autopilot/config`, body);
    } catch (e) {
      console.warn('[object-preset-durable] ptz framing failed', e);
      return { ok: false, error: String(e.message || e) };
    }
  }

  function renderActivePreset(name) {
    document.querySelectorAll('[data-object-preset]').forEach((btn) => {
      btn.classList.toggle('active', btn.dataset.objectPreset === name);
    });

    const state = $('objectPresetState');

    if (state) {
      state.textContent = `preset: ${name || '--'}`;
    }

    if (typeof window.paintActivePreset === 'function') {
      try { window.paintActivePreset(name); } catch (_) {}
    }
  }

  async function stopPtzBecausePresetSelection() {
    try {
      await postJson(`${autopilotBaseUrl()}/api/autopilot/stop`, {});
    } catch (_) {}

    try {
      await postJson(`${autopilotBaseUrl()}/api/control/stop`, {});
    } catch (_) {}
  }

  async function selectObjectPresetNoArm(name) {
    let settings = await restoreCustomFromLocalIfMissing();
    const presets = allPresets(settings);
    const preset = Object.assign({}, BUILTIN[name] || {}, presets[name] || {});

    if (!preset || !Object.keys(preset).length) {
      throw new Error(`preset not found: ${name}`);
    }

    await stopPtzBecausePresetSelection();

    const backend = await applyDetectionBackend(preset);
    const ptz = await applyPtzFramingOnly(preset);

    settings = await readSettings();
    const custom = Object.assign({}, settings.objectPresetsCustom || {});

    settings.activeObjectPreset = name;
    settings.ptzArmed = false;
    settings.detectorEnabled = true;
    settings.detectorSelectedClasses = preset.classes || [];
    settings.operatorModel = preset.model || preset.operatorModel || settings.operatorModel || '';
    settings.operatorDetectionLimit = Number(preset.max_detections || 10);
    settings.operatorDetectEvery = Number(preset.detect_every_n_frames || 1);
    settings.operatorDetectionAreaMode = String(preset.detection_mode || 'full_frame');
    settings.objectPresetTrackingMode = preset.tracking_mode || 'single_auto';
    settings.objectPresetLossBehavior = preset.loss_behavior || 'continuous_wide_scan_x';
    settings.lastAppliedObjectPreset = {
      name: name,
      label: preset.label || name,
      tracking_mode: settings.objectPresetTrackingMode,
      loss_behavior: settings.objectPresetLossBehavior,
      ts: Math.floor(Date.now() / 1000)
    };

    settings.objectPresetsCustom = custom;

    await writeSettings(settings);
    saveLocalCustomBackup(custom);

    if ($('operatorDetectionLimit')) $('operatorDetectionLimit').value = String(settings.operatorDetectionLimit);
    if ($('operatorDetectEvery')) $('operatorDetectEvery').value = String(settings.operatorDetectEvery);
    if ($('operatorDetectionAreaMode')) $('operatorDetectionAreaMode').value = String(settings.operatorDetectionAreaMode);

    renderActivePreset(name);

    const line = $('ptzStateLine');

    if (line) {
      line.textContent =
        `preset selected, PTZ not armed: ${name} limit=${settings.operatorDetectionLimit} every=${settings.operatorDetectEvery}`;
    }

    if (typeof window.syncPtzRunStateHard === 'function') {
      try { await window.syncPtzRunStateHard(); } catch (_) {}
    }

    console.log('[object-preset-no-arm]', { name, preset, backend, ptz });

    return { name, preset, backend, ptz };
  }

  async function saveDetectionToActivePreset(reason) {
    let settings = await restoreCustomFromLocalIfMissing();
    const active = activePresetName(settings);
    const preset = currentPresetFromUi(settings, active);

    const backend = await applyDetectionBackend(preset);
    const ptz = await applyPtzFramingOnly(preset);

    settings = await readSettings();

    const custom = Object.assign({}, settings.objectPresetsCustom || {});
    custom[active] = preset;

    settings.objectPresetsCustom = custom;
    settings.activeObjectPreset = active;
    settings.detectorEnabled = true;
    settings.detectorSelectedClasses = preset.classes || [];
    settings.operatorModel = preset.model || '';
    settings.operatorDetectionLimit = Number(preset.max_detections || 10);
    settings.operatorDetectEvery = Number(preset.detect_every_n_frames || 1);
    settings.operatorDetectionAreaMode = String(preset.detection_mode || 'full_frame');
    settings.objectPresetTrackingMode = preset.tracking_mode || 'single_auto';
    settings.objectPresetLossBehavior = preset.loss_behavior || 'continuous_wide_scan_x';
    settings.lastEditedObjectPreset = {
      name: active,
      label: preset.label || active,
      reason: reason || 'manual',
      ts: Math.floor(Date.now() / 1000)
    };
    settings.lastAppliedObjectPreset = {
      name: active,
      label: preset.label || active,
      tracking_mode: preset.tracking_mode || 'single_auto',
      loss_behavior: preset.loss_behavior || 'continuous_wide_scan_x',
      ts: Math.floor(Date.now() / 1000)
    };

    await writeSettings(settings);
    saveLocalCustomBackup(custom);

    const verify = await readSettings();
    const saved = verify?.objectPresetsCustom?.[active];

    const ok =
      saved &&
      Number(saved.max_detections) === Number(preset.max_detections) &&
      Number(saved.detect_every_n_frames) === Number(preset.detect_every_n_frames) &&
      String(saved.detection_mode) === String(preset.detection_mode);

    const state = $('objectPresetState');

    if (state) {
      state.textContent = ok
        ? `preset: ${active} saved OK`
        : `preset: ${active} SAVE VERIFY FAILED`;
    }

    const line = $('ptzStateLine');

    if (line) {
      line.textContent = ok
        ? `saved ${active}: limit=${preset.max_detections} every=${preset.detect_every_n_frames} area=${preset.detection_mode}`
        : `save verify failed: ${active}`;
    }

    console.log('[object-preset-durable-save]', { active, preset, backend, ptz, verify: saved, ok });

    return {
      ok: ok,
      active: active,
      preset: preset,
      backend: backend,
      ptz: ptz,
      verify: saved
    };
  }

  async function syncUiFromActivePreset() {
    const settings = await restoreCustomFromLocalIfMissing();
    const active = activePresetName(settings);
    const preset = settings?.objectPresetsCustom?.[active] || BUILTIN[active];

    if (!preset) return;

    if ($('operatorDetectionLimit') && preset.max_detections != null) {
      $('operatorDetectionLimit').value = String(preset.max_detections);
    }

    if ($('operatorDetectEvery') && preset.detect_every_n_frames != null) {
      $('operatorDetectEvery').value = String(preset.detect_every_n_frames);
    }

    if ($('operatorDetectionAreaMode') && preset.detection_mode) {
      $('operatorDetectionAreaMode').value = String(preset.detection_mode);
    }

    if ($('operatorModelSelect') && (preset.model || preset.operatorModel)) {
      const wanted = String(preset.model || preset.operatorModel || '');
      const wantedBase = shortModelName(wanted);

      for (const opt of Array.from($('operatorModelSelect').options || [])) {
        if (opt.value === wanted || shortModelName(opt.value) === wantedBase) {
          $('operatorModelSelect').value = opt.value;
          break;
        }
      }
    }

    renderActivePreset(active);
  }

  function removeOldSaveButtons() {
    for (const id of [
      'objectPresetSaveCurrentBtn',
      'objectPresetHardSaveBtn',
      'objectPresetDurableSaveBtn'
    ]) {
      const el = $(id);
      if (el) el.remove();
    }
  }

  function ensureDurableSaveButton() {
    removeOldSaveButtons();

    const panel = $('objectPresetPanel');
    if (!panel || $('objectPresetDurableSaveBtn')) return;

    const btn = document.createElement('button');
    btn.id = 'objectPresetDurableSaveBtn';
    btn.className = 'small-btn';
    btn.textContent = 'SAVE DETECTION TO PRESET';
    btn.title = 'Save Model / Limit / Detect FPS / Area to active object preset';

    btn.addEventListener('click', async (e) => {
      e.preventDefault();
      e.stopPropagation();
      e.stopImmediatePropagation();

      try {
        await saveDetectionToActivePreset('manual_button');
      } catch (err) {
        console.error('[object-preset-durable-save]', err);

        if ($('ptzStateLine')) {
          $('ptzStateLine').textContent = `preset save error: ${String(err.message || err)}`;
        }
      }
    }, true);

    panel.appendChild(btn);
  }

  function replaceObjectPresetButtonHandlers() {
    document.querySelectorAll('[data-object-preset]').forEach((oldBtn) => {
      if (oldBtn.__noAutoArmButtonReplaced) return;

      const btn = oldBtn.cloneNode(true);
      btn.__presetBound = true;
      btn.__noAutoArmButtonReplaced = true;

      btn.addEventListener('click', async (e) => {
        e.preventDefault();
        e.stopPropagation();
        e.stopImmediatePropagation();

        const name = btn.dataset.objectPreset;

        try {
          await selectObjectPresetNoArm(name);
        } catch (err) {
          console.error('[object-preset-no-arm]', err);

          if ($('ptzStateLine')) {
            $('ptzStateLine').textContent = `preset select error: ${String(err.message || err)}`;
          }
        }
      }, true);

      oldBtn.replaceWith(btn);
    });
  }

  function bindControlAutoSave() {
    if (document.__objectPresetDurableControlBound) return;
    document.__objectPresetDurableControlBound = true;

    document.addEventListener('change', (e) => {
      const id = String(e.target?.id || '');

      if ([
        'operatorModelSelect',
        'operatorDetectionLimit',
        'operatorDetectEvery',
        'operatorDetectionAreaMode'
      ].includes(id)) {
        setTimeout(() => {
          saveDetectionToActivePreset(id).catch((err) => {
            console.error('[object-preset-durable-autosave]', err);
          });
        }, 450);
      }
    }, true);
  }

  function boot() {
    replaceObjectPresetButtonHandlers();
    ensureDurableSaveButton();
    bindControlAutoSave();

    restoreCustomFromLocalIfMissing()
      .then(() => syncUiFromActivePreset())
      .catch(() => {});
  }

  if (document.readyState === 'loading') {
    document.addEventListener('DOMContentLoaded', boot, { once: true });
  } else {
    boot();
  }

  setTimeout(boot, 500);
  setTimeout(boot, 1500);
  setTimeout(boot, 3000);
  setTimeout(syncUiFromActivePreset, 2500);

  window.selectObjectPresetNoArm = selectObjectPresetNoArm;
  window.saveDetectionToActivePreset = saveDetectionToActivePreset;
  window.syncUiFromActivePreset = syncUiFromActivePreset;
})();
// OBJECT_PRESET_NO_AUTO_ARM_DURABLE_SAVE_END
</script>
'''

if "OBJECT_PRESET_NO_AUTO_ARM_DURABLE_SAVE_START" not in s:
    if "</body>" not in s:
        raise SystemExit("ERROR: </body> not found")
    s = s.replace("</body>", script + "\n</body>", 1)
    p.write_text(s, encoding="utf-8")
    print("OK patched web/index.html")
    print("Backup:", bak)
else:
    print("OK already patched")
