from pathlib import Path
import time

p = Path("web/index.html")
s = p.read_text(encoding="utf-8", errors="ignore")

bak = p.with_suffix(p.suffix + f".bak_operator_controls_hard_persist_apply_{int(time.time())}")
bak.write_text(s, encoding="utf-8")

script = r'''
<script>
// OPERATOR_CONTROLS_HARD_PERSIST_APPLY_START
(function () {
  const $ = (id) => document.getElementById(id);

  const BUILTIN_OBJECT_PRESETS_MIN = {
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

  const PRESERVE_SETTINGS_KEYS = [
    'objectPresetsCustom',
    'activeObjectPreset',
    'lastAppliedObjectPreset',
    'lastEditedObjectPreset',
    'searchPresetsCustom',
    'activeSearchPreset',
    'lastAppliedSearchPreset',
    'lastEditedSearchPreset',
    'ptzArmed',
    'objectPresetTrackingMode',
    'objectPresetLossBehavior'
  ];

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

  async function readSettingsHard() {
    try {
      const cfg = await getJson('/api/settings');
      return cfg && typeof cfg === 'object' ? cfg : {};
    } catch (_) {
      return {};
    }
  }

  async function writeSettingsHard(next) {
    next = Object.assign({}, next || {});
    next.config_version = Math.max(16, Number(next.config_version || 16));

    try {
      localStorage.setItem('pantilt_ui_settings_v2', JSON.stringify(next));
    } catch (_) {}

    await postJson('/api/settings', next);
    return next;
  }

  function activeObjectPresetName(settings) {
    return String(
      settings?.lastAppliedObjectPreset?.name ||
      settings?.activeObjectPreset ||
      'person_single'
    );
  }

  function shortModelName(v) {
    return String(v || '').split('/').filter(Boolean).pop() || String(v || '');
  }

  function selectedModelValue(settings, preset) {
    const el = $('operatorModelSelect');

    return String(
      el?.value ||
      settings.operatorModel ||
      preset.model ||
      preset.operatorModel ||
      preset.current_model ||
      ''
    ).trim();
  }

  function selectedLimitValue(settings, preset) {
    return Number(
      $('operatorDetectionLimit')?.value ||
      settings.operatorDetectionLimit ||
      preset.max_detections ||
      10
    );
  }

  function selectedEveryValue(settings, preset) {
    return Number(
      $('operatorDetectEvery')?.value ||
      settings.operatorDetectEvery ||
      preset.detect_every_n_frames ||
      1
    );
  }

  function selectedAreaValue(settings, preset) {
    return String(
      $('operatorDetectionAreaMode')?.value ||
      settings.operatorDetectionAreaMode ||
      preset.detection_mode ||
      'full_frame'
    );
  }

  function classesForActivePreset(settings, preset, active) {
    if (Array.isArray(preset.classes) && preset.classes.length) {
      return preset.classes.map(Number).filter(Number.isFinite);
    }

    if (Array.isArray(settings.detectorSelectedClasses) && settings.detectorSelectedClasses.length) {
      return settings.detectorSelectedClasses.map(Number).filter(Number.isFinite);
    }

    if (active === 'person_single' || active === 'people') return [0];
    if (active === 'car_single' || active === 'cars') return [2, 3, 5, 7];
    if (active === 'airplane_single' || active === 'airplanes') return [4];
    if (active === 'bird_single' || active === 'birds') return [14];

    return [];
  }

  function currentPtzForPreset(settings, preset) {
    return Object.assign(
      {},
      preset.ptz || {},
      settings.ptzAutoZoom || {},
      settings.ptzConfig || {}
    );
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

  function buildActivePresetFromUi(settings) {
    const active = activeObjectPresetName(settings);
    const custom = Object.assign({}, settings.objectPresetsCustom || {});
    const base = Object.assign({}, BUILTIN_OBJECT_PRESETS_MIN[active] || {});
    const old = Object.assign({}, base, custom[active] || {});

    const model = selectedModelValue(settings, old);
    const maxDet = selectedLimitValue(settings, old);
    const every = selectedEveryValue(settings, old);
    const area = selectedAreaValue(settings, old);
    const classes = classesForActivePreset(settings, old, active);
    const ptz = currentPtzForPreset(settings, old);

    const preset = Object.assign({}, old, {
      label: old.label || active,
      model: model,
      operatorModel: model,
      classes: classes,
      detection_mode: area,
      max_detections: maxDet,
      max_raw_candidates: Math.max(20, Number(old.max_raw_candidates || maxDet * 5)),
      detect_every_n_frames: every,
      tracking_mode: old.tracking_mode || settings.objectPresetTrackingMode || settings?.lastAppliedObjectPreset?.tracking_mode || 'single_auto',
      loss_behavior: old.loss_behavior || settings.objectPresetLossBehavior || settings?.lastAppliedObjectPreset?.loss_behavior || 'continuous_wide_scan_x',
      ptz: ptz
    });

    return { active, custom, preset };
  }

  async function applyBackendOperatorConfig(preset) {
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
    const lim = await postJson('/api/detection/limits', {
      max_detections: maxDet,
      max_raw_candidates: maxRaw
    });
    const thr = await postJson('/api/detection/throttle', {
      detect_every_n_frames: every
    });
    const roi = await postJson('/api/detection/roi_config', await roiPayload(area));

    return { detector: det, limits: lim, throttle: thr, roi: roi };
  }

  async function saveActiveObjectPresetHard(reason) {
    const oldSettings = await readSettingsHard();
    const { active, custom, preset } = buildActivePresetFromUi(oldSettings);

    const backend = await applyBackendOperatorConfig(preset);

    custom[active] = preset;

    const next = Object.assign({}, oldSettings, {
      objectPresetsCustom: custom,
      activeObjectPreset: active,
      detectorEnabled: true,
      detectorSelectedClasses: preset.classes,
      operatorModel: preset.model || '',
      operatorDetectionLimit: preset.max_detections,
      operatorDetectEvery: preset.detect_every_n_frames,
      operatorDetectionAreaMode: preset.detection_mode,
      objectPresetTrackingMode: preset.tracking_mode,
      objectPresetLossBehavior: preset.loss_behavior,
      lastEditedObjectPreset: {
        name: active,
        label: preset.label || active,
        reason: reason || 'ui',
        ts: Math.floor(Date.now() / 1000)
      },
      lastAppliedObjectPreset: Object.assign({}, oldSettings.lastAppliedObjectPreset || {}, {
        name: active,
        label: preset.label || active,
        tracking_mode: preset.tracking_mode,
        loss_behavior: preset.loss_behavior,
        ts: Math.floor(Date.now() / 1000)
      })
    });

    await writeSettingsHard(next);

    const state = $('objectPresetState');
    if (state) {
      state.textContent = `preset: ${active} saved`;
    }

    const line = $('ptzStateLine');
    if (line) {
      line.textContent =
        `saved ${active}: model=${shortModelName(preset.model)} limit=${preset.max_detections} every=${preset.detect_every_n_frames} area=${preset.detection_mode}`;
    }

    console.log('[object-preset-hard-save]', { active, preset, backend });

    return { active, preset, backend };
  }

  function protectSaveSettingsFromWipingPresets() {
    if (window.__objectPresetProtectSaveSettingsInstalled) return;
    window.__objectPresetProtectSaveSettingsInstalled = true;

    const original = window.saveSettings;

    if (typeof original !== 'function') return;

    window.saveSettings = async function (...args) {
      const before = await readSettingsHard();

      let result;
      try {
        result = await original.apply(this, args);
      } catch (e) {
        throw e;
      }

      try {
        const after = await readSettingsHard();
        let changed = false;

        for (const key of PRESERVE_SETTINGS_KEYS) {
          if (before[key] !== undefined && after[key] === undefined) {
            after[key] = before[key];
            changed = true;
          }
        }

        if (changed) {
          await writeSettingsHard(after);
          console.warn('[object-preset-hard-save] restored preserved keys after saveSettings');
        }
      } catch (e) {
        console.warn('[object-preset-hard-save] preserve failed', e);
      }

      return result;
    };
  }

  let saveTimer = null;

  function scheduleHardSave(reason) {
    if (saveTimer) clearTimeout(saveTimer);

    saveTimer = setTimeout(() => {
      saveTimer = null;

      saveActiveObjectPresetHard(reason).catch((e) => {
        console.error('[object-preset-hard-save]', e);

        const line = $('ptzStateLine');
        if (line) {
          line.textContent = `preset save error: ${String(e.message || e)}`;
        }
      });
    }, 700);
  }

  function ensureSaveButton() {
    if ($('objectPresetHardSaveBtn')) return;

    const panel = $('objectPresetPanel');
    if (!panel) return;

    const btn = document.createElement('button');
    btn.id = 'objectPresetHardSaveBtn';
    btn.className = 'small-btn';
    btn.textContent = 'SAVE DETECTION TO PRESET';
    btn.title = 'Save Model / Limit / Detect FPS / Area to active object preset and apply backend';

    btn.addEventListener('click', (e) => {
      e.preventDefault();
      e.stopPropagation();
      saveActiveObjectPresetHard('manual_button').catch(console.error);
    }, true);

    panel.appendChild(btn);
  }

  function bindOperatorControlsHard() {
    if (document.__operatorControlsHardPersistBound) return;
    document.__operatorControlsHardPersistBound = true;

    document.addEventListener('change', (e) => {
      const id = String(e.target?.id || '');

      if ([
        'operatorModelSelect',
        'operatorDetectionLimit',
        'operatorDetectEvery',
        'operatorDetectionAreaMode'
      ].includes(id)) {
        scheduleHardSave(id);
      }
    }, true);

    document.addEventListener('click', (e) => {
      const id = String(e.target?.id || '');

      if (id === 'operatorModelApplyBtn') {
        setTimeout(() => scheduleHardSave('operatorModelApplyBtn'), 1000);
      }

      if (id === 'operatorLimitApplyBtn') {
        setTimeout(() => scheduleHardSave('operatorLimitApplyBtn'), 1000);
      }

      if (id === 'operatorAreaApplyBtn') {
        setTimeout(() => scheduleHardSave('operatorAreaApplyBtn'), 1000);
      }

      if (e.target && e.target.matches && e.target.matches('[data-object-preset]')) {
        setTimeout(() => scheduleHardSave('object_preset_click'), 1200);
      }
    }, true);
  }

  async function syncUiFromSavedActivePreset() {
    const settings = await readSettingsHard();
    const active = activeObjectPresetName(settings);
    const preset = settings?.objectPresetsCustom?.[active];

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

    if ($('operatorModelSelect') && preset.model) {
      for (const opt of Array.from($('operatorModelSelect').options || [])) {
        if (opt.value === preset.model || shortModelName(opt.value) === shortModelName(preset.model)) {
          $('operatorModelSelect').value = opt.value;
          break;
        }
      }
    }
  }

  function boot() {
    protectSaveSettingsFromWipingPresets();
    ensureSaveButton();
    bindOperatorControlsHard();

    syncUiFromSavedActivePreset().catch(() => {});
  }

  if (document.readyState === 'loading') {
    document.addEventListener('DOMContentLoaded', boot, { once: true });
  } else {
    boot();
  }

  setTimeout(boot, 500);
  setTimeout(boot, 1500);
  setTimeout(boot, 3000);

  window.saveActiveObjectPresetHard = saveActiveObjectPresetHard;
  window.syncUiFromSavedActivePreset = syncUiFromSavedActivePreset;
})();
// OPERATOR_CONTROLS_HARD_PERSIST_APPLY_END
</script>
'''

if "OPERATOR_CONTROLS_HARD_PERSIST_APPLY_START" not in s:
    if "</body>" not in s:
        raise SystemExit("ERROR: </body> not found")
    s = s.replace("</body>", script + "\n</body>", 1)
    p.write_text(s, encoding="utf-8")
    print("OK patched web/index.html")
    print("Backup:", bak)
else:
    print("OK already patched")
