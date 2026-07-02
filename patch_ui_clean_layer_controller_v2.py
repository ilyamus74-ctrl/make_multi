from pathlib import Path
import re
import time

p = Path("web/index.html")
s = p.read_text(encoding="utf-8", errors="ignore")

bak = p.with_suffix(p.suffix + f".bak_before_clean_layer_controller_v2_{int(time.time())}")
bak.write_text(s, encoding="utf-8")

REMOVE_MARKERS = [
    "OBJECT_PRESET_MANAGER_START",
    "OBJECT_PRESET_BUTTONS_ACTIVE_FIX_START",
    "OBJECT_TRACKING_LOGS_BUTTON_START",
    "OBJECT_PRESET_AUTO_LEARN_OPERATOR_CONTROLS_START",
    "OPERATOR_CONTROLS_HARD_PERSIST_APPLY_START",
    "OBJECT_PRESET_NO_AUTO_ARM_DURABLE_SAVE_START",
    "PTZ_START_STOP_ACTIVE_STATE_FIX_START",
    "SINGLE_PRESET_AUTO_ARM",
    "SEARCH_PRESET_SELECTOR_START",
    "PTZ_CLEAN_LAYER_CONTROLLER_START"
]

script_re = re.compile(r"<script\b[^>]*>.*?</script>", re.IGNORECASE | re.DOTALL)

removed = {}

def replace_script(match):
    block = match.group(0)

    hits = [
        marker
        for marker in REMOVE_MARKERS
        if marker in block
    ]

    if not hits:
        return block

    key = ",".join(hits)

    removed[key] = removed.get(key, 0) + 1

    return "\n<!-- removed conflicting ptz ui controller block -->\n"

s = script_re.sub(replace_script, s)

controller = r'''
<script>
// PTZ_CLEAN_LAYER_CONTROLLER_START
(function () {
  const $ = (id) => document.getElementById(id);

  const OBJECT_PRESETS = {
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
      loss_behavior: 'operator_select',
      ptz: {
        target_x: 0.50,
        target_y: 0.47,
        auto_zoom_enable: true,
        auto_zoom_target_h: 0.58,
        auto_zoom_deadzone: 0.10,
        auto_zoom_cmd: 8,
        auto_zoom_sign: 1,
        auto_zoom_period_ms: 450
      }
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
      loss_behavior: 'operator_select',
      ptz: {
        target_x: 0.50,
        target_y: 0.50,
        auto_zoom_enable: true,
        auto_zoom_target_h: 0.42,
        auto_zoom_deadzone: 0.12,
        auto_zoom_cmd: 8,
        auto_zoom_sign: 1,
        auto_zoom_period_ms: 500
      }
    },
    airplane_single: {
      label: 'САМОЛЁТ',
      classes: [4],
      detection_mode: 'hybrid',
      max_detections: 8,
      max_raw_candidates: 60,
      detect_every_n_frames: 1,
      tracking_mode: 'single_auto',
      loss_behavior: 'continuous_wide_scan_x',
      ptz: {
        target_x: 0.50,
        target_y: 0.45,
        auto_zoom_enable: true,
        auto_zoom_target_h: 0.30,
        auto_zoom_deadzone: 0.08,
        auto_zoom_cmd: 10,
        auto_zoom_sign: 1,
        auto_zoom_period_ms: 350
      }
    },
    airplanes: {
      label: 'САМОЛЁТЫ',
      classes: [4],
      detection_mode: 'hybrid',
      max_detections: 15,
      max_raw_candidates: 100,
      detect_every_n_frames: 1,
      tracking_mode: 'multi_operator',
      loss_behavior: 'operator_select',
      ptz: {
        target_x: 0.50,
        target_y: 0.45,
        auto_zoom_enable: true,
        auto_zoom_target_h: 0.25,
        auto_zoom_deadzone: 0.10,
        auto_zoom_cmd: 10,
        auto_zoom_sign: 1,
        auto_zoom_period_ms: 400
      }
    },
    bird_single: {
      label: 'ПТИЦА',
      classes: [14],
      detection_mode: 'hybrid',
      max_detections: 10,
      max_raw_candidates: 80,
      detect_every_n_frames: 1,
      tracking_mode: 'single_auto',
      loss_behavior: 'continuous_wide_scan_x',
      ptz: {
        target_x: 0.50,
        target_y: 0.45,
        auto_zoom_enable: true,
        auto_zoom_target_h: 0.24,
        auto_zoom_deadzone: 0.07,
        auto_zoom_cmd: 12,
        auto_zoom_sign: 1,
        auto_zoom_period_ms: 300
      }
    },
    birds: {
      label: 'ПТИЦЫ',
      classes: [14],
      detection_mode: 'hybrid',
      max_detections: 20,
      max_raw_candidates: 120,
      detect_every_n_frames: 1,
      tracking_mode: 'multi_operator',
      loss_behavior: 'operator_select',
      ptz: {
        target_x: 0.50,
        target_y: 0.45,
        auto_zoom_enable: true,
        auto_zoom_target_h: 0.20,
        auto_zoom_deadzone: 0.08,
        auto_zoom_cmd: 12,
        auto_zoom_sign: 1,
        auto_zoom_period_ms: 300
      }
    }
  };

  const SEARCH_PRESETS = {
    lost_step_wait: 'Lost: step wide, sweep, wait',
    lost_wide_cycle: 'Lost: wide 3 frames cycle',
    lost_nested_loop: 'Lost: short + preset1 + pause'
  };

  const PTZ_FRAMING_KEYS = new Set([
    'target_x',
    'target_y',
    'auto_zoom_enable',
    'auto_zoom_target_h',
    'auto_zoom_deadzone',
    'auto_zoom_cmd',
    'auto_zoom_sign',
    'auto_zoom_period_ms',
    'zoom_scale_enable',
    'zoom_scale_min',
    'zoom_scale_max',
    'zoom_scale_smoothing'
  ]);

  function autopilotBaseUrl() {
    return `${location.protocol}//${location.hostname}:8090`;
  }

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
      const data = await getJson('/api/settings');
      return data && typeof data === 'object' ? data : {};
    } catch (_) {
      return {};
    }
  }

  async function writeSettings(settings) {
    const next = Object.assign({}, settings || {});
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

  function sanitizePtz(ptz) {
    const out = {};

    for (const [k, v] of Object.entries(ptz || {})) {
      if (PTZ_FRAMING_KEYS.has(k)) {
        out[k] = v;
      }
    }

    return out;
  }

  function activeObjectName(settings) {
    return String(settings.activeObjectPreset || settings?.lastAppliedObjectPreset?.name || 'person_single');
  }

  function mergedPresets(settings) {
    return Object.assign({}, OBJECT_PRESETS, settings.objectPresetsCustom || {});
  }

  function getActivePreset(settings) {
    const name = activeObjectName(settings);
    const presets = mergedPresets(settings);
    const preset = Object.assign({}, OBJECT_PRESETS[name] || {}, presets[name] || {});
    preset.ptz = sanitizePtz(preset.ptz || {});
    return { name, preset };
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

  function fullRoi(w, h) {
    return { id: 'full_frame', enabled: true, x: 0, y: 0, w, h, every_n_frames: 1, classes: [] };
  }

  function skyRoi(w, h) {
    return { id: 'sky_top', enabled: true, x: 0, y: 0, w, h: Math.max(1, Math.round(h * 0.68)), every_n_frames: 1, classes: [] };
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

  async function roiPayload(mode) {
    const sz = await frameSize();
    const w = sz.w;
    const h = sz.h;

    if (mode === 'tiled') return { detection_mode: 'tiled', rois: [] };
    if (mode === 'roi') return { detection_mode: 'roi', rois: [skyRoi(w, h)] };
    if (mode === 'multi_roi') return { detection_mode: 'multi_roi', rois: [skyRoi(w, h), centerRoi(w, h)] };
    if (mode === 'hybrid') return { detection_mode: 'hybrid', rois: [fullRoi(w, h), skyRoi(w, h)] };

    return { detection_mode: 'full_frame', rois: [] };
  }

  function nval(id, fallback) {
    const el = $(id);
    const x = Number(el?.value);
    return Number.isFinite(x) ? x : fallback;
  }

  function sval(id, fallback) {
    const el = $(id);
    const v = String(el?.value || '').trim();
    return v || fallback;
  }

  function syncUiFromPreset(preset) {
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
  }

  function presetFromUi(settings, name) {
    const presets = mergedPresets(settings);
    const old = Object.assign({}, OBJECT_PRESETS[name] || {}, presets[name] || {});

    const maxDet = nval('operatorDetectionLimit', Number(old.max_detections || settings.operatorDetectionLimit || 10));
    const every = nval('operatorDetectEvery', Number(old.detect_every_n_frames || settings.operatorDetectEvery || 1));
    const area = sval('operatorDetectionAreaMode', String(old.detection_mode || settings.operatorDetectionAreaMode || 'full_frame'));
    const model = sval('operatorModelSelect', String(old.model || old.operatorModel || settings.operatorModel || ''));

    return Object.assign({}, old, {
      label: old.label || name,
      model,
      operatorModel: model,
      classes: Array.isArray(old.classes) ? old.classes.map(Number).filter(Number.isFinite) : [],
      detection_mode: area,
      max_detections: maxDet,
      max_raw_candidates: Math.max(20, Number(old.max_raw_candidates || maxDet * 5)),
      detect_every_n_frames: every,
      tracking_mode: old.tracking_mode || 'single_auto',
      loss_behavior: old.loss_behavior || 'continuous_wide_scan_x',
      ptz: sanitizePtz(old.ptz || {})
    });
  }

  async function applyDetection(preset) {
    const classes = Array.isArray(preset.classes) ? preset.classes.map(Number).filter(Number.isFinite) : [];
    const model = String(preset.model || preset.operatorModel || '').trim();
    const maxDet = Number(preset.max_detections || 10);
    const maxRaw = Number(preset.max_raw_candidates || Math.max(20, maxDet * 5));
    const every = Number(preset.detect_every_n_frames || 1);
    const mode = String(preset.detection_mode || 'full_frame');

    const detectorPayload = {
      detect_enabled: true,
      selected_classes: classes
    };

    if (model) {
      detectorPayload.current_model = model;
    }

    const detector = await postJson('/api/detector/config', detectorPayload);
    const limits = await postJson('/api/detection/limits', {
      max_detections: maxDet,
      max_raw_candidates: maxRaw
    });
    const throttle = await postJson('/api/detection/throttle', {
      detect_every_n_frames: every
    });
    const roi = await postJson('/api/detection/roi_config', await roiPayload(mode));

    return { detector, limits, throttle, roi };
  }

  async function applyPtzFraming(preset) {
    const body = sanitizePtz(preset.ptz || {});
    if (!Object.keys(body).length) return { ok: true, skipped: true };
    return await postJson(`${autopilotBaseUrl()}/api/autopilot/config`, body);
  }

  function ensureObjectPanel() {
    let old = $('objectPresetPanel');

    if (old && !old.__cleanPanel) {
      old.remove();
      old = null;
    }

    if (old) return;

    const parent = $('detectionMenu') || document.querySelector('.controls-panel') || document.body;
    const box = document.createElement('div');

    box.id = 'objectPresetPanel';
    box.__cleanPanel = true;
    box.className = 'menu-body';
    box.style.marginTop = '6px';

    box.innerHTML = `
      <strong>OBJECT PRESETS:</strong>
      <button class="small-btn" data-clean-object-preset="person_single">ЧЕЛОВЕК</button>
      <button class="small-btn" data-clean-object-preset="people">ЛЮДИ</button>
      <button class="small-btn" data-clean-object-preset="car_single">МАШИНА</button>
      <button class="small-btn" data-clean-object-preset="cars">МАШИНЫ</button>
      <button class="small-btn" data-clean-object-preset="airplane_single">САМОЛЁТ</button>
      <button class="small-btn" data-clean-object-preset="airplanes">САМОЛЁТЫ</button>
      <button class="small-btn" data-clean-object-preset="bird_single">ПТИЦА</button>
      <button class="small-btn" data-clean-object-preset="birds">ПТИЦЫ</button>
      <button id="objectPresetCleanSaveBtn" class="small-btn">SAVE DETECTION TO PRESET</button>
      <button id="objectTrackingLogsBtn" class="small-btn">LOGS</button>
      <code id="objectPresetState">preset: --</code>
    `;

    parent.appendChild(box);
  }

  function ensureSearchPanel() {
    if ($('searchPresetPanel')) return;

    const objectPanel = $('objectPresetPanel');
    if (!objectPanel || !objectPanel.parentNode) return;

    const row = document.createElement('div');

    row.id = 'searchPresetPanel';
    row.style.marginTop = '6px';
    row.style.display = 'flex';
    row.style.gap = '6px';
    row.style.alignItems = 'center';
    row.style.flexWrap = 'wrap';

    const label = document.createElement('strong');
    label.textContent = 'SEARCH PRESET:';

    const select = document.createElement('select');
    select.id = 'searchPresetSelect';

    for (const [id, text] of Object.entries(SEARCH_PRESETS)) {
      const opt = document.createElement('option');
      opt.value = id;
      opt.textContent = text;
      select.appendChild(opt);
    }

    const state = document.createElement('code');
    state.id = 'searchPresetState';
    state.textContent = 'search: --';

    row.appendChild(label);
    row.appendChild(select);
    row.appendChild(state);

    objectPanel.parentNode.insertBefore(row, objectPanel.nextSibling);
  }

  function paintObjectPreset(name) {
    document.querySelectorAll('[data-clean-object-preset]').forEach(btn => {
      const on = btn.dataset.cleanObjectPreset === name;
      btn.classList.toggle('active', on);
      btn.classList.toggle('object-preset-active', on);
      btn.setAttribute('aria-pressed', on ? 'true' : 'false');
    });

    if ($('objectPresetState')) {
      $('objectPresetState').textContent = `preset: ${name || '--'}`;
    }
  }

  function paintPtz(ap, settings) {
    const start = $('ptzStartBtn');
    const stop = $('ptzStopBtn');
    const chip = $('ptzRunStateChip');

    const running = Boolean(ap && ap.enabled === true);
    const armed = Boolean(settings && settings.ptzArmed === true);

    if (start) {
      start.classList.remove('ptz-active-running', 'ptz-active-armed');
      start.textContent = running ? 'PTZ ACTIVE' : (armed ? 'PTZ ARMED' : 'START PTZ');

      if (running) start.classList.add('ptz-active-running');
      else if (armed) start.classList.add('ptz-active-armed');
    }

    if (stop) {
      stop.textContent = running || armed ? 'STOP PTZ' : 'STOP';
    }

    if (chip) {
      chip.textContent = running
        ? `ptz: ACTIVE / ${ap.mode || ''}`
        : (armed ? 'ptz: ARMED / SEARCH' : 'ptz: STOPPED');
    }
  }

  async function syncPtz() {
    const ap = await getJson(`${autopilotBaseUrl()}/api/autopilot/state`).catch(() => ({ enabled: false, mode: 'offline' }));
    const settings = await readSettings();
    paintPtz(ap, settings);
    return { ap, settings };
  }

  async function selectObjectPreset(name) {
    let settings = await readSettings();
    const presets = mergedPresets(settings);
    const preset = Object.assign({}, OBJECT_PRESETS[name] || {}, presets[name] || {});
    preset.ptz = sanitizePtz(preset.ptz || {});

    await postJson(`${autopilotBaseUrl()}/api/autopilot/stop`, {}).catch(() => {});
    await postJson(`${autopilotBaseUrl()}/api/control/stop`, {}).catch(() => {});

    const detection = await applyDetection(preset);
    const ptz = await applyPtzFraming(preset);

    settings = await readSettings();

    settings.activeObjectPreset = name;
    settings.ptzArmed = false;
    settings.controlMode = 'manual';
    settings.detectorEnabled = true;
    settings.detectorSelectedClasses = Array.isArray(preset.classes) ? preset.classes : [];
    settings.operatorModel = preset.model || preset.operatorModel || settings.operatorModel || '';
    settings.operatorDetectionLimit = Number(preset.max_detections || 10);
    settings.operatorDetectEvery = Number(preset.detect_every_n_frames || 1);
    settings.operatorDetectionAreaMode = String(preset.detection_mode || 'full_frame');
    settings.objectPresetTrackingMode = preset.tracking_mode || 'single_auto';
    settings.objectPresetLossBehavior = preset.loss_behavior || 'continuous_wide_scan_x';
    settings.lastAppliedObjectPreset = {
      name,
      label: preset.label || name,
      tracking_mode: settings.objectPresetTrackingMode,
      loss_behavior: settings.objectPresetLossBehavior,
      ts: Math.floor(Date.now() / 1000)
    };

    await writeSettings(settings);

    syncUiFromPreset(preset);
    paintObjectPreset(name);
    await syncPtz();

    if ($('ptzStateLine')) {
      $('ptzStateLine').textContent =
        `preset selected, PTZ OFF: ${name} limit=${settings.operatorDetectionLimit} every=${settings.operatorDetectEvery}`;
    }

    return { ok: true, name, preset, detection, ptz };
  }

  async function saveDetectionToPreset(reason) {
    let settings = await readSettings();
    const name = activeObjectName(settings);
    const preset = presetFromUi(settings, name);

    const detection = await applyDetection(preset);
    const ptz = await applyPtzFraming(preset);

    settings = await readSettings();

    const custom = Object.assign({}, settings.objectPresetsCustom || {});
    custom[name] = preset;

    settings.objectPresetsCustom = custom;
    settings.activeObjectPreset = name;
    settings.detectorEnabled = true;
    settings.detectorSelectedClasses = preset.classes || [];
    settings.operatorModel = preset.model || '';
    settings.operatorDetectionLimit = Number(preset.max_detections || 10);
    settings.operatorDetectEvery = Number(preset.detect_every_n_frames || 1);
    settings.operatorDetectionAreaMode = String(preset.detection_mode || 'full_frame');
    settings.objectPresetTrackingMode = preset.tracking_mode || 'single_auto';
    settings.objectPresetLossBehavior = preset.loss_behavior || 'continuous_wide_scan_x';
    settings.lastEditedObjectPreset = {
      name,
      label: preset.label || name,
      reason: reason || 'manual',
      ts: Math.floor(Date.now() / 1000)
    };

    await writeSettings(settings);

    const verify = await readSettings();
    const saved = verify?.objectPresetsCustom?.[name] || {};

    const ok =
      Number(saved.max_detections) === Number(preset.max_detections) &&
      Number(saved.detect_every_n_frames) === Number(preset.detect_every_n_frames) &&
      String(saved.detection_mode) === String(preset.detection_mode);

    paintObjectPreset(name);

    if ($('objectPresetState')) {
      $('objectPresetState').textContent = ok ? `preset: ${name} saved OK` : `preset: ${name} SAVE FAIL`;
    }

    if ($('ptzStateLine')) {
      $('ptzStateLine').textContent = ok
        ? `saved ${name}: limit=${preset.max_detections} every=${preset.detect_every_n_frames} area=${preset.detection_mode}`
        : `save verify failed: ${name}`;
    }

    return { ok, name, preset, detection, ptz, saved };
  }

  async function setSearchPreset(name) {
    const settings = await readSettings();

    settings.activeSearchPreset = name;
    settings.lastAppliedSearchPreset = {
      name,
      label: SEARCH_PRESETS[name] || name,
      ts: Math.floor(Date.now() / 1000)
    };

    await writeSettings(settings);

    if ($('searchPresetState')) {
      $('searchPresetState').textContent = `search: ${name}`;
    }

    if ($('ptzStateLine')) {
      $('ptzStateLine').textContent = `search preset saved: ${name}`;
    }
  }

  async function startPtz() {
    const settings = await readSettings();

    settings.ptzArmed = true;
    settings.controlMode = 'ptz';

    await writeSettings(settings);
    await syncPtz();

    if ($('ptzStateLine')) {
      $('ptzStateLine').textContent = 'PTZ ARMED: daemon controls runtime';
    }
  }

  async function stopPtz() {
    const settings = await readSettings();

    settings.ptzArmed = false;
    settings.controlMode = 'manual';

    await writeSettings(settings);

    await postJson(`${autopilotBaseUrl()}/api/autopilot/stop`, {}).catch(() => {});
    await postJson(`${autopilotBaseUrl()}/api/control/stop`, {}).catch(() => {});
    await postJson('/api/tracker/clear', {}).catch(() => {});

    await syncPtz();

    if ($('ptzStateLine')) {
      $('ptzStateLine').textContent = 'PTZ stopped/disarmed';
    }
  }

  async function syncUi() {
    const settings = await readSettings();
    const { name, preset } = getActivePreset(settings);

    syncUiFromPreset(preset);
    paintObjectPreset(name);

    const search = settings.activeSearchPreset || 'lost_step_wait';

    if ($('searchPresetSelect') && SEARCH_PRESETS[search]) {
      $('searchPresetSelect').value = search;
    }

    if ($('searchPresetState')) {
      $('searchPresetState').textContent = `search: ${search}`;
    }

    await syncPtz();
  }

  function bindHandlers() {
    if (document.__ptzCleanLayerBound) return;
    document.__ptzCleanLayerBound = true;

    document.addEventListener('click', async (e) => {
      const btn = e.target && e.target.closest ? e.target.closest('[data-clean-object-preset]') : null;

      if (!btn) return;

      e.preventDefault();
      e.stopPropagation();
      e.stopImmediatePropagation();

      try {
        await selectObjectPreset(btn.dataset.cleanObjectPreset);
      } catch (err) {
        console.error('[clean select preset]', err);

        if ($('ptzStateLine')) {
          $('ptzStateLine').textContent = `preset error: ${String(err.message || err)}`;
        }
      }
    }, true);

    document.addEventListener('click', async (e) => {
      if (e.target && e.target.id === 'objectPresetCleanSaveBtn') {
        e.preventDefault();
        e.stopPropagation();
        e.stopImmediatePropagation();

        try {
          await saveDetectionToPreset('manual_button');
        } catch (err) {
          console.error('[clean save preset]', err);

          if ($('ptzStateLine')) {
            $('ptzStateLine').textContent = `save error: ${String(err.message || err)}`;
          }
        }
      }
    }, true);

    document.addEventListener('change', async (e) => {
      if (e.target && e.target.id === 'searchPresetSelect') {
        await setSearchPreset(String(e.target.value || 'lost_step_wait'));
      }
    }, true);

    document.addEventListener('click', (e) => {
      if (e.target && e.target.id === 'objectTrackingLogsBtn') {
        e.preventDefault();
        e.stopPropagation();
        window.open(`${location.protocol}//${location.hostname}:8091/`, '_blank', 'noopener,noreferrer');
      }
    }, true);

    const start = $('ptzStartBtn');

    if (start && !start.__cleanLayerBound) {
      start.__cleanLayerBound = true;
      start.onclick = null;
      start.removeAttribute('onclick');

      start.addEventListener('click', async (e) => {
        e.preventDefault();
        e.stopPropagation();
        e.stopImmediatePropagation();

        try {
          await startPtz();
        } catch (err) {
          console.error('[clean start ptz]', err);

          if ($('ptzStateLine')) {
            $('ptzStateLine').textContent = `start error: ${String(err.message || err)}`;
          }
        }
      }, true);
    }

    const stop = $('ptzStopBtn');

    if (stop && !stop.__cleanLayerBound) {
      stop.__cleanLayerBound = true;
      stop.onclick = null;
      stop.removeAttribute('onclick');

      stop.addEventListener('click', async (e) => {
        e.preventDefault();
        e.stopPropagation();
        e.stopImmediatePropagation();

        try {
          await stopPtz();
        } catch (err) {
          console.error('[clean stop ptz]', err);

          if ($('ptzStateLine')) {
            $('ptzStateLine').textContent = `stop error: ${String(err.message || err)}`;
          }
        }
      }, true);
    }

    document.addEventListener('change', (e) => {
      const id = String(e.target?.id || '');

      if ([
        'operatorModelSelect',
        'operatorDetectionLimit',
        'operatorDetectEvery',
        'operatorDetectionAreaMode'
      ].includes(id)) {
        window.clearTimeout(window.__ptzCleanAutosaveTimer);

        window.__ptzCleanAutosaveTimer = window.setTimeout(() => {
          saveDetectionToPreset(id).catch(err => console.error('[clean autosave]', err));
        }, 700);
      }
    }, true);
  }

  function installStyle() {
    if ($('ptzCleanLayerStyle')) return;

    const st = document.createElement('style');
    st.id = 'ptzCleanLayerStyle';
    st.textContent = `
      [data-clean-object-preset].active,
      [data-clean-object-preset].object-preset-active {
        outline: 2px solid #00ff66 !important;
        background: rgba(0,255,102,.22) !important;
        color: #fff !important;
        border-color: #00ff66 !important;
      }

      #ptzStartBtn.ptz-active-running {
        background: #2e7d32 !important;
        color: #fff !important;
      }

      #ptzStartBtn.ptz-active-armed {
        background: #9c7b00 !important;
        color: #fff !important;
      }
    `;

    document.head.appendChild(st);
  }

  function boot() {
    installStyle();
    ensureObjectPanel();
    ensureSearchPanel();
    bindHandlers();
    syncUi().catch(err => console.error('[clean sync]', err));
  }

  if (document.readyState === 'loading') {
    document.addEventListener('DOMContentLoaded', boot, { once: true });
  } else {
    boot();
  }

  setTimeout(boot, 500);
  setTimeout(boot, 1500);
  setInterval(() => syncPtz().catch(() => {}), 1500);

  window.ptzCleanLayer = {
    selectObjectPreset,
    saveDetectionToPreset,
    setSearchPreset,
    startPtz,
    stopPtz,
    syncUi
  };
})();
// PTZ_CLEAN_LAYER_CONTROLLER_END
</script>
'''

if "</body>" not in s:
    raise SystemExit("ERROR: </body> not found")

s = s.replace("</body>", controller + "\n</body>", 1)

p.write_text(s, encoding="utf-8")

print("OK patched web/index.html")
print("Backup:", bak)
print("")
print("Removed blocks:")
for k, v in sorted(removed.items()):
    print(v, k)
