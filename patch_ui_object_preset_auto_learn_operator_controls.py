from pathlib import Path
import time

p = Path("web/index.html")
s = p.read_text(encoding="utf-8", errors="ignore")

bak = p.with_suffix(p.suffix + f".bak_object_preset_auto_learn_{int(time.time())}")
bak.write_text(s, encoding="utf-8")

script = r'''
<script>
// OBJECT_PRESET_AUTO_LEARN_OPERATOR_CONTROLS_START
(function () {
  const $ = (id) => document.getElementById(id);

  const BUILTIN_LABELS = {
    person_single: 'ЧЕЛОВЕК',
    people: 'ЛЮДИ',
    car_single: 'МАШИНА',
    cars: 'МАШИНЫ',
    airplane_single: 'САМОЛЁТ',
    airplanes: 'САМОЛЁТЫ',
    bird_single: 'ПТИЦА',
    birds: 'ПТИЦЫ'
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

  async function readSettings() {
    try {
      return await getJson('/api/settings');
    } catch (_) {
      return {};
    }
  }

  async function saveSettings(settings) {
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

  function activePresetName(settings) {
    return String(
      settings?.lastAppliedObjectPreset?.name ||
      settings?.activeObjectPreset ||
      'person_single'
    );
  }

  function currentClassesFromSettings(settings) {
    const arr = settings.detectorSelectedClasses;
    if (Array.isArray(arr)) {
      return arr.map(Number).filter(Number.isFinite);
    }

    const active = activePresetName(settings);

    if (active === 'person_single' || active === 'people') return [0];
    if (active === 'car_single' || active === 'cars') return [2, 3, 5, 7];
    if (active === 'airplane_single' || active === 'airplanes') return [4];
    if (active === 'bird_single' || active === 'birds') return [14];

    return [];
  }

  function currentPtzFromSettings(settings) {
    return Object.assign(
      {},
      settings.ptzAutoZoom || {},
      settings.ptzConfig || {}
    );
  }

  function currentOperatorPreset(settings) {
    const active = activePresetName(settings);
    const custom = Object.assign({}, settings.objectPresetsCustom || {});
    const old = Object.assign({}, custom[active] || {});

    const limitEl = $('operatorDetectionLimit');
    const everyEl = $('operatorDetectEvery');
    const areaEl = $('operatorDetectionAreaMode');
    const modelEl = $('operatorModelSelect');

    const maxDet = Number(
      limitEl?.value ||
      settings.operatorDetectionLimit ||
      old.max_detections ||
      10
    );

    const every = Number(
      everyEl?.value ||
      settings.operatorDetectEvery ||
      old.detect_every_n_frames ||
      1
    );

    const area = String(
      areaEl?.value ||
      settings.operatorDetectionAreaMode ||
      old.detection_mode ||
      'full_frame'
    );

    const model = String(
      modelEl?.value ||
      settings.operatorModel ||
      old.model ||
      ''
    ).trim();

    const preset = Object.assign({}, old, {
      label: old.label || settings?.lastAppliedObjectPreset?.label || BUILTIN_LABELS[active] || active,
      model: model,
      operatorModel: model,
      classes: currentClassesFromSettings(settings),
      detection_mode: area,
      max_detections: maxDet,
      max_raw_candidates: Math.max(20, Number(old.max_raw_candidates || maxDet * 5)),
      detect_every_n_frames: every,
      tracking_mode: old.tracking_mode || settings.objectPresetTrackingMode || settings?.lastAppliedObjectPreset?.tracking_mode || 'single_auto',
      loss_behavior: old.loss_behavior || settings.objectPresetLossBehavior || settings?.lastAppliedObjectPreset?.loss_behavior || 'continuous_wide_scan_x',
      ptz: Object.assign({}, old.ptz || {}, currentPtzFromSettings(settings))
    });

    return { active, custom, preset };
  }

  async function saveActiveObjectPresetFromCurrentUi(reason) {
    const settings = await readSettings();
    const { active, custom, preset } = currentOperatorPreset(settings);

    custom[active] = preset;

    settings.objectPresetsCustom = custom;
    settings.activeObjectPreset = active;

    settings.operatorModel = preset.model || '';
    settings.operatorDetectionLimit = preset.max_detections;
    settings.operatorDetectEvery = preset.detect_every_n_frames;
    settings.operatorDetectionAreaMode = preset.detection_mode;
    settings.detectorSelectedClasses = preset.classes;

    settings.lastEditedObjectPreset = {
      name: active,
      label: preset.label || active,
      reason: reason || 'ui',
      ts: Math.floor(Date.now() / 1000)
    };

    await saveSettings(settings);

    const state = $('objectPresetState');
    if (state) {
      state.textContent = `preset: ${active} saved ${reason || ''}`;
    }

    const line = $('ptzStateLine');
    if (line) {
      line.textContent = `object preset saved: ${active} model=${shortModelName(preset.model)} every=${preset.detect_every_n_frames} area=${preset.detection_mode}`;
    }

    return preset;
  }

  async function applyPresetModelIfNeeded() {
    const settings = await readSettings();
    const active = activePresetName(settings);
    const preset = settings?.objectPresetsCustom?.[active];

    if (!preset) return;

    const model = String(preset.model || preset.operatorModel || '').trim();
    if (!model) return;

    const select = $('operatorModelSelect');

    if (select) {
      for (const opt of Array.from(select.options || [])) {
        if (opt.value === model || shortModelName(opt.value) === shortModelName(model)) {
          select.value = opt.value;
          break;
        }
      }
    }

    await postJson('/api/detector/config', {
      current_model: model
    }).catch(() => {});
  }

  let saveTimer = null;

  function schedulePresetSave(reason) {
    if (saveTimer) clearTimeout(saveTimer);

    saveTimer = setTimeout(() => {
      saveTimer = null;
      saveActiveObjectPresetFromCurrentUi(reason).catch((e) => {
        console.error('[object-preset-auto-learn]', e);
      });
    }, 350);
  }

  function ensureSaveButton() {
    if ($('objectPresetSaveCurrentBtn')) return;

    const panel = $('objectPresetPanel');
    if (!panel) return;

    const btn = document.createElement('button');
    btn.id = 'objectPresetSaveCurrentBtn';
    btn.className = 'small-btn';
    btn.textContent = 'SAVE CURRENT TO PRESET';
    btn.title = 'Save current Model / Limit / Detect FPS / Area into active object preset';

    btn.addEventListener('click', async (e) => {
      e.preventDefault();
      e.stopPropagation();
      await saveActiveObjectPresetFromCurrentUi('manual_button');
    }, true);

    panel.appendChild(btn);
  }

  function bindAutoLearn() {
    if (document.__objectPresetAutoLearnBound) return;
    document.__objectPresetAutoLearnBound = true;

    document.addEventListener('change', (e) => {
      const id = String(e.target?.id || '');

      if ([
        'operatorModelSelect',
        'operatorDetectionLimit',
        'operatorDetectEvery',
        'operatorDetectionAreaMode'
      ].includes(id)) {
        schedulePresetSave(id);
      }
    }, true);

    document.addEventListener('click', (e) => {
      const id = String(e.target?.id || '');

      if (id === 'operatorModelApplyBtn') {
        setTimeout(() => schedulePresetSave('operatorModelApplyBtn'), 700);
      }

      if (e.target && e.target.matches && e.target.matches('[data-object-preset]')) {
        setTimeout(() => applyPresetModelIfNeeded().catch(() => {}), 700);
      }
    }, true);
  }

  function boot() {
    ensureSaveButton();
    bindAutoLearn();
  }

  if (document.readyState === 'loading') {
    document.addEventListener('DOMContentLoaded', boot, { once: true });
  } else {
    boot();
  }

  setTimeout(boot, 500);
  setTimeout(boot, 1500);
  setTimeout(boot, 3000);

  window.saveActiveObjectPresetFromCurrentUi = saveActiveObjectPresetFromCurrentUi;
})();
// OBJECT_PRESET_AUTO_LEARN_OPERATOR_CONTROLS_END
</script>
'''

if "OBJECT_PRESET_AUTO_LEARN_OPERATOR_CONTROLS_START" not in s:
    if "</body>" not in s:
        raise SystemExit("ERROR: </body> not found")
    s = s.replace("</body>", script + "\n</body>", 1)
    p.write_text(s, encoding="utf-8")
    print("OK patched web/index.html")
    print("Backup:", bak)
else:
    print("OK already patched")
