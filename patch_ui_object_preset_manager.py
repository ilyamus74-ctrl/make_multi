from pathlib import Path
import time

p = Path("web/index.html")
s = p.read_text(encoding="utf-8")

bak = p.with_suffix(p.suffix + f".bak_object_preset_manager_{int(time.time())}")
bak.write_text(s, encoding="utf-8")

script = r'''
<script>
// OBJECT_PRESET_MANAGER_START
(function () {
  const $ = (id) => document.getElementById(id);

  const BUILTIN_PRESETS = {
    person_single: {
      label: 'ЧЕЛОВЕК',
      classes: [0],
      detection_mode: 'full_frame',
      max_detections: 5,
      max_raw_candidates: 25,
      detect_every_n_frames: 1,
      ptz: {
        target_x: 0.50,
        target_y: 0.46,
        min_pan: 3,
        min_tilt: 3,
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
      ptz: {
        target_x: 0.50,
        target_y: 0.47,
        min_pan: 3,
        min_tilt: 3,
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
      ptz: {
        target_x: 0.50,
        target_y: 0.50,
        min_pan: 3,
        min_tilt: 2,
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
      ptz: {
        target_x: 0.50,
        target_y: 0.50,
        min_pan: 3,
        min_tilt: 2,
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
      ptz: {
        target_x: 0.50,
        target_y: 0.45,
        min_pan: 4,
        min_tilt: 3,
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
      ptz: {
        target_x: 0.50,
        target_y: 0.45,
        min_pan: 4,
        min_tilt: 3,
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
      ptz: {
        target_x: 0.50,
        target_y: 0.45,
        min_pan: 4,
        min_tilt: 4,
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
      ptz: {
        target_x: 0.50,
        target_y: 0.45,
        min_pan: 4,
        min_tilt: 4,
        auto_zoom_enable: true,
        auto_zoom_target_h: 0.20,
        auto_zoom_deadzone: 0.08,
        auto_zoom_cmd: 12,
        auto_zoom_sign: 1,
        auto_zoom_period_ms: 300
      }
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
      return await getJson('/api/settings');
    } catch (_) {
      return {};
    }
  }

  async function saveSettingsMerge(patch) {
    const cfg = await readSettings();
    const next = Object.assign({}, cfg || {}, patch || {});
    next.config_version = Math.max(16, Number(next.config_version || 16));

    try {
      localStorage.setItem('pantilt_ui_settings_v2', JSON.stringify(next));
    } catch (_) {}

    await postJson('/api/settings', next);
    return next;
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
    return { id: 'sky_top', enabled: true, x: 0, y: 0, w, h: Math.max(1, Math.round(h * 0.68)), every_n_frames: 1, classes: [] };
  }

  function centerRoi(w, h) {
    const rw = Math.round(w * 0.60);
    const rh = Math.round(h * 0.55);
    return { id: 'center', enabled: true, x: Math.max(0, Math.round((w - rw) / 2)), y: Math.max(0, Math.round((h - rh) / 2)), w: Math.max(1, rw), h: Math.max(1, rh), every_n_frames: 1, classes: [] };
  }

  function fullRoi(w, h) {
    return { id: 'full_frame', enabled: true, x: 0, y: 0, w, h, every_n_frames: 1, classes: [] };
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

  async function applyPreset(name, preset) {
    const classes = Array.isArray(preset.classes) ? preset.classes.map(Number).filter(Number.isFinite) : [];
    const mode = String(preset.detection_mode || 'full_frame');
    const maxDet = Number(preset.max_detections || 10);
    const maxRaw = Number(preset.max_raw_candidates || Math.max(20, maxDet * 5));
    const every = Number(preset.detect_every_n_frames || 1);
    const ptz = Object.assign({}, preset.ptz || {});

    await postJson('/api/detector/config', {
      detect_enabled: true,
      selected_classes: classes
    });

    await postJson('/api/detection/limits', {
      max_detections: maxDet,
      max_raw_candidates: maxRaw
    });

    await postJson('/api/detection/throttle', {
      detect_every_n_frames: every
    });

    await postJson('/api/detection/roi_config', await roiPayload(mode));

    if (Object.keys(ptz).length) {
      await postJson(`${autopilotBaseUrl()}/api/autopilot/config`, ptz);
    }

    await saveSettingsMerge({
      activeObjectPreset: name,
      detectorEnabled: true,
      detectorSelectedClasses: classes,
      operatorDetectionLimit: maxDet,
      operatorDetectEvery: every,
      operatorDetectionAreaMode: mode,
      ptzConfig: {
        target_x: ptz.target_x,
        target_y: ptz.target_y,
        min_pan: ptz.min_pan,
        min_tilt: ptz.min_tilt
      },
      ptzAutoZoom: ptz
    });

    try {
      if (typeof window.refreshDetectionUiHard === 'function') await window.refreshDetectionUiHard();
    } catch (_) {}

    const line = $('ptzStateLine');
    if (line) line.textContent = `preset=${name} applied`;
  }

  function allPresetsFromSettings(settings) {
    return Object.assign({}, BUILTIN_PRESETS, settings.objectPresetsCustom || {});
  }

  function ensurePresetPanel() {
    const detMenu = $('detectionMenu');
    if (!detMenu || $('objectPresetPanel')) return;

    const box = document.createElement('div');
    box.id = 'objectPresetPanel';
    box.className = 'menu-body';
    box.style.marginTop = '6px';
    box.innerHTML = `
      <strong>OBJECT PRESETS:</strong>
      <button class="small-btn" data-object-preset="person_single">ЧЕЛОВЕК</button>
      <button class="small-btn" data-object-preset="people">ЛЮДИ</button>
      <button class="small-btn" data-object-preset="car_single">МАШИНА</button>
      <button class="small-btn" data-object-preset="cars">МАШИНЫ</button>
      <button class="small-btn" data-object-preset="airplane_single">САМОЛЁТ</button>
      <button class="small-btn" data-object-preset="airplanes">САМОЛЁТЫ</button>
      <button class="small-btn" data-object-preset="bird_single">ПТИЦА</button>
      <button class="small-btn" data-object-preset="birds">ПТИЦЫ</button>
      <button id="objectPresetCustomBtn" class="small-btn">CUSTOM</button>
      <code id="objectPresetState">preset: --</code>
    `;

    detMenu.appendChild(box);
  }

  function ensureCustomDialog() {
    if ($('objectPresetDialog')) return;

    const dlg = document.createElement('dialog');
    dlg.id = 'objectPresetDialog';
    dlg.style.maxWidth = '760px';
    dlg.style.background = '#111';
    dlg.style.color = '#eee';
    dlg.style.border = '1px solid #444';
    dlg.style.borderRadius = '8px';
    dlg.innerHTML = `
      <form method="dialog" style="display:grid;gap:8px;">
        <h3>Custom object preset</h3>

        <label>Name/id <input id="customPresetName" value="custom_room" style="width:260px"></label>
        <label>Label <input id="customPresetLabel" value="Мой пресет" style="width:260px"></label>

        <fieldset>
          <legend>Objects / COCO classes</legend>
          <label><input type="checkbox" class="customCls" value="0"> person</label>
          <label><input type="checkbox" class="customCls" value="2"> car</label>
          <label><input type="checkbox" class="customCls" value="3"> motorcycle</label>
          <label><input type="checkbox" class="customCls" value="5"> bus</label>
          <label><input type="checkbox" class="customCls" value="7"> truck</label>
          <label><input type="checkbox" class="customCls" value="4"> airplane</label>
          <label><input type="checkbox" class="customCls" value="14"> bird</label>
        </fieldset>

        <label>Area
          <select id="customPresetArea">
            <option value="full_frame">FULL FRAME</option>
            <option value="roi">ROI SKY</option>
            <option value="multi_roi">MULTI ROI</option>
            <option value="tiled">TILED 2x2</option>
            <option value="hybrid">HYBRID FULL+ROI</option>
          </select>
        </label>

        <label>Limit <input id="customPresetLimit" type="number" min="1" max="50" value="10"></label>
        <label>Detect every N frames <input id="customPresetEvery" type="number" min="1" max="20" value="1"></label>

        <fieldset>
          <legend>Framing / PTZ</legend>
          <label>Center X <input id="customPresetTargetX" type="number" min="0.1" max="0.9" step="0.01" value="0.50"></label>
          <label>Center Y <input id="customPresetTargetY" type="number" min="0.1" max="0.9" step="0.01" value="0.46"></label>
          <label>Min Pan <input id="customPresetMinPan" type="number" min="0" max="50" value="3"></label>
          <label>Min Tilt <input id="customPresetMinTilt" type="number" min="0" max="50" value="3"></label>
        </fieldset>

        <fieldset>
          <legend>Auto Zoom</legend>
          <label><input id="customPresetAutoZoom" type="checkbox" checked> enabled</label>
          <label>Target object height <input id="customPresetZoomH" type="number" min="0.15" max="0.95" step="0.01" value="0.60"></label>
          <label>Deadzone <input id="customPresetZoomDz" type="number" min="0.02" max="0.30" step="0.01" value="0.08"></label>
          <label>Zoom cmd <input id="customPresetZoomCmd" type="number" min="1" max="60" value="10"></label>
          <label>Zoom sign
            <select id="customPresetZoomSign">
              <option value="1">normal</option>
              <option value="-1">invert</option>
            </select>
          </label>
        </fieldset>

        <p style="opacity:.8;">
          Подсказка: человек в комнате — person + FULL FRAME + height 0.65–0.72.
          Птица/самолёт — bird/airplane + HYBRID/ROI + height 0.20–0.35.
        </p>

        <menu>
          <button value="cancel">Cancel</button>
          <button id="customPresetSaveBtn" value="default">Save preset</button>
          <button id="customPresetSaveApplyBtn" value="default">Save + Apply</button>
        </menu>
      </form>
    `;

    document.body.appendChild(dlg);
  }

  function readCustomPresetFromDialog() {
    const name = String($('customPresetName')?.value || 'custom_preset').trim().replace(/[^a-zA-Z0-9_-]/g, '_');
    const label = String($('customPresetLabel')?.value || name).trim();
    const classes = Array.from(document.querySelectorAll('.customCls:checked')).map(x => Number(x.value)).filter(Number.isFinite);
    const maxDet = Number($('customPresetLimit')?.value || 10);

    return {
      name,
      preset: {
        label,
        classes,
        detection_mode: String($('customPresetArea')?.value || 'full_frame'),
        max_detections: maxDet,
        max_raw_candidates: Math.max(20, maxDet * 5),
        detect_every_n_frames: Number($('customPresetEvery')?.value || 1),
        ptz: {
          target_x: Number($('customPresetTargetX')?.value || 0.5),
          target_y: Number($('customPresetTargetY')?.value || 0.46),
          min_pan: Number($('customPresetMinPan')?.value || 3),
          min_tilt: Number($('customPresetMinTilt')?.value || 3),
          auto_zoom_enable: Boolean($('customPresetAutoZoom')?.checked),
          auto_zoom_target_h: Number($('customPresetZoomH')?.value || 0.60),
          auto_zoom_deadzone: Number($('customPresetZoomDz')?.value || 0.08),
          auto_zoom_cmd: Number($('customPresetZoomCmd')?.value || 10),
          auto_zoom_sign: Number($('customPresetZoomSign')?.value || 1),
          auto_zoom_period_ms: 350
        }
      }
    };
  }

  async function saveCustomPreset(applyNow) {
    const data = readCustomPresetFromDialog();
    const settings = await readSettings();
    const custom = Object.assign({}, settings.objectPresetsCustom || {});
    custom[data.name] = data.preset;

    await saveSettingsMerge({
      objectPresetsCustom: custom,
      activeObjectPreset: data.name
    });

    if (applyNow) {
      await applyPreset(data.name, data.preset);
    }

    renderActivePreset(data.name);
    return data;
  }

  function renderActivePreset(name) {
    const el = $('objectPresetState');
    if (el) el.textContent = `preset: ${name || '--'}`;

    document.querySelectorAll('[data-object-preset]').forEach(btn => {
      btn.classList.toggle('active', btn.dataset.objectPreset === name);
    });
  }

  function bindPresetButtons() {
    document.querySelectorAll('[data-object-preset]').forEach(btn => {
      if (btn.__presetBound) return;
      btn.__presetBound = true;

      btn.addEventListener('click', async (e) => {
        e.preventDefault();
        e.stopPropagation();
        e.stopImmediatePropagation();

        const name = btn.dataset.objectPreset;
        const settings = await readSettings();
        const presets = allPresetsFromSettings(settings);

        if (!presets[name]) return;

        try {
          await applyPreset(name, presets[name]);
          renderActivePreset(name);
        } catch (err) {
          console.error('[object-preset]', name, err);
          const line = $('ptzStateLine');
          if (line) line.textContent = `preset error ${name}: ${String(err.message || err)}`;
        }
      }, true);
    });

    const customBtn = $('objectPresetCustomBtn');
    if (customBtn && !customBtn.__presetCustomBound) {
      customBtn.__presetCustomBound = true;
      customBtn.addEventListener('click', (e) => {
        e.preventDefault();
        ensureCustomDialog();
        $('objectPresetDialog')?.showModal();
      }, true);
    }

    const saveBtn = $('customPresetSaveBtn');
    if (saveBtn && !saveBtn.__presetSaveBound) {
      saveBtn.__presetSaveBound = true;
      saveBtn.addEventListener('click', async (e) => {
        e.preventDefault();
        await saveCustomPreset(false);
        $('objectPresetDialog')?.close();
      }, true);
    }

    const saveApplyBtn = $('customPresetSaveApplyBtn');
    if (saveApplyBtn && !saveApplyBtn.__presetSaveApplyBound) {
      saveApplyBtn.__presetSaveApplyBound = true;
      saveApplyBtn.addEventListener('click', async (e) => {
        e.preventDefault();
        await saveCustomPreset(true);
        $('objectPresetDialog')?.close();
      }, true);
    }
  }

  async function boot() {
    ensurePresetPanel();
    ensureCustomDialog();
    bindPresetButtons();

    const settings = await readSettings();
    renderActivePreset(settings.activeObjectPreset || 'person_single');
  }

  if (document.readyState === 'loading') {
    document.addEventListener('DOMContentLoaded', boot, { once: true });
  } else {
    boot();
  }

  setTimeout(boot, 500);
  setTimeout(boot, 1500);
  setTimeout(boot, 3000);
})();
// OBJECT_PRESET_MANAGER_END
</script>
'''

if "OBJECT_PRESET_MANAGER_START" not in s:
    if "</body>" not in s:
        raise SystemExit("ERROR: </body> not found")
    s = s.replace("</body>", script + "\n</body>", 1)
    p.write_text(s, encoding="utf-8")
    print(f"OK patched web/index.html, backup={bak}")
else:
    print("OK already installed")
