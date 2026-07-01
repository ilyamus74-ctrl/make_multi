from pathlib import Path
import time

p = Path("web/index.html")
s = p.read_text(encoding="utf-8", errors="ignore")

bak = p.with_suffix(p.suffix + f".bak_single_preset_auto_arm_{int(time.time())}")
bak.write_text(s, encoding="utf-8")

script = r'''
<script>
// SINGLE_PRESET_AUTO_ARM_START
(function () {
  const $ = (id) => document.getElementById(id);

  const SINGLE_PRESETS = {
    person_single: [0],
    car_single: [2, 3, 5, 7],
    airplane_single: [4],
    bird_single: [14]
  };

  const MULTI_PRESETS = new Set([
    'people',
    'cars',
    'airplanes',
    'birds'
  ]);

  function sleep(ms) {
    return new Promise(resolve => setTimeout(resolve, ms));
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

  async function saveSettingsPatch(patch) {
    const old = await readSettings();
    const next = Object.assign({}, old || {}, patch || {});
    next.config_version = Math.max(16, Number(next.config_version || 16));

    try {
      localStorage.setItem('pantilt_ui_settings_v2', JSON.stringify(next));
    } catch (_) {}

    await postJson('/api/settings', next);
    return next;
  }

  function activePresetNameFromSettings(settings) {
    return String(
      settings?.lastAppliedObjectPreset?.name ||
      settings?.activeObjectPreset ||
      'person_single'
    );
  }

  function paintPreset(name) {
    document.querySelectorAll('[data-object-preset]').forEach(btn => {
      const on = btn.dataset.objectPreset === name;
      btn.classList.toggle('object-preset-active', on);
      btn.classList.toggle('active', on);
    });

    const state = $('objectPresetState');
    if (state) {
      state.textContent = `preset: ${name}`;
      state.classList.add('object-preset-active-state');
    }
  }

  function paintPtzRun(ap) {
    const running = ap && ap.enabled === true;

    const start = $('ptzStartBtn');
    const stop = $('ptzStopBtn');

    if (start) start.classList.toggle('ptz-run-active', running);
    if (stop) stop.classList.toggle('ptz-stop-active', !running);

    if ($('ptzStateLine') && ap) {
      const prefix = running ? 'PTZ ACTIVE' : 'PTZ STOPPED';
      const old = $('ptzStateLine').textContent || '';
      if (!old || old.startsWith('PTZ ACTIVE') || old.startsWith('PTZ STOPPED')) {
        $('ptzStateLine').textContent = `${prefix} mode=${ap.mode} enabled=${ap.enabled}`;
      }
    }
  }

  function ensureStyle() {
    if ($('singlePresetAutoArmStyle')) return;

    const st = document.createElement('style');
    st.id = 'singlePresetAutoArmStyle';
    st.textContent = `
      [data-object-preset].object-preset-active {
        outline: 2px solid #00ff66 !important;
        background: rgba(0,255,102,.22) !important;
        color: #fff !important;
        box-shadow: 0 0 8px rgba(0,255,102,.45) !important;
      }

      #objectPresetState.object-preset-active-state {
        color: #00ff66 !important;
        background: rgba(0,255,102,.14) !important;
      }

      #ptzStartBtn.ptz-run-active {
        outline: 2px solid #00ff66 !important;
        background: rgba(0,255,102,.22) !important;
        color: #fff !important;
      }

      #ptzStopBtn.ptz-stop-active {
        outline: 2px solid #ffcc00 !important;
        background: rgba(255,204,0,.18) !important;
        color: #fff !important;
      }
    `;
    document.head.appendChild(st);
  }

  async function chooseBestDetection(classes, attempts = 35, delayMs = 250) {
    const wanted = new Set((classes || []).map(Number));

    for (let i = 0; i < attempts; i++) {
      const det = await getJson('/api/detections');
      const items = Array.isArray(det.items) ? det.items : [];

      const candidates = items
        .filter(item => {
          const cls = Number(item.cls);
          return !wanted.size || wanted.has(cls);
        })
        .map(item => ({
          item,
          id: Number(item.id),
          score: Number(item.prop || item.score || item.conf || 0)
        }))
        .filter(x => Number.isFinite(x.id))
        .sort((a, b) => b.score - a.score);

      if (candidates.length) return candidates[0].item;

      if ($('ptzStateLine')) {
        $('ptzStateLine').textContent = `AUTO ARM waiting detections ${i + 1}/${attempts}`;
      }

      await sleep(delayMs);
    }

    throw new Error(`no matching detections for classes=${Array.from(wanted).join(',')}`);
  }

  async function autoSelectAndStartPreset(name) {
    const classes = SINGLE_PRESETS[name];

    if (!classes) {
      await saveSettingsPatch({
        activeObjectPreset: name,
        ptzArmed: false,
        lastAppliedObjectPreset: {
          name,
          tracking_mode: MULTI_PRESETS.has(name) ? 'multi_operator' : 'manual_select',
          ts: Math.floor(Date.now() / 1000)
        }
      });

      if ($('ptzStateLine')) {
        $('ptzStateLine').textContent = `preset=${name} applied, multi/manual target mode`;
      }

      return;
    }

    await saveSettingsPatch({
      activeObjectPreset: name,
      controlMode: 'ptz',
      ptzArmed: true,
      lastAppliedObjectPreset: {
        name,
        tracking_mode: 'single_auto',
        ts: Math.floor(Date.now() / 1000)
      }
    });

    paintPreset(name);

    if ($('controlMode')) $('controlMode').value = 'ptz';
    if (typeof safeSyncControlModeUi === 'function') safeSyncControlModeUi();

    if ($('ptzStateLine')) $('ptzStateLine').textContent = `AUTO ARM ${name}: waiting target...`;

    const target = await chooseBestDetection(classes, 35, 250);

    await postJson('/api/tracker/clear', {});
    await postJson('/api/tracker/select', { track_id: Number(target.id) });

    await sleep(450);

    const tr = await getJson('/api/tracker/state');

    if (tr.mode !== 'TRACKING' || tr.selected_box_valid !== true) {
      throw new Error(`tracker not ready: mode=${tr.mode} valid=${tr.selected_box_valid}`);
    }

    const apStart = await postJson(`${autopilotBaseUrl()}/api/autopilot/start`, {});
    await sleep(250);

    const ap = await getJson(`${autopilotBaseUrl()}/api/autopilot/state`);
    paintPtzRun(ap);

    if ($('ptzStateLine')) {
      $('ptzStateLine').textContent =
        `AUTO ARM OK preset=${name} target=${target.id} cls=${target.cls} PTZ=${ap.enabled}/${ap.mode}`;
    }

    return { target, tracker: tr, autopilot: apStart };
  }

  async function syncUiState() {
    ensureStyle();

    const settings = await readSettings();
    const name = activePresetNameFromSettings(settings);
    paintPreset(name);

    try {
      const ap = await getJson(`${autopilotBaseUrl()}/api/autopilot/state`);
      paintPtzRun(ap);
    } catch (_) {}
  }

  function bindPresetAutoArm() {
    document.addEventListener('click', event => {
      const btn = event.target && event.target.closest ? event.target.closest('[data-object-preset]') : null;
      if (!btn) return;

      const name = btn.dataset.objectPreset;
      paintPreset(name);

      setTimeout(async () => {
        try {
          await autoSelectAndStartPreset(name);
        } catch (err) {
          console.error('[single-preset-auto-arm]', err);
          await saveSettingsPatch({
            activeObjectPreset: name,
            ptzArmed: false
          }).catch(() => {});

          if ($('ptzStateLine')) {
            $('ptzStateLine').textContent = `AUTO ARM failed: ${String(err.message || err)}`;
          }
        }
      }, 1000);
    }, true);
  }

  function bindStartStopPersistence() {
    const start = $('ptzStartBtn');
    if (start && !start.__ptzArmPersistBound) {
      start.__ptzArmPersistBound = true;
      start.addEventListener('click', async () => {
        const settings = await readSettings();
        const name = activePresetNameFromSettings(settings);

        await saveSettingsPatch({
          activeObjectPreset: name,
          controlMode: 'ptz',
          ptzArmed: true
        }).catch(() => {});

        setTimeout(syncUiState, 800);
      }, true);
    }

    const stop = $('ptzStopBtn');
    if (stop && !stop.__ptzArmPersistBound) {
      stop.__ptzArmPersistBound = true;
      stop.addEventListener('click', async () => {
        await saveSettingsPatch({
          ptzArmed: false
        }).catch(() => {});

        setTimeout(syncUiState, 800);
      }, true);
    }
  }

  async function boot() {
    ensureStyle();
    bindStartStopPersistence();
    await syncUiState();
  }

  if (document.readyState === 'loading') {
    document.addEventListener('DOMContentLoaded', boot, { once: true });
  } else {
    boot();
  }

  bindPresetAutoArm();

  setTimeout(boot, 500);
  setTimeout(boot, 1500);
  setTimeout(boot, 3000);
  setInterval(syncUiState, 5000);

  window.singlePresetAutoArm = autoSelectAndStartPreset;
})();
// SINGLE_PRESET_AUTO_ARM_END
</script>
'''

if "SINGLE_PRESET_AUTO_ARM_START" not in s:
    if "</body>" not in s:
        raise SystemExit("ERROR: </body> not found")
    s = s.replace("</body>", script + "\n</body>", 1)
    p.write_text(s, encoding="utf-8")
    print(f"OK patched web/index.html")
    print(f"Backup: {bak}")
else:
    print("OK already patched")
