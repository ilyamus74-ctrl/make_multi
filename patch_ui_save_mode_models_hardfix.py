from pathlib import Path
import time

p = Path("web/index.html")
s = p.read_text(encoding="utf-8")
bak = p.with_suffix(p.suffix + f".bak_save_mode_models_hardfix_{int(time.time())}")
bak.write_text(s, encoding="utf-8")

changed = False

# 1) Add fallback normalizeStreamUrl before restoreSettings/buildSettingsPayload uses it.
anchor = "  async function restoreSettings() {\n"
helper = r'''
  // SAVE_MODE_MODELS_HARDFIX_START
  function normalizeStreamUrl(value) {
    const raw = String(value || '').trim();
    if (!raw) {
      try {
        if (typeof defaultMjpegUrl === 'function') return defaultMjpegUrl();
      } catch (_) {}
      return `${location.protocol}//${location.hostname}:8080/stream`;
    }

    try {
      const u = new URL(raw, location.href);
      if (!u.pathname || u.pathname === '/') u.pathname = '/stream';
      return u.toString();
    } catch (_) {
      return raw;
    }
  }
  // SAVE_MODE_MODELS_HARDFIX_END

'''
if "SAVE_MODE_MODELS_HARDFIX_START" not in s:
    if anchor not in s:
        raise SystemExit("ERROR: restoreSettings anchor not found")
    s = s.replace(anchor, helper + anchor, 1)
    changed = True

# 2) Add independent late boot script before </body>.
late_script = r'''
<script>
// SAVE_MODE_MODELS_LATE_FIX_START
(function () {
  const $ = (id) => document.getElementById(id);

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

  function modeNorm(v) {
    v = String(v || '').toLowerCase().trim();
    if (v === 'ptz') return 'ptz';
    if (v === 'auto') return 'auto';
    if (v === 'assist') return 'assist';
    return 'manual';
  }

  function renderMode(mode) {
    mode = modeNorm(mode);

    const setActive = (id, on) => {
      const el = $(id);
      if (!el) return;
      el.classList.toggle('active', !!on);
    };

    if ($('controlMode')) $('controlMode').value = mode;

    setActive('modeManualBtn', mode === 'manual');
    setActive('modePtzBtn', mode === 'ptz');
    setActive('modeAssistBtn', mode === 'assist');
    setActive('modeAutoBtn', mode === 'auto');

    setActive('ptzModeManualBtn', mode === 'manual');
    setActive('ptzModePtzBtn', mode === 'ptz');
    setActive('ptzModeAssistBtn', mode === 'assist');
    setActive('ptzModeAutoBtn', mode === 'auto');

    if ($('controlModeChip')) $('controlModeChip').textContent = `MODE: ${mode.toUpperCase()}`;

    const line = $('ptzStateLine');
    if (line && (!line.textContent || line.textContent === 'idle' || /^mode=/.test(line.textContent))) {
      line.textContent = `mode=${mode}`;
    }
  }

  async function readSettings() {
    try {
      const cfg = await getJson('/api/settings');
      return cfg && typeof cfg === 'object' ? cfg : {};
    } catch (_) {
      return {};
    }
  }

  async function saveMode(mode) {
    mode = modeNorm(mode);
    const cfg = await readSettings();
    cfg.config_version = Math.max(16, Number(cfg.config_version || 16));
    cfg.controlMode = mode;

    try {
      localStorage.setItem('pantilt_ui_settings_v2', JSON.stringify(cfg));
    } catch (_) {}

    await postJson('/api/settings', cfg);
    renderMode(mode);
    return cfg;
  }

  function shortModelName(v) {
    return String(v || '').split('/').filter(Boolean).pop() || String(v || '');
  }

  function dedupeModels(models) {
    const out = [];
    const seen = new Set();

    for (const m of Array.isArray(models) ? models : []) {
      const val = String(m || '').trim();
      if (!val) continue;

      const base = shortModelName(val);
      if (seen.has(base)) continue;

      seen.add(base);
      out.push(val);
    }

    return out;
  }

  async function refreshModelsHard() {
    const select = $('operatorModelSelect');
    if (!select) return;

    let cfg = null;
    try {
      cfg = await getJson('/api/detector/config');
    } catch (e) {
      if ($('operatorModelState')) $('operatorModelState').textContent = `model: api error ${String(e.message || e)}`;
      return;
    }

    const current = String(cfg.current_model || cfg.current || '').trim();
    const currentBase = shortModelName(current);
    const models = dedupeModels(cfg.models || []);

    select.innerHTML = '';

    if (!models.length && current) models.push(current);

    if (!models.length) {
      const opt = document.createElement('option');
      opt.value = '';
      opt.textContent = 'no models';
      select.appendChild(opt);
      select.disabled = true;
      if ($('operatorModelState')) $('operatorModelState').textContent = 'model: no models from API';
      return;
    }

    for (const model of models) {
      const opt = document.createElement('option');
      opt.value = model;
      opt.textContent = shortModelName(model);
      opt.title = model;
      select.appendChild(opt);
    }

    select.disabled = false;

    let selected = '';
    for (const opt of Array.from(select.options)) {
      if (opt.value === current || shortModelName(opt.value) === currentBase) {
        selected = opt.value;
        break;
      }
    }

    select.value = selected || models[0];

    if ($('operatorModelState')) {
      $('operatorModelState').textContent = `model: ${shortModelName(select.value)}`;
    }
  }

  function installModeHandlers() {
    const bind = (id, mode) => {
      const el = $(id);
      if (!el || el.__hardModeBound) return;
      el.__hardModeBound = true;

      el.addEventListener('click', async (e) => {
        e.preventDefault();
        e.stopPropagation();
        e.stopImmediatePropagation();

        try {
          if (mode === 'manual') {
            await postJson(`${autopilotBaseUrl()}/api/autopilot/stop`, {}).catch(() => {});
            await postJson(`${autopilotBaseUrl()}/api/control/stop`, {}).catch(() => {});
          }

          await saveMode(mode);

          if (mode === 'ptz') {
            try {
              const tr = await getJson('/api/tracker/state');
              const ap = await getJson(`${autopilotBaseUrl()}/api/autopilot/state`);
              if ($('ptzStateLine')) {
                $('ptzStateLine').textContent =
                  `mode=ptz tracker=${tr.mode} id=${tr.selected_track_id} valid=${tr.selected_box_valid} ptz=${ap.enabled}/${ap.mode}`;
              }
            } catch (_) {}
          }
        } catch (err) {
          if ($('ptzStateLine')) $('ptzStateLine').textContent = `mode save error: ${String(err.message || err)}`;
          console.error('[mode hardfix]', err);
        }
      }, true);
    };

    bind('modeManualBtn', 'manual');
    bind('ptzModeManualBtn', 'manual');
    bind('modePtzBtn', 'ptz');
    bind('ptzModePtzBtn', 'ptz');
  }

  function installModelApplyHandler() {
    const btn = $('operatorModelApplyBtn');
    const select = $('operatorModelSelect');
    if (!btn || !select || btn.__hardModelBound) return;

    btn.__hardModelBound = true;
    btn.addEventListener('click', async (e) => {
      e.preventDefault();
      e.stopPropagation();
      e.stopImmediatePropagation();

      const model = String(select.value || '').trim();
      if (!model) return;

      if ($('operatorModelState')) $('operatorModelState').textContent = `switching: ${shortModelName(model)}`;

      try {
        await postJson(`${autopilotBaseUrl()}/api/autopilot/stop`, {}).catch(() => {});
        await postJson('/api/detector/config', { current_model: model });
        await refreshModelsHard();
      } catch (err) {
        if ($('operatorModelState')) $('operatorModelState').textContent = `model error: ${String(err.message || err)}`;
        console.error('[model hardfix]', err);
      }
    }, true);
  }

  async function bootHardFix() {
    installModeHandlers();
    installModelApplyHandler();

    const cfg = await readSettings();
    renderMode(cfg.controlMode || 'manual');

    await refreshModelsHard().catch((e) => console.error('[models hardfix]', e));
  }

  if (document.readyState === 'loading') {
    document.addEventListener('DOMContentLoaded', bootHardFix, { once: true });
  } else {
    bootHardFix();
  }

  setTimeout(bootHardFix, 500);
  setTimeout(bootHardFix, 1500);
  setTimeout(refreshModelsHard, 3000);
})();
// SAVE_MODE_MODELS_LATE_FIX_END
</script>
'''

if "SAVE_MODE_MODELS_LATE_FIX_START" not in s:
    if "</body>" not in s:
        raise SystemExit("ERROR: </body> not found")
    s = s.replace("</body>", late_script + "\n</body>", 1)
    changed = True

if changed:
    p.write_text(s, encoding="utf-8")
    print(f"OK patched {p}, backup={bak}")
else:
    print("OK already patched")
