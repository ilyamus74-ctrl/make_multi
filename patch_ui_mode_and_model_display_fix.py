from pathlib import Path
import time

p = Path("web/index.html")
s = p.read_text(encoding="utf-8")
bak = p.with_suffix(p.suffix + f".bak_mode_model_display_fix_{int(time.time())}")
bak.write_text(s, encoding="utf-8")

changed = False

# 1) Add stronger model select CSS.
css_anchor = "#detectionToggleBtn.detect-off {\n"
css_patch = """
#operatorModelSelect {
  min-width: 360px;
  max-width: 720px;
}
#operatorModelSelect option {
  color: #000;
  background: #fff;
}
"""
if css_patch.strip() not in s:
    if css_anchor not in s:
        raise SystemExit("ERROR: CSS anchor not found")
    s = s.replace(css_anchor, css_patch + "\n" + css_anchor, 1)
    changed = True

# 2) Inject robust UI state synchronizer before main </script> end.
# Put it before the first QA_SWEEP block, still inside the main script scope.
anchor = "  // QA_SWEEP_GLOBAL_HELPERS_START\n"
patch = r'''
  // OPERATOR_STATE_RENDER_FIX_START
  function normalizeControlModeValue(v) {
    v = String(v || '').toLowerCase().trim();
    if (v === 'ptz') return 'ptz';
    if (v === 'auto') return 'auto';
    if (v === 'assist') return 'assist';
    return 'manual';
  }

  function renderControlModeButtons(mode) {
    mode = normalizeControlModeValue(mode);
    const setActive = (id, on) => {
      const el = $(id);
      if (!el) return;
      el.classList.toggle('active', !!on);
    };

    setActive('modeManualBtn', mode === 'manual');
    setActive('modePtzBtn', mode === 'ptz');
    setActive('modeAssistBtn', mode === 'assist');
    setActive('modeAutoBtn', mode === 'auto');

    setActive('ptzModeManualBtn', mode === 'manual');
    setActive('ptzModePtzBtn', mode === 'ptz');
    setActive('ptzModeAssistBtn', mode === 'assist');
    setActive('ptzModeAutoBtn', mode === 'auto');

    const chipText = mode === 'ptz' ? 'MODE: PTZ' : `MODE: ${mode.toUpperCase()}`;
    if ($('controlModeChip')) $('controlModeChip').textContent = chipText;
  }

  function forceControlModeUiSync(reason = 'sync') {
    try {
      const el = $('controlMode');
      const mode = normalizeControlModeValue(el?.value || 'manual');
      if (el) el.value = mode;
      renderControlModeButtons(mode);
      if ($('ptzStateLine') && reason === 'boot') {
        const prev = $('ptzStateLine').textContent || '';
        if (!prev || prev === 'idle') $('ptzStateLine').textContent = `mode=${mode}`;
      }
    } catch (e) {
      try { console.warn('forceControlModeUiSync failed', reason, e); } catch (_) {}
    }
  }

  async function loadRemoteModeAndRender() {
    try {
      const r = await fetch('/api/settings', { cache: 'no-store' });
      if (!r.ok) throw new Error(String(r.status));
      const cfg = await r.json();
      const mode = normalizeControlModeValue(cfg.controlMode || 'manual');
      if ($('controlMode')) $('controlMode').value = mode;
      renderControlModeButtons(mode);
    } catch (_) {
      forceControlModeUiSync('remote_failed');
    }
  }

  function shortModelName(model) {
    return String(model || '').split('/').filter(Boolean).pop() || String(model || '');
  }

  function dedupeModels(models) {
    const out = [];
    const seen = new Set();
    for (const m of Array.isArray(models) ? models : []) {
      const val = String(m || '').trim();
      if (!val) continue;
      const key = shortModelName(val);
      if (seen.has(key)) continue;
      seen.add(key);
      out.push(val);
    }
    return out;
  }

  // Override model select renderer: show basename, keep full path as value.
  window.syncOperatorModelSelectFixed = function(models, currentModel) {
    const select = $('operatorModelSelect');
    if (!select) return;

    const normalized = dedupeModels(models);
    const current = String(currentModel || '').trim();
    const currentBase = shortModelName(current);

    select.innerHTML = '';

    if (!normalized.length && current) normalized.push(current);

    if (!normalized.length) {
      const opt = document.createElement('option');
      opt.value = '';
      opt.textContent = 'no models';
      select.appendChild(opt);
      select.disabled = true;
      if ($('operatorModelState')) $('operatorModelState').textContent = 'model: unavailable';
      return;
    }

    for (const model of normalized) {
      const opt = document.createElement('option');
      opt.value = model;
      opt.textContent = shortModelName(model);
      opt.title = model;
      select.appendChild(opt);
    }

    select.disabled = false;

    let selectedValue = '';
    for (const opt of [...select.options]) {
      if (opt.value === current || shortModelName(opt.value) === currentBase) {
        selectedValue = opt.value;
        break;
      }
    }
    select.value = selectedValue || normalized[0];

    if ($('operatorModelState')) $('operatorModelState').textContent = `model: ${shortModelName(select.value)}`;
  };

  // Wrap old refreshDetectorConfig if it exists.
  const oldRefreshDetectorConfig = typeof refreshDetectorConfig === 'function' ? refreshDetectorConfig : null;
  if (oldRefreshDetectorConfig && !window.__refreshDetectorConfigModeModelFixWrapped) {
    window.__refreshDetectorConfigModeModelFixWrapped = true;
    refreshDetectorConfig = async function() {
      const cfg = await apiGetJson('/api/detector/config');
      detectorEnabled = parseBool(cfg.detect_enabled, false);
      detectorSelectedClasses = Array.isArray(cfg.selected_classes) ? cfg.selected_classes.map(v => Number(v)).filter(Number.isFinite) : [];
      setDetectionClassCheckboxes(detectorSelectedClasses);
      window.syncOperatorModelSelectFixed(Array.isArray(cfg.models) ? cfg.models : [], String(cfg.current_model || ''));
      updateDetectionUi(cfg);
      return cfg;
    };
  }

  // Hard boot sync. This is intentionally independent from the old mode code.
  setTimeout(() => loadRemoteModeAndRender(), 100);
  setTimeout(() => forceControlModeUiSync('boot'), 300);
  setTimeout(() => forceControlModeUiSync('boot'), 1000);
  setInterval(() => forceControlModeUiSync('interval'), 2000);
  // OPERATOR_STATE_RENDER_FIX_END

'''
if "OPERATOR_STATE_RENDER_FIX_START" not in s:
    if anchor not in s:
        raise SystemExit("ERROR: QA_SWEEP anchor not found")
    s = s.replace(anchor, patch + "\n" + anchor, 1)
    changed = True

# 3) Fix obvious recursion bug if still present.
bad = "safeSyncControlModeUi();\n        return;"
good = "syncControlModeUi();\n        return;"
if bad in s:
    s = s.replace(bad, good, 1)
    changed = True

if changed:
    p.write_text(s, encoding="utf-8")
    print(f"OK patched {p}, backup={bak}")
else:
    print("OK already patched")
