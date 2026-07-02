from pathlib import Path
import time
import re

ROOT = Path('/root/new_yolo8')
P = ROOT / 'web' / 'index.html'

s = P.read_text(encoding='utf-8', errors='ignore')
bak = P.with_suffix(P.suffix + f'.bak_ui_detection_hydrate_f5_v1_{int(time.time())}')
bak.write_text(s, encoding='utf-8')

changes = []

# 1) Strengthen the clean saveSettings guard so legacy saveSettings() cannot
# overwrite persistent detection fields from stale DOM defaults during boot/F5.
old_keep = """        objectPresetTrackingMode: before.objectPresetTrackingMode,
        objectPresetLossBehavior: before.objectPresetLossBehavior
      };"""
new_keep = """        objectPresetTrackingMode: before.objectPresetTrackingMode,
        objectPresetLossBehavior: before.objectPresetLossBehavior,
        operatorModel: before.operatorModel,
        detectorSelectedClasses: before.detectorSelectedClasses,
        operatorDetectionLimit: before.operatorDetectionLimit,
        operatorDetectEvery: before.operatorDetectEvery,
        operatorDetectionAreaMode: before.operatorDetectionAreaMode
      };"""

if old_keep in s and 'operatorDetectionAreaMode: before.operatorDetectionAreaMode' not in s:
    s = s.replace(old_keep, new_keep, 1)
    changes.append('extended saveSettings guard with detection fields')
elif 'operatorDetectionAreaMode: before.operatorDetectionAreaMode' in s:
    changes.append('saveSettings guard already has detection fields')
else:
    raise SystemExit('anchor not found: saveSettings keep object')

# 2) Insert a read-only UI hydrate helper inside PTZ clean layer.
helper_marker = 'PTZ_CLEAN_DETECTION_UI_HYDRATE_F5_START'
helper = r'''

  // PTZ_CLEAN_DETECTION_UI_HYDRATE_F5_START
  function cleanNumber(value, fallback) {
    const n = Number(value);
    return Number.isFinite(n) ? n : fallback;
  }

  function activeDetectionValuesFromSettings(settings) {
    settings = settings || {};
    const name = activeObjectName(settings);
    const preset = getActivePreset(settings) || {};

    const maxDet = cleanNumber(
      preset.max_detections ?? settings.operatorDetectionLimit,
      10
    );

    const maxRaw = cleanNumber(
      preset.max_raw_candidates,
      Math.max(20, maxDet * 5)
    );

    const every = cleanNumber(
      preset.detect_every_n_frames ?? settings.operatorDetectEvery,
      1
    );

    const area = String(
      preset.detection_mode || settings.operatorDetectionAreaMode || 'full_frame'
    );

    return { name, preset, maxDet, maxRaw, every, area };
  }

  async function hydrateDetectionControlsFromSettings(reason) {
    const settings = await readSettings();
    const d = activeDetectionValuesFromSettings(settings);

    cleanActiveObjectPresetName = d.name;

    if ($('operatorDetectionLimit')) {
      $('operatorDetectionLimit').value = String(d.maxDet);
    }

    if ($('operatorLimitState')) {
      $('operatorLimitState').textContent = `limit: ${d.maxDet}/${d.maxRaw}`;
    }

    if ($('operatorDetectEvery')) {
      $('operatorDetectEvery').value = String(d.every);
    }

    if ($('operatorDetectEveryState')) {
      $('operatorDetectEveryState').textContent = `detect: 1/${d.every}`;
    }

    if ($('operatorDetectionAreaMode')) {
      $('operatorDetectionAreaMode').value = d.area;
    }

    if ($('operatorDetectionAreaState')) {
      $('operatorDetectionAreaState').textContent = `area: ${d.area}`;
    }

    if ($('operatorModelState')) {
      const model = d.preset.model || d.preset.operatorModel || settings.operatorModel || '';
      if (model) $('operatorModelState').textContent = `model: ${model.split('/').pop()}`;
    }

    paintObjectPreset(d.name);

    if ($('objectPresetState')) {
      $('objectPresetState').textContent = `preset: ${d.name} limit=${d.maxDet} detect=1/${d.every} area=${d.area}`;
    }

    try {
      if (typeof updateOperatorMenuSummaries === 'function') updateOperatorMenuSummaries();
    } catch (_) {}

    window.__ptzCleanLayerHydrated = true;
    window.__ptzCleanLayerLastHydrate = {
      reason: reason || 'manual',
      activeObjectPreset: d.name,
      operatorDetectionLimit: d.maxDet,
      operatorDetectEvery: d.every,
      operatorDetectionAreaMode: d.area,
      ts: Date.now()
    };

    return window.__ptzCleanLayerLastHydrate;
  }

  function scheduleDetectionUiHydrate(reason) {
    [0, 250, 750, 1500, 3000, 5000, 8000, 12000].forEach((ms) => {
      setTimeout(() => {
        hydrateDetectionControlsFromSettings(`${reason || 'boot'}:${ms}`).catch((err) => {
          console.error('[ptz-clean-detection-hydrate]', err);
        });
      }, ms);
    });
  }
  // PTZ_CLEAN_DETECTION_UI_HYDRATE_F5_END
'''

if helper_marker not in s:
    anchor = '  function bindHandlers() {'
    idx = s.find(anchor)
    if idx < 0:
        raise SystemExit('anchor not found: function bindHandlers')
    s = s[:idx] + helper + '\n' + s[idx:]
    changes.append('inserted detection UI hydrate helper')
else:
    changes.append('detection UI hydrate helper already present')

# 3) Schedule hydrate during clean layer boot.
if 'scheduleDetectionUiHydrate(\'clean_layer_boot\')' not in s:
    boot_anchor = '  boot();\n  setTimeout(boot, 500);'
    boot_repl = "  boot();\n  scheduleDetectionUiHydrate('clean_layer_boot');\n  setTimeout(boot, 500);"
    if boot_anchor in s:
        s = s.replace(boot_anchor, boot_repl, 1)
        changes.append('scheduled detection hydrate after boot')
    else:
        export_anchor = '  window.ptzCleanLayer = {'
        idx = s.find(export_anchor)
        if idx < 0:
            raise SystemExit('anchor not found: boot schedule/export')
        s = s[:idx] + "  scheduleDetectionUiHydrate('clean_layer_boot');\n\n" + s[idx:]
        changes.append('scheduled detection hydrate before export')
else:
    changes.append('detection hydrate schedule already present')

# 4) Export manual rebuild/hydrate handles.
if 'hydrateDetectionControlsFromSettings' not in re.search(r'window\.ptzCleanLayer\s*=\s*\{.*?\n\s*\};', s, re.S).group(0):
    m = re.search(r'(  window\.ptzCleanLayer\s*=\s*\{)(.*?)(\n  \};)', s, re.S)
    if not m:
        raise SystemExit('anchor not found: window.ptzCleanLayer export block')

    head, body, tail = m.group(1), m.group(2), m.group(3)
    body_r = body.rstrip()
    if body_r and not body_r.endswith(','):
        body_r += ','

    addition = """
    hydrateDetectionControlsFromSettings,
    rebuildPanels: async function () {
      await boot();
      return hydrateDetectionControlsFromSettings('manual_rebuild');
    }"""

    new_block = head + body_r + addition + tail
    s = s[:m.start()] + new_block + s[m.end():]
    changes.append('exported hydrate/rebuild helpers')
else:
    changes.append('hydrate helper already exported')

P.write_text(s, encoding='utf-8')

print('OK patched web/index.html UI detection hydrate F5 v1')
print('Backup:', bak)
print('Changes:')
for c in changes:
    print(' -', c)