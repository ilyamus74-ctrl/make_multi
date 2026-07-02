#!/usr/bin/env python3
from pathlib import Path
import time
import re

ROOT = Path('/root/new_yolo8')
if not ROOT.exists():
    ROOT = Path.cwd()

p = ROOT / 'web' / 'index.html'
s = p.read_text(encoding='utf-8', errors='ignore')

bak = p.with_suffix(p.suffix + f'.bak_ui_persist_detection_to_active_preset_v3_{int(time.time())}')
bak.write_text(s, encoding='utf-8')

changed = []

# 1) Strengthen saveSettings guard: never preserve an empty objectPresetsCustom.
old = "objectPresetsCustom: before.objectPresetsCustom,"
new = "objectPresetsCustom: (before.objectPresetsCustom && Object.keys(before.objectPresetsCustom).length ? before.objectPresetsCustom : Object.assign({}, OBJECT_PRESETS || {})),"
if old in s and new not in s:
    s = s.replace(old, new, 1)
    changed.append('saveSettings guard: fallback objectPresetsCustom to OBJECT_PRESETS when empty')

# 2) Insert persist helper inside the clean layer, before bindHandlers(), where OBJECT_PRESETS/readSettings/writeSettings are in scope.
marker = "\n  function bindHandlers() {\n"
helper = r'''

  // PTZ_CLEAN_DETECTION_ACTIVE_PRESET_PERSIST_START
  async function persistActiveDetectionControlsToPreset(reason = 'ui_change') {
    const settings = await readSettings();
    const active = activeObjectName(settings);
    const presets = mergedPresets(settings);
    const base = Object.assign({}, OBJECT_PRESETS[active] || {}, presets[active] || {});

    const limitEl = $('operatorDetectionLimit');
    const everyEl = $('operatorDetectEvery');
    const areaEl = $('operatorDetectionAreaMode');

    const maxDet = Number(limitEl?.value || settings.operatorDetectionLimit || base.max_detections || 10);
    const maxRaw = Math.max(
      Number(base.max_raw_candidates || settings.operatorMaxRawCandidates || 0),
      Math.max(20, maxDet * 5)
    );
    const every = Number(everyEl?.value || settings.operatorDetectEvery || base.detect_every_n_frames || 1);
    const area = String(areaEl?.value || settings.operatorDetectionAreaMode || base.detection_mode || 'full_frame');

    const custom = Object.assign({}, settings.objectPresetsCustom || {});

    // Contract safety: objectPresetsCustom must never become empty from browser UI.
    if (!Object.keys(custom).length) {
      Object.assign(custom, OBJECT_PRESETS || {});
    }

    const nextPreset = Object.assign({}, base, custom[active] || {}, {
      max_detections: maxDet,
      max_raw_candidates: maxRaw,
      detect_every_n_frames: every,
      detection_mode: area
    });

    if (!Array.isArray(nextPreset.classes)) {
      nextPreset.classes = Array.isArray(settings.detectorSelectedClasses) ? settings.detectorSelectedClasses : [];
    }

    custom[active] = nextPreset;

    const next = Object.assign({}, settings, {
      objectPresetsCustom: custom,
      activeObjectPreset: active,
      detectorEnabled: true,
      detectorSelectedClasses: Array.isArray(nextPreset.classes) ? nextPreset.classes : [],
      operatorModel: nextPreset.model || nextPreset.operatorModel || settings.operatorModel || '',
      operatorDetectionLimit: maxDet,
      operatorDetectEvery: every,
      operatorDetectionAreaMode: area
    });

    await writeSettings(next);

    window.__ptzCleanLayerLastDetectionPersist = {
      reason,
      activeObjectPreset: active,
      operatorDetectionLimit: maxDet,
      operatorDetectEvery: every,
      operatorDetectionAreaMode: area,
      ts: Date.now()
    };

    try {
      await hydrateDetectionControlsFromSettings('persist_' + reason);
    } catch (_) {}

    return next;
  }

  function schedulePersistActiveDetectionControlsToPreset(reason = 'ui_change') {
    for (const delay of [250, 750, 1500]) {
      setTimeout(() => {
        persistActiveDetectionControlsToPreset(reason).catch(err => {
          console.error('[ptz-clean-detection-persist]', reason, err);
        });
      }, delay);
    }
  }
  // PTZ_CLEAN_DETECTION_ACTIVE_PRESET_PERSIST_END
'''
if 'PTZ_CLEAN_DETECTION_ACTIVE_PRESET_PERSIST_START' not in s:
    if marker not in s:
        raise SystemExit('bindHandlers marker not found')
    s = s.replace(marker, helper + marker, 1)
    changed.append('inserted active preset detection persist helper')

# 3) Bind clean-layer capture listeners for detection controls and apply buttons.
old_bind = """  function bindHandlers() {
    if (document.__ptzCleanLayerBound) return;
    document.__ptzCleanLayerBound = true;

    document.addEventListener('click', async (e) => {"""
new_bind = """  function bindHandlers() {
    if (document.__ptzCleanLayerBound) return;
    document.__ptzCleanLayerBound = true;

    document.addEventListener('change', (e) => {
      const id = e.target && e.target.id ? String(e.target.id) : '';
      if (id === 'operatorDetectionLimit' || id === 'operatorDetectEvery' || id === 'operatorDetectionAreaMode') {
        schedulePersistActiveDetectionControlsToPreset('change_' + id);
      }
    }, true);

    document.addEventListener('click', (e) => {
      const target = e.target && e.target.closest ? e.target.closest('#operatorLimitApplyBtn,#operatorDetectEveryApplyBtn,#operatorDetectionAreaApplyBtn') : null;
      if (target) {
        schedulePersistActiveDetectionControlsToPreset('click_' + target.id);
      }
    }, true);

    document.addEventListener('click', async (e) => {"""
if old_bind in s and new_bind not in s:
    s = s.replace(old_bind, new_bind, 1)
    changed.append('bound detection control change/apply persistence')

# 4) Export helper in window.ptzCleanLayer.
old_export = """    hydrateDetectionControlsFromSettings,
    rebuildPanels"""
new_export = """    hydrateDetectionControlsFromSettings,
    persistActiveDetectionControlsToPreset,
    rebuildPanels"""
if old_export in s and new_export not in s:
    s = s.replace(old_export, new_export, 1)
    changed.append('exported persistActiveDetectionControlsToPreset')
else:
    # fallback: insert after hydrateDetectionControlsFromSettings in object literal if present
    if 'persistActiveDetectionControlsToPreset' not in s:
        s2 = re.sub(r'(window\.ptzCleanLayer\s*=\s*\{[^}]*?hydrateDetectionControlsFromSettings,)', r'\1\n    persistActiveDetectionControlsToPreset,', s, flags=re.S)
        if s2 != s:
            s = s2
            changed.append('exported persist helper via regex')

# 5) Add a boot-time scheduled hydrate/persist safety pass after scheduleDetectionUiHydrate call.
needle = "scheduleDetectionUiHydrate('clean_layer_boot');"
insert = """scheduleDetectionUiHydrate('clean_layer_boot');
  setTimeout(() => persistActiveDetectionControlsToPreset('boot_safety').catch(() => {}), 2500);"""
if needle in s and insert not in s:
    s = s.replace(needle, insert, 1)
    changed.append('added boot safety persist pass')

p.write_text(s, encoding='utf-8')

print('OK patched web/index.html UI active preset detection persist v3')
print('Backup:', bak)
print('Changes:')
for item in changed:
    print(' -', item)
if not changed:
    print(' - no changes; markers may already exist')