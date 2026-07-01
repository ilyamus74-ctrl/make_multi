from pathlib import Path
import shutil, time

p = Path("web/index.html")
s = p.read_text(encoding="utf-8")

backup = p.with_suffix(f".html.bak_zoom_calib_standalone_sync_{int(time.time())}")
shutil.copy2(p, backup)
print("Backup:", backup)

block = r'''
<script>
// ZOOM_CALIB_STANDALONE_SYNC_START
(function () {
  const $ = (id) => document.getElementById(id);

  const defs = [
    { label: 'near', prefix: 'Near', defaultDistanceM: 1, defaultTagId: 7 },
    { label: 'mid',  prefix: 'Mid',  defaultDistanceM: 5, defaultTagId: -1 },
    { label: 'far',  prefix: 'Far',  defaultDistanceM: 10, defaultTagId: 12 }
  ];

  let activeLabel = 'near';
  let saveTimer = null;

  function id(def, field) {
    return `zoomAnchor${def.prefix}${field}`;
  }

  function setVal(el, v) {
    if (!el) return;
    el.value = String(v);
  }

  function setChecked(el, v) {
    if (!el) return;
    el.checked = !!v;
  }

  function normNum(v, fallback) {
    const n = Number(v);
    return Number.isFinite(n) ? n : fallback;
  }

  function getAnchorFromUi(def) {
    return {
      label: def.label,
      enabled: !!$(id(def, 'Enabled'))?.checked,
      tag_id: Math.round(normNum($(id(def, 'TagId'))?.value, def.defaultTagId)),
      distance_mm: normNum($(id(def, 'DistanceM'))?.value, def.defaultDistanceM) * 1000,
      tag_size_mm: normNum($(id(def, 'SizeMm'))?.value, 160)
    };
  }

  function getActiveAnchor() {
    const anchors = defs.map(getAnchorFromUi);
    return anchors.find(a => a.label === activeLabel) || anchors[0];
  }

  function updateSummary() {
    const a = getActiveAnchor();
    const txt = `active=${activeLabel} id=${a.tag_id} distance=${(a.distance_mm / 1000).toFixed(2)}m size=${a.tag_size_mm}mm`;

    if ($('zoomActiveAnchorSummary')) $('zoomActiveAnchorSummary').textContent = txt;
    if ($('zoomCalibMenuSummary')) $('zoomCalibMenuSummary').textContent = txt;
  }

  function applySettingsToUi(s) {
    activeLabel = s.active_anchor_label || activeLabel || 'near';

    const byLabel = new Map((Array.isArray(s.anchors) ? s.anchors : []).map(a => [String(a.label || ''), a]));

    defs.forEach(def => {
      const a = byLabel.get(def.label) || {
        enabled: true,
        tag_id: def.label === 'near' ? normNum(s.anchor_tag_id, def.defaultTagId) : def.defaultTagId,
        distance_mm: def.label === 'near' ? normNum(s.anchor_distance_mm, def.defaultDistanceM * 1000) : def.defaultDistanceM * 1000,
        tag_size_mm: normNum(s.tag_size_mm, 160)
      };

      setChecked($(id(def, 'Enabled')), a.enabled !== false);
      setVal($(id(def, 'TagId')), Math.round(normNum(a.tag_id, def.defaultTagId)));
      setVal($(id(def, 'DistanceM')), (normNum(a.distance_mm, def.defaultDistanceM * 1000) / 1000).toFixed(2).replace(/\.00$/, ''));
      setVal($(id(def, 'SizeMm')), normNum(a.tag_size_mm, normNum(s.tag_size_mm, 160)));
    });

    setVal($('zoomImpulseMs'), normNum(s.impulse_ms, 170));
    setVal($('zoomSettleMs'), normNum(s.settle_ms, 190));
    setVal($('zoomSamples'), normNum(s.samples, 12));
    setVal($('zoomCmdAbs'), normNum(s.cmd_abs, 34));
    setVal($('zoomWideHoldMs'), normNum(s.wide_hold_ms, 1500));

    if ($('zoomWideSign')) {
      $('zoomWideSign').value = String(normNum(s.wide_cmd_sign, -1) < 0 ? -1 : 1);
    }

    if ($('zoomCalibDirection')) {
      $('zoomCalibDirection').value =
        s.calibration_direction === 'tele_to_wide' ? 'tele_to_wide' : 'wide_to_tele';
    }

    updateSummary();

    if ($('zoomAprilTagCalibStatus')) {
      $('zoomAprilTagCalibStatus').textContent = 'settings loaded';
    }
  }

  async function loadSettings() {
    const r = await fetch('/api/zoom_calibration/settings?ts=' + Date.now(), { cache: 'no-store' });
    if (!r.ok) throw new Error('GET settings failed: ' + r.status);
    const s = await r.json();
    applySettingsToUi(s);
    window.__zoomCalibStandaloneLastSettings = s;
    return s;
  }

  function readSettingsFromUi() {
    const anchors = defs.map(getAnchorFromUi);
    const active = anchors.find(a => a.label === activeLabel) || anchors[0];

    return {
      tag_size_mm: Number(active?.tag_size_mm || 160),
      anchor_tag_id: Number(active?.tag_id ?? 7),
      anchor_distance_mm: Number(active?.distance_mm || 1000),
      samples: Number($('zoomSamples')?.value || 12),
      impulse_ms: Number($('zoomImpulseMs')?.value || 170),
      settle_ms: Number($('zoomSettleMs')?.value || 190),
      cmd_abs: Number($('zoomCmdAbs')?.value || 34),
      wide_cmd_sign: Number($('zoomWideSign')?.value || -1),
      wide_hold_ms: Number($('zoomWideHoldMs')?.value || 1500),
      calibration_direction: $('zoomCalibDirection')?.value || 'wide_to_tele',
      active_anchor_label: activeLabel,
      anchors
    };
  }

  async function saveSettings() {
    const payload = readSettingsFromUi();

    if ($('zoomAprilTagCalibStatus')) {
      $('zoomAprilTagCalibStatus').textContent = 'settings saving...';
    }

    const r = await fetch('/api/zoom_calibration/settings', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify(payload)
    });

    if (!r.ok) throw new Error('POST settings failed: ' + r.status);

    const res = await r.json().catch(() => ({ ok: true }));

    if ($('zoomAprilTagCalibStatus')) {
      $('zoomAprilTagCalibStatus').textContent = 'settings saved';
    }

    window.__zoomCalibStandaloneLastSave = payload;
    updateSummary();

    return res;
  }

  function scheduleSave(delay = 250) {
    updateSummary();
    if (saveTimer) clearTimeout(saveTimer);
    saveTimer = setTimeout(() => {
      saveTimer = null;
      saveSettings().catch(e => {
        if ($('zoomAprilTagCalibStatus')) $('zoomAprilTagCalibStatus').textContent = 'settings save error';
        console.error('[zoom-calib-standalone] save failed', e);
      });
    }, delay);
  }

  function flushSave() {
    if (saveTimer) {
      clearTimeout(saveTimer);
      saveTimer = null;
    }
    return saveSettings().catch(e => {
      if ($('zoomAprilTagCalibStatus')) $('zoomAprilTagCalibStatus').textContent = 'settings save error';
      console.error('[zoom-calib-standalone] save failed', e);
    });
  }

  function isZoomCalibField(el) {
    if (!el || !el.id) return false;
    if (String(el.id).startsWith('zoomAnchor')) return true;
    return [
      'zoomImpulseMs',
      'zoomSettleMs',
      'zoomSamples',
      'zoomCmdAbs',
      'zoomWideHoldMs',
      'zoomCalibDirection',
      'zoomWideSign'
    ].includes(el.id);
  }

  function installAutosave() {
    if (window.__zoomCalibStandaloneAutosaveInstalled) return;
    window.__zoomCalibStandaloneAutosaveInstalled = true;

    document.addEventListener('input', e => {
      if (!isZoomCalibField(e.target)) return;
      scheduleSave(250);
    }, true);

    document.addEventListener('change', e => {
      if (!isZoomCalibField(e.target)) return;
      flushSave();
    }, true);
  }

  function installAnchorButtons() {
    defs.forEach(def => {
      const selectBtn = $(`zoomSelect${def.prefix}Btn`);
      const runBtn = $(`zoomRun${def.prefix}Btn`);

      if (selectBtn && !selectBtn.__zoomStandaloneBound) {
        selectBtn.__zoomStandaloneBound = true;
        selectBtn.addEventListener('click', e => {
          e.preventDefault();
          activeLabel = def.label;
          flushSave();
          updateSummary();
        }, true);
      }

      if (runBtn && !runBtn.__zoomStandaloneBound) {
        runBtn.__zoomStandaloneBound = true;
        runBtn.addEventListener('click', async e => {
          e.preventDefault();
          activeLabel = def.label;
          await flushSave();

          if (typeof runAprilTagZoomCalibrationFromUi === 'function') {
            runAprilTagZoomCalibrationFromUi();
          } else {
            fetch('/api/zoom_calibration', {
              method: 'POST',
              headers: { 'Content-Type': 'application/json' },
              body: JSON.stringify({ ...readSettingsFromUi(), mode: 'apriltag_zoom_table' })
            }).catch(console.error);
          }

          updateSummary();
        }, true);
      }
    });
  }

  function boot() {
    window.__zoomCalibStandaloneSyncInstalled = true;
    installAutosave();
    installAnchorButtons();

    loadSettings().catch(e => {
      if ($('zoomAprilTagCalibStatus')) $('zoomAprilTagCalibStatus').textContent = 'settings load error';
      console.error('[zoom-calib-standalone] load failed', e);
    });

    /*
     * Firefox/Chromium can restore form values after script init.
     * Re-apply backend values a few times after page load.
     */
    setTimeout(() => loadSettings().catch(() => {}), 250);
    setTimeout(() => loadSettings().catch(() => {}), 1000);
  }

  window.zoomCalibStandaloneLoad = loadSettings;
  window.zoomCalibStandaloneSave = saveSettings;

  if (document.readyState === 'loading') {
    document.addEventListener('DOMContentLoaded', boot, { once: true });
  } else {
    boot();
  }

  window.addEventListener('pageshow', () => {
    setTimeout(() => loadSettings().catch(() => {}), 100);
  });
})();
// ZOOM_CALIB_STANDALONE_SYNC_END
</script>
'''

if "ZOOM_CALIB_STANDALONE_SYNC_START" in s:
    print("SKIP: standalone sync already exists")
else:
    if "</body>" in s:
        s = s.replace("</body>", block + "\n</body>", 1)
    elif "</html>" in s:
        s = s.replace("</html>", block + "\n</html>", 1)
    else:
        s += "\n" + block + "\n"
    p.write_text(s, encoding="utf-8")
    print("OK: inserted standalone sync script")
