from pathlib import Path
import shutil, time

p = Path("web/index.html")
s = p.read_text(encoding="utf-8")

backup = p.with_suffix(f".html.bak_zoom_calib_autosave_delegate_{int(time.time())}")
shutil.copy2(p, backup)
print("Backup:", backup)

old = '''  let zoomCalibSettingsSaveTimer = null;
  function scheduleZoomCalibSettingsSave() {
    updateZoomActiveAnchorSummary();
    if ($('zoomAprilTagCalibStatus')) $('zoomAprilTagCalibStatus').textContent = 'settings saving...';
    if (zoomCalibSettingsSaveTimer) clearTimeout(zoomCalibSettingsSaveTimer);
    zoomCalibSettingsSaveTimer = setTimeout(() => {
      saveZoomCalibSettingsFromUi().catch(e => {
        if ($('zoomAprilTagCalibStatus')) $('zoomAprilTagCalibStatus').textContent = 'settings save error';
        ptzLog('ZOOM CALIB SETTINGS save error', { error: String(e.message || e) });
      });
    }, 400);
  }
'''

new = '''  let zoomCalibSettingsSaveTimer = null;

  function scheduleZoomCalibSettingsSave(delayMs = 400) {
    updateZoomActiveAnchorSummary();
    if ($('zoomAprilTagCalibStatus')) $('zoomAprilTagCalibStatus').textContent = 'settings saving...';
    if (zoomCalibSettingsSaveTimer) clearTimeout(zoomCalibSettingsSaveTimer);
    zoomCalibSettingsSaveTimer = setTimeout(() => {
      zoomCalibSettingsSaveTimer = null;
      saveZoomCalibSettingsFromUi().catch(e => {
        if ($('zoomAprilTagCalibStatus')) $('zoomAprilTagCalibStatus').textContent = 'settings save error';
        ptzLog('ZOOM CALIB SETTINGS save error', { error: String(e.message || e) });
      });
    }, Math.max(0, Number(delayMs || 0)));
  }

  function flushZoomCalibSettingsSaveNow() {
    updateZoomActiveAnchorSummary();
    if (zoomCalibSettingsSaveTimer) {
      clearTimeout(zoomCalibSettingsSaveTimer);
      zoomCalibSettingsSaveTimer = null;
    }
    if ($('zoomAprilTagCalibStatus')) $('zoomAprilTagCalibStatus').textContent = 'settings saving...';
    return saveZoomCalibSettingsFromUi().catch(e => {
      if ($('zoomAprilTagCalibStatus')) $('zoomAprilTagCalibStatus').textContent = 'settings save error';
      ptzLog('ZOOM CALIB SETTINGS save error', { error: String(e.message || e) });
    });
  }

  function isZoomCalibAutosaveElement(el) {
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

  function installZoomCalibAutosaveDelegate() {
    if (window.__zoomCalibAutosaveDelegateInstalled) return;
    window.__zoomCalibAutosaveDelegateInstalled = true;

    document.addEventListener('input', (e) => {
      if (!isZoomCalibAutosaveElement(e.target)) return;
      scheduleZoomCalibSettingsSave(250);
    }, true);

    document.addEventListener('change', (e) => {
      if (!isZoomCalibAutosaveElement(e.target)) return;
      flushZoomCalibSettingsSaveNow();
    }, true);
  }

  installZoomCalibAutosaveDelegate();
'''

if old not in s:
    raise SystemExit("ERROR: scheduleZoomCalibSettingsSave block not found")

s = s.replace(old, new, 1)
p.write_text(s, encoding="utf-8")

print("OK: autosave delegate installed")
