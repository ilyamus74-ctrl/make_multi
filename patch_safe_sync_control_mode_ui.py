from pathlib import Path
import shutil, time

p = Path("web/index.html")
s = p.read_text(encoding="utf-8")

backup = p.with_suffix(f".html.bak_safe_sync_control_mode_ui_{int(time.time())}")
shutil.copy2(p, backup)
print("Backup:", backup)

helper = r'''  function safeSyncControlModeUi() {
    try {
      if (typeof syncControlModeUi === 'function') {
        syncControlModeUi();
        return;
      }
    } catch (_) {}

    try {
      const selectedMode = $('controlMode')?.value || 'manual';
      const mode = selectedMode.toUpperCase();
      const isAuto = selectedMode === 'auto';

      $('modeManualBtn')?.classList.toggle('active', selectedMode === 'manual');
      $('modePtzBtn')?.classList.toggle('active', selectedMode === 'ptz');
      $('ptzModeManualBtn')?.classList.toggle('active', selectedMode === 'manual');
      $('ptzModePtzBtn')?.classList.toggle('active', selectedMode === 'ptz');
      $('modeAssistBtn')?.classList.toggle('active', selectedMode === 'assist');
      $('modeAutoBtn')?.classList.toggle('active', isAuto);

      if ($('controlModeChip')) {
        $('controlModeChip').textContent =
          zoomAprilTagCalibrationActive && selectedMode === 'ptz'
            ? 'ACTIVE MODE: PTZ / CALIB SAFE'
            : `ACTIVE MODE: ${mode}`;
      }
    } catch (e) {
      try { ptzLog('safeSyncControlModeUi error', { error: String(e.message || e) }); } catch (_) {}
    }
  }

'''

anchor = "  async function runAprilTagZoomCalibrationFromUi() {\n"

if "function safeSyncControlModeUi()" not in s:
    if anchor not in s:
        raise SystemExit("ERROR: runAprilTagZoomCalibrationFromUi anchor not found")
    s = s.replace(anchor, helper + anchor, 1)
    print("OK: inserted safeSyncControlModeUi")
else:
    print("SKIP: safeSyncControlModeUi already exists")

# Replace direct calls, but not inside the helper text.
s = s.replace("      syncControlModeUi();", "      safeSyncControlModeUi();")
s = s.replace("    syncControlModeUi();", "    safeSyncControlModeUi();")
s = s.replace("  syncControlModeUi();", "  safeSyncControlModeUi();")

p.write_text(s, encoding="utf-8")
print("DONE")
