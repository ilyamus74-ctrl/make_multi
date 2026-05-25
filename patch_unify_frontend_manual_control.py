from pathlib import Path
import shutil
import time

p = Path("web/index.html")
s = p.read_text(encoding="utf-8")

backup = p.with_suffix(f".html.bak_unify_frontend_manual_control_{int(time.time())}")
shutil.copy2(p, backup)
print(f"backup: {backup}")

# 1) Replace legacy manualDrive direct J/Z with safe wrapper.
old_manual = """  function manualDrive(pan, tilt, zoom) {
    if (zoomAprilTagCalibrationActive || zoomCalibInProgress) {
      pan = 0;
      tilt = 0;
      zoom = 0;
    }
    if ($('auto').checked || $('controlMode').value === 'manual') connectWs();
    if (!ws || ws.readyState !== 1) return;
    const safeLimited = applySafeModeLimits(pan, tilt, zoom);
    const limited = applyCommandLimits(safeLimited.pan, safeLimited.tilt, safeLimited.zoom);
    seq = (seq + 1) & 0xFFFF;
    sendLine(`J ${seq} ${limited.pan} ${limited.tilt}`);
    sendLine(`Z ${limited.zoom}`);

    updateZoomStateFromCommand(limited.zoom);
  }
"""

new_manual = """  function manualDrive(pan, tilt, zoom) {
    // Legacy compatibility wrapper.
    // Do not send raw J from frontend anymore.
    // Normal pan/tilt must go through backend manual_drive.
    if (zoomAprilTagCalibrationActive || zoomCalibInProgress) {
      pan = 0;
      tilt = 0;
      zoom = 0;
    }

    const safeLimited = applySafeModeLimits(pan, tilt, zoom);
    const limited = applyCommandLimits(safeLimited.pan, safeLimited.tilt, safeLimited.zoom);

    const panNorm = Math.max(-1, Math.min(1, Number(limited.pan || 0) / 100.0));
    const tiltNorm = Math.max(-1, Math.min(1, Number(limited.tilt || 0) / 100.0));

    if (Math.abs(panNorm) > 0.001 || Math.abs(tiltNorm) > 0.001) {
      manualInputDrive(panNorm, tiltNorm, 'legacy_manualDrive', 500).catch(e =>
        ptzLog('LEGACY manualDrive backend error', { error: String(e.message || e) })
      );
    } else {
      manualInputStop('legacy_manualDrive_stop').catch(() => {});
    }

    // Zoom-only still uses existing Z path for now.
    if (Number(limited.zoom || 0) !== 0) {
      sendZoomOnly(limited.zoom);
    }
  }
"""

if old_manual not in s:
    raise SystemExit("ERROR: legacy manualDrive block not found")
s = s.replace(old_manual, new_manual, 1)
print("OK: legacy manualDrive no longer sends raw J")


# 2) Fix bindTouchControls: PTZ mode must not block pan/tilt.
old_start = """      if ($('controlMode').value === 'ptz') {
        sendZoomOnly(touchState.zoom);
        return;
      }

      const [panNorm, tiltNorm] = directionForTouchCmd(cmd);
      startManualHold(panNorm, tiltNorm, `touch_${cmd}`);
      sendZoomOnly(touchState.zoom);
"""

new_start = """      if (cmd === 'zin' || cmd === 'zout') {
        sendZoomOnly(touchState.zoom);
        return;
      }

      if (cmd === 'stop') {
        stopManualHold('touch_stop_button').catch(() => {});
        sendZoomOnly(0);
        return;
      }

      const [panNorm, tiltNorm] = directionForTouchCmd(cmd);
      startManualHold(panNorm, tiltNorm, `touch_${cmd}`);
"""

if old_start not in s:
    raise SystemExit("ERROR: bindTouchControls onStart PTZ block not found")
s = s.replace(old_start, new_start, 1)
print("OK: touch pan/tilt no longer blocked in PTZ mode")


old_end = """      if ($('controlMode').value === 'ptz') {
        sendZoomOnly(0);
        return;
      }

      stopManualHold('touch_stop').catch(() => {});
      sendZoomOnly(0);
"""

new_end = """      stopManualHold('touch_stop').catch(() => {});
      sendZoomOnly(0);
"""

if old_end not in s:
    raise SystemExit("ERROR: bindTouchControls onEnd PTZ block not found")
s = s.replace(old_end, new_end, 1)
print("OK: touch release always stops manual hold")


# 3) Q/A: use physical key code too.
old_keydown = """    const key = e.key.toLowerCase();
    const map = {
      arrowup: 'up',
      arrowdown: 'down',
      arrowleft: 'left',
      arrowright: 'right',
      q: 'zin',
      a: 'zout',
    };
    const action = map[key];
    if (!action) return;
    e.preventDefault();
    keyboardState[action] = true;

    if (key === 'q') {
      keyboardZoomSampleStep(-1, 'Q_WIDE');
      return;
    }
    if (key === 'a') {
      keyboardZoomSampleStep(1, 'A_TELE');
      return;
    }
"""

new_keydown = """    const key = e.key.toLowerCase();
    const code = String(e.code || '');
    const isQ = key === 'q' || code === 'KeyQ';
    const isA = key === 'a' || code === 'KeyA';

    const map = {
      arrowup: 'up',
      arrowdown: 'down',
      arrowleft: 'left',
      arrowright: 'right',
    };

    let action = map[key];
    if (isQ) action = 'zin';
    if (isA) action = 'zout';

    if (!action) return;
    e.preventDefault();
    keyboardState[action] = true;

    if (isQ) {
      keyboardZoomSampleStep(-1, 'Q_WIDE');
      return;
    }
    if (isA) {
      keyboardZoomSampleStep(1, 'A_TELE');
      return;
    }
"""

if old_keydown not in s:
    raise SystemExit("ERROR: keydown Q/A block not found")
s = s.replace(old_keydown, new_keydown, 1)
print("OK: keydown Q/A uses KeyboardEvent.code too")


old_keyup = """    const key = e.key.toLowerCase();
    const map = {
      arrowup: 'up',
      arrowdown: 'down',
      arrowleft: 'left',
      arrowright: 'right',
      q: 'zin',
      a: 'zout',
    };
    const action = map[key];
    if (!action) return;
    e.preventDefault();
    keyboardState[action] = false;

    if (key === 'q' || key === 'a') {
      return;
    }
"""

new_keyup = """    const key = e.key.toLowerCase();
    const code = String(e.code || '');
    const isQ = key === 'q' || code === 'KeyQ';
    const isA = key === 'a' || code === 'KeyA';

    const map = {
      arrowup: 'up',
      arrowdown: 'down',
      arrowleft: 'left',
      arrowright: 'right',
    };

    let action = map[key];
    if (isQ) action = 'zin';
    if (isA) action = 'zout';

    if (!action) return;
    e.preventDefault();
    keyboardState[action] = false;

    if (isQ || isA) {
      return;
    }
"""

if old_keyup not in s:
    raise SystemExit("ERROR: keyup Q/A block not found")
s = s.replace(old_keyup, new_keyup, 1)
print("OK: keyup Q/A uses KeyboardEvent.code too")


# 4) Add stronger CSS hide for legacy controls if not already strong enough.
css_marker = "</style>"
hide_css = """
/* Hide legacy control panel; keep DOM only for old references during migration. */
#controlsSpoiler,
#settingsToggleBtn {
  display: none !important;
  visibility: hidden !important;
  pointer-events: none !important;
  height: 0 !important;
  max-height: 0 !important;
  overflow: hidden !important;
}
"""
if "#controlsSpoiler" not in s or "pointer-events: none !important" not in s:
    if css_marker not in s:
        print("WARN: </style> not found, CSS hide not inserted")
    else:
        s = s.replace(css_marker, hide_css + "\n" + css_marker, 1)
        print("OK: strengthened legacy controls hide CSS")
else:
    print("SKIP: legacy controls hide CSS already present")

p.write_text(s, encoding="utf-8")
print("DONE")
