from pathlib import Path
import shutil

p = Path("web/index.html")
s = p.read_text(encoding="utf-8")

backup = p.with_suffix(".html.bak_zoom_keys_ptz")
if not backup.exists():
    shutil.copy2(p, backup)
    print(f"backup: {backup}")

# 1. In PTZ mode keep touch controls usable for zoom buttons.
s = s.replace(
    "$('touchControls').classList.toggle('disabled', isAuto || isPtz);",
    "$('touchControls').classList.toggle('disabled', isAuto);"
)

# 2. Manual mode should auto-connect WS even if old Auto-connect checkbox is off/hidden.
s = s.replace(
    "if ($('auto').checked) connectWs();\n    if (!ws || ws.readyState !== 1) return;",
    "if ($('auto').checked || $('controlMode').value === 'manual') connectWs();\n    if (!ws || ws.readyState !== 1) return;"
)

# 3. Add zoom-only sender before applyKeyboardDrive().
marker = "  function applyKeyboardDrive() {"
insert = r'''  function withWsReady(fn) {
    if (ws && ws.readyState === 1) {
      fn();
      return;
    }
    connectWs();
    let done = false;
    const tryRun = () => {
      if (done) return;
      if (ws && ws.readyState === 1) {
        done = true;
        fn();
      }
    };
    setTimeout(tryRun, 80);
    setTimeout(tryRun, 250);
  }

  function sendZoomOnly(zoom) {
    if (zoomCalibInProgress) zoom = 0;
    const safeLimited = applySafeModeLimits(0, 0, zoom);
    const limited = applyCommandLimits(0, 0, safeLimited.zoom);

    withWsReady(() => {
      sendLine(`Z ${limited.zoom}`);
      updateZoomStateFromCommand(limited.zoom);
      $('zoomStick').textContent = `zoom=${limited.zoom} (PTZ zoom-only)`;
    });
  }

'''
if marker not in s:
    raise SystemExit("applyKeyboardDrive marker not found")
if "function sendZoomOnly(zoom)" not in s:
    s = s.replace(marker, insert + marker, 1)

# 4. In PTZ mode keyboard arrows are ignored, but Q/A sends Z only.
old = r'''  function applyKeyboardDrive() {
    const pan = (keyboardState.right ? 35 : 0) + (keyboardState.left ? -35 : 0);
    let tilt = (keyboardState.up ? -35 : 0) + (keyboardState.down ? 35 : 0);
    if (invertTiltAxis) tilt = -tilt;
    const zoom = (keyboardState.zin ? -45 : 0) + (keyboardState.zout ? 45 : 0);
    manualDrive(pan, tilt, zoom);
  }'''

new = r'''  function applyKeyboardDrive() {
    const pan = (keyboardState.right ? 35 : 0) + (keyboardState.left ? -35 : 0);
    let tilt = (keyboardState.up ? -35 : 0) + (keyboardState.down ? 35 : 0);
    if (invertTiltAxis) tilt = -tilt;
    const zoom = (keyboardState.zin ? -45 : 0) + (keyboardState.zout ? 45 : 0);

    if ($('controlMode').value === 'ptz') {
      // In PTZ mode server autopilot owns pan/tilt.
      // Keyboard Q/A is still allowed for optical zoom, but must not send J 0 0.
      sendZoomOnly(zoom);
      return;
    }

    manualDrive(pan, tilt, zoom);
  }'''

if old not in s:
    raise SystemExit("applyKeyboardDrive block not found")
s = s.replace(old, new, 1)

# 5. In PTZ mode touch arrows are ignored, touch zoom remains allowed.
old = r'''    const onStart = (btn) => {
      const cmd = btn.dataset.cmd;
      if (!map[cmd]) return;
      touchState = map[cmd];
      btn.classList.add('active');
      manualDrive(touchState.pan, touchState.tilt, touchState.zoom);
    };
    const onEnd = (btn) => {
      btn.classList.remove('active');
      touchState = { pan: 0, tilt: 0, zoom: 0 };
      manualDrive(0, 0, 0);
    };'''

new = r'''    const onStart = (btn) => {
      const cmd = btn.dataset.cmd;
      if (!map[cmd]) return;

      if ($('controlMode').value === 'ptz' && !['zin', 'zout', 'stop'].includes(cmd)) {
        return;
      }

      touchState = map[cmd];
      btn.classList.add('active');

      if ($('controlMode').value === 'ptz') {
        sendZoomOnly(touchState.zoom);
        return;
      }

      manualDrive(touchState.pan, touchState.tilt, touchState.zoom);
    };
    const onEnd = (btn) => {
      btn.classList.remove('active');
      touchState = { pan: 0, tilt: 0, zoom: 0 };

      if ($('controlMode').value === 'ptz') {
        sendZoomOnly(0);
        return;
      }

      manualDrive(0, 0, 0);
    };'''

if old not in s:
    raise SystemExit("touch control block not found")
s = s.replace(old, new, 1)

p.write_text(s, encoding="utf-8")
print("patched web/index.html")
