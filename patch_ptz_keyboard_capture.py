from pathlib import Path
import shutil, time

p = Path("web/index.html")
s = p.read_text(encoding="utf-8")

backup = p.with_suffix(f".html.bak_ptz_keyboard_capture_{int(time.time())}")
shutil.copy2(p, backup)
print("Backup:", backup)

anchor = '''  window.addEventListener('keydown', (e) => {
'''

insert = r'''  /*
   * PTZ keyboard capture guard.
   * After ZOOM CALIB UI changes, focused inputs/page scroll can steal Arrow keys.
   * This capture handler runs before browser/page handlers and owns Arrow keys in PTZ mode.
   */
  function ptzKeyboardCaptureDirection(e) {
    const key = String(e.key || '').toLowerCase();
    if (key === 'arrowleft') return { keyName: 'ArrowLeft', pan: -1, tilt: 0, source: 'keyboard_left' };
    if (key === 'arrowright') return { keyName: 'ArrowRight', pan: 1, tilt: 0, source: 'keyboard_right' };
    if (key === 'arrowup') return { keyName: 'ArrowUp', pan: 0, tilt: -1, source: 'keyboard_up' };
    if (key === 'arrowdown') return { keyName: 'ArrowDown', pan: 0, tilt: 1, source: 'keyboard_down' };
    return null;
  }

  function ptzKeyboardCaptureEnabled() {
    return $('controlMode')?.value === 'ptz';
  }

  window.__ptzKeyCaptureInstalled = true;

  window.addEventListener('keydown', (e) => {
    const dir = ptzKeyboardCaptureDirection(e);
    if (!dir || !ptzKeyboardCaptureEnabled()) return;

    e.preventDefault();
    e.stopPropagation();
    e.stopImmediatePropagation();

    try {
      const el = document.activeElement;
      const tag = String(el?.tagName || '').toUpperCase();
      if (tag === 'INPUT' || tag === 'TEXTAREA' || tag === 'SELECT' || el?.isContentEditable) {
        el.blur?.();
      }
    } catch (_) {}

    if (e.repeat) return;

    cancelKeyboardHoldStopTimer?.();

    manualKeyState.add(dir.keyName);
    updateKeyboardStateFromManualKeys();
    keyboardTapHoldVector = currentManualKeyVector();

    const kmode = getKeyboardPtzMode();
    if (kmode === 'tap_hold') {
      if (!keyboardTapHoldTimer && !keyboardTapHoldActive && !keyboardMotionInFlight) {
        startKeyboardTapHoldTimer();
      }
    } else {
      keyboardMotionLoop();
    }

    ptzLog?.('PTZ KEY CAPTURE DOWN', {
      key: dir.keyName,
      mode: kmode,
      vector: currentManualKeyVector()
    });
  }, true);

  window.addEventListener('keyup', (e) => {
    const dir = ptzKeyboardCaptureDirection(e);
    if (!dir || !ptzKeyboardCaptureEnabled()) return;

    e.preventDefault();
    e.stopPropagation();
    e.stopImmediatePropagation();

    const beforeRelease = currentManualKeyVector();
    manualKeyState.delete(dir.keyName);
    updateKeyboardStateFromManualKeys();

    if (getKeyboardPtzMode() === 'tap_hold') {
      if (!keyboardTapHoldActive && manualKeyState.size === 0) {
        clearKeyboardTapHoldTimer();
        const v = keyboardTapHoldVector || beforeRelease;
        if (Math.abs(v.pan) > 0.001 || Math.abs(v.tilt) > 0.001) {
          manualInputJPulse(v.pan, v.tilt, keyboardPulseSourceFromVector(v.pan, v.tilt)).catch(err =>
            ptzLog('PTZ KEY CAPTURE TAP PULSE error', { error: String(err.message || err) })
          );
        }
        keyboardTapHoldVector = { pan: 0, tilt: 0 };
      }
    }

    ptzLog?.('PTZ KEY CAPTURE UP', {
      key: dir.keyName,
      vector: currentManualKeyVector()
    });
  }, true);

'''

if 'PTZ keyboard capture guard' in s:
    print("SKIP: capture patch already exists")
else:
    if anchor not in s:
        raise SystemExit("ERROR: keydown anchor not found")
    s = s.replace(anchor, insert + anchor, 1)
    p.write_text(s, encoding="utf-8")
    print("OK: inserted PTZ keyboard capture handler")
