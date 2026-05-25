from pathlib import Path
import re
import shutil
import time

p = Path("web/index.html")
s = p.read_text(encoding="utf-8")

backup = p.with_suffix(f".html.bak_keyboard_hold_debounce_{int(time.time())}")
shutil.copy2(p, backup)
print(f"backup: {backup}")

# 1) Remove dangling broken touchControls CSS selector if still present.
s = re.sub(
    r'\n\s*/\* Hide legacy control panel; keep DOM only for old references during migration\. \*/\s*'
    r'\n\s*/\* Touch overlay removed: keep forced hidden if stale cached DOM exists\. \*/\s*'
    r'\n\s*#touchControls,\s*',
    '\n',
    s,
    count=1
)

# 2) Make first manual hold tick immediate.
old = """    manualHoldSource = source;
    manualHoldActive = true;
    if (!manualHoldTimer) {
"""

new = """    manualHoldSource = source;
    manualHoldActive = true;
    manualHoldLastSentAt = 0;
    if (!manualHoldTimer) {
"""

if old not in s:
    print("WARN: startManualHold immediate tick insertion point not found or already patched")
else:
    s = s.replace(old, new, 1)
    print("OK: startManualHold resets manualHoldLastSentAt")

# 3) Replace keyboard handlers with debounced keyup logic.
start = s.find("  window.addEventListener('keydown', (e) => {")
if start < 0:
    raise SystemExit("ERROR: keydown handler start not found")

end_marker = "  });\n\n</script>"
end = s.find(end_marker, start)
if end < 0:
    raise SystemExit("ERROR: keyup handler end marker not found")
end += len("  });")

replacement = r'''  let keyboardHoldStopTimer = null;
  const KEYBOARD_HOLD_STOP_DELAY_MS = 240;

  function cancelKeyboardHoldStopTimer() {
    if (keyboardHoldStopTimer) {
      clearTimeout(keyboardHoldStopTimer);
      keyboardHoldStopTimer = null;
    }
  }

  function scheduleKeyboardHoldStop() {
    cancelKeyboardHoldStopTimer();
    keyboardHoldStopTimer = setTimeout(() => {
      keyboardHoldStopTimer = null;
      if (manualKeyState.size === 0) {
        stopManualHold('keyboard_keyup_debounced').catch(() => {});
      }
    }, KEYBOARD_HOLD_STOP_DELAY_MS);
  }

  function keyToManualDirection(e) {
    const key = String(e.key || '').toLowerCase();
    if (key === 'arrowleft') return { keyName: 'ArrowLeft', pan: -1, tilt: 0, source: 'keyboard_left' };
    if (key === 'arrowright') return { keyName: 'ArrowRight', pan: 1, tilt: 0, source: 'keyboard_right' };
    if (key === 'arrowup') return { keyName: 'ArrowUp', pan: 0, tilt: -1, source: 'keyboard_up' };
    if (key === 'arrowdown') return { keyName: 'ArrowDown', pan: 0, tilt: 1, source: 'keyboard_down' };
    return null;
  }

  window.addEventListener('keydown', (e) => {
    if (e.key === 'Escape') {
      panicStop('esc');
      return;
    }
    if (shouldIgnoreKeyboardEvent(e)) return;

    const key = String(e.key || '').toLowerCase();
    const code = String(e.code || '');
    const isQ = key === 'q' || code === 'KeyQ';
    const isA = key === 'a' || code === 'KeyA';

    if (isQ || isA) {
      e.preventDefault();
      if (e.repeat) return;
      keyboardState[isQ ? 'zin' : 'zout'] = true;
      if (isQ) keyboardZoomSampleStep(1, 'Q_TELE');
      if (isA) keyboardZoomSampleStep(-1, 'A_WIDE');
      return;
    }

    const dir = keyToManualDirection(e);
    if (!dir) return;

    e.preventDefault();
    cancelKeyboardHoldStopTimer();

    keyboardState.up = dir.keyName === 'ArrowUp';
    keyboardState.down = dir.keyName === 'ArrowDown';
    keyboardState.left = dir.keyName === 'ArrowLeft';
    keyboardState.right = dir.keyName === 'ArrowRight';

    // On remote browsers keyup/keydown can flicker. Always refresh hold on keydown,
    // including repeat events.
    manualKeyState.clear();
    manualKeyState.add(dir.keyName);
    startManualHold(dir.pan, dir.tilt, dir.source);
  });

  window.addEventListener('keyup', (e) => {
    if (shouldIgnoreKeyboardEvent(e)) return;

    const key = String(e.key || '').toLowerCase();
    const code = String(e.code || '');
    const isQ = key === 'q' || code === 'KeyQ';
    const isA = key === 'a' || code === 'KeyA';

    if (isQ || isA) {
      e.preventDefault();
      keyboardState[isQ ? 'zin' : 'zout'] = false;
      return;
    }

    const dir = keyToManualDirection(e);
    if (!dir) return;

    e.preventDefault();

    keyboardState.up = false;
    keyboardState.down = false;
    keyboardState.left = false;
    keyboardState.right = false;

    manualKeyState.delete(dir.keyName);
    scheduleKeyboardHoldStop();
  });'''

s = s[:start] + replacement + s[end:]

p.write_text(s, encoding="utf-8")
print("OK: replaced keyboard handlers with debounced hold stop")
