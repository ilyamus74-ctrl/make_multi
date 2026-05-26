from pathlib import Path
import shutil
import time
import re

p = Path("web/index.html")
s = p.read_text(encoding="utf-8")

backup = p.with_suffix(f".html.bak_keyboard_diagonal_j_pulse_{int(time.time())}")
shutil.copy2(p, backup)
print("backup:", backup)

# 1) Insert combined keyboard pulse loop after manualKeyState declaration
anchor = "  const manualKeyState = new Set();\n"

insert = '''  let keyboardPulseInFlight = false;
  const KEYBOARD_J_PULSE_GAP_MS = 15;

  function updateKeyboardStateFromManualKeys() {
    keyboardState.up = manualKeyState.has('ArrowUp');
    keyboardState.down = manualKeyState.has('ArrowDown');
    keyboardState.left = manualKeyState.has('ArrowLeft');
    keyboardState.right = manualKeyState.has('ArrowRight');
  }

  function currentManualKeyVector() {
    let pan = 0;
    let tilt = 0;

    if (manualKeyState.has('ArrowLeft')) pan -= 1;
    if (manualKeyState.has('ArrowRight')) pan += 1;
    if (manualKeyState.has('ArrowUp')) tilt -= 1;
    if (manualKeyState.has('ArrowDown')) tilt += 1;

    pan = Math.max(-1, Math.min(1, pan));
    tilt = Math.max(-1, Math.min(1, tilt));

    return { pan, tilt };
  }

  function keyboardPulseSourceFromVector(pan, tilt) {
    if (pan < 0 && tilt < 0) return 'keyboard_left_up';
    if (pan < 0 && tilt > 0) return 'keyboard_left_down';
    if (pan > 0 && tilt < 0) return 'keyboard_right_up';
    if (pan > 0 && tilt > 0) return 'keyboard_right_down';
    if (pan < 0) return 'keyboard_left';
    if (pan > 0) return 'keyboard_right';
    if (tilt < 0) return 'keyboard_up';
    if (tilt > 0) return 'keyboard_down';
    return 'keyboard_none';
  }

  async function keyboardPulseLoop() {
    if (keyboardPulseInFlight) return;

    keyboardPulseInFlight = true;
    try {
      while (manualKeyState.size > 0) {
        const v = currentManualKeyVector();

        if (Math.abs(v.pan) > 0.001 || Math.abs(v.tilt) > 0.001) {
          await manualInputJPulse(
            v.pan,
            v.tilt,
            keyboardPulseSourceFromVector(v.pan, v.tilt)
          );
        }

        await new Promise(resolve => setTimeout(resolve, KEYBOARD_J_PULSE_GAP_MS));
      }
    } catch (e) {
      ptzLog('KEYBOARD J PULSE LOOP error', { error: String(e.message || e) });
    } finally {
      keyboardPulseInFlight = false;

      // If a key appeared while the loop was finishing, restart.
      if (manualKeyState.size > 0) {
        keyboardPulseLoop();
      }
    }
  }

'''

if "function currentManualKeyVector()" not in s:
    if anchor not in s:
        raise SystemExit("ERROR: manualKeyState anchor not found")
    s = s.replace(anchor, anchor + "\n" + insert, 1)
    print("OK: inserted combined keyboard pulse loop")
else:
    print("SKIP: combined keyboard pulse loop already exists")


# 2) Replace keydown manual axis handling block
patterns = [
r'''    keyboardState\.up = dir\.keyName === 'ArrowUp';
    keyboardState\.down = dir\.keyName === 'ArrowDown';
    keyboardState\.left = dir\.keyName === 'ArrowLeft';
    keyboardState\.right = dir\.keyName === 'ArrowRight';

    // Keyboard arrows use direct J pulse:
    // one keydown = one micro movement\. No hold/ramp/keyup stop\.
    manualKeyState\.clear\(\);
    manualKeyState\.add\(dir\.keyName\);
    manualInputJPulse\(dir\.pan, dir\.tilt, dir\.source\)\.catch\(e =>
      ptzLog\('MANUAL J PULSE error', \{ source: dir\.source, error: String\(e\.message \|\| e\) \}\)
    \);
''',
r'''    keyboardState\.up = dir\.keyName === 'ArrowUp';
    keyboardState\.down = dir\.keyName === 'ArrowDown';
    keyboardState\.left = dir\.keyName === 'ArrowLeft';
    keyboardState\.right = dir\.keyName === 'ArrowRight';

    // On remote browsers keyup/keydown can flicker\. Always refresh hold on keydown,
    // including repeat events\.
    manualKeyState\.clear\(\);
    manualKeyState\.add\(dir\.keyName\);
    startManualHold\(dir\.pan, dir\.tilt, dir\.source\);
'''
]

replacement = '''    manualKeyState.add(dir.keyName);
    updateKeyboardStateFromManualKeys();
    keyboardPulseLoop();
'''

done = False
for pat in patterns:
    s2, n = re.subn(pat, replacement, s, count=1)
    if n:
        s = s2
        done = True
        print("OK: keydown now keeps multiple axes and starts pulse loop")
        break

if not done:
    if "keyboardPulseLoop();" in s:
        print("SKIP: keydown already patched")
    else:
        raise SystemExit("ERROR: keydown manual axis block not found")


# 3) Replace keyup manual axis handling block
patterns = [
r'''    keyboardState\.up = false;
    keyboardState\.down = false;
    keyboardState\.left = false;
    keyboardState\.right = false;

    manualKeyState\.delete\(dir\.keyName\);
    if \(manualHoldActive\) \{
      scheduleKeyboardHoldStop\(\);
    \}
''',
r'''    keyboardState\.up = false;
    keyboardState\.down = false;
    keyboardState\.left = false;
    keyboardState\.right = false;

    manualKeyState\.delete\(dir\.keyName\);
    scheduleKeyboardHoldStop\(\);
'''
]

replacement = '''    manualKeyState.delete(dir.keyName);
    updateKeyboardStateFromManualKeys();
'''

done = False
for pat in patterns:
    s2, n = re.subn(pat, replacement, s, count=1)
    if n:
        s = s2
        done = True
        print("OK: keyup now only updates active key set")
        break

if not done:
    if "updateKeyboardStateFromManualKeys();" in s:
        print("SKIP: keyup already patched")
    else:
        raise SystemExit("ERROR: keyup manual axis block not found")


p.write_text(s, encoding="utf-8")
print("DONE")
