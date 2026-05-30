from pathlib import Path
import shutil, time

p = Path("web/index.html")
s = p.read_text(encoding="utf-8")

backup = p.with_suffix(f".html.bak_keyboard_hold_smooth_{int(time.time())}")
shutil.copy2(p, backup)
print("backup:", backup)

# 1. Add keyboard mode selector near J Pulse ms
old = '''        <label title="Duration of direct J pulse for keyboard micro movement">
          J Pulse ms
          <input id="ptzTuneJPulseMs" type="range" min="1" max="120" step="1" value="70" style="width:130px">
          <input id="ptzTuneJPulseMsNum" type="number" min="1" max="120" step="1" value="70" style="width:64px">
        </label>
'''

new = '''        <label title="Duration of direct J pulse for keyboard micro movement">
          J Pulse ms
          <input id="ptzTuneJPulseMs" type="range" min="1" max="120" step="1" value="70" style="width:130px">
          <input id="ptzTuneJPulseMsNum" type="number" min="1" max="120" step="1" value="70" style="width:64px">
        </label>
        <label title="Keyboard PTZ behavior">
          Keyboard
          <select id="ptzTuneKeyboardMode" class="small-select">
            <option value="hold" selected>HOLD SMOOTH</option>
            <option value="pulse">PULSE STEP</option>
          </select>
        </label>
'''

if 'id="ptzTuneKeyboardMode"' not in s:
    if old not in s:
        raise SystemExit("ERROR: J Pulse HTML anchor not found")
    s = s.replace(old, new, 1)
    print("OK: inserted Keyboard mode selector")
else:
    print("SKIP: Keyboard mode selector already exists")


# 2. Insert keyboard mode init after J Pulse init block
anchor = '''  if ($('ptzTuneJPulseMsNum')) {
    $('ptzTuneJPulseMsNum').onchange = e => syncJPulseInputs(e.target.value);
  }

'''

insert = '''  if ($('ptzTuneKeyboardMode')) {
    const savedKeyboardMode = localStorage.getItem('ptzTuneKeyboardMode') || 'hold';
    $('ptzTuneKeyboardMode').value = savedKeyboardMode === 'pulse' ? 'pulse' : 'hold';
    $('ptzTuneKeyboardMode').onchange = e => {
      const mode = e.target.value === 'pulse' ? 'pulse' : 'hold';
      localStorage.setItem('ptzTuneKeyboardMode', mode);
      ptzLog('KEYBOARD PTZ MODE', { mode });
    };
  }

'''

if "localStorage.getItem('ptzTuneKeyboardMode')" not in s:
    if anchor not in s:
        raise SystemExit("ERROR: J Pulse init anchor not found")
    s = s.replace(anchor, anchor + insert, 1)
    print("OK: inserted Keyboard mode init")
else:
    print("SKIP: Keyboard mode init already exists")


# 3. Insert smooth hold helpers before applyKeyboardDrive()
anchor = '''  function applyKeyboardDrive() {
'''

insert = '''  function getKeyboardPtzMode() {
    const el = $('ptzTuneKeyboardMode');
    return el && el.value === 'pulse' ? 'pulse' : 'hold';
  }

  const KEYBOARD_HOLD_REFRESH_MS = 220;
  const KEYBOARD_HOLD_BACKEND_MS = 750;
  const KEYBOARD_HOLD_LOOP_SLEEP_MS = 35;

  async function manualInputJHold(panNorm, tiltNorm, source = 'keyboard_hold') {
    panNorm = Math.max(-1, Math.min(1, Number(panNorm || 0)));
    tiltNorm = Math.max(-1, Math.min(1, Number(tiltNorm || 0)));

    const res = await apiPostJson(`${autopilotBaseUrl()}/api/control/manual_drive`, {
      pan: panNorm,
      tilt: tiltNorm,
      source,
      hold_ms: KEYBOARD_HOLD_BACKEND_MS
    });

    ptzLog('MANUAL J HOLD BACKEND', { ...res, source });
    return res;
  }

  let keyboardMotionInFlight = false;

  async function keyboardMotionLoop() {
    if (keyboardMotionInFlight) return;

    keyboardMotionInFlight = true;

    let lastPan = null;
    let lastTilt = null;
    let lastRefreshTs = 0;
    let usedHoldMode = false;

    try {
      while (manualKeyState.size > 0) {
        const v = currentManualKeyVector();
        const mode = getKeyboardPtzMode();

        if (Math.abs(v.pan) < 0.001 && Math.abs(v.tilt) < 0.001) {
          await new Promise(resolve => setTimeout(resolve, KEYBOARD_HOLD_LOOP_SLEEP_MS));
          continue;
        }

        if (mode === 'pulse') {
          await manualInputJPulse(
            v.pan,
            v.tilt,
            keyboardPulseSourceFromVector(v.pan, v.tilt)
          );
          await new Promise(resolve => setTimeout(resolve, KEYBOARD_J_PULSE_GAP_MS));
          continue;
        }

        usedHoldMode = true;

        const now = performance.now();
        const changed = v.pan !== lastPan || v.tilt !== lastTilt;
        const expired = (now - lastRefreshTs) >= KEYBOARD_HOLD_REFRESH_MS;

        if (changed || expired) {
          await manualInputJHold(
            v.pan,
            v.tilt,
            keyboardPulseSourceFromVector(v.pan, v.tilt)
          );

          lastPan = v.pan;
          lastTilt = v.tilt;
          lastRefreshTs = performance.now();
        }

        await new Promise(resolve => setTimeout(resolve, KEYBOARD_HOLD_LOOP_SLEEP_MS));
      }
    } catch (e) {
      ptzLog('KEYBOARD MOTION LOOP error', { error: String(e.message || e) });
    } finally {
      keyboardMotionInFlight = false;

      if (usedHoldMode) {
        manualInputStop('keyboard_hold_keyup_stop').catch(e =>
          ptzLog('KEYBOARD HOLD STOP error', { error: String(e.message || e) })
        );
      }

      if (manualKeyState.size > 0) {
        keyboardMotionLoop();
      }
    }
  }

'''

if 'async function keyboardMotionLoop()' not in s:
    if anchor not in s:
        raise SystemExit("ERROR: applyKeyboardDrive anchor not found")
    s = s.replace(anchor, insert + anchor, 1)
    print("OK: inserted smooth hold keyboard loop")
else:
    print("SKIP: smooth hold keyboard loop already exists")


# 4. Replace keydown call keyboardPulseLoop() -> keyboardMotionLoop()
old = '''    keyboardPulseLoop();
  });
'''

new = '''    keyboardMotionLoop();
  });
'''

if old in s:
    s = s.replace(old, new, 1)
    print("OK: keydown now starts keyboardMotionLoop")
elif 'keyboardMotionLoop();' in s:
    print("SKIP: keydown already uses keyboardMotionLoop")
else:
    raise SystemExit("ERROR: keydown keyboardPulseLoop call not found")

p.write_text(s, encoding="utf-8")
print("DONE")
