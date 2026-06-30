from pathlib import Path
import shutil, time

p = Path("web/index.html")
s = p.read_text(encoding="utf-8")

backup = p.with_suffix(f".html.bak_tap_hold_profile_controls_{int(time.time())}")
shutil.copy2(p, backup)
print("backup:", backup)

# 1. HTML: add Nudge inputs and tap_hold option
old = '''        <label>Min Tilt <input id="ptzTuneMinTilt" type="number" step="1"></label>
        <label>Deadzone <input id="ptzTuneDeadzone" type="number" step="0.001"></label>
        <button id="ptzTuneApplyConfigBtn" class="small-btn">APPLY SPEED</button>
'''
new = '''        <label>Min Tilt <input id="ptzTuneMinTilt" type="number" step="1"></label>
        <label>Deadzone <input id="ptzTuneDeadzone" type="number" step="0.001"></label>
        <label title="Pulse single-click pan command">Nudge Pan <input id="ptzTuneNudgePan" type="number" min="0" max="100" step="1" value="12"></label>
        <label title="Pulse single-click tilt command">Nudge Tilt <input id="ptzTuneNudgeTilt" type="number" min="0" max="100" step="1" value="12"></label>
        <button id="ptzTuneApplyConfigBtn" class="small-btn">APPLY SPEED</button>
'''
if 'id="ptzTuneNudgePan"' not in s:
    if old not in s:
        raise SystemExit("ERROR: PTZ tune HTML anchor not found")
    s = s.replace(old, new, 1)
    print("OK: inserted Nudge Pan/Tilt inputs")
else:
    print("SKIP: Nudge inputs already exist")

old = '''            <option value="hold" selected>HOLD SMOOTH</option>
            <option value="pulse">PULSE STEP</option>
'''
new = '''            <option value="tap_hold" selected>TAP=PULSE / HOLD=SMOOTH</option>
            <option value="hold">HOLD SMOOTH</option>
            <option value="pulse">PULSE STEP</option>
'''
if '<option value="tap_hold"' not in s:
    if old not in s:
        raise SystemExit("ERROR: Keyboard mode options anchor not found")
    s = s.replace(old, new, 1)
    print("OK: added tap_hold keyboard mode")
else:
    print("SKIP: tap_hold option already exists")

# 2. fillPtzTuneFieldsFromConfig: add nudge/manual mode.
# If function is compact/one-line, patch by inserting after existing fields via common anchors.
old = '''if($('ptzTuneMinTilt')) $('ptzTuneMinTilt').value=Number(c.min_tilt??0);'''
new = '''if($('ptzTuneMinTilt')) $('ptzTuneMinTilt').value=Number(c.min_tilt??0); if($('ptzTuneNudgePan')) $('ptzTuneNudgePan').value=Number(c.nudge_pan??c.max_pan??12); if($('ptzTuneNudgeTilt')) $('ptzTuneNudgeTilt').value=Number(c.nudge_tilt??c.max_tilt??12); if($('ptzTuneKeyboardMode') && c.manual_mode) { $('ptzTuneKeyboardMode').value = (c.manual_mode === 'pulse' || c.manual_mode === 'hold') ? c.manual_mode : 'tap_hold'; localStorage.setItem('ptzTuneKeyboardMode', $('ptzTuneKeyboardMode').value); }'''
if old in s:
    s = s.replace(old, new, 1)
    print("OK: fillPtzTuneFieldsFromConfig handles nudge/manual_mode")
elif 'ptzTuneNudgePan' in s and 'c.nudge_pan' in s:
    print("SKIP: fillPtzTuneFieldsFromConfig already patched")
else:
    print("WARN: exact fillPtzTuneFieldsFromConfig anchor not found; will still save/load via explicit loaders")

# 3. save payload add nudge/manual_mode
old = '''      focal_px: focalPx,
      j_pulse_ms: getJPulseMs()
    });
'''
new = '''      focal_px: focalPx,
      j_pulse_ms: getJPulseMs(),
      nudge_pan: Number($('ptzTuneNudgePan')?.value || $('ptzTuneMaxPan')?.value || 0),
      nudge_tilt: Number($('ptzTuneNudgeTilt')?.value || $('ptzTuneMaxTilt')?.value || 0),
      manual_mode: $('ptzTuneKeyboardMode')?.value || 'tap_hold'
    });
'''
if old in s:
    s = s.replace(old, new, 1)
    print("OK: SAVE SPEED POINT sends nudge/manual_mode")
elif 'nudge_pan: Number' in s and 'manual_mode:' in s:
    print("SKIP: save payload already patched")
else:
    raise SystemExit("ERROR: save payload anchor not found")

# 4. apply config should also keep current local nudge/mode saved in localStorage only; backend save point gets fields.
# 5. keyboard mode init default tap_hold
old = '''    const savedKeyboardMode = localStorage.getItem('ptzTuneKeyboardMode') || 'hold';
    $('ptzTuneKeyboardMode').value = savedKeyboardMode === 'pulse' ? 'pulse' : 'hold';
    $('ptzTuneKeyboardMode').onchange = e => {
      const mode = e.target.value === 'pulse' ? 'pulse' : 'hold';
'''
new = '''    const savedKeyboardMode = localStorage.getItem('ptzTuneKeyboardMode') || 'tap_hold';
    $('ptzTuneKeyboardMode').value = (savedKeyboardMode === 'pulse' || savedKeyboardMode === 'hold') ? savedKeyboardMode : 'tap_hold';
    $('ptzTuneKeyboardMode').onchange = e => {
      const mode = (e.target.value === 'pulse' || e.target.value === 'hold') ? e.target.value : 'tap_hold';
'''
if old in s:
    s = s.replace(old, new, 1)
    print("OK: keyboard mode init supports tap_hold")
elif "|| 'tap_hold'" in s:
    print("SKIP: keyboard mode init already supports tap_hold")
else:
    raise SystemExit("ERROR: keyboard mode init anchor not found")

# 6. getKeyboardPtzMode supports tap_hold
old = '''  function getKeyboardPtzMode() {
    const el = $('ptzTuneKeyboardMode');
    return el && el.value === 'pulse' ? 'pulse' : 'hold';
  }
'''
new = '''  function getKeyboardPtzMode() {
    const el = $('ptzTuneKeyboardMode');
    const v = el ? String(el.value || 'tap_hold') : 'tap_hold';
    return (v === 'pulse' || v === 'hold') ? v : 'tap_hold';
  }
'''
if old in s:
    s = s.replace(old, new, 1)
    print("OK: getKeyboardPtzMode supports tap_hold")
elif "return (v === 'pulse' || v === 'hold')" in s:
    print("SKIP: getKeyboardPtzMode already patched")
else:
    raise SystemExit("ERROR: getKeyboardPtzMode anchor not found")

# 7. add tap-hold state after manualKeyState declaration
anchor = '''  const manualKeyState = new Set();

'''
insert = '''  let keyboardTapHoldTimer = null;
  let keyboardTapHoldActive = false;
  let keyboardTapHoldVector = { pan: 0, tilt: 0 };
  const KEYBOARD_TAP_HOLD_THRESHOLD_MS = 170;

  function clearKeyboardTapHoldTimer() {
    if (keyboardTapHoldTimer) {
      clearTimeout(keyboardTapHoldTimer);
      keyboardTapHoldTimer = null;
    }
  }

  function startKeyboardTapHoldTimer() {
    clearKeyboardTapHoldTimer();
    keyboardTapHoldActive = false;
    keyboardTapHoldTimer = setTimeout(() => {
      keyboardTapHoldTimer = null;
      if (manualKeyState.size > 0) {
        keyboardTapHoldActive = true;
        keyboardMotionLoop();
      }
    }, KEYBOARD_TAP_HOLD_THRESHOLD_MS);
  }

'''
if 'KEYBOARD_TAP_HOLD_THRESHOLD_MS' not in s:
    if anchor not in s:
        raise SystemExit("ERROR: manualKeyState anchor not found")
    s = s.replace(anchor, anchor + insert, 1)
    print("OK: added tap-hold state")
else:
    print("SKIP: tap-hold state already exists")

# 8. keydown branch
old = '''    manualKeyState.add(dir.keyName);
    updateKeyboardStateFromManualKeys();
    keyboardMotionLoop();
  });
'''
new = '''    manualKeyState.add(dir.keyName);
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
  });
'''
if old in s:
    s = s.replace(old, new, 1)
    print("OK: keydown supports tap_hold")
elif 'startKeyboardTapHoldTimer();' in s:
    print("SKIP: keydown already patched")
else:
    raise SystemExit("ERROR: keydown anchor not found")

# 9. keyup branch
old = '''    manualKeyState.delete(dir.keyName);
    updateKeyboardStateFromManualKeys();
  });
'''
new = '''    const beforeRelease = currentManualKeyVector();
    manualKeyState.delete(dir.keyName);
    updateKeyboardStateFromManualKeys();

    if (getKeyboardPtzMode() === 'tap_hold') {
      if (!keyboardTapHoldActive && manualKeyState.size === 0) {
        clearKeyboardTapHoldTimer();
        const v = keyboardTapHoldVector || beforeRelease;
        if (Math.abs(v.pan) > 0.001 || Math.abs(v.tilt) > 0.001) {
          manualInputJPulse(v.pan, v.tilt, keyboardPulseSourceFromVector(v.pan, v.tilt)).catch(e =>
            ptzLog('TAP-HOLD PULSE error', { error: String(e.message || e) })
          );
        }
        keyboardTapHoldVector = { pan: 0, tilt: 0 };
      }
    }
  });
'''
if old in s:
    s = s.replace(old, new, 1)
    print("OK: keyup supports tap pulse")
elif 'TAP-HOLD PULSE error' in s:
    print("SKIP: keyup already patched")
else:
    raise SystemExit("ERROR: keyup anchor not found")

# 10. keyboardMotionLoop finally resets tap hold active after stop
old = '''      keyboardMotionInFlight = false;

      if (usedHoldMode) {
'''
new = '''      keyboardMotionInFlight = false;
      keyboardTapHoldActive = false;
      clearKeyboardTapHoldTimer();

      if (usedHoldMode) {
'''
if old in s:
    s = s.replace(old, new, 1)
    print("OK: keyboardMotionLoop resets tap_hold")
elif 'keyboardTapHoldActive = false;' in s:
    print("SKIP: loop already resets tap_hold")
else:
    print("WARN: loop finally anchor not found")

p.write_text(s, encoding="utf-8")
print("DONE")
