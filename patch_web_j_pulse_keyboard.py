from pathlib import Path
import shutil
import time

p = Path("web/index.html")
s = p.read_text(encoding="utf-8")

backup = p.with_suffix(f".html.bak_j_pulse_keyboard_{int(time.time())}")
shutil.copy2(p, backup)
print("backup:", backup)

# 1) Add J Pulse slider after APPLY SPEED
old = '''        <button id="ptzTuneApplyConfigBtn" class="small-btn">APPLY SPEED</button>
        <code id="ptzTuneStatus">sample=0 zoom=0.00 focal=0 points=0 stepsSinceHome=0</code>
'''

new = '''        <button id="ptzTuneApplyConfigBtn" class="small-btn">APPLY SPEED</button>
        <label title="Duration of direct J pulse for keyboard micro movement">
          J Pulse ms
          <input id="ptzTuneJPulseMs" type="range" min="1" max="120" step="1" value="70" style="width:130px">
          <input id="ptzTuneJPulseMsNum" type="number" min="1" max="120" step="1" value="70" style="width:64px">
        </label>
        <code id="ptzTuneStatus">sample=0 zoom=0.00 focal=0 points=0 stepsSinceHome=0</code>
'''

if 'id="ptzTuneJPulseMs"' not in s:
    if old not in s:
        raise SystemExit("ERROR: APPLY SPEED HTML anchor not found")
    s = s.replace(old, new, 1)
    print("OK: inserted J Pulse ms slider")
else:
    print("SKIP: J Pulse ms slider already exists")


# 2) Add JS helpers after manualInputStop
anchor = '''  async function manualInputStop(source = 'manual_ui') {
    return backendManualStop(source);
  }

'''

insert = '''  function getJPulseMs() {
    const slider = $('ptzTuneJPulseMs');
    const num = $('ptzTuneJPulseMsNum');
    const raw = Number((num && num.value) || (slider && slider.value) || 70);
    return Math.max(1, Math.min(120, Math.round(raw)));
  }

  function syncJPulseInputs(v) {
    v = Math.max(1, Math.min(120, Math.round(Number(v || 70))));
    if ($('ptzTuneJPulseMs')) $('ptzTuneJPulseMs').value = String(v);
    if ($('ptzTuneJPulseMsNum')) $('ptzTuneJPulseMsNum').value = String(v);
    try { localStorage.setItem('ptzTuneJPulseMs', String(v)); } catch (_) {}
    return v;
  }

  async function manualInputJPulse(panNorm, tiltNorm, source = 'manual_j_pulse') {
    panNorm = Math.max(-1, Math.min(1, Number(panNorm || 0)));
    tiltNorm = Math.max(-1, Math.min(1, Number(tiltNorm || 0)));

    const pulseMs = getJPulseMs();
    const res = await apiPostJson(`${autopilotBaseUrl()}/api/control/manual_j_pulse`, {
      pan: panNorm,
      tilt: tiltNorm,
      source,
      pulse_ms: pulseMs
    });

    ptzLog('MANUAL J PULSE BACKEND', { ...res, source });
    return res;
  }

'''

if 'function getJPulseMs()' not in s:
    if anchor not in s:
        raise SystemExit("ERROR: manualInputStop anchor not found")
    s = s.replace(anchor, anchor + insert, 1)
    print("OK: inserted manualInputJPulse helpers")
else:
    print("SKIP: manualInputJPulse helpers already exist")


# 3) Initialize slider before keyboard hotkeys block
init_anchor = '''  const manualKeyState = new Set();

'''

init_code = '''  if ($('ptzTuneJPulseMs')) {
    const savedPulse = Number(localStorage.getItem('ptzTuneJPulseMs') || 70);
    syncJPulseInputs(savedPulse);
    $('ptzTuneJPulseMs').oninput = e => syncJPulseInputs(e.target.value);
  }
  if ($('ptzTuneJPulseMsNum')) {
    $('ptzTuneJPulseMsNum').onchange = e => syncJPulseInputs(e.target.value);
  }

'''

if "localStorage.getItem('ptzTuneJPulseMs')" not in s:
    if init_anchor not in s:
        raise SystemExit("ERROR: manualKeyState anchor not found")
    s = s.replace(init_anchor, init_code + init_anchor, 1)
    print("OK: inserted J Pulse slider init")
else:
    print("SKIP: J Pulse slider init already exists")


# 4) Replace keyboard arrows: startManualHold -> manualInputJPulse
old = '''    // On remote browsers keyup/keydown can flicker. Always refresh hold on keydown,
    // including repeat events.
    manualKeyState.clear();
    manualKeyState.add(dir.keyName);
    startManualHold(dir.pan, dir.tilt, dir.source);
'''

new = '''    // Keyboard arrows use direct J pulse:
    // one keydown = one micro movement. No hold/ramp/keyup stop.
    manualKeyState.clear();
    manualKeyState.add(dir.keyName);
    manualInputJPulse(dir.pan, dir.tilt, dir.source).catch(e =>
      ptzLog('MANUAL J PULSE error', { source: dir.source, error: String(e.message || e) })
    );
'''

if old in s:
    s = s.replace(old, new, 1)
    print("OK: keyboard arrows switched to manual_j_pulse")
elif "manualInputJPulse(dir.pan, dir.tilt, dir.source)" in s:
    print("SKIP: keyboard already uses manual_j_pulse")
else:
    raise SystemExit("ERROR: keyboard startManualHold block not found")


# 5) keyup should not schedule hold stop for pulse mode
old = '''    manualKeyState.delete(dir.keyName);
    scheduleKeyboardHoldStop();
'''

new = '''    manualKeyState.delete(dir.keyName);
    if (manualHoldActive) {
      scheduleKeyboardHoldStop();
    }
'''

if old in s:
    s = s.replace(old, new, 1)
    print("OK: keyup no longer sends delayed stop for pulse mode")
elif "if (manualHoldActive) {" in s and "scheduleKeyboardHoldStop();" in s:
    print("SKIP: keyup stop already guarded")
else:
    raise SystemExit("ERROR: keyup scheduleKeyboardHoldStop block not found")

p.write_text(s, encoding="utf-8")
print("DONE")
