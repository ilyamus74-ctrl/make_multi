from pathlib import Path
import time

p = Path("web/index.html")
s = p.read_text(encoding="utf-8")
bak = p.with_suffix(p.suffix + f".bak_persistent_operator_state_{int(time.time())}")
bak.write_text(s, encoding="utf-8")

changed = False

def replace_once(old, new, label):
    global s, changed
    if old in s:
        s = s.replace(old, new, 1)
        changed = True
    else:
        print(f"WARN: anchor not found: {label}")

# 1) Default mode must be manual, not assist.
if "controlMode: 'assist'" in s:
    s = s.replace("controlMode: 'assist'", "controlMode: 'manual'")
    changed = True

# 2) Fix safeSyncControlModeUi recursion typo.
if "safeSyncControlModeUi();\n        return;" in s:
    s = s.replace("safeSyncControlModeUi();\n        return;", "syncControlModeUi();\n        return;", 1)
    changed = True

# 3) Insert persistent helpers after apiPostJson.
anchor = '''  async function apiPostJson(url, payload = {}) {
    const r = await fetch(url, {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify(payload),
    });
    if (!r.ok) throw new Error(`${r.status} ${url}`);
    return await r.json();
  }
'''
helper = r'''  async function apiPostJson(url, payload = {}) {
    const r = await fetch(url, {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify(payload),
    });
    if (!r.ok) throw new Error(`${r.status} ${url}`);
    return await r.json();
  }

  let settingsSaveTimer = null;
  function scheduleSettingsSave(reason = 'ui') {
    if (settingsSaveTimer) clearTimeout(settingsSaveTimer);
    settingsSaveTimer = setTimeout(() => {
      settingsSaveTimer = null;
      try {
        saveSettings();
      } catch (e) {
        try { ptzLog('SETTINGS SAVE error', { reason, error: String(e.message || e) }); } catch (_) {}
      }
    }, 250);
  }

  async function enterManualMode() {
    if ($('controlMode')) $('controlMode').value = 'manual';
    try {
      if (typeof setControlMode === 'function') setControlMode('manual');
      else safeSyncControlModeUi();
    } catch (_) {
      safeSyncControlModeUi();
    }

    try { await apiPostJson(`${autopilotBaseUrl()}/api/autopilot/stop`, {}); } catch (_) {}
    try { await manualInputStop('manual_mode'); } catch (_) {}

    await saveSettings();
    await ptzPrecheck().catch(() => {});
  }

'''
if "let settingsSaveTimer = null;" not in s:
    replace_once(anchor, helper, "apiPostJson helper insert")

# 4) Restore saved fields into UI.
old = """    $('controlMode').value = saved.controlMode;
"""
new = """    const restoredMode = ['manual','ptz','assist','auto'].includes(String(saved.controlMode || ''))
      ? String(saved.controlMode)
      : 'manual';
    $('controlMode').value = restoredMode;
"""
replace_once(old, new, "restore controlMode manual default")

old = """    zoomState = clamp01(Number(saved.zoomState ?? defaults.zoomState));
    zoomStateLastTs = Date.now();
    renderZoomState();
"""
new = """    zoomState = clamp01(Number(saved.zoomState ?? defaults.zoomState));

    if ($('operatorDetectionLimit') && saved.operatorDetectionLimit != null) {
      $('operatorDetectionLimit').value = String(saved.operatorDetectionLimit);
    }
    if ($('operatorDetectEvery') && saved.operatorDetectEvery != null) {
      $('operatorDetectEvery').value = String(saved.operatorDetectEvery);
    }
    if ($('operatorDetectionAreaMode') && saved.operatorDetectionAreaMode) {
      $('operatorDetectionAreaMode').value = String(saved.operatorDetectionAreaMode);
    }

    const pc = saved.ptzConfig || {};
    if ($('ptzInvertTilt') && pc.invert_tilt != null) $('ptzInvertTilt').checked = Boolean(pc.invert_tilt);
    if ($('ptzInvertPan') && pc.invert_pan != null) $('ptzInvertPan').checked = Boolean(pc.invert_pan);
    if ($('ptzTargetX') && pc.target_x != null) $('ptzTargetX').value = Number(pc.target_x).toFixed(2);
    if ($('ptzTargetY') && pc.target_y != null) $('ptzTargetY').value = Number(pc.target_y).toFixed(2);
    if ($('ptzMinPan') && pc.min_pan != null) $('ptzMinPan').value = String(pc.min_pan);
    if ($('ptzMinTilt') && pc.min_tilt != null) $('ptzMinTilt').value = String(pc.min_tilt);

    const ts = saved.ptzTune || {};
    if ($('ptzTuneKp') && ts.kp != null) $('ptzTuneKp').value = ts.kp;
    if ($('ptzTuneKd') && ts.kd != null) $('ptzTuneKd').value = ts.kd;
    if ($('ptzTuneMaxPan') && ts.max_pan != null) $('ptzTuneMaxPan').value = ts.max_pan;
    if ($('ptzTuneMaxTilt') && ts.max_tilt != null) $('ptzTuneMaxTilt').value = ts.max_tilt;
    if ($('ptzTuneMaxAccel') && ts.max_accel != null) $('ptzTuneMaxAccel').value = ts.max_accel;
    if ($('ptzTuneMinPan') && ts.min_pan != null) $('ptzTuneMinPan').value = ts.min_pan;
    if ($('ptzTuneMinTilt') && ts.min_tilt != null) $('ptzTuneMinTilt').value = ts.min_tilt;
    if ($('ptzTuneDeadzone') && ts.deadzone != null) $('ptzTuneDeadzone').value = ts.deadzone;
    if ($('ptzTuneCurve') && ts.ptz_curve != null) $('ptzTuneCurve').value = ts.ptz_curve;
    if ($('ptzTuneLeadMs') && ts.ptz_lead_ms != null) $('ptzTuneLeadMs').value = ts.ptz_lead_ms;
    if ($('ptzTuneNudgePan') && ts.nudge_pan != null) $('ptzTuneNudgePan').value = ts.nudge_pan;
    if ($('ptzTuneNudgeTilt') && ts.nudge_tilt != null) $('ptzTuneNudgeTilt').value = ts.nudge_tilt;
    if ($('ptzTuneKeyboardMode') && ts.manual_mode) $('ptzTuneKeyboardMode').value = ts.manual_mode;
    if (ts.j_pulse_ms != null && typeof syncJPulseInputs === 'function') syncJPulseInputs(ts.j_pulse_ms);

    zoomStateLastTs = Date.now();
    renderZoomState();
"""
replace_once(old, new, "restore extended settings")

# 5) Add fields to backend/local settings JSON.
old = """      controlMode: $('controlMode').value,
      landscapeLock: $('toggleLandscapeLock').dataset.locked === '1',
"""
new = """      controlMode: $('controlMode').value,
      operatorModel: $('operatorModelSelect')?.value || '',
      operatorDetectionLimit: Number($('operatorDetectionLimit')?.value || 10),
      operatorDetectEvery: Number($('operatorDetectEvery')?.value || 1),
      operatorDetectionAreaMode: $('operatorDetectionAreaMode')?.value || 'full_frame',
      ptzConfig: (typeof readPtzConfigFromUi === 'function') ? readPtzConfigFromUi() : {},
      ptzTune: {
        kp: Number($('ptzTuneKp')?.value || 0),
        kd: Number($('ptzTuneKd')?.value || 0),
        max_pan: Number($('ptzTuneMaxPan')?.value || 0),
        max_tilt: Number($('ptzTuneMaxTilt')?.value || 0),
        max_accel: Number($('ptzTuneMaxAccel')?.value || 0),
        min_pan: Number($('ptzTuneMinPan')?.value || 0),
        min_tilt: Number($('ptzTuneMinTilt')?.value || 0),
        deadzone: Number($('ptzTuneDeadzone')?.value || 0),
        ptz_curve: Number($('ptzTuneCurve')?.value || 1.0),
        ptz_lead_ms: Number($('ptzTuneLeadMs')?.value || 0),
        nudge_pan: Number($('ptzTuneNudgePan')?.value || 0),
        nudge_tilt: Number($('ptzTuneNudgeTilt')?.value || 0),
        j_pulse_ms: (typeof getJPulseMs === 'function') ? getJPulseMs() : Number($('ptzTuneJPulseMsNum')?.value || 70),
        manual_mode: $('ptzTuneKeyboardMode')?.value || 'tap_hold'
      },
      landscapeLock: $('toggleLandscapeLock').dataset.locked === '1',
"""
replace_once(old, new, "buildSettingsPayload extended fields")

# 6) Manual buttons must also stop backend autopilot.
s2 = s.replace(
    "$('modeManualBtn').onclick = () => setControlMode('manual');",
    "$('modeManualBtn').onclick = () => enterManualMode().catch(e => ptzLog('MANUAL MODE error', { error: String(e.message || e) }));"
)
s2 = s2.replace(
    "if ($('ptzModeManualBtn')) $('ptzModeManualBtn').onclick = () => setControlMode('manual');",
    "if ($('ptzModeManualBtn')) $('ptzModeManualBtn').onclick = () => enterManualMode().catch(e => ptzLog('MANUAL MODE error', { error: String(e.message || e) }));"
)
if s2 != s:
    s = s2
    changed = True

# 7) Do not kill autonomous PTZ on page close/reload.
old = "  window.addEventListener('beforeunload', () => panicStop('unload'));\n"
new = r'''  window.addEventListener('beforeunload', () => {
    try { saveSettings(); } catch (_) {}
    // Do not stop autonomous PTZ on page reload/close.
    // Only manual browser-driven movement is stopped.
    if (($('controlMode')?.value || 'manual') === 'manual') {
      try {
        const blob = new Blob(['{}'], { type: 'application/json' });
        navigator.sendBeacon(`${autopilotBaseUrl()}/api/control/stop`, blob);
      } catch (_) {}
    }
  });
'''
if old in s:
    s = s.replace(old, new, 1)
    changed = True

# 8) Save speed point with full UI speed values, not only j_pulse/curve.
old = """      focal_px: focalPx,
      j_pulse_ms: getJPulseMs(),
"""
new = """      focal_px: focalPx,
      kp: Number($('ptzTuneKp')?.value || 0),
      ki: 0,
      kd: Number($('ptzTuneKd')?.value || 0),
      deadzone: Number($('ptzTuneDeadzone')?.value || 0),
      max_pan: Number($('ptzTuneMaxPan')?.value || 0),
      max_tilt: Number($('ptzTuneMaxTilt')?.value || 0),
      max_accel: Number($('ptzTuneMaxAccel')?.value || 0),
      min_pan: Number($('ptzTuneMinPan')?.value || 0),
      min_tilt: Number($('ptzTuneMinTilt')?.value || 0),
      j_pulse_ms: getJPulseMs(),
"""
replace_once(old, new, "full speed point save payload")

# 9) Generic autosave for operator fields.
anchor = "  bindTouchControls();\n"
insert = r'''  bindTouchControls();

  document.addEventListener('change', (e) => {
    const id = String(e.target?.id || '');
    const persistIds = new Set([
      'operatorModelSelect',
      'operatorDetectionLimit',
      'operatorDetectEvery',
      'operatorDetectionAreaMode',
      'detectionToggleBtn',
      'ptzInvertTilt',
      'ptzInvertPan',
      'ptzTargetX',
      'ptzTargetY',
      'ptzMinPan',
      'ptzMinTilt',
      'ptzTuneKp',
      'ptzTuneKd',
      'ptzTuneMaxPan',
      'ptzTuneMaxTilt',
      'ptzTuneMaxAccel',
      'ptzTuneMinPan',
      'ptzTuneMinTilt',
      'ptzTuneDeadzone',
      'ptzTuneCurve',
      'ptzTuneLeadMs',
      'ptzTuneNudgePan',
      'ptzTuneNudgeTilt',
      'ptzTuneJPulseMs',
      'ptzTuneJPulseMsNum',
      'ptzTuneKeyboardMode'
    ]);
    if (persistIds.has(id) || id.startsWith('ptzTune') || id.startsWith('operator')) {
      scheduleSettingsSave('operator_field_change');
    }
  }, true);

'''
if "operator_field_change" not in s:
    replace_once(anchor, insert, "operator autosave listener")

if changed:
    p.write_text(s, encoding="utf-8")
    print(f"OK UI patched, backup={bak}")
else:
    print("OK UI already patched or anchors missed")
