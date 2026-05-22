from pathlib import Path

# 1. ptz_autopilot: listen on LAN, not only localhost
p = Path("ptz_autopilot.cpp")
s = p.read_text()

s = s.replace(
    "a.sin_addr.s_addr=htonl(INADDR_LOOPBACK);",
    "a.sin_addr.s_addr=htonl(INADDR_ANY);"
)

p.write_text(s)
print("patched ptz_autopilot.cpp: control API bind 0.0.0.0")


# 2. web UI: make top row wrap and add visible mode buttons into PTZ panel
p = Path("web/index.html")
s = p.read_text()

s = s.replace(
    ".floating-row{display:flex;gap:8px;align-items:center}",
    ".floating-row{display:flex;gap:8px;align-items:center;flex-wrap:wrap}"
)

s = s.replace(
    'style="position:absolute;left:10px;bottom:10px;z-index:4;background:var(--controls-bg);border:1px solid var(--controls-border);border-radius:8px;padding:6px 10px;"',
    'style="position:absolute;left:10px;bottom:10px;z-index:4;background:var(--controls-bg);border:1px solid var(--controls-border);border-radius:8px;padding:6px 10px;flex-wrap:wrap;max-height:96px;overflow:auto;align-items:center;"'
)

s = s.replace(
    'Boxes: <code>DET=yellow TRACK=cyan SEL=green</code>',
    'Boxes: <code>DET=yellow TRACK=cyan SEL=red</code>'
)

old = '''      <div class="row" id="ptzPanel">
        <b>PTZ:</b>
        <button id="ptzPrecheckBtn" class="small-btn">PRECHECK</button>'''

new = '''      <div class="row" id="ptzPanel">
        <b>MODE:</b>
        <button id="ptzModeManualBtn" class="small-btn">Manual</button>
        <button id="ptzModeAssistBtn" class="small-btn">Assist</button>
        <button id="ptzModeAutoBtn" class="small-btn">Auto</button>
        <button id="ptzModePtzBtn" class="small-btn">PTZ</button>
        <span style="opacity:.55">|</span>
        <b>PTZ:</b>
        <button id="ptzPrecheckBtn" class="small-btn">PRECHECK</button>'''

if old not in s:
    print("WARN: PTZ panel insertion pattern not found")
else:
    s = s.replace(old, new, 1)

old = '''  $('modeManualBtn').onclick = () => setControlMode('manual');
  $('modeAssistBtn').onclick = () => setControlMode('assist');
  $('modeAutoBtn').onclick = () => setControlMode('auto');
  $('ptzPrecheckBtn').onclick = () => ptzPrecheck().catch(e => ptzLog('PRECHECK error', { error: String(e.message || e) }));'''

new = '''  $('modeManualBtn').onclick = () => setControlMode('manual');
  $('modeAssistBtn').onclick = () => setControlMode('assist');
  $('modeAutoBtn').onclick = () => setControlMode('auto');

  const enterPtzMode = async () => {
    $('controlMode').value = 'ptz';
    if ($('auto')) $('auto').checked = false;
    syncControlModeUi();
    saveSettings();
    disconnectWs();
    await ptzPrecheck();
  };

  if ($('ptzModeManualBtn')) $('ptzModeManualBtn').onclick = () => setControlMode('manual');
  if ($('ptzModeAssistBtn')) $('ptzModeAssistBtn').onclick = () => setControlMode('assist');
  if ($('ptzModeAutoBtn')) $('ptzModeAutoBtn').onclick = () => setControlMode('auto');
  if ($('ptzModePtzBtn')) $('ptzModePtzBtn').onclick = enterPtzMode;

  $('ptzPrecheckBtn').onclick = () => ptzPrecheck().catch(e => ptzLog('PRECHECK error', { error: String(e.message || e) }));'''

if old not in s:
    print("WARN: mode button wiring pattern not found")
else:
    s = s.replace(old, new, 1)

old = '''  $('modePtzBtn').onclick = async () => {
    $('controlMode').value = 'ptz';
    if ($('auto')) $('auto').checked = false;
    syncControlModeUi();
    saveSettings();
    disconnectWs();
    await ptzPrecheck();
  };'''

new = '''  $('modePtzBtn').onclick = enterPtzMode;'''

if old not in s:
    print("WARN: modePtzBtn handler pattern not found")
else:
    s = s.replace(old, new, 1)

s = s.replace(
    "$('touchControls').classList.toggle('disabled', isAuto);",
    "$('touchControls').classList.toggle('disabled', isAuto || isPtz);"
)

s = s.replace(
    "$('controlMode').value = 'manual';\n    if ($('auto')) $('auto').checked = false;",
    "$('controlMode').value = 'ptz';\n    if ($('auto')) $('auto').checked = false;"
)

p.write_text(s)
print("patched web/index.html")
