from pathlib import Path
import shutil

p = Path("web/index.html")
s = p.read_text(encoding="utf-8")

backup = p.with_suffix(".html.bak_operator_cleanup")
if not backup.exists():
    shutil.copy2(p, backup)
    print(f"backup: {backup}")

# 1. Hide legacy Assist/Auto buttons inside visible PTZ panel.
s = s.replace(
    '<button id="ptzModeAssistBtn" class="small-btn">Assist</button>',
    '<button id="ptzModeAssistBtn" class="small-btn hidden">Assist</button>'
)
s = s.replace(
    '<button id="ptzModeAutoBtn" class="small-btn">Auto</button>',
    '<button id="ptzModeAutoBtn" class="small-btn hidden">Auto</button>'
)

# 2. Collapse Advanced by default.
s = s.replace(
    '<details id="controlsSpoiler" class="controls-spoiler" open>',
    '<details id="controlsSpoiler" class="controls-spoiler">'
)

# 3. Reduce polling pressure.
s = s.replace(
    'setInterval(refreshPtzTelemetry, 500);',
    'setInterval(refreshPtzTelemetry, 1000);'
)

# 4. Replace setControlMode with safer logic.
old = '''  function setControlMode(mode) {
    if (mode !== 'manual' && mode !== 'assist' && mode !== 'auto' && mode !== 'ptz') return;
    if ($('controlMode').value === mode) return;
    if (mode === 'ptz') {
      if ($('auto')) $('auto').checked = false;
      disconnectWs();
    } else if (mode === 'manual') {
      fetch(`${autopilotBaseUrl()}/api/autopilot/stop`, { method: 'POST' }).catch(() => {});
    }
    $('controlMode').value = mode;
    syncControlModeUi();
    saveSettings();
    panicStop(`mode-quick-switch-${mode}`);
  }'''

new = '''  function setControlMode(mode) {
    if (mode !== 'manual' && mode !== 'assist' && mode !== 'auto' && mode !== 'ptz') return;
    if ($('controlMode').value === mode) return;

    if (mode === 'ptz') {
      $('controlMode').value = 'ptz';
      if ($('auto')) $('auto').checked = false;
      disconnectWs();
      syncControlModeUi();
      saveSettings();
      ptzPrecheck().catch(e => ptzLog('PTZ mode precheck error', { error: String(e.message || e) }));
      return;
    }

    if (mode === 'manual') {
      fetch(`${autopilotBaseUrl()}/api/autopilot/stop`, { method: 'POST' }).catch(() => {});
    }

    $('controlMode').value = mode;
    syncControlModeUi();
    saveSettings();
    panicStop(`mode-quick-switch-${mode}`);
  }'''

if old not in s:
    raise SystemExit("setControlMode block not found; repo changed or patch already applied")

s = s.replace(old, new, 1)

# 5. Make controlMode onchange use same path, otherwise select PTZ still calls panicStop directly.
old = '''  $('controlMode').onchange = () => {
    syncControlModeUi();
    saveSettings();
    panicStop('mode-switch');
  };'''

new = '''  $('controlMode').onchange = () => {
    setControlMode($('controlMode').value);
  };'''

if old in s:
    s = s.replace(old, new, 1)
else:
    print("WARN: controlMode onchange block not found; skipped")

p.write_text(s, encoding="utf-8")
print("patched web/index.html")
