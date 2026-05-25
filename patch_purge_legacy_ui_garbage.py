from pathlib import Path
import re
import shutil
import time

p = Path("web/index.html")
s = p.read_text(encoding="utf-8")

backup = p.with_suffix(f".html.bak_purge_legacy_ui_garbage_{int(time.time())}")
shutil.copy2(p, backup)
print(f"backup: {backup}")

# ---------------------------------------------------------------------
# 1) Remove old duplicate Q/W hotkey block.
# This block used keyboardZoomPulse() and /api/zoom/pulse and conflicts
# with the newer keyboardZoomSampleStep() Q/A flow.
# ---------------------------------------------------------------------
pattern = re.compile(
    r'\n\s*async function keyboardZoomPulse[\s\S]*?// SAMPLE_INIT_HOTKEYS_REPAIR_END\s*\n',
    re.M
)

s2, n = pattern.subn('\n  // Legacy keyboardZoomPulse block removed. Q/A are handled by keyboardZoomSampleStep().\n', s, count=1)
if n != 1:
    print("WARN: old keyboardZoomPulse block not found or already removed")
else:
    s = s2
    print("OK: removed old keyboardZoomPulse / Q_WIDE hotkey block")


# ---------------------------------------------------------------------
# 2) Remove visible settings gear button.
# It only opened old controlsSpoiler.
# ---------------------------------------------------------------------
s2, n = re.subn(
    r'\s*<button id="settingsToggleBtn"[\s\S]*?</button>\s*',
    '\n',
    s,
    count=1
)
if n:
    s = s2
    print("OK: removed visible settingsToggleBtn")
else:
    print("WARN: visible settingsToggleBtn not found")


# ---------------------------------------------------------------------
# 3) Replace old controlsSpoiler details with hidden compatibility stubs.
# We remove visible legacy UI but keep old IDs that old JS still reads.
# ---------------------------------------------------------------------
start = s.find('<details id="controlsSpoiler"')
if start >= 0:
    end = s.find('</details>', start)
    if end < 0:
        raise SystemExit("ERROR: controlsSpoiler closing </details> not found")
    end += len('</details>')

    legacy_stubs = r'''
      <!-- Legacy UI removed. Hidden stubs keep old JS references from crashing during migration. -->
      <div id="legacyCompatStubs" hidden aria-hidden="true" style="display:none!important">
        <span id="ws" class="ok">CONNECTED</span>
        <code id="wsurl">-</code>
        <span id="gp" class="bad">NOT FOUND</span>
        <code id="tx">-</code>
        <code id="ack">-</code>
        <span id="axes">-</span>
        <span id="zoomStick">-</span>

        <code id="zoomStateValue">0.00</code>
        <input id="zoomTravelSec" type="number" value="12">
        <button id="zoomMarkWide"></button>
        <button id="zoomMarkTele"></button>
        <button id="zoomRunCalibrationBtn"></button>
        <code id="zoomCalibStatus">idle</code>

        <code id="msBtn">-</code>
        <span id="msLegend">X← B→ A↓ Y↑</span>

        <select id="controlMode">
          <option value="manual" selected>MANUAL</option>
          <option value="assist">ASSIST</option>
          <option value="auto">AUTO</option>
          <option value="ptz">PTZ</option>
        </select>
        <input id="auto" type="checkbox" checked>
        <input id="port" type="number" value="8765">
        <input id="path" type="text" value="/ws">
        <input id="hz" type="number" value="50">
        <input id="dz" type="number" value="5">

        <select id="controlProfile">
          <option value="indoor" selected>indoor</option>
          <option value="outdoor">outdoor</option>
          <option value="night">night</option>
        </select>
        <input id="safeLostSec" type="number" value="2">
        <input id="safeRestoreSec" type="number" value="3">

        <input id="confL0Enabled" type="checkbox" checked>
        <input id="confW0" type="number" value="0.50">
        <input id="confL1Enabled" type="checkbox" checked>
        <input id="confW1" type="number" value="0.30">
        <input id="confL2Enabled" type="checkbox" checked>
        <input id="confW2" type="number" value="0.20">
        <input id="confL3Enabled" type="checkbox" checked>
        <input id="confW3" type="number" value="0.25">

        <code id="confDbgRevision">--/--</code>
        <code id="confDbgFused">--→--</code>
        <code id="confDbgLayers">L0-- L1-- L2-- L3--</code>

        <input id="invertTilt" type="checkbox" checked>
        <input id="swapMicroButtons" type="checkbox">

        <input id="pidKp" type="number" value="32">
        <input id="pidKi" type="number" value="0">
        <input id="pidKd" type="number" value="18">
        <input id="pidDeadzone" type="number" value="4">
        <input id="pidMaxSpeed" type="number" value="40">
        <input id="pidMaxAccel" type="number" value="8">
        <input id="pidPanHold" type="number" value="2">
        <input id="pidPanRelease" type="number" value="5">

        <input id="reacquirePanStep" type="number" value="18">
        <input id="reacquireTiltStep" type="number" value="8">
        <input id="reacquirePanCadence" type="number" value="8">
        <input id="reacquireTiltCadence" type="number" value="16">
        <input id="reacquireHoldMs" type="number" value="700">
        <input id="reacquireMaxLostSec" type="number" value="8">

        <input id="showDetOverlay" type="checkbox" checked>
        <input id="selectTargetByTap" type="checkbox" checked>
        <button id="clearSelectedTarget"></button>

        <code id="pidTarget">none</code>
        <code id="pidSelectedId">auto</code>
        <code id="pidCmd">0,0</code>

        <button id="home"></button>
        <button id="stop"></button>
        <button id="center"></button>
        <button id="connect"></button>
        <button id="disconnect"></button>
        <button id="exportSettings"></button>
        <button id="importSettings"></button>
        <button id="exportDiagnostics"></button>
        <input id="importSettingsFile" type="file">
      </div>
'''
    s = s[:start] + legacy_stubs + s[end:]
    print("OK: replaced controlsSpoiler with hidden compatibility stubs")
else:
    print("WARN: controlsSpoiler details not found or already removed")


# ---------------------------------------------------------------------
# 4) Remove touchControls visible HTML if still present.
# ---------------------------------------------------------------------
s2, n = re.subn(
    r'\n\s*<div class="touch-controls" id="touchControls">[\s\S]*?</div>\s*',
    '\n',
    s,
    count=1
)
if n:
    s = s2
    print("OK: removed touchControls HTML")
else:
    print("SKIP: touchControls HTML not found")


# ---------------------------------------------------------------------
# 5) Remove touchControls CSS blocks / guard blocks.
# ---------------------------------------------------------------------
# Remove simple .touch-controls CSS blocks.
s = re.sub(r'\n\s*\.touch-controls\s*\{[^}]*\}\s*', '\n', s)
s = re.sub(r'\n\s*\.touch-controls\.disabled\s+\.touch-btn\s*\{[^}]*\}\s*', '\n', s)

# Remove old #touchControls guard CSS.
s = re.sub(r'\n\s*/\* Touch overlay removed:[\s\S]*?#touchControls,[\s\S]*?\.touch-controls\s*\{[^}]*\}\s*', '\n', s)

# Remove controlsSpoiler/settingsToggle CSS hide blocks.
s = re.sub(r'\n\s*#controlsSpoiler,\s*\n\s*#settingsToggleBtn\s*\{[^}]*\}\s*', '\n', s)
s = re.sub(r'\n\s*/\* Hide legacy control panel;[\s\S]*?#controlsSpoiler,\s*\n\s*#settingsToggleBtn\s*\{[^}]*\}\s*', '\n', s)


# ---------------------------------------------------------------------
# 6) Remove / neutralize touchControls JS references.
# ---------------------------------------------------------------------
s = s.replace("    $('touchControls').classList.toggle('disabled', isAuto);\n", "")

# Replace bindTouchControls with no-op.
def replace_function(text, name, replacement):
    start = text.find(f"  function {name}(")
    if start < 0:
        return text, 0
    brace = text.find("{", start)
    if brace < 0:
        return text, 0
    depth = 0
    i = brace
    while i < len(text):
        ch = text[i]
        if ch == "{":
            depth += 1
        elif ch == "}":
            depth -= 1
            if depth == 0:
                end = i + 1
                return text[:start] + replacement + text[end:], 1
        i += 1
    return text, 0

noop_bind = """  function bindTouchControls() {
    // On-screen touch controls were removed. Keyboard/manual backend control remains.
  }
"""
s, n = replace_function(s, "bindTouchControls", noop_bind)
if n:
    print("OK: bindTouchControls replaced with no-op")
else:
    print("WARN: bindTouchControls not found")


p.write_text(s, encoding="utf-8")
print("DONE")
