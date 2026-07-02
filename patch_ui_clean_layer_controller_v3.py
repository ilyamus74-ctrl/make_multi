from pathlib import Path
import time

p = Path("web/index.html")
s = p.read_text(encoding="utf-8", errors="ignore")

bak = p.with_suffix(p.suffix + f".bak_clean_layer_controller_v3_{int(time.time())}")
bak.write_text(s, encoding="utf-8")

changed = False

# 1. Remove stale old operator autosave listener inserted into the old main UI.
# This old listener calls saveSettings() on LIMIT/FPS/AREA changes and can overwrite activeObjectPreset.
removed_autosave = 0

while "operator_field_change" in s:
    idx = s.find("operator_field_change")

    start = s.rfind("document.addEventListener('change'", 0, idx)

    if start < 0:
        start = s.rfind('document.addEventListener("change"', 0, idx)

    end = s.find("}, true);", idx)

    if start >= 0 and end >= 0:
        end2 = s.find("\n", end)

        if end2 < 0:
            end2 = end + len("}, true);")

        s = (
            s[:start]
            + "\n  /* removed stale operator_field_change autosave by clean layer v3 */\n"
            + s[end2 + 1:]
        )
        removed_autosave += 1
        changed = True
    else:
        s = s.replace("operator_field_change", "operator_field_change_removed")
        removed_autosave += 1
        changed = True

# 2. Pin active object preset inside the clean controller.
old = """  function activeObjectName(settings) {
    return String(settings.activeObjectPreset || settings?.lastAppliedObjectPreset?.name || 'person_single');
  }
"""

new = """  let cleanActiveObjectPresetName = null;

  function activeObjectName(settings) {
    return String(
      cleanActiveObjectPresetName ||
      settings.activeObjectPreset ||
      settings?.lastAppliedObjectPreset?.name ||
      'person_single'
    );
  }
"""

if old in s and "let cleanActiveObjectPresetName = null;" not in s:
    s = s.replace(old, new, 1)
    changed = True
else:
    print("WARN: activeObjectName anchor not replaced or already patched")

# 3. When preset is selected, immediately pin it.
old = """  async function selectObjectPreset(name) {
    let settings = await readSettings();
"""

new = """  async function selectObjectPreset(name) {
    cleanActiveObjectPresetName = String(name || cleanActiveObjectPresetName || 'person_single');

    let settings = await readSettings();
"""

if old in s and "cleanActiveObjectPresetName = String(name || cleanActiveObjectPresetName" not in s:
    s = s.replace(old, new, 1)
    changed = True
else:
    print("WARN: selectObjectPreset pin anchor not replaced or already patched")

# 4. On sync, remember the currently active preset.
old = """    const { name, preset } = getActivePreset(settings);

    syncUiFromPreset(preset);
"""

new = """    const { name, preset } = getActivePreset(settings);

    cleanActiveObjectPresetName = String(name || cleanActiveObjectPresetName || 'person_single');

    syncUiFromPreset(preset);
"""

if old in s and "cleanActiveObjectPresetName = String(name || cleanActiveObjectPresetName || 'person_single');" not in s:
    s = s.replace(old, new, 1)
    changed = True
else:
    print("WARN: syncUi pin anchor not replaced or already patched")

# 5. When object preset selection stops PTZ, also update DOM controlMode.
old = """    settings.ptzArmed = false;
    settings.controlMode = 'manual';
"""

new = """    settings.ptzArmed = false;
    settings.controlMode = 'manual';

    if ($('controlMode')) {
      $('controlMode').value = 'manual';
    }
"""

if old in s:
    s = s.replace(old, new, 1)
    changed = True
else:
    print("WARN: selectObjectPreset controlMode anchor not found")

# 6. When START PTZ is pressed, update DOM controlMode too.
old = """    settings.ptzArmed = true;
    settings.controlMode = 'ptz';

    await writeSettings(settings);
"""

new = """    settings.ptzArmed = true;
    settings.controlMode = 'ptz';

    if ($('controlMode')) {
      $('controlMode').value = 'ptz';
    }

    await writeSettings(settings);
"""

if old in s:
    s = s.replace(old, new, 1)
    changed = True
else:
    print("WARN: startPtz controlMode anchor not found")

# 7. When STOP PTZ is pressed, update DOM controlMode too.
old = """    settings.ptzArmed = false;
    settings.controlMode = 'manual';

    await writeSettings(settings);
"""

new = """    settings.ptzArmed = false;
    settings.controlMode = 'manual';

    if ($('controlMode')) {
      $('controlMode').value = 'manual';
    }

    await writeSettings(settings);
"""

if old in s:
    s = s.replace(old, new, 1)
    changed = True
else:
    print("WARN: stopPtz controlMode anchor not found")

if changed:
    p.write_text(s, encoding="utf-8")
    print("OK patched web/index.html")
else:
    print("NO CHANGES")

print("Backup:", bak)
print("removed_autosave_blocks =", removed_autosave)
