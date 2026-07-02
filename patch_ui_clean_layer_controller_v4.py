from pathlib import Path
import time

p = Path("web/index.html")
s = p.read_text(encoding="utf-8", errors="ignore")

bak = p.with_suffix(p.suffix + f".bak_clean_layer_controller_v4_{int(time.time())}")
bak.write_text(s, encoding="utf-8")

changed = False
removed_blocks = 0

def remove_operator_field_change_once(text):
    marker = "operator_field_change"
    idx = text.find(marker)

    if idx < 0:
        return text, False

    start1 = text.rfind("document.addEventListener('change'", 0, idx)
    start2 = text.rfind('document.addEventListener("change"', 0, idx)
    start = max(start1, start2)

    if start < 0:
        return text.replace(marker, "operator_autosave_removed", 1), True

    end = text.find("}, true);", idx)

    if end < 0:
        return text.replace(marker, "operator_autosave_removed", 1), True

    end = end + len("}, true);")

    return text[:start] + "\n  /* removed stale operator autosave by clean layer v4 */\n" + text[end:], True

for _ in range(20):
    s2, did = remove_operator_field_change_once(s)

    if not did:
        break

    s = s2
    removed_blocks += 1
    changed = True

# Важное: не оставляем старый marker в файле вообще.
s = s.replace("operator_field_change", "operator_autosave_removed")

# Pin active object preset inside clean controller.
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

# Keep hidden/select controlMode DOM in sync so old base code cannot save stale "ptz".
old = """    settings.ptzArmed = false;
    settings.controlMode = 'manual';

    settings.detectorEnabled = true;
"""

new = """    settings.ptzArmed = false;
    settings.controlMode = 'manual';

    if ($('controlMode')) {
      $('controlMode').value = 'manual';
    }

    settings.detectorEnabled = true;
"""

if old in s:
    s = s.replace(old, new, 1)
    changed = True

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

# Hard guard: do not allow old global saveSettings to overwrite activeObjectPreset/ptzArmed/controlMode
# from stale DOM/localStorage after clean layer is active.
guard = r'''
<script>
// PTZ_CLEAN_SAVESETTINGS_GUARD_START
(function () {
  const $ = (id) => document.getElementById(id);

  async function getJson(url) {
    const r = await fetch(url, { cache: 'no-store' });
    if (!r.ok) throw new Error(`${r.status} ${url}`);
    return await r.json();
  }

  async function postJson(url, body) {
    const r = await fetch(url, {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify(body || {})
    });

    if (!r.ok) throw new Error(`${r.status} ${url}`);
    return await r.json().catch(() => ({ ok: true }));
  }

  async function readSettings() {
    try {
      const data = await getJson('/api/settings');
      return data && typeof data === 'object' ? data : {};
    } catch (_) {
      return {};
    }
  }

  async function writeSettings(settings) {
    const next = Object.assign({}, settings || {});
    next.config_version = Math.max(16, Number(next.config_version || 16));
    await postJson('/api/settings', next);
    return next;
  }

  function installGuard() {
    if (window.__ptzCleanSaveSettingsGuardInstalled) return;
    window.__ptzCleanSaveSettingsGuardInstalled = true;

    const original = window.saveSettings;

    if (typeof original !== 'function') return;

    window.saveSettings = async function (...args) {
      const before = await readSettings();

      const keep = {
        activeObjectPreset: before.activeObjectPreset,
        lastAppliedObjectPreset: before.lastAppliedObjectPreset,
        objectPresetsCustom: before.objectPresetsCustom,
        activeSearchPreset: before.activeSearchPreset,
        lastAppliedSearchPreset: before.lastAppliedSearchPreset,
        ptzArmed: before.ptzArmed,
        controlMode: before.controlMode,
        objectPresetTrackingMode: before.objectPresetTrackingMode,
        objectPresetLossBehavior: before.objectPresetLossBehavior
      };

      let result = await original.apply(this, args);

      const after = await readSettings();

      let changed = false;

      for (const [key, val] of Object.entries(keep)) {
        if (val !== undefined && JSON.stringify(after[key]) !== JSON.stringify(val)) {
          after[key] = val;
          changed = true;
        }
      }

      if (changed) {
        await writeSettings(after);
      }

      return result;
    };
  }

  if (document.readyState === 'loading') {
    document.addEventListener('DOMContentLoaded', installGuard, { once: true });
  } else {
    installGuard();
  }

  setTimeout(installGuard, 500);
  setTimeout(installGuard, 1500);
})();
// PTZ_CLEAN_SAVESETTINGS_GUARD_END
</script>
'''

if "PTZ_CLEAN_SAVESETTINGS_GUARD_START" not in s:
    if "</body>" not in s:
        raise SystemExit("ERROR: </body> not found")
    s = s.replace("</body>", guard + "\n</body>", 1)
    changed = True

if changed:
    p.write_text(s, encoding="utf-8")
    print("OK patched web/index.html")
else:
    print("NO CHANGES")

print("Backup:", bak)
print("removed_operator_autosave_blocks =", removed_blocks)
