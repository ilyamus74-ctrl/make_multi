from pathlib import Path
import time

p = Path("web/index.html")
s = p.read_text(encoding="utf-8", errors="ignore")

bak = p.with_suffix(p.suffix + f".bak_object_preset_buttons_active_{int(time.time())}")
bak.write_text(s, encoding="utf-8")

script = r'''
<script>
// OBJECT_PRESET_BUTTONS_ACTIVE_FIX_START
(function () {
  const $ = (id) => document.getElementById(id);

  function ensurePresetActiveStyle() {
    if ($('objectPresetButtonsActiveStyle')) return;

    const st = document.createElement('style');
    st.id = 'objectPresetButtonsActiveStyle';
    st.textContent = `
      [data-object-preset].object-preset-active,
      [data-object-preset].active {
        outline: 2px solid #00ff66 !important;
        background: rgba(0, 255, 102, 0.24) !important;
        color: #ffffff !important;
        box-shadow: 0 0 10px rgba(0, 255, 102, 0.50) !important;
        border-color: #00ff66 !important;
      }

      #objectPresetState.object-preset-active-state {
        color: #00ff66 !important;
        background: rgba(0, 255, 102, 0.16) !important;
        border: 1px solid rgba(0, 255, 102, 0.45) !important;
      }
    `;
    document.head.appendChild(st);
  }

  async function getSettings() {
    try {
      const r = await fetch('/api/settings', { cache: 'no-store' });
      if (!r.ok) throw new Error(String(r.status));
      return await r.json();
    } catch (_) {
      return {};
    }
  }

  async function saveSettingsPatch(patch) {
    let old = {};

    try {
      old = await getSettings();
    } catch (_) {
      old = {};
    }

    const next = Object.assign({}, old || {}, patch || {});
    next.config_version = Math.max(16, Number(next.config_version || 16));

    try {
      localStorage.setItem('pantilt_ui_settings_v2', JSON.stringify(next));
    } catch (_) {}

    try {
      await fetch('/api/settings', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify(next)
      });
    } catch (_) {}

    return next;
  }

  function resolveActivePreset(settings) {
    const last = settings && settings.lastAppliedObjectPreset && settings.lastAppliedObjectPreset.name;
    if (last) return String(last);

    const active = settings && settings.activeObjectPreset;
    if (active) return String(active);

    const chip = $('objectPresetState');
    const text = chip ? String(chip.textContent || '') : '';
    const m = text.match(/preset:\s*([a-zA-Z0-9_-]+)/);
    if (m) return m[1];

    return 'person_single';
  }

  function paintActivePreset(name) {
    name = String(name || 'person_single');

    document.querySelectorAll('[data-object-preset]').forEach(btn => {
      const on = btn.dataset.objectPreset === name;

      btn.classList.toggle('object-preset-active', on);
      btn.classList.toggle('active', on);

      if (on) {
        btn.setAttribute('aria-pressed', 'true');
      } else {
        btn.setAttribute('aria-pressed', 'false');
      }
    });

    const chip = $('objectPresetState');
    if (chip) {
      chip.textContent = `preset: ${name}`;
      chip.classList.add('object-preset-active-state');
    }
  }

  async function syncActivePresetButtons() {
    ensurePresetActiveStyle();

    const settings = await getSettings();
    const name = resolveActivePreset(settings);

    paintActivePreset(name);

    if (settings.activeObjectPreset !== name) {
      await saveSettingsPatch({
        activeObjectPreset: name
      });
    }

    return name;
  }

  function bindPresetClicksForHighlight() {
    document.querySelectorAll('[data-object-preset]').forEach(btn => {
      if (btn.__activePresetHighlightBound) return;
      btn.__activePresetHighlightBound = true;

      btn.addEventListener('click', () => {
        const name = btn.dataset.objectPreset;
        paintActivePreset(name);

        saveSettingsPatch({
          activeObjectPreset: name,
          lastAppliedObjectPreset: {
            name,
            ts: Math.floor(Date.now() / 1000)
          }
        }).catch(() => {});

        setTimeout(syncActivePresetButtons, 500);
        setTimeout(syncActivePresetButtons, 1500);
        setTimeout(syncActivePresetButtons, 3000);
      }, true);
    });
  }

  async function bootPresetHighlight() {
    ensurePresetActiveStyle();
    bindPresetClicksForHighlight();
    await syncActivePresetButtons();
  }

  if (document.readyState === 'loading') {
    document.addEventListener('DOMContentLoaded', bootPresetHighlight, { once: true });
  } else {
    bootPresetHighlight();
  }

  setTimeout(bootPresetHighlight, 300);
  setTimeout(bootPresetHighlight, 1000);
  setTimeout(bootPresetHighlight, 2500);
  setInterval(syncActivePresetButtons, 5000);

  window.syncActivePresetButtons = syncActivePresetButtons;
  window.paintActivePreset = paintActivePreset;
})();
// OBJECT_PRESET_BUTTONS_ACTIVE_FIX_END
</script>
'''

if "OBJECT_PRESET_BUTTONS_ACTIVE_FIX_START" not in s:
    if "</body>" not in s:
        raise SystemExit("ERROR: </body> not found")
    s = s.replace("</body>", script + "\n</body>", 1)
    p.write_text(s, encoding="utf-8")
    print("OK patched web/index.html")
    print("Backup:", bak)
else:
    print("OK already patched")
