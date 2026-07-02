from pathlib import Path
import time

p = Path("web/index.html")
s = p.read_text(encoding="utf-8", errors="ignore")

bak = p.with_suffix(p.suffix + f".bak_search_preset_selector_{int(time.time())}")
bak.write_text(s, encoding="utf-8")

script = r'''
<script>
// SEARCH_PRESET_SELECTOR_START
(function () {
  const $ = (id) => document.getElementById(id);

  const SEARCH_PRESETS = {
    lost_step_wait: 'Lost: step wide, sweep, wait',
    lost_wide_cycle: 'Lost: wide 3 frames cycle',
    lost_nested_loop: 'Lost: short + preset1 + pause'
  };

  async function getSettings() {
    try {
      const r = await fetch('/api/settings', { cache: 'no-store' });
      if (!r.ok) throw new Error(String(r.status));
      return await r.json();
    } catch (_) {
      return {};
    }
  }

  async function postSettingsPatch(patch) {
    const old = await getSettings();
    const next = Object.assign({}, old || {}, patch || {});
    next.config_version = Math.max(16, Number(next.config_version || 16));

    try {
      localStorage.setItem('pantilt_ui_settings_v2', JSON.stringify(next));
    } catch (_) {}

    const r = await fetch('/api/settings', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify(next)
    });

    if (!r.ok) throw new Error(String(r.status));

    return next;
  }

  function ensureSearchPresetUi() {
    if ($('searchPresetPanel')) return;

    const objectPanel = $('objectPresetPanel');
    if (!objectPanel || !objectPanel.parentNode) return;

    const div = document.createElement('div');
    div.id = 'searchPresetPanel';
    div.style.marginTop = '6px';
    div.style.display = 'flex';
    div.style.gap = '6px';
    div.style.alignItems = 'center';
    div.style.flexWrap = 'wrap';

    const label = document.createElement('span');
    label.textContent = 'SEARCH PRESET';
    label.style.opacity = '0.8';

    const sel = document.createElement('select');
    sel.id = 'searchPresetSelect';

    for (const [id, name] of Object.entries(SEARCH_PRESETS)) {
      const opt = document.createElement('option');
      opt.value = id;
      opt.textContent = name;
      sel.appendChild(opt);
    }

    const status = document.createElement('code');
    status.id = 'searchPresetStatus';
    status.textContent = 'search: --';

    sel.addEventListener('change', async () => {
      const id = sel.value;

      await postSettingsPatch({
        activeSearchPreset: id,
        lastAppliedSearchPreset: {
          name: id,
          label: SEARCH_PRESETS[id] || id,
          ts: Math.floor(Date.now() / 1000)
        }
      });

      status.textContent = 'search: ' + id;

      if ($('ptzStateLine')) {
        $('ptzStateLine').textContent = 'search preset saved: ' + id;
      }
    });

    div.appendChild(label);
    div.appendChild(sel);
    div.appendChild(status);

    objectPanel.parentNode.insertBefore(div, objectPanel.nextSibling);
  }

  async function syncSearchPresetUi() {
    ensureSearchPresetUi();

    const sel = $('searchPresetSelect');
    const status = $('searchPresetStatus');

    if (!sel) return;

    const settings = await getSettings();
    const active = settings.activeSearchPreset || 'lost_step_wait';

    if (SEARCH_PRESETS[active]) {
      sel.value = active;
    }

    if (status) {
      status.textContent = 'search: ' + active;
    }
  }

  function boot() {
    ensureSearchPresetUi();
    syncSearchPresetUi().catch(() => {});
  }

  if (document.readyState === 'loading') {
    document.addEventListener('DOMContentLoaded', boot, { once: true });
  } else {
    boot();
  }

  setTimeout(boot, 500);
  setTimeout(boot, 1500);
  setTimeout(boot, 3000);

  window.syncSearchPresetUi = syncSearchPresetUi;
})();
// SEARCH_PRESET_SELECTOR_END
</script>
'''

if "SEARCH_PRESET_SELECTOR_START" not in s:
    if "</body>" not in s:
        raise SystemExit("ERROR: </body> not found")
    s = s.replace("</body>", script + "\n</body>", 1)
    p.write_text(s, encoding="utf-8")
    print("OK patched web/index.html")
    print("Backup:", bak)
else:
    print("OK already patched")
