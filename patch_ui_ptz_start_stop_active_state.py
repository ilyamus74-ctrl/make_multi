from pathlib import Path
import time

p = Path("web/index.html")
s = p.read_text(encoding="utf-8", errors="ignore")

bak = p.with_suffix(p.suffix + f".bak_ptz_start_stop_active_state_{int(time.time())}")
bak.write_text(s, encoding="utf-8")

script = r'''
<script>
// PTZ_START_STOP_ACTIVE_STATE_FIX_START
(function () {
  const $ = (id) => document.getElementById(id);

  function autopilotBaseUrlHard() {
    return `${location.protocol}//${location.hostname}:8090`;
  }

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

  async function readSettingsHard() {
    try {
      return await getJson('/api/settings');
    } catch (_) {
      return {};
    }
  }

  async function saveSettingsPatchHard(patch) {
    const old = await readSettingsHard();
    const next = Object.assign({}, old || {}, patch || {});
    next.config_version = Math.max(16, Number(next.config_version || 16));

    try {
      localStorage.setItem('pantilt_ui_settings_v2', JSON.stringify(next));
    } catch (_) {}

    try {
      await postJson('/api/settings', next);
    } catch (_) {}

    return next;
  }

  function ensurePtzRunStyle() {
    if ($('ptzStartStopActiveStyle')) return;

    const st = document.createElement('style');
    st.id = 'ptzStartStopActiveStyle';
    st.textContent = `
      #ptzStartBtn.ptz-active-running {
        outline: 2px solid #00ff66 !important;
        background: rgba(0, 255, 102, 0.25) !important;
        border-color: #00ff66 !important;
        color: #ffffff !important;
        box-shadow: 0 0 10px rgba(0, 255, 102, 0.55) !important;
      }

      #ptzStartBtn.ptz-active-armed {
        outline: 2px solid #ffcc00 !important;
        background: rgba(255, 204, 0, 0.20) !important;
        border-color: #ffcc00 !important;
        color: #ffffff !important;
      }

      #ptzStopBtn.ptz-active-running {
        outline: 2px solid #ff4444 !important;
        background: rgba(255, 68, 68, 0.24) !important;
        border-color: #ff4444 !important;
        color: #ffffff !important;
        box-shadow: 0 0 10px rgba(255, 68, 68, 0.45) !important;
      }

      #ptzStopBtn.ptz-stopped {
        outline: none !important;
        box-shadow: none !important;
        opacity: 0.75;
      }

      #ptzRunStateChip {
        margin-left: 6px;
      }

      #ptzRunStateChip.ptz-running-chip {
        color: #00ff66 !important;
        background: rgba(0, 255, 102, 0.14) !important;
        border: 1px solid rgba(0, 255, 102, 0.45) !important;
      }

      #ptzRunStateChip.ptz-armed-chip {
        color: #ffcc00 !important;
        background: rgba(255, 204, 0, 0.14) !important;
        border: 1px solid rgba(255, 204, 0, 0.45) !important;
      }

      #ptzRunStateChip.ptz-stopped-chip {
        color: #ddd !important;
        background: rgba(255, 255, 255, 0.08) !important;
        border: 1px solid rgba(255, 255, 255, 0.18) !important;
      }
    `;
    document.head.appendChild(st);
  }

  function ensurePtzRunChip() {
    if ($('ptzRunStateChip')) return;

    const start = $('ptzStartBtn');
    if (!start || !start.parentNode) return;

    const chip = document.createElement('code');
    chip.id = 'ptzRunStateChip';
    chip.textContent = 'ptz: --';

    start.parentNode.insertBefore(chip, start.nextSibling);
  }

  function setBtnText(el, text) {
    if (!el) return;
    if (!el.dataset.originalText) {
      el.dataset.originalText = el.textContent || '';
    }
    el.textContent = text;
  }

  function clearPtzClasses() {
    const start = $('ptzStartBtn');
    const stop = $('ptzStopBtn');
    const chip = $('ptzRunStateChip');

    for (const el of [start, stop, chip]) {
      if (!el) continue;
      el.classList.remove(
        'ptz-run-active',
        'ptz-stop-active',
        'ptz-active-running',
        'ptz-active-armed',
        'ptz-stopped',
        'ptz-running-chip',
        'ptz-armed-chip',
        'ptz-stopped-chip'
      );
    }
  }

  function paintPtzRunState(ap, settings) {
    ensurePtzRunStyle();
    ensurePtzRunChip();

    const start = $('ptzStartBtn');
    const stop = $('ptzStopBtn');
    const chip = $('ptzRunStateChip');

    const running = Boolean(ap && ap.enabled === true);
    const armed = Boolean(settings && settings.ptzArmed === true);

    clearPtzClasses();

    if (running) {
      if (start) {
        start.classList.add('ptz-active-running');
        setBtnText(start, 'PTZ ACTIVE');
      }

      if (stop) {
        stop.classList.add('ptz-active-running');
        setBtnText(stop, 'STOP PTZ');
      }

      if (chip) {
        chip.classList.add('ptz-running-chip');
        chip.textContent = `ptz: ACTIVE ${ap.mode || ''}`;
      }

      if ($('controlMode')) $('controlMode').value = 'ptz';
      if (typeof safeSyncControlModeUi === 'function') safeSyncControlModeUi();

      return;
    }

    if (armed) {
      if (start) {
        start.classList.add('ptz-active-armed');
        setBtnText(start, 'PTZ ARMED');
      }

      if (stop) {
        stop.classList.add('ptz-stopped');
        setBtnText(stop, 'STOP PTZ');
      }

      if (chip) {
        chip.classList.add('ptz-armed-chip');
        chip.textContent = 'ptz: ARMED / SEARCH';
      }

      return;
    }

    if (start) {
      setBtnText(start, 'START PTZ');
    }

    if (stop) {
      stop.classList.add('ptz-stopped');
      setBtnText(stop, 'STOP PTZ');
    }

    if (chip) {
      chip.classList.add('ptz-stopped-chip');
      chip.textContent = 'ptz: STOPPED';
    }
  }

  async function syncPtzRunStateHard() {
    ensurePtzRunStyle();
    ensurePtzRunChip();

    let ap = {};
    let settings = {};

    try {
      ap = await getJson(`${autopilotBaseUrlHard()}/api/autopilot/state`);
    } catch (_) {
      ap = {};
    }

    settings = await readSettingsHard();

    paintPtzRunState(ap, settings);

    // Backend is authoritative: if autopilot is actually running, persist ptzArmed=true.
    if (ap && ap.enabled === true && settings.ptzArmed !== true) {
      await saveSettingsPatchHard({
        ptzArmed: true,
        controlMode: 'ptz'
      });
    }

    return { ap, settings };
  }

  function bindStartStopPersistHard() {
    const start = $('ptzStartBtn');
    const stop = $('ptzStopBtn');

    if (start && !start.__ptzRunStateHardBound) {
      start.__ptzRunStateHardBound = true;

      start.addEventListener('click', () => {
        saveSettingsPatchHard({
          ptzArmed: true,
          controlMode: 'ptz'
        }).catch(() => {});

        setTimeout(syncPtzRunStateHard, 400);
        setTimeout(syncPtzRunStateHard, 1200);
        setTimeout(syncPtzRunStateHard, 2500);
      }, true);
    }

    if (stop && !stop.__ptzRunStateHardBound) {
      stop.__ptzRunStateHardBound = true;

      stop.addEventListener('click', () => {
        saveSettingsPatchHard({
          ptzArmed: false
        }).catch(() => {});

        setTimeout(syncPtzRunStateHard, 400);
        setTimeout(syncPtzRunStateHard, 1200);
        setTimeout(syncPtzRunStateHard, 2500);
      }, true);
    }
  }

  async function bootPtzRunStateHard() {
    ensurePtzRunStyle();
    ensurePtzRunChip();
    bindStartStopPersistHard();
    await syncPtzRunStateHard();
  }

  if (document.readyState === 'loading') {
    document.addEventListener('DOMContentLoaded', bootPtzRunStateHard, { once: true });
  } else {
    bootPtzRunStateHard();
  }

  setTimeout(bootPtzRunStateHard, 300);
  setTimeout(bootPtzRunStateHard, 1000);
  setTimeout(bootPtzRunStateHard, 2500);
  setInterval(syncPtzRunStateHard, 1000);

  window.syncPtzRunStateHard = syncPtzRunStateHard;
})();
// PTZ_START_STOP_ACTIVE_STATE_FIX_END
</script>
'''

if "PTZ_START_STOP_ACTIVE_STATE_FIX_START" not in s:
    if "</body>" not in s:
        raise SystemExit("ERROR: </body> not found")
    s = s.replace("</body>", script + "\n</body>", 1)
    p.write_text(s, encoding="utf-8")
    print("OK patched web/index.html")
    print("Backup:", bak)
else:
    print("OK already patched")
