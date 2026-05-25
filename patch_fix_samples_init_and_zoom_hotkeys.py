from pathlib import Path
import shutil
import re
import time

p = Path("web/index.html")
s = p.read_text(encoding="utf-8")

backup = p.with_suffix(f".html.bak_fix_samples_init_hotkeys_{int(time.time())}")
shutil.copy2(p, backup)
print(f"backup: {backup}")

insert_marker = "  // PTZ speed tune sample init must never block video stream startup."
idx = s.find(insert_marker)
if idx < 0:
    raise SystemExit("ERROR: init marker not found")

# Remove old injected block if any.
s = re.sub(
    r"\n\s*// SAMPLE_INIT_HOTKEYS_REPAIR_START.*?// SAMPLE_INIT_HOTKEYS_REPAIR_END\n",
    "\n",
    s,
    flags=re.S
)

repair = r'''
  // SAMPLE_INIT_HOTKEYS_REPAIR_START

  async function refreshPtzTuneSpeedProfile() {
    try {
      const data = await apiGetJson(`${autopilotBaseUrl()}/api/autopilot/speed_profile`);
      ptzTuneSpeedProfile = data || { points: [] };
      ptzTuneSavedPoints = Array.isArray(data?.points) ? data.points.length : 0;
      updatePtzTuneSampleButtonUi();
      updateOperatorMenuSummaries?.();
      return ptzTuneSpeedProfile;
    } catch (e) {
      ptzTuneSpeedProfile = { points: [] };
      ptzTuneSavedPoints = 0;
      ptzLog('PTZ SPEED PROFILE load error', { error: String(e.message || e) });
      return ptzTuneSpeedProfile;
    }
  }

  function makeFallbackZoomSamples(count) {
    count = Math.max(2, Math.min(64, Number(count || 13)));
    const maxIdx = count - 1;
    const out = [];
    for (let i = 0; i < count; i++) {
      const ratio = maxIdx > 0 ? i / maxIdx : 0;
      out.push({
        profile_idx: i,
        zoom_ratio: ratio,
        focal_px: 0,
        fallback: true
      });
    }
    return out;
  }

  async function initPtzSpeedTuneSamples() {
    let points = [];

    try {
      const mp = await apiGetJson('/api/zoom/master_profile');
      if (Array.isArray(mp?.profile_points)) {
        points = mp.profile_points
          .map((p, i) => ({
            profile_idx: Number(p.profile_idx ?? i),
            zoom_ratio: Number(p.zoom_ratio ?? 0),
            focal_px: Number(p.focal_px ?? 0),
            fallback: false
          }))
          .filter(p => Number.isFinite(p.profile_idx))
          .sort((a, b) => Number(a.profile_idx) - Number(b.profile_idx));
      }
    } catch (e) {
      ptzLog('PTZ ZOOM SAMPLES master profile load error', { error: String(e.message || e) });
    }

    // Fallback if master profile is missing/broken.
    if (!points.length) {
      let count = 13;

      try {
        const zs = await apiGetJson('/api/zoom/state');
        count = Number(zs.zoom_sample_count || 0) || count;
      } catch (_) {}

      if ($('zoomSamples')) {
        const uiSamples = Number($('zoomSamples').value || 0);
        if (uiSamples > 0) count = uiSamples + 1;
      }

      points = makeFallbackZoomSamples(count);
      ptzLog('PTZ ZOOM SAMPLES fallback generated', { count: points.length });
    }

    ptzTuneZoomSamples = points;
    renderPtzTuneSampleButtons(points);

    await refreshPtzTuneSpeedProfile();

    try {
      const zs = await apiGetJson('/api/zoom/state');
      const idx = Number(zs.zoom_sample_idx);
      if (Number.isFinite(idx)) {
        ptzTuneCurrentSample = Math.max(0, Math.min(ptzTuneMaxSampleIdx(), idx));
      } else {
        ptzTuneCurrentSample = 0;
      }
      tuneStatusSet?.(Number(zs.zoom_ratio || 0), Number(zs.focal_px || 0));
    } catch (_) {
      ptzTuneCurrentSample = 0;
      tuneStatusSet?.(0, 0);
    }

    updatePtzTuneSampleButtonUi();

    // Try loading saved speed for active sample, but do not fail page init.
    ptzTuneLoadSpeedForSample(ptzTuneCurrentSample).catch(e =>
      ptzLog('PTZ SPEED sample load error', {
        sample: ptzTuneCurrentSample,
        error: String(e.message || e)
      })
    );

    ptzLog('PTZ ZOOM SAMPLES ready', {
      samples: points.length,
      current: ptzTuneCurrentSample
    });

    return points;
  }

  function isTypingTarget(el) {
    if (!el) return false;
    const tag = String(el.tagName || '').toLowerCase();
    return tag === 'input' || tag === 'textarea' || tag === 'select' || Boolean(el.isContentEditable);
  }

  async function keyboardZoomPulse(direction, reason) {
    const cmd = direction > 0 ? getTeleCmd() : getWideCmd();
    const hold = Number($('zoomImpulseMs')?.value || 170);
    await zoomTest(cmd, hold);
    ptzLog('KEYBOARD ZOOM', { reason, cmd, hold });
  }

  if (!window.__ptzZoomHotkeysInstalled) {
    window.__ptzZoomHotkeysInstalled = true;

    window.addEventListener('keydown', (ev) => {
      if (isTypingTarget(ev.target)) return;
      if (ev.repeat) return;

      const k = String(ev.key || '').toLowerCase();

      // Q/W hotkeys requested for quick zoom testing.
      // Q = WIDE impulse, W = TELE impulse.
      if (k === 'q') {
        ev.preventDefault();
        keyboardZoomPulse(-1, 'Q_WIDE').catch(e =>
          ptzLog('KEYBOARD ZOOM error', { key: 'q', error: String(e.message || e) })
        );
      } else if (k === 'w') {
        ev.preventDefault();
        keyboardZoomPulse(1, 'W_TELE').catch(e =>
          ptzLog('KEYBOARD ZOOM error', { key: 'w', error: String(e.message || e) })
        );
      }
    }, true);
  }

  // SAMPLE_INIT_HOTKEYS_REPAIR_END

'''

s = s[:idx] + repair + "\n" + s[idx:]

p.write_text(s, encoding="utf-8")
print("OK: inserted sample init + Q/W hotkeys repair")
