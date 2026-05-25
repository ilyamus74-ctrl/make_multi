from pathlib import Path
import shutil
import time
import re

p = Path("web/index.html")
s = p.read_text(encoding="utf-8")

backup = p.with_suffix(f".html.bak_backend_samples_key_latch_{int(time.time())}")
shutil.copy2(p, backup)
print(f"backup: {backup}")

def replace_function(src, name, replacement):
    # supports "function name" and "async function name"
    starts = [f"async function {name}", f"function {name}"]
    start = -1
    for marker in starts:
        start = src.find(marker)
        if start >= 0:
            break
    if start < 0:
        raise SystemExit(f"ERROR: function {name} not found")

    brace = src.find("{", start)
    if brace < 0:
        raise SystemExit(f"ERROR: opening brace not found for {name}")

    depth = 0
    i = brace
    in_str = None
    esc = False

    while i < len(src):
        ch = src[i]

        if esc:
            esc = False
            i += 1
            continue

        if ch == "\\":
            esc = True
            i += 1
            continue

        if in_str:
            if ch == in_str:
                in_str = None
            i += 1
            continue

        if ch in ("'", '"', "`"):
            in_str = ch
            i += 1
            continue

        if ch == "{":
            depth += 1
        elif ch == "}":
            depth -= 1
            if depth == 0:
                end = i + 1
                return src[:start] + replacement.strip() + src[end:]

        i += 1

    raise SystemExit(f"ERROR: end of function {name} not found")


# --------------------------------------------------------------------
# 1. Replace ptzTuneGoToSample with backend-only path.
# --------------------------------------------------------------------

new_ptzTuneGoToSample = r'''
async function ptzTuneGoToSample(sampleIdx, opts = {}) {
  sampleIdx = Math.round(Number(sampleIdx || 0));

  const target = (ptzTuneZoomSamples || []).find(p => Number(p.profile_idx) === Number(sampleIdx));
  if (!target) {
    ptzLog('PTZ TUNE SAMPLE error', { error: 'sample_not_found', sampleIdx });
    return { ok: false, error: 'sample_not_found' };
  }

  if (ptzTuneZoomMoveBusy) {
    ptzLog('PTZ TUNE SAMPLE busy', { sampleIdx });
    return { ok: false, error: 'frontend_zoom_move_busy' };
  }

  ptzTuneZoomMoveBusy = true;
  setPtzTuneSampleButtonsDisabled?.(true);

  try {
    const res = await apiPostJson('/api/zoom/go_to_sample', {
      profile_idx: Number(sampleIdx),
      mode: opts.forceAbsolute ? 'absolute_wide' : 'relative'
    });

    if (!res || res.ok === false) {
      throw new Error(res?.error || 'backend_go_to_sample_failed');
    }

    ptzTuneCurrentSample = Number(res.to_sample ?? res.target_sample ?? sampleIdx);

    // Sync exact backend state after movement.
    let zs = null;
    try {
      zs = await apiGetJson('/api/zoom/state');
      const idx = Number(zs.zoom_sample_idx);
      if (Number.isFinite(idx)) ptzTuneCurrentSample = idx;
    } catch (_) {}

    await ptzTuneLoadSpeedForSample(ptzTuneCurrentSample).catch(() => {});
    await refreshPtzTelemetry?.().catch(() => {});
    updatePtzTuneSampleButtonUi?.();

    if (zs) {
      tuneStatusSet?.(Number(zs.zoom_ratio || res.zoom_ratio || 0), Number(zs.focal_px || res.focal_px || 0));
    } else {
      tuneStatusSet?.(Number(res.zoom_ratio || 0), Number(res.focal_px || 0));
    }

    ptzLog('PTZ TUNE SAMPLE BACKEND', res);
    return res;
  } catch (e) {
    ptzLog('PTZ TUNE SAMPLE backend error', {
      sampleIdx,
      error: String(e.message || e)
    });
    return { ok: false, error: String(e.message || e) };
  } finally {
    ptzTuneZoomMoveBusy = false;
    setPtzTuneSampleButtonsDisabled?.(false);
  }
}
'''

s = replace_function(s, "ptzTuneGoToSample", new_ptzTuneGoToSample)

# --------------------------------------------------------------------
# 2. Replace keyboard zoom block with key-latched Q/W.
# --------------------------------------------------------------------

block_pattern = re.compile(
    r"\n\s*let keyboardZoomBusy = false;.*?// SAMPLE_INIT_HOTKEYS_REPAIR_END",
    re.S
)

new_keyboard_block = r'''
  let keyboardZoomBusy = false;
  let keyboardZoomLastTs = 0;
  const KEYBOARD_ZOOM_MIN_INTERVAL_MS = 500;
  const keyboardZoomPressedKeys = new Set();

  async function getCurrentZoomSampleSafe() {
    try {
      const zs = await apiGetJson('/api/zoom/state');
      const idx = Number(zs.zoom_sample_idx);
      if (Number.isFinite(idx)) {
        ptzTuneCurrentSample = idx;
        return idx;
      }
    } catch (_) {}
    return Number(ptzTuneCurrentSample || 0);
  }

  async function keyboardZoomPulse(direction, reason) {
    const now = Date.now();

    if (keyboardZoomBusy) {
      return { ok: false, skipped: true, reason: 'frontend_busy' };
    }

    if (now - keyboardZoomLastTs < KEYBOARD_ZOOM_MIN_INTERVAL_MS) {
      return { ok: false, skipped: true, reason: 'frontend_throttle' };
    }

    const currentSample = await getCurrentZoomSampleSafe();
    const maxSample = typeof ptzTuneMaxSampleIdx === 'function' ? ptzTuneMaxSampleIdx() : currentSample;

    // Do not physically twitch servo when already at mechanical edge.
    if (direction === 'wide' && currentSample <= 0) {
      ptzLog('KEYBOARD ZOOM skipped edge', { direction, sample: currentSample });
      return { ok: false, skipped: true, reason: 'edge_wide' };
    }

    if (direction === 'tele' && currentSample >= maxSample) {
      ptzLog('KEYBOARD ZOOM skipped edge', { direction, sample: currentSample, maxSample });
      return { ok: false, skipped: true, reason: 'edge_tele' };
    }

    keyboardZoomBusy = true;
    keyboardZoomLastTs = now;

    try {
      const hold = Number($('zoomImpulseMs')?.value || 170);
      const res = await apiPostJson('/api/zoom/pulse', {
        direction,
        steps: 1,
        hold_ms: hold,
        source: reason || 'keyboard'
      });

      ptzLog('KEYBOARD ZOOM BACKEND', res);

      try {
        const zs = await apiGetJson('/api/zoom/state');
        const idx = Number(zs.zoom_sample_idx);
        if (Number.isFinite(idx)) ptzTuneCurrentSample = idx;
        tuneStatusSet?.(Number(zs.zoom_ratio || 0), Number(zs.focal_px || 0));
      } catch (_) {}

      await refreshPtzTelemetry?.().catch(() => {});
      updatePtzTuneSampleButtonUi?.();

      return res;
    } catch (e) {
      ptzLog('KEYBOARD ZOOM error', {
        direction,
        reason,
        error: String(e.message || e)
      });
      return { ok: false, error: String(e.message || e) };
    } finally {
      keyboardZoomBusy = false;
    }
  }

  if (!window.__ptzZoomHotkeysInstalled) {
    window.__ptzZoomHotkeysInstalled = true;

    window.addEventListener('keydown', (ev) => {
      if (isTypingTarget(ev.target)) return;

      const k = String(ev.key || '').toLowerCase();
      if (k !== 'q' && k !== 'w') return;

      ev.preventDefault();

      // Robust key latch. Some browsers/remote desktops can generate repeated
      // keydown events without reliable ev.repeat behavior.
      if (keyboardZoomPressedKeys.has(k)) return;
      keyboardZoomPressedKeys.add(k);

      if (k === 'q') {
        keyboardZoomPulse('wide', 'Q_WIDE').catch(e =>
          ptzLog('KEYBOARD ZOOM error', { key: 'q', error: String(e.message || e) })
        );
      } else if (k === 'w') {
        keyboardZoomPulse('tele', 'W_TELE').catch(e =>
          ptzLog('KEYBOARD ZOOM error', { key: 'w', error: String(e.message || e) })
        );
      }
    }, true);

    window.addEventListener('keyup', (ev) => {
      const k = String(ev.key || '').toLowerCase();
      if (k === 'q' || k === 'w') {
        keyboardZoomPressedKeys.delete(k);
      }
    }, true);

    window.addEventListener('blur', () => {
      keyboardZoomPressedKeys.clear();
    });
  }

  // SAMPLE_INIT_HOTKEYS_REPAIR_END
'''

s2, n = block_pattern.subn("\n" + new_keyboard_block, s, count=1)
if n != 1:
    raise SystemExit(f"ERROR: keyboard zoom block not replaced, replacements={n}")

p.write_text(s2, encoding="utf-8")
print("OK: sample buttons now backend-only; Q/W key-latched")
