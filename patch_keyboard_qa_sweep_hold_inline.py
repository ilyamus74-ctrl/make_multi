from pathlib import Path
import shutil, time

p = Path("web/index.html")
s = p.read_text(encoding="utf-8")

backup = p.with_suffix(f".html.bak_keyboard_qa_sweep_hold_inline_{int(time.time())}")
shutil.copy2(p, backup)
print("Backup:", backup)

changed = False

insert_after = """  async function keyboardZoomSampleStep(delta, source) {
    try {
      const zs = await apiGetJson('/api/zoom/state');
      const cur = Number(zs.zoom_sample_idx || 0);
      const count = Number(zs.zoom_sample_count || 0);
      const maxIdx = count > 0 ? count - 1 : cur;
      const target = Math.max(0, Math.min(maxIdx, cur + delta));

      if (target === cur) {
        ptzLog('KEYBOARD ZOOM SAMPLE skipped edge', { source, cur, target });
        return;
      }

      const res = await apiPostJson('/api/zoom/go_to_sample', {
        profile_idx: target,
        mode: 'relative'
      });

      ptzLog('KEYBOARD ZOOM SAMPLE', { source, from: cur, to: target, res });

      if (typeof ptzTuneCurrentSample !== 'undefined') {
        ptzTuneCurrentSample = target;
      }

      try {
        await ptzTuneLoadSpeedForSample?.(target);
      } catch (_) {}

      updatePtzTuneSampleButtonUi?.();
    } catch (e) {
      ptzLog('KEYBOARD ZOOM SAMPLE error', { source, error: String(e.message || e) });
    }
  }


"""

helper = """  // Q/A in SWEEP TIME STEPS: real hold zoom.
  let keyboardQaSweepHold = null;

  function keyboardQaIsSweepMode() {
    return ($('zoomMoveMode')?.value || '') === 'sweep_time_steps';
  }

  function keyboardQaSamples() {
    return Math.max(2, Math.min(100, Number($('zoomSamples')?.value || 10)));
  }

  function keyboardQaFullSweepMs() {
    return Math.max(100, Number($('zoomFullSweepMs')?.value || $('zoomWideHoldMs')?.value || 1500));
  }

  function keyboardQaStepMs() {
    return keyboardQaFullSweepMs() / Math.max(1, keyboardQaSamples() - 1);
  }

  function keyboardQaWideCmd() {
    const cmdAbs = Math.max(1, Number($('zoomCmdAbs')?.value || 34));
    const sign = Number($('zoomWideSign')?.value || 1) < 0 ? -1 : 1;
    return sign * cmdAbs;
  }

  function keyboardQaTeleCmd() {
    return -keyboardQaWideCmd();
  }

  async function keyboardQaSweepStart(isQ, source) {
    if (keyboardQaSweepHold) return;

    let cur = 0;
    let count = keyboardQaSamples();

    try {
      const zs = await apiGetJson('/api/zoom/state');
      cur = Number(zs.zoom_sample_idx || 0);
      count = Number(zs.zoom_sample_count || count) || count;
    } catch (_) {}

    const direction = isQ ? 1 : -1;
    const cmd = isQ ? keyboardQaTeleCmd() : keyboardQaWideCmd();

    keyboardQaSweepHold = {
      source,
      direction,
      cmd,
      startTs: performance.now(),
      startIdx: Math.max(0, Math.min(Math.max(1, count) - 1, cur)),
      count: Math.max(2, count)
    };

    sendZoomOnly(cmd);

    ptzLog('KEYBOARD QA SWEEP HOLD START', {
      source,
      cmd,
      start_idx: keyboardQaSweepHold.startIdx,
      samples: keyboardQaSweepHold.count,
      full_sweep_ms: keyboardQaFullSweepMs(),
      step_ms: keyboardQaStepMs()
    });

    if ($('ptzTuneStatus')) {
      $('ptzTuneStatus').textContent =
        `Q/A sweep hold START ${source} sample=${keyboardQaSweepHold.startIdx} cmd=${cmd}`;
    }
  }

  async function keyboardQaSweepStop(reason = 'keyup') {
    if (!keyboardQaSweepHold) return;

    const h = keyboardQaSweepHold;
    keyboardQaSweepHold = null;

    sendZoomOnly(0);

    const heldMs = Math.max(0, performance.now() - h.startTs);
    const step = keyboardQaStepMs();
    const rawDelta = heldMs / Math.max(1, step);

    // Round to nearest sample; one very short tap may stay in same sample.
    const delta = Math.max(0, Math.round(rawDelta));
    const maxIdx = Math.max(0, h.count - 1);
    const target = Math.max(0, Math.min(maxIdx, h.startIdx + h.direction * delta));
    const ratio = maxIdx > 0 ? target / maxIdx : 0;

    try {
      await apiPostJson('/api/zoom/state', {
        zoom_sample_idx: target,
        zoom_ratio: ratio,
        zoom_confidence: 0.75,
        zoom_source: 'keyboard_qa_sweep_hold'
      });
    } catch (e) {
      ptzLog('KEYBOARD QA SWEEP state update error', { error: String(e.message || e) });
    }

    if (typeof ptzTuneCurrentSample !== 'undefined') {
      ptzTuneCurrentSample = target;
    }

    try {
      await ptzTuneLoadSpeedForSample?.(target);
    } catch (_) {}

    updatePtzTuneSampleButtonUi?.();

    ptzLog('KEYBOARD QA SWEEP HOLD STOP', {
      reason,
      source: h.source,
      start_idx: h.startIdx,
      target_idx: target,
      held_ms: Math.round(heldMs),
      raw_delta: Number(rawDelta.toFixed(2)),
      delta,
      step_ms: Number(step.toFixed(1))
    });

    if ($('ptzTuneStatus')) {
      $('ptzTuneStatus').textContent =
        `Q/A sweep hold: ${h.startIdx}→${target}, held=${Math.round(heldMs)}ms step=${step.toFixed(1)}ms`;
    }
  }

  window.keyboardQaSweepStop = keyboardQaSweepStop;

"""

if "KEYBOARD QA SWEEP HOLD START" not in s:
    if insert_after not in s:
        raise SystemExit("ERROR: keyboardZoomSampleStep block not found")
    s = s.replace(insert_after, insert_after + helper, 1)
    print("OK: inserted Q/A sweep hold helpers")
    changed = True
else:
    print("SKIP: Q/A sweep hold helpers already present")

old_keydown = """    if (isQ || isA) {
      e.preventDefault();
      if (e.repeat) return;
      keyboardState[isQ ? 'zin' : 'zout'] = true;
      if (isQ) keyboardZoomSampleStep(1, 'Q_TELE');
      if (isA) keyboardZoomSampleStep(-1, 'A_WIDE');
      return;
    }
"""

new_keydown = """    if (isQ || isA) {
      e.preventDefault();
      if (e.repeat) return;

      keyboardState[isQ ? 'zin' : 'zout'] = true;

      if (keyboardQaIsSweepMode()) {
        keyboardQaSweepStart(isQ, isQ ? 'Q_TELE_SWEEP_HOLD' : 'A_WIDE_SWEEP_HOLD')
          .catch(e => {
            sendZoomOnly(0);
            keyboardQaSweepHold = null;
            ptzLog('KEYBOARD QA SWEEP START error', { error: String(e.message || e) });
          });
      } else {
        if (isQ) keyboardZoomSampleStep(1, 'Q_TELE');
        if (isA) keyboardZoomSampleStep(-1, 'A_WIDE');
      }

      return;
    }
"""

if old_keydown in s:
    s = s.replace(old_keydown, new_keydown, 1)
    print("OK: Q/A keydown now uses sweep hold when movement=sweep")
    changed = True
else:
    print("WARN: Q/A keydown block not found or already patched")

old_keyup = """    if (isQ || isA) {
      e.preventDefault();
      keyboardState[isQ ? 'zin' : 'zout'] = false;
      return;
    }
"""

new_keyup = """    if (isQ || isA) {
      e.preventDefault();
      keyboardState[isQ ? 'zin' : 'zout'] = false;

      if (keyboardQaIsSweepMode()) {
        keyboardQaSweepStop('keyup').catch(e => {
          sendZoomOnly(0);
          keyboardQaSweepHold = null;
          ptzLog('KEYBOARD QA SWEEP STOP error', { error: String(e.message || e) });
        });
      }

      return;
    }
"""

if old_keyup in s:
    s = s.replace(old_keyup, new_keyup, 1)
    print("OK: Q/A keyup now stops real sweep hold")
    changed = True
else:
    print("WARN: Q/A keyup block not found or already patched")

# Add safety stops.
if "keyboardQaSweepStop('window_blur')" not in s:
    marker = """  window.addEventListener('keyup', (e) => {
    if (shouldIgnoreKeyboardEvent(e)) return;
"""
    safety = """  window.addEventListener('blur', () => {
    if (keyboardQaSweepHold) keyboardQaSweepStop('window_blur').catch(() => sendZoomOnly(0));
  });

  document.addEventListener('visibilitychange', () => {
    if (document.hidden && keyboardQaSweepHold) keyboardQaSweepStop('visibility_hidden').catch(() => sendZoomOnly(0));
  });

"""
    if marker in s:
        s = s.replace(marker, safety + marker, 1)
        print("OK: added Q/A sweep safety stop on blur/hidden")
        changed = True
    else:
        print("WARN: safety insert point not found")

if changed:
    p.write_text(s, encoding="utf-8")
    print("DONE")
else:
    print("No changes")
