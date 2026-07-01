from pathlib import Path
import shutil, time

p = Path("web/index.html")
s = p.read_text(encoding="utf-8")

backup = p.with_suffix(f".html.bak_fix_keyboard_qa_sweep_scope_{int(time.time())}")
shutil.copy2(p, backup)
print("Backup:", backup)

marker = """  window.addEventListener('keydown', (e) => {
    if (e.key === 'Escape') {
"""

helpers = r'''  // QA_SWEEP_GLOBAL_HELPERS_START
  // Q/A sweep helpers must be in the same top-level scope as keyboard handlers.
  var keyboardQaSweepHold = null;

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
      step_ms: Number(keyboardQaStepMs().toFixed(1))
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
  // QA_SWEEP_GLOBAL_HELPERS_END

'''

if "QA_SWEEP_GLOBAL_HELPERS_START" in s:
    print("SKIP: helpers already present")
else:
    if marker not in s:
        raise SystemExit("ERROR: keydown marker not found")
    s = s.replace(marker, helpers + marker, 1)
    p.write_text(s, encoding="utf-8")
    print("OK: inserted Q/A sweep helpers in main keyboard scope")
