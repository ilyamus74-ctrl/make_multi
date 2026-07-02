#!/usr/bin/env python3
from pathlib import Path
import time

ROOT = Path('/root/new_yolo8')
WEB = ROOT / 'web' / 'index.html'
MARKER_START = 'PTZ_QA_SWEEP_PROFILE_IDX_BACKEND_START'
MARKER_END = 'PTZ_QA_SWEEP_PROFILE_IDX_BACKEND_END'
MARKER_V2 = 'PTZ_QA_SAMPLE_STEP_MOVEMENT_MODE_V2'
MARKER_V4 = 'PTZ_QA_SAMPLE_STEP_MOVEMENT_MODE_V4'

block = r'''

  // PTZ_QA_SWEEP_PROFILE_IDX_BACKEND_START
  // PTZ_QA_SAMPLE_STEP_MOVEMENT_MODE_V2
  // PTZ_QA_SAMPLE_STEP_MOVEMENT_MODE_V4
  // Canonical Q/A sample-step handler.
  // Q/A selects only target profile_idx. ZOOM CALIB Movement selects physical movement:
  //   legacy_impulse or sweep_time_steps.
  // Backend /api/zoom/go_to_sample remains the source of real zoom/frame movement.
  var ptzQaSweepProfileIdxBusy = false;

  function ptzQaSweepIsTypingTarget(el) {
    try {
      const tag = String(el?.tagName || '').toUpperCase();
      return tag === 'INPUT' || tag === 'TEXTAREA' || tag === 'SELECT' || Boolean(el?.isContentEditable);
    } catch (_) {
      return false;
    }
  }

  function ptzQaSweepMovementMode() {
    const raw = String($('zoomMoveMode')?.value || 'legacy_impulse');
    return raw === 'sweep_time_steps' ? 'sweep_time_steps' : 'legacy_impulse';
  }

  function ptzQaSweepDomSampleCount() {
    const buttons = Array.from(document.querySelectorAll('.sample-btn[data-sample-idx]'));
    const maxBtn = buttons
      .map(b => Number(b.dataset.sampleIdx))
      .filter(Number.isFinite)
      .reduce((m, v) => Math.max(m, v), -1);
    if (maxBtn >= 0) return maxBtn + 1;
    return Math.max(2, Math.min(100, Number($('zoomSamples')?.value || 10)));
  }

  async function ptzQaSweepCurrentSampleState() {
    let z = null;
    try { z = await apiGetJson('/api/zoom/state'); } catch (_) {}

    let idx = Number(z?.zoom_sample_idx);
    if (!Number.isFinite(idx)) idx = Number(typeof ptzTuneCurrentSample !== 'undefined' ? ptzTuneCurrentSample : 0);
    if (!Number.isFinite(idx)) idx = 0;

    let count = Number(z?.zoom_sample_count);
    if (!Number.isFinite(count) || count < 2) count = ptzQaSweepDomSampleCount();
    count = Math.max(2, Math.min(100, Math.round(count)));

    idx = Math.max(0, Math.min(count - 1, Math.round(idx)));
    return { idx, count, zoom: z || {} };
  }

  async function ptzQaSweepProfileIdxStep(delta, source) {
    if (ptzQaSweepProfileIdxBusy) return { ok: false, error: 'qa_sample_step_busy' };
    ptzQaSweepProfileIdxBusy = true;

    const movementMode = ptzQaSweepMovementMode();

    try {
      const st = await ptzQaSweepCurrentSampleState();
      const target = Math.max(0, Math.min(st.count - 1, st.idx + Number(delta || 0)));

      if (target === st.idx) {
        if ($('ptzTuneStatus')) $('ptzTuneStatus').textContent = `Q/A sample edge sample=${st.idx} movement=${movementMode}`;
        try { ptzLog?.('Q/A SAMPLE STEP edge', { source, idx: st.idx, target, count: st.count, movementMode }); } catch (_) {}
        return { ok: true, skipped: true, from: st.idx, to: target, movementMode };
      }

      const moveRes = await apiPostJson('/api/zoom/go_to_sample', {
        profile_idx: target,
        mode: 'relative',
        movement_mode: movementMode,
        zoom_move_mode: movementMode,
        source
      });

      if (!moveRes || moveRes.ok === false) {
        throw new Error(moveRes?.error || 'zoom_go_to_sample_failed');
      }

      let finalIdx = Number(moveRes.to_sample ?? moveRes.target_sample ?? moveRes.profile_idx ?? target);
      if (!Number.isFinite(finalIdx)) finalIdx = target;
      finalIdx = Math.max(0, Math.min(st.count - 1, Math.round(finalIdx)));

      const speedRes = await apiPostJson(`${autopilotBaseUrl()}/api/autopilot/speed_profile/apply_nearest`, {
        profile_idx: finalIdx,
        source
      });

      if (typeof ptzTuneCurrentSample !== 'undefined') ptzTuneCurrentSample = finalIdx;
      try { await ptzTuneLoadSpeedForSample?.(finalIdx); } catch (_) {}
      try { await refreshPtzTuneSpeedProfile?.(); } catch (_) {}
      try { updatePtzTuneSampleButtonUi?.(); } catch (_) {}
      try { await refreshPtzTelemetry?.(); } catch (_) {}

      if ($('ptzTuneStatus')) {
        $('ptzTuneStatus').textContent = `Q/A profile_idx step ${st.idx}→${finalIdx} movement=${movementMode} source=${source}`;
      }

      try {
        ptzLog?.('Q/A SAMPLE STEP', {
          source,
          from: st.idx,
          target,
          finalIdx,
          movementMode,
          moveRes,
          speedRes
        });
      } catch (_) {}

      return { ok: true, from: st.idx, to: finalIdx, movementMode, moveRes, speedRes };
    } catch (err) {
      if ($('ptzTuneStatus')) $('ptzTuneStatus').textContent = `Q/A sample step error: ${String(err.message || err)}`;
      try { ptzLog?.('Q/A SAMPLE STEP error', { source, movementMode, error: String(err.message || err) }); } catch (_) {}
      return { ok: false, movementMode, error: String(err.message || err) };
    } finally {
      ptzQaSweepProfileIdxBusy = false;
    }
  }

  window.ptzQaSweepProfileIdxStep = ptzQaSweepProfileIdxStep;
  window.ptzQaSweepMovementMode = ptzQaSweepMovementMode;

  window.addEventListener('keydown', (e) => {
    const key = String(e.key || '').toLowerCase();
    const code = String(e.code || '');
    const isQ = key === 'q' || code === 'KeyQ';
    const isA = key === 'a' || code === 'KeyA';

    if (!isQ && !isA) return;
    if (ptzQaSweepIsTypingTarget(e.target)) return;

    e.preventDefault();
    e.stopPropagation();
    e.stopImmediatePropagation();

    if (e.repeat) return;

    if (isQ) {
      ptzQaSweepProfileIdxStep(1, 'keyboard_Q_profile_idx_sweep').catch(() => {});
    } else if (isA) {
      ptzQaSweepProfileIdxStep(-1, 'keyboard_A_profile_idx_sweep').catch(() => {});
    }
  }, true);

  window.addEventListener('keyup', (e) => {
    const key = String(e.key || '').toLowerCase();
    const code = String(e.code || '');
    const isQ = key === 'q' || code === 'KeyQ';
    const isA = key === 'a' || code === 'KeyA';

    if (!isQ && !isA) return;
    if (ptzQaSweepIsTypingTarget(e.target)) return;

    e.preventDefault();
    e.stopPropagation();
    e.stopImmediatePropagation();
  }, true);
  // PTZ_QA_SWEEP_PROFILE_IDX_BACKEND_END
'''

if not WEB.exists():
    raise SystemExit(f'Missing {WEB}')

text = WEB.read_text(encoding='utf-8', errors='ignore')
if MARKER_V4 in text:
    print('Already patched:', MARKER_V4)
    raise SystemExit(0)
if MARKER_START not in text or MARKER_END not in text:
    raise SystemExit('Missing Q/A profile_idx block; apply patch_ui_qa_sweep_profile_idx_v1.py first')

backup = WEB.with_name(WEB.name + f'.bak_qa_sample_step_movement_mode_v4_{int(time.time())}')
backup.write_text(text, encoding='utf-8')

start = text.find(MARKER_START)
end = text.find(MARKER_END, start)
if start < 0 or end < 0 or end <= start:
    raise SystemExit('Could not locate canonical Q/A block')

# Include the full line containing the end marker.
line_end = text.find('\n', end)
if line_end < 0:
    line_end = len(text)
else:
    line_end += 1

# Include indentation/comment prefix before start marker.
line_start = text.rfind('\n', 0, start)
if line_start < 0:
    line_start = 0
else:
    line_start += 1

new_text = text[:line_start] + block + text[line_end:]
WEB.write_text(new_text, encoding='utf-8')

print('OK patched web/index.html Q/A sample-step movement mode v4')
print('Backup:', backup)
print('Changes:')
print(' - replaces canonical Q/A block instead of searching old helper names')
print(' - Q/A owns both LEGACY IMPULSE and SWEEP TIME STEPS')
print(' - Q/A chooses profile_idx target only')
print(' - /api/zoom/go_to_sample receives movement_mode/zoom_move_mode')
print(' - applies PTZ speed profile by final profile_idx')