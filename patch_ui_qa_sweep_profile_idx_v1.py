#!/usr/bin/env python3
from pathlib import Path
import time

p = Path('/root/new_yolo8/web/index.html')
s = p.read_text(encoding='utf-8', errors='ignore')

MARK = 'PTZ_QA_SWEEP_PROFILE_IDX_BACKEND_START'
if MARK in s:
    print('Already patched web/index.html Q/A sweep profile_idx v1')
    raise SystemExit(0)

bak = p.with_name(p.name + f'.bak_qa_sweep_profile_idx_v1_{int(time.time())}')
bak.write_text(s, encoding='utf-8')

block = r'''

  // PTZ_QA_SWEEP_PROFILE_IDX_BACKEND_START
  // Canonical Q/A SWEEP contract:
  // Q/A must move the real backend zoom sample through /api/zoom/go_to_sample
  // using profile_idx, then apply PTZ SPEED TUNE for the same profile_idx.
  // This high-priority capture handler is intentionally registered before older Q/A handlers.
  var ptzQaSweepProfileIdxBusy = false;

  function ptzQaSweepProfileIdxKey(e) {
    const key = String(e?.key || '').toLowerCase();
    const code = String(e?.code || '');
    const isQ = key === 'q' || code === 'KeyQ';
    const isA = key === 'a' || code === 'KeyA';
    return { ok: isQ || isA, isQ, isA };
  }

  function ptzQaSweepProfileIdxTypingTarget(target) {
    const tag = String(target?.tagName || '').toUpperCase();
    return tag === 'INPUT' || tag === 'TEXTAREA' || tag === 'SELECT' || Boolean(target?.isContentEditable);
  }

  function ptzQaSweepProfileIdxEnabled() {
    return String($('zoomMoveMode')?.value || '') === 'sweep_time_steps';
  }

  function ptzQaSweepProfileIdxClamp(v, lo, hi) {
    return Math.max(lo, Math.min(hi, v));
  }

  function ptzQaSweepProfileIdxSetStatus(msg) {
    if ($('ptzTuneStatus')) $('ptzTuneStatus').textContent = msg;
  }

  async function ptzQaSweepProfileIdxStep(isQ, reason = 'keyboard_qa_sweep_profile_idx') {
    if (ptzQaSweepProfileIdxBusy) return { ok: false, skipped: 'busy' };
    ptzQaSweepProfileIdxBusy = true;

    try {
      const z0 = await apiGetJson('/api/zoom/state');
      const count = Math.max(2, Number(z0?.zoom_sample_count || $('zoomSamples')?.value || 10));
      const maxIdx = count - 1;

      let cur = Number(z0?.zoom_sample_idx);
      if (!Number.isFinite(cur) && typeof ptzTuneCurrentSample !== 'undefined') {
        cur = Number(ptzTuneCurrentSample);
      }
      if (!Number.isFinite(cur)) cur = 0;
      cur = ptzQaSweepProfileIdxClamp(Math.round(cur), 0, maxIdx);

      // Q = TELE = sample +1, A = WIDE = sample -1.
      const direction = isQ ? 1 : -1;
      const target = ptzQaSweepProfileIdxClamp(cur + direction, 0, maxIdx);

      if (target === cur) {
        ptzQaSweepProfileIdxSetStatus(`Q/A sweep edge sample=${cur}`);
        try { window.ptzLog?.('QA SWEEP PROFILE_IDX edge', { reason, cur, target, count }); } catch (_) {}
        return { ok: true, skipped: 'edge', cur, target };
      }

      ptzQaSweepProfileIdxSetStatus(`Q/A sweep moving ${cur}→${target}`);

      const moveRes = await apiPostJson('/api/zoom/go_to_sample', {
        profile_idx: target,
        mode: 'relative',
        source: reason
      });

      if (!moveRes || moveRes.ok === false) {
        throw new Error(moveRes?.error || 'go_to_sample_failed');
      }

      let z1 = null;
      try { z1 = await apiGetJson('/api/zoom/state'); } catch (_) {}

      let finalIdx = Number(z1?.zoom_sample_idx);
      if (!Number.isFinite(finalIdx)) finalIdx = Number(moveRes?.to_sample ?? moveRes?.target_sample ?? target);
      if (!Number.isFinite(finalIdx)) finalIdx = target;
      finalIdx = ptzQaSweepProfileIdxClamp(Math.round(finalIdx), 0, maxIdx);

      const finalRatio = Number(z1?.zoom_ratio ?? moveRes?.zoom_ratio ?? (maxIdx > 0 ? finalIdx / maxIdx : 0));

      if (typeof ptzTuneCurrentSample !== 'undefined') {
        ptzTuneCurrentSample = finalIdx;
      }

      let speedRes = null;
      try {
        speedRes = await apiPostJson(`${autopilotBaseUrl()}/api/autopilot/speed_profile/apply_nearest`, {
          profile_idx: finalIdx,
          zoom_ratio: Number.isFinite(finalRatio) ? finalRatio : undefined,
          source: reason
        });
      } catch (e) {
        speedRes = { ok: false, error: String(e?.message || e) };
      }

      try {
        if (typeof ptzTuneLoadSpeedForSample === 'function') {
          await ptzTuneLoadSpeedForSample(finalIdx);
        }
      } catch (_) {}

      try { await refreshPtzTelemetry?.(); } catch (_) {}
      try { await refreshPtzTuneSpeedProfile?.(); } catch (_) {}
      try { await initPtzSpeedTuneSamples?.(); } catch (_) {}
      try { updatePtzTuneSampleButtonUi?.(); } catch (_) {}
      try { updateOperatorMenuSummaries?.(); } catch (_) {}

      ptzQaSweepProfileIdxSetStatus(`Q/A sweep ${cur}→${finalIdx} zoom=${Number(finalRatio || 0).toFixed(2)}`);
      try { window.ptzLog?.('QA SWEEP PROFILE_IDX MOVE', { reason, from: cur, target, finalIdx, moveRes, speedRes }); } catch (_) {}

      return { ok: true, from: cur, to: finalIdx, moveRes, speedRes };
    } finally {
      ptzQaSweepProfileIdxBusy = false;
    }
  }

  window.ptzQaSweepProfileIdxStep = ptzQaSweepProfileIdxStep;

  window.addEventListener('keydown', (e) => {
    const k = ptzQaSweepProfileIdxKey(e);
    if (!k.ok) return;
    if (!ptzQaSweepProfileIdxEnabled()) return;
    if (ptzQaSweepProfileIdxTypingTarget(e.target)) return;

    e.preventDefault();
    e.stopPropagation();
    e.stopImmediatePropagation();

    if (e.repeat) return;

    ptzQaSweepProfileIdxStep(k.isQ, k.isQ ? 'keyboard_Q_profile_idx_sweep' : 'keyboard_A_profile_idx_sweep')
      .catch(err => {
        ptzQaSweepProfileIdxSetStatus(`Q/A sweep error: ${String(err?.message || err)}`);
        try { window.ptzLog?.('QA SWEEP PROFILE_IDX error', { error: String(err?.message || err) }); } catch (_) {}
      });
  }, true);

  window.addEventListener('keyup', (e) => {
    const k = ptzQaSweepProfileIdxKey(e);
    if (!k.ok) return;
    if (!ptzQaSweepProfileIdxEnabled()) return;
    if (ptzQaSweepProfileIdxTypingTarget(e.target)) return;

    e.preventDefault();
    e.stopPropagation();
    e.stopImmediatePropagation();
  }, true);
  // PTZ_QA_SWEEP_PROFILE_IDX_BACKEND_END
'''

anchor = '  // QA_SWEEP_GLOBAL_HELPERS_END\n'
if anchor in s:
    s = s.replace(anchor, anchor + block + '\n', 1)
else:
    # Fallback: insert before the first capture Q/A handler block that was added by old sweep patches.
    needle = "  window.addEventListener('keydown', (e) => {\n    const key = String(e.key || '').toLowerCase();\n    const code = String(e.code || '');\n    const isQ = key === 'q' || code === 'KeyQ';\n    const isA = key === 'a' || code === 'KeyA';\n\n    if (!isQ && !isA) return;\n    if (isTypingTarget(e.target)) return;\n    if (!isSweepMode()) return;"
    if needle not in s:
        raise SystemExit('anchor not found for Q/A sweep profile_idx patch')
    s = s.replace(needle, block + '\n\n' + needle, 1)

p.write_text(s, encoding='utf-8')
print('OK patched web/index.html Q/A sweep profile_idx v1')
print('Backup:', bak)
print('Changes:')
print(' - inserted high-priority Q/A SWEEP handler before older Q/A handlers')
print(' - Q/A uses /api/zoom/go_to_sample with profile_idx')
print(' - applies PTZ speed profile by profile_idx after zoom move')
print(' - updates PTZ SPEED TUNE current sample and sample buttons')