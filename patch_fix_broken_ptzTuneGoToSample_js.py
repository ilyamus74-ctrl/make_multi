from pathlib import Path
import shutil
import time

p = Path("web/index.html")
s = p.read_text(encoding="utf-8")

backup = p.with_suffix(f".html.bak_fix_broken_ptzTuneGoToSample_{int(time.time())}")
shutil.copy2(p, backup)
print(f"backup: {backup}")

start = s.find("async function ptzTuneGoToSample")
end = s.find("\n  async function tuneMoveSteps", start)

if start < 0:
    raise SystemExit("ERROR: start marker not found: async function ptzTuneGoToSample")
if end < 0:
    raise SystemExit("ERROR: end marker not found: async function tuneMoveSteps")

clean_func = r'''
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
      tuneStatusSet?.(
        Number(zs.zoom_ratio || res.zoom_ratio || 0),
        Number(zs.focal_px || res.focal_px || 0)
      );
    } else {
      tuneStatusSet?.(
        Number(res.zoom_ratio || 0),
        Number(res.focal_px || 0)
      );
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

s = s[:start] + clean_func + s[end:]

p.write_text(s, encoding="utf-8")
print("OK: replaced broken ptzTuneGoToSample block")
