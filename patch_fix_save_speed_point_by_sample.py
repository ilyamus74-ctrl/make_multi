from pathlib import Path
import re
import shutil
import time

p = Path("web/index.html")
s = p.read_text(encoding="utf-8")

backup = p.with_suffix(f".html.bak_fix_save_speed_point_by_sample_{int(time.time())}")
shutil.copy2(p, backup)
print(f"backup: {backup}")

pattern = re.compile(
    r"\$\('ptzTuneSavePointBtn'\)\.onclick\s*=\s*\(\)\s*=>\s*\(async\s*\(\)\s*=>\s*\{.*?\n\s*\}\)\(\)\.catch\(e\s*=>\s*ptzLog\('PTZ TUNE SAVE error'.*?\)\);",
    re.S
)

replacement = r'''$('ptzTuneSavePointBtn').onclick = () => (async () => {
    const zs = await apiGetJson('/api/zoom/state');

    const sampleIdxRaw = Number(zs.zoom_sample_idx ?? ptzTuneCurrentSample ?? 0);
    const maxSampleIdx = typeof ptzTuneMaxSampleIdx === 'function' ? ptzTuneMaxSampleIdx() : sampleIdxRaw;
    const sampleIdx = Math.max(0, Math.min(maxSampleIdx, Number.isFinite(sampleIdxRaw) ? sampleIdxRaw : 0));

    const target = (ptzTuneZoomSamples || []).find(p => Number(p.profile_idx) === sampleIdx) || {
      profile_idx: sampleIdx,
      zoom_ratio: Number(zs.zoom_ratio || 0),
      focal_px: Number(zs.focal_px || 0)
    };

    const zoomRatio = Number(target.zoom_ratio ?? zs.zoom_ratio ?? 0);
    const focalPx = Number(target.focal_px ?? zs.focal_px ?? 0);

    // Keep current backend speed config as the source of truth.
    // Operator should press APPLY SPEED before SAVE SPEED POINT.
    const res = await apiPostJson(`${autopilotBaseUrl()}/api/autopilot/speed_profile/save_point`, {
      profile_idx: sampleIdx,
      step: sampleIdx, // legacy compatibility
      zoom_ratio: zoomRatio,
      focal_px: focalPx
    });

    await refreshPtzTuneSpeedProfile();
    tuneStatusSet(zoomRatio, focalPx);
    updatePtzTuneSampleButtonUi?.();

    ptzLog(`SPEED SAMPLE SAVED sample=${sampleIdx} zoom=${zoomRatio.toFixed(3)} focal=${Math.round(focalPx)}`, res);
  })().catch(e => ptzLog('PTZ TUNE SAVE error', { error: String(e.message || e) }));'''

s2, n = pattern.subn(replacement, s, count=1)

if n != 1:
    raise SystemExit(f"ERROR: SAVE SPEED POINT handler not replaced, replacements={n}")

p.write_text(s2, encoding="utf-8")
print("OK: SAVE SPEED POINT now saves by profile_idx/current sample")
