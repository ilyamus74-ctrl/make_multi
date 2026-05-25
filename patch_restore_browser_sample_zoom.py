from pathlib import Path
import shutil
import re

p = Path("web/index.html")
s = p.read_text(encoding="utf-8")

backup = p.with_suffix(".html.bak_restore_browser_sample_zoom")
if not backup.exists():
    shutil.copy2(p, backup)
    print(f"backup: {backup}")

# Replace backend-driven ptzTuneGoToSample if present.
pattern = re.compile(
    r"async function ptzTuneGoToSample\(sampleIdx[^)]*\)\s*\{.*?\n\s*\}\n(?=\s*async function|\s*function|\s*\$\('ptzTune|\s*if \(\$\(')",
    re.S
)

replacement = r'''
async function ptzTuneMoveRelativeSamples(delta) {
  delta = Math.round(Number(delta || 0));
  if (delta === 0) return;

  const cmd = delta > 0 ? getTeleCmd() : getWideCmd();
  const impulseMs = Number($('zoomImpulseMs')?.value || 170);
  const steps = Math.abs(delta);

  for (let i = 0; i < steps; i++) {
    await zoomTest(cmd, impulseMs);
    await sleepMs(80);
  }
}

async function ptzTuneSetZoomStateForSample(sampleIdx, source) {
  const target = (ptzTuneZoomSamples || []).find(p => Number(p.profile_idx) === Number(sampleIdx));
  if (!target) throw new Error(`zoom sample not found: ${sampleIdx}`);

  ptzTuneCurrentSample = Number(sampleIdx);

  const res = await apiPostJson('/api/zoom/state', {
    zoom_ratio: Number(target.zoom_ratio || 0),
    zoom_confidence: 1,
    zoom_source: source || `ptz_tune_sample_${sampleIdx}`,
    zoom_sample_idx: Number(sampleIdx)
  });

  await refreshPtzTelemetry().catch(() => {});
  updatePtzTuneSampleButtonUi();
  return res;
}

async function ptzTuneGoToSample(sampleIdx, opts = {}) {
  sampleIdx = Math.round(Number(sampleIdx || 0));

  const target = (ptzTuneZoomSamples || []).find(p => Number(p.profile_idx) === Number(sampleIdx));
  if (!target) {
    ptzLog('PTZ TUNE SAMPLE error', { error: 'sample_not_found', sampleIdx });
    return;
  }

  if (ptzTuneZoomMoveBusy) {
    ptzLog('PTZ TUNE SAMPLE busy', { sampleIdx });
    return;
  }

  ptzTuneZoomMoveBusy = true;

  try {
    await ptzStop().catch(() => {});

    const current = Number(ptzTuneCurrentSample || 0);
    const delta = sampleIdx - current;

    if (opts.forceAbsolute) {
      await ptzTuneRehomeWidePhysical();
      if (sampleIdx > 0) await ptzTuneMoveRelativeSamples(sampleIdx);
    } else {
      await ptzTuneMoveRelativeSamples(delta);
    }

    await ptzTuneSetZoomStateForSample(sampleIdx, `ptz_tune_browser_${current}_to_${sampleIdx}`);
    await ptzTuneLoadSpeedForSample(sampleIdx).catch(() => {});
    await refreshPtzTelemetry().catch(() => {});
    updatePtzTuneSampleButtonUi();

    ptzLog('PTZ TUNE SAMPLE moved', {
      mode: opts.forceAbsolute ? 'browser_absolute_wide' : 'browser_relative',
      from: current,
      to: sampleIdx,
      delta
    });
  } catch (e) {
    ptzLog('PTZ TUNE SAMPLE move error', {
      sampleIdx,
      error: String(e.message || e)
    });
  } finally {
    ptzTuneZoomMoveBusy = false;
  }
}
'''

if "async function ptzTuneGoToSample" not in s:
    raise SystemExit("ptzTuneGoToSample not found")

s2, n = pattern.subn(replacement, s, count=1)
if n != 1:
    raise SystemExit(f"failed to replace ptzTuneGoToSample, replacements={n}")

p.write_text(s2, encoding="utf-8")
print("patched web/index.html: restored browser-driven sample movement")
