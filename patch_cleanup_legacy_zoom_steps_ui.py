from pathlib import Path
import re
import shutil
import time

p = Path("web/index.html")
s = p.read_text(encoding="utf-8")

backup = p.with_suffix(f".html.bak_cleanup_legacy_zoom_steps_{int(time.time())}")
shutil.copy2(p, backup)
print(f"backup: {backup}")

def replace_function(src, name, replacement=""):
    markers = [f"async function {name}", f"function {name}"]
    start = -1
    for marker in markers:
        start = src.find(marker)
        if start >= 0:
            break
    if start < 0:
        print(f"WARN: function {name} not found")
        return src

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
                return src[:start] + replacement + src[end:]

        i += 1

    raise SystemExit(f"ERROR: end of function {name} not found")

# 1) Remove old Advanced manual zoom steps UI, keep only backend rehome-current button.
old_adv = re.compile(
    r'\s*<details id="ptzTuneAdvancedManualZoom">.*?</details>',
    re.S
)

new_adv = '''
        <button id="ptzTuneRehomeCurrentSampleBtn" class="small-btn" title="Backend nearest-edge rehome and return to current sample">REHOME CURRENT</button>
'''

s2, n = old_adv.subn(new_adv, s, count=1)
if n != 1:
    raise SystemExit(f"ERROR: advanced manual zoom block not replaced, replacements={n}")
s = s2
print("OK: removed Advanced manual zoom steps UI")

# 2) Remove old browser-side zoom movement helpers.
s = replace_function(s, "ptzTuneMoveRelativeSamples", "\n")
s = replace_function(s, "ptzTuneSetZoomStateForSample", "\n")
s = replace_function(s, "tuneMoveSteps", "\n")
print("OK: removed old browser-side sample movement functions")

# 3) Replace old handlers for REHOME WIDE / TELE +1 / TELE +2 / WIDE -1.
start = s.find("  $('ptzTuneRehomeWideBtn').onclick")
end = s.find("  $('ptzTuneApplyConfigBtn').onclick", start)

if start < 0 or end < 0:
    raise SystemExit(f"ERROR: cannot find old handler block start={start} end={end}")

new_handlers = r'''  if ($('ptzTuneRehomeCurrentSampleBtn')) $('ptzTuneRehomeCurrentSampleBtn').onclick = () => (async () => {
    const sampleIdx = Number(ptzTuneCurrentSample || 0);

    const res = await apiPostJson('/api/zoom/rehome_current_sample', {
      profile_idx: sampleIdx,
      nearest_edge: true
    });

    if (!res || res.ok === false) {
      throw new Error(res?.error || 'backend_rehome_failed');
    }

    ptzTuneCurrentSample = Number(res.target_sample ?? sampleIdx);

    try {
      const zs = await apiGetJson('/api/zoom/state');
      const idx = Number(zs.zoom_sample_idx);
      if (Number.isFinite(idx)) ptzTuneCurrentSample = idx;
      tuneStatusSet?.(Number(zs.zoom_ratio || 0), Number(zs.focal_px || 0));
    } catch (_) {}

    await ptzTuneLoadSpeedForSample(ptzTuneCurrentSample).catch(() => {});
    await refreshPtzTelemetry?.().catch(() => {});
    updatePtzTuneSampleButtonUi?.();

    ptzLog('PTZ TUNE REHOME CURRENT BACKEND', res);
  })().catch(e => ptzLog('PTZ TUNE REHOME CURRENT error', { error: String(e.message || e) }));

'''

s = s[:start] + new_handlers + s[end:]
print("OK: replaced old advanced zoom handlers with backend rehome-current handler")

p.write_text(s, encoding="utf-8")
print("DONE: legacy manual zoom-step UI cleanup complete")
