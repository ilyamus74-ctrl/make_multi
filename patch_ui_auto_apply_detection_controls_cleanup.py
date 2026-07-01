from pathlib import Path
import time

p = Path("web/index.html")
s = p.read_text(encoding="utf-8")

bak = p.with_suffix(p.suffix + f".bak_auto_apply_detection_controls_cleanup_{int(time.time())}")
bak.write_text(s, encoding="utf-8")

late_script = r'''
<script>
// DETECTION_AUTO_APPLY_CLEANUP_START
(function () {
  const $ = (id) => document.getElementById(id);

  async function getJson(url) {
    const r = await fetch(url, { cache: 'no-store' });
    if (!r.ok) throw new Error(`${r.status} ${url}`);
    return await r.json();
  }

  async function postJson(url, body) {
    const r = await fetch(url, {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify(body || {})
    });
    if (!r.ok) throw new Error(`${r.status} ${url}`);
    return await r.json().catch(() => ({ ok: true }));
  }

  function autopilotBaseUrl() {
    return `${location.protocol}//${location.hostname}:8090`;
  }

  function shortModelName(v) {
    return String(v || '').split('/').filter(Boolean).pop() || String(v || '');
  }

  async function readSettings() {
    try {
      const cfg = await getJson('/api/settings');
      return cfg && typeof cfg === 'object' ? cfg : {};
    } catch (_) {
      return {};
    }
  }

  async function saveSettingsMerge(patch) {
    const cfg = await readSettings();
    const next = Object.assign({}, cfg, patch || {});
    next.config_version = Math.max(16, Number(next.config_version || 16));

    try {
      localStorage.setItem('pantilt_ui_settings_v2', JSON.stringify(next));
    } catch (_) {}

    await postJson('/api/settings', next);
    return next;
  }

  function hideLegacyApplyControls() {
    const ids = [
      'operatorModelApplyBtn',
      'operatorLimitApplyBtn',
      'operatorDetectEveryApplyBtn',
      'operatorDetectionAreaApplyBtn'
    ];

    for (const id of ids) {
      const el = $(id);
      if (!el) continue;
      el.style.display = 'none';
      el.setAttribute('aria-hidden', 'true');
      el.disabled = true;
    }

    const stateIds = [
      'operatorModelState',
      'operatorLimitState',
      'operatorDetectEveryState',
      'operatorDetectionAreaState'
    ];

    for (const id of stateIds) {
      const el = $(id);
      if (!el) continue;
      el.style.display = 'none';
    }
  }

  async function applyModelFromSelect() {
    const select = $('operatorModelSelect');
    if (!select) return;

    const model = String(select.value || '').trim();
    if (!model) return;

    try {
      const line = $('detectionMenuSummary');
      if (line) line.textContent = `switching model ${shortModelName(model)}`;

      await postJson(`${autopilotBaseUrl()}/api/autopilot/stop`, {}).catch(() => {});
      const res = await postJson('/api/detector/config', {
        current_model: model
      });

      await saveSettingsMerge({
        operatorModel: model
      });

      if (line) {
        line.textContent = `model ${shortModelName(model)}`;
      }

      console.log('[det-auto-cleanup] model applied', res);
    } catch (err) {
      console.error('[det-auto-cleanup] model apply failed', err);
      const line = $('detectionMenuSummary');
      if (line) line.textContent = `model error ${String(err.message || err)}`;
    }
  }

  async function applyLimitFromSelect() {
    const el = $('operatorDetectionLimit');
    if (!el) return;

    const maxDet = Number(el.value || 10);
    const maxRaw = Math.max(20, maxDet * 5);

    const res = await postJson('/api/detection/limits', {
      max_detections: maxDet,
      max_raw_candidates: maxRaw
    });

    await saveSettingsMerge({
      operatorDetectionLimit: Number(res.max_detections || maxDet)
    });

    console.log('[det-auto-cleanup] limit applied', res);
  }

  async function applyDetectEveryFromSelect() {
    const el = $('operatorDetectEvery');
    if (!el) return;

    const n = Number(el.value || 1);

    const res = await postJson('/api/detection/throttle', {
      detect_every_n_frames: n
    });

    await saveSettingsMerge({
      operatorDetectEvery: Number(res.detect_every_n_frames || n)
    });

    console.log('[det-auto-cleanup] detect fps applied', res);
  }

  async function frameSize() {
    try {
      const d = await getJson('/api/detections');
      return {
        w: Math.max(1, Number(d.width || 1920)),
        h: Math.max(1, Number(d.height || 1080))
      };
    } catch (_) {
      return { w: 1920, h: 1080 };
    }
  }

  function skyRoi(w, h) {
    return {
      id: 'sky_top',
      enabled: true,
      x: 0,
      y: 0,
      w: w,
      h: Math.max(1, Math.round(h * 0.68)),
      every_n_frames: 1,
      classes: []
    };
  }

  function centerRoi(w, h) {
    const rw = Math.round(w * 0.60);
    const rh = Math.round(h * 0.55);
    return {
      id: 'center',
      enabled: true,
      x: Math.max(0, Math.round((w - rw) / 2)),
      y: Math.max(0, Math.round((h - rh) / 2)),
      w: Math.max(1, rw),
      h: Math.max(1, rh),
      every_n_frames: 1,
      classes: []
    };
  }

  function fullRoi(w, h) {
    return {
      id: 'full_frame',
      enabled: true,
      x: 0,
      y: 0,
      w: w,
      h: h,
      every_n_frames: 1,
      classes: []
    };
  }

  async function buildAreaPayload(mode) {
    const sz = await frameSize();
    const w = sz.w;
    const h = sz.h;

    mode = String(mode || 'full_frame');

    if (mode === 'full_frame') {
      return { detection_mode: 'full_frame', rois: [] };
    }

    if (mode === 'tiled') {
      return { detection_mode: 'tiled', rois: [] };
    }

    if (mode === 'roi') {
      return { detection_mode: 'roi', rois: [skyRoi(w, h)] };
    }

    if (mode === 'multi_roi') {
      return { detection_mode: 'multi_roi', rois: [skyRoi(w, h), centerRoi(w, h)] };
    }

    if (mode === 'hybrid') {
      return { detection_mode: 'hybrid', rois: [fullRoi(w, h), skyRoi(w, h)] };
    }

    return { detection_mode: 'full_frame', rois: [] };
  }

  async function applyAreaFromSelect() {
    const el = $('operatorDetectionAreaMode');
    if (!el) return;

    const mode = String(el.value || 'full_frame');
    const payload = await buildAreaPayload(mode);

    const res = await postJson('/api/detection/roi_config', payload);

    await saveSettingsMerge({
      operatorDetectionAreaMode: payload.detection_mode
    });

    console.log('[det-auto-cleanup] area applied', res);
  }

  function bindAutoApply(id, fn) {
    const el = $(id);
    if (!el || el.__autoApplyCleanupBound) return;

    el.__autoApplyCleanupBound = true;
    el.addEventListener('change', async (e) => {
      e.preventDefault();
      e.stopPropagation();

      try {
        await fn();
      } catch (err) {
        console.error('[det-auto-cleanup]', id, err);
      }
    }, true);
  }

  function boot() {
    hideLegacyApplyControls();

    bindAutoApply('operatorModelSelect', applyModelFromSelect);
    bindAutoApply('operatorDetectionLimit', applyLimitFromSelect);
    bindAutoApply('operatorDetectEvery', applyDetectEveryFromSelect);
    bindAutoApply('operatorDetectionAreaMode', applyAreaFromSelect);
  }

  if (document.readyState === 'loading') {
    document.addEventListener('DOMContentLoaded', boot, { once: true });
  } else {
    boot();
  }

  setTimeout(boot, 500);
  setTimeout(boot, 1500);
  setTimeout(boot, 3000);
})();
// DETECTION_AUTO_APPLY_CLEANUP_END
</script>
'''

if "DETECTION_AUTO_APPLY_CLEANUP_START" not in s:
    if "</body>" not in s:
        raise SystemExit("ERROR: </body> not found")
    s = s.replace("</body>", late_script + "\n</body>", 1)
    p.write_text(s, encoding="utf-8")
    print(f"OK patched {p}, backup={bak}")
else:
    print("OK already installed")
