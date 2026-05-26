from pathlib import Path
import shutil, time

p = Path("web/index.html")
s = p.read_text(encoding="utf-8")

backup = p.with_suffix(f".html.bak_detection_area_mode_{int(time.time())}")
shutil.copy2(p, backup)
print("backup:", backup)

# 1. Insert UI controls after DETECT FPS state
old = '''          <button id="operatorDetectEveryApplyBtn" class="small-btn">APPLY</button>
          <code id="operatorDetectEveryState">detect: --</code>

          <div class="det-row">
'''

new = '''          <button id="operatorDetectEveryApplyBtn" class="small-btn">APPLY</button>
          <code id="operatorDetectEveryState">detect: --</code>

          <b>AREA:</b>
          <select id="operatorDetectionAreaMode" class="small-select">
            <option value="full_frame" selected>FULL FRAME</option>
            <option value="roi">ROI SKY</option>
            <option value="multi_roi">MULTI ROI</option>
            <option value="tiled">TILED 2x2</option>
            <option value="hybrid">HYBRID FULL+ROI</option>
          </select>
          <button id="operatorDetectionAreaApplyBtn" class="small-btn">APPLY AREA</button>
          <code id="operatorDetectionAreaState">area: full_frame</code>

          <div class="det-row">
'''

if 'id="operatorDetectionAreaMode"' not in s:
    if old not in s:
        raise SystemExit("ERROR: detection area HTML anchor not found")
    s = s.replace(old, new, 1)
    print("OK: inserted detection area UI")
else:
    print("SKIP: detection area UI already exists")


# 2. Replace old helper text
old = '''          <span style="opacity:.8">Detection ON/OFF runs YOLO. Objects choose what YOLO keeps. Search area is FULL FRAME by default.</span>
'''

new = '''          <span style="opacity:.8">Detection ON/OFF runs YOLO. Objects choose classes. AREA controls full frame / ROI / tiled / hybrid search.</span>
'''

if old in s:
    s = s.replace(old, new, 1)
    print("OK: updated detection help text")


# 3. Insert JS before closing main script
anchor = '''</script>



<!-- FORCE_VIDEO_BOOTSTRAP_START -->
'''

js = r'''
<script>
// DETECTION_AREA_MODE_UI_START
(function () {
  function $(id) { return document.getElementById(id); }

  async function getJson(url) {
    const r = await fetch(url, { cache: 'no-store' });
    if (!r.ok) throw new Error(String(r.status) + ' ' + url);
    return await r.json();
  }

  async function postJson(url, payload) {
    const r = await fetch(url, {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify(payload || {})
    });
    if (!r.ok) throw new Error(String(r.status) + ' ' + url);
    return await r.json();
  }

  function logArea(msg, obj) {
    try {
      if (typeof ptzLog === 'function') ptzLog(msg, obj);
      else console.log(msg, obj || '');
    } catch (_) {}
  }

  async function getFrameSize() {
    try {
      const d = await getJson('/api/detections');
      const w = Number(d.width || 1920);
      const h = Number(d.height || 1080);
      return { w: Math.max(1, w), h: Math.max(1, h) };
    } catch (_) {
      return { w: 1920, h: 1080 };
    }
  }

  function makeSkyRoi(w, h) {
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

  function makeCenterRoi(w, h) {
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

  function makeFullRoi(w, h) {
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

  async function buildAreaConfig(mode) {
    const { w, h } = await getFrameSize();

    if (mode === 'full_frame') {
      return { detection_mode: 'full_frame', rois: [] };
    }

    if (mode === 'tiled') {
      return { detection_mode: 'tiled', rois: [] };
    }

    if (mode === 'roi') {
      return {
        detection_mode: 'roi',
        rois: [makeSkyRoi(w, h)]
      };
    }

    if (mode === 'multi_roi') {
      return {
        detection_mode: 'multi_roi',
        rois: [
          makeSkyRoi(w, h),
          makeCenterRoi(w, h)
        ]
      };
    }

    if (mode === 'hybrid') {
      return {
        detection_mode: 'hybrid',
        rois: [
          makeFullRoi(w, h),
          makeSkyRoi(w, h)
        ]
      };
    }

    return { detection_mode: 'full_frame', rois: [] };
  }

  async function applyDetectionAreaMode(mode) {
    const payload = await buildAreaConfig(mode);
    const res = await postJson('/api/detection/roi_config', payload);

    localStorage.setItem('operatorDetectionAreaMode', mode);

    const state = $('operatorDetectionAreaState');
    if (state) {
      const rois = Array.isArray(res.rois) ? res.rois.length : 0;
      state.textContent = 'area: ' + (res.detection_mode || mode) + ' rois=' + rois;
    }

    logArea('DETECTION AREA MODE', { mode, payload, res });
    return res;
  }

  async function loadDetectionAreaMode() {
    try {
      const cfg = await getJson('/api/detection/roi_config');
      const mode = cfg.detection_mode || localStorage.getItem('operatorDetectionAreaMode') || 'full_frame';

      if ($('operatorDetectionAreaMode')) {
        $('operatorDetectionAreaMode').value = mode;
      }

      if ($('operatorDetectionAreaState')) {
        const rois = Array.isArray(cfg.rois) ? cfg.rois.length : 0;
        $('operatorDetectionAreaState').textContent = 'area: ' + mode + ' rois=' + rois;
      }
    } catch (e) {
      if ($('operatorDetectionAreaState')) {
        $('operatorDetectionAreaState').textContent = 'area: error';
      }
      logArea('DETECTION AREA LOAD error', { error: String(e.message || e) });
    }
  }

  function initDetectionAreaModeUi() {
    const btn = $('operatorDetectionAreaApplyBtn');
    const sel = $('operatorDetectionAreaMode');

    if (btn && sel) {
      btn.onclick = () => {
        applyDetectionAreaMode(sel.value).catch(e =>
          logArea('DETECTION AREA APPLY error', { error: String(e.message || e) })
        );
      };
    }

    loadDetectionAreaMode();
  }

  if (document.readyState === 'loading') {
    document.addEventListener('DOMContentLoaded', initDetectionAreaModeUi);
  } else {
    initDetectionAreaModeUi();
  }
})();
// DETECTION_AREA_MODE_UI_END
</script>



<!-- FORCE_VIDEO_BOOTSTRAP_START -->
'''

if 'DETECTION_AREA_MODE_UI_START' not in s:
    if anchor not in s:
        raise SystemExit("ERROR: closing script anchor not found")
    s = s.replace(anchor, js, 1)
    print("OK: inserted detection area JS")
else:
    print("SKIP: detection area JS already exists")

p.write_text(s, encoding="utf-8")
print("DONE")
