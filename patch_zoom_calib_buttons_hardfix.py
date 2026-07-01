from pathlib import Path
import shutil, time

p = Path("web/index.html")
s = p.read_text(encoding="utf-8")

backup = p.with_suffix(f".html.bak_zoom_calib_buttons_hardfix_{int(time.time())}")
shutil.copy2(p, backup)
print("Backup:", backup)

block = r'''
<script>
// ZOOM_CALIB_BUTTONS_HARDFIX_START

function getActiveZoomAnchorFromUi() {
  const defs = [
    { label: 'near', prefix: 'Near', defaultDistanceM: 1, defaultTagId: 7 },
    { label: 'mid',  prefix: 'Mid',  defaultDistanceM: 5, defaultTagId: -1 },
    { label: 'far',  prefix: 'Far',  defaultDistanceM: 10, defaultTagId: 12 }
  ];

  const $ = (id) => document.getElementById(id);

  let activeLabel = 'near';
  try {
    const txt = String($('zoomActiveAnchorSummary')?.textContent || '');
    const m = txt.match(/active=([a-z]+)/i);
    if (m) activeLabel = m[1].toLowerCase();
  } catch (_) {}

  const anchors = defs.map(def => ({
    label: def.label,
    enabled: !!$(`zoomAnchor${def.prefix}Enabled`)?.checked,
    tag_id: Number($(`zoomAnchor${def.prefix}TagId`)?.value ?? def.defaultTagId),
    distance_mm: Number($(`zoomAnchor${def.prefix}DistanceM`)?.value || def.defaultDistanceM) * 1000,
    tag_size_mm: Number($(`zoomAnchor${def.prefix}SizeMm`)?.value || 160)
  }));

  return anchors.find(a => a.label === activeLabel && a.enabled) ||
    anchors.find(a => a.enabled) ||
    anchors.find(a => a.label === activeLabel) ||
    anchors[0];
}

(function () {
  const $ = (id) => document.getElementById(id);

  function setStatus(txt) {
    if ($('zoomAprilTagCalibStatus')) $('zoomAprilTagCalibStatus').textContent = txt;
  }

  function defs() {
    return [
      { label: 'near', prefix: 'Near', defaultDistanceM: 1, defaultTagId: 7 },
      { label: 'mid',  prefix: 'Mid',  defaultDistanceM: 5, defaultTagId: -1 },
      { label: 'far',  prefix: 'Far',  defaultDistanceM: 10, defaultTagId: 12 }
    ];
  }

  function activeLabelFromUi() {
    const txt = String($('zoomActiveAnchorSummary')?.textContent || '');
    const m = txt.match(/active=([a-z]+)/i);
    return m ? m[1].toLowerCase() : 'near';
  }

  function readAnchor(def) {
    return {
      label: def.label,
      enabled: !!$(`zoomAnchor${def.prefix}Enabled`)?.checked,
      tag_id: Number($(`zoomAnchor${def.prefix}TagId`)?.value ?? def.defaultTagId),
      distance_mm: Number($(`zoomAnchor${def.prefix}DistanceM`)?.value || def.defaultDistanceM) * 1000,
      tag_size_mm: Number($(`zoomAnchor${def.prefix}SizeMm`)?.value || 160)
    };
  }

  function readPayload() {
    const anchors = defs().map(readAnchor);
    const activeLabel = activeLabelFromUi();
    const active = anchors.find(a => a.label === activeLabel) || anchors[0];

    return {
      tag_size_mm: Number(active?.tag_size_mm || 160),
      anchor_tag_id: Number(active?.tag_id ?? 7),
      anchor_distance_mm: Number(active?.distance_mm || 1000),
      samples: Number($('zoomSamples')?.value || 12),
      impulse_ms: Number($('zoomImpulseMs')?.value || 170),
      settle_ms: Number($('zoomSettleMs')?.value || 190),
      cmd_abs: Number($('zoomCmdAbs')?.value || 34),
      wide_cmd_sign: Number($('zoomWideSign')?.value || -1),
      wide_hold_ms: Number($('zoomWideHoldMs')?.value || 1500),
      calibration_direction: $('zoomCalibDirection')?.value || 'wide_to_tele',
      active_anchor_label: activeLabel,
      anchors
    };
  }

  async function postJson(url, payload) {
    const r = await fetch(url, {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify(payload || {})
    });
    if (!r.ok) throw new Error(`${url} ${r.status}`);
    return await r.json().catch(() => ({ ok: true }));
  }

  async function getJson(url) {
    const r = await fetch(url, { cache: 'no-store' });
    if (!r.ok) throw new Error(`${url} ${r.status}`);
    return await r.json();
  }

  async function saveSettingsNow() {
    const payload = readPayload();
    setStatus('settings saving...');
    const res = await postJson('/api/zoom_calibration/settings', payload);
    setStatus('settings saved');
    return res;
  }

  async function runCalib() {
    await saveSettingsNow();

    const payload = readPayload();
    payload.mode = 'apriltag_zoom_table';

    setStatus('calibration starting...');

    const res = await postJson('/api/zoom_calibration', payload);

    if (res.started || res.ok) {
      setStatus('calibration running...');
    } else {
      setStatus('calibration not started');
    }

    console.log('[zoom-calib] run result', res);
    return res;
  }

  async function buildMaster() {
    setStatus('building master profile...');
    const res = await postJson('/api/zoom_calibration/build_master_profile', {});
    setStatus(res.ok ? `master ok points=${res.points_total_clean ?? res.points ?? ''}` : `master failed ${res.error || ''}`);
    console.log('[zoom-calib] build master result', res);
    return res;
  }

  async function testAprilTag() {
    setStatus('testing apriltag...');
    const res = await getJson('/api/apriltag/test?ts=' + Date.now());
    setStatus(`apriltag tags=${res.tags_found || 0} ids=${(res.ids || []).join(',') || 'none'} ${res.message || ''}`);
    console.log('[zoom-calib] apriltag test', res);
    return res;
  }

  function bindHard(id, fn) {
    const el = $(id);
    if (!el || el.__zoomHardBound) return;
    el.__zoomHardBound = true;
    el.addEventListener('click', (e) => {
      e.preventDefault();
      e.stopPropagation();
      e.stopImmediatePropagation();
      fn().catch(err => {
        console.error(`[zoom-calib] ${id} failed`, err);
        setStatus(`error: ${String(err.message || err)}`);
      });
    }, true);
  }

  function boot() {
    window.__zoomCalibButtonsHardfixInstalled = true;

    bindHard('zoomRunAprilTagCalibBtn', runCalib);
    bindHard('zoomBuildMasterProfileBtn', buildMaster);
    bindHard('apriltagTestBtn', testAprilTag);

    console.log('[zoom-calib] hardfix buttons installed');
  }

  if (document.readyState === 'loading') {
    document.addEventListener('DOMContentLoaded', boot, { once: true });
  } else {
    boot();
  }

  window.zoomCalibHardRun = runCalib;
  window.zoomCalibHardBuildMaster = buildMaster;
  window.zoomCalibHardTestAprilTag = testAprilTag;
})();

// ZOOM_CALIB_BUTTONS_HARDFIX_END
</script>
'''

if "ZOOM_CALIB_BUTTONS_HARDFIX_START" in s:
    print("SKIP: hardfix already installed")
else:
    if "</body>" in s:
        s = s.replace("</body>", block + "\n</body>", 1)
    elif "</html>" in s:
        s = s.replace("</html>", block + "\n</html>", 1)
    else:
        s += "\n" + block + "\n"

    p.write_text(s, encoding="utf-8")
    print("OK: hardfix inserted")
