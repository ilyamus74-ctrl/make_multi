from pathlib import Path
import shutil

p = Path("web/index.html")
s = p.read_text(encoding="utf-8")

backup = p.with_suffix(".html.bak_ptz_apply_on_change")
if not backup.exists():
    shutil.copy2(p, backup)
    print(f"backup: {backup}")

# 1. Add helper to sync UI from autopilot state.
marker = '''async function ptzApplyConfig() {'''
insert = r'''
function syncPtzConfigUiFromState(ap) {
  if (!ap || ap.ok !== true) return;

  if ($('ptzInvertTilt')) $('ptzInvertTilt').checked = Boolean(ap.invert_tilt);
  if ($('ptzInvertPan')) $('ptzInvertPan').checked = Boolean(ap.invert_pan);

  if ($('ptzTargetX') && Number.isFinite(Number(ap.target_x))) {
    $('ptzTargetX').value = Number(ap.target_x).toFixed(2);
  }
  if ($('ptzTargetY') && Number.isFinite(Number(ap.target_y))) {
    $('ptzTargetY').value = Number(ap.target_y).toFixed(2);
  }
  if ($('ptzMinPan') && Number.isFinite(Number(ap.min_pan))) {
    $('ptzMinPan').value = String(Number(ap.min_pan));
  }
  if ($('ptzMinTilt') && Number.isFinite(Number(ap.min_tilt))) {
    $('ptzMinTilt').value = String(Number(ap.min_tilt));
  }
}

async function refreshPtzConfigFromServer() {
  const ap = await apiGetJson(`${autopilotBaseUrl()}/api/autopilot/state`);
  syncPtzConfigUiFromState(ap);
  return ap;
}

'''
if insert.strip() not in s:
    s = s.replace(marker, insert + "\n" + marker, 1)

# 2. In ptzPrecheck(), after autopilot state fetch, sync the UI.
old = '''    const ap = await apiGetJson(`${autopilotBaseUrl()}/api/autopilot/state`);
    const roi = await apiGetJson('/api/detection/roi_config');'''

new = '''    const ap = await apiGetJson(`${autopilotBaseUrl()}/api/autopilot/state`);
    syncPtzConfigUiFromState(ap);
    const roi = await apiGetJson('/api/detection/roi_config');'''

if old not in s:
    print("WARN: ptzPrecheck autopilot sync pattern not found")
else:
    s = s.replace(old, new, 1)

# 3. Add onchange handlers for PTZ config controls.
old = '''  $('ptzApplyConfigBtn').onclick = () =>
    ptzApplyConfig().catch(e => ptzLog('PTZ CONFIG error', { error: String(e.message || e) }));'''

new = '''  $('ptzApplyConfigBtn').onclick = () =>
    ptzApplyConfig().catch(e => ptzLog('PTZ CONFIG error', { error: String(e.message || e) }));

  ['ptzInvertTilt','ptzInvertPan','ptzTargetX','ptzTargetY','ptzMinPan','ptzMinTilt'].forEach(id => {
    if ($(id)) {
      $(id).onchange = () =>
        ptzApplyConfig().catch(e => ptzLog('PTZ CONFIG onchange error', {
          field: id,
          error: String(e.message || e)
        }));
    }
  });'''

if old not in s:
    raise SystemExit("ptzApplyConfigBtn handler block not found")
s = s.replace(old, new, 1)

# 4. On init, load current autopilot config into GUI.
old = '''  refreshDetectionLimits().catch(e => ptzLog('LIMIT init error', { error: String(e.message || e) }));
  refreshDetectionThrottle().catch(e => ptzLog('THROTTLE init error', { error: String(e.message || e) }));'''

new = '''  refreshDetectionLimits().catch(e => ptzLog('LIMIT init error', { error: String(e.message || e) }));
  refreshDetectionThrottle().catch(e => ptzLog('THROTTLE init error', { error: String(e.message || e) }));
  refreshPtzConfigFromServer().catch(e => ptzLog('PTZ CONFIG init error', { error: String(e.message || e) }));'''

if old not in s:
    print("WARN: init refresh insertion pattern not found")
else:
    s = s.replace(old, new, 1)

p.write_text(s, encoding="utf-8")
print("patched web/index.html")
