from pathlib import Path
import shutil, time

p = Path("web/index.html")
s = p.read_text(encoding="utf-8")

backup = p.with_suffix(f".html.bak_apriltag_overlay_only_zoom_calib_{int(time.time())}")
shutil.copy2(p, backup)
print("Backup:", backup)

# 1) CSS: overlay hidden by default, visible only when body has zoom-calib-active
old_css = ".apriltag-overlay{position:absolute;inset:0;width:100%;height:100%;pointer-events:none;z-index:3}"
new_css = """.apriltag-overlay{position:absolute;inset:0;width:100%;height:100%;pointer-events:none;z-index:3;display:none}
body.zoom-calib-active .apriltag-overlay{display:block}"""

if old_css in s:
    s = s.replace(old_css, new_css, 1)
    print("OK: CSS overlay visibility patched")
elif "body.zoom-calib-active .apriltag-overlay" in s:
    print("SKIP: CSS already patched")
else:
    raise SystemExit("ERROR: apriltag-overlay CSS anchor not found")

# 2) Insert controller helpers after latestAprilTag
old = '''  let latestAprilTag = null;
'''
new = '''  let latestAprilTag = null;

  function isZoomCalibOverlayWanted() {
    const menuOpen = !!$('zoomCalibMenu')?.open;
    return menuOpen || running;
  }

  function clearAprilTagOverlay() {
    const canvas = $('apriltagOverlay');
    if (!canvas) return;
    const ctx = canvas.getContext('2d');
    if (ctx) ctx.clearRect(0, 0, canvas.width || 1, canvas.height || 1);
  }

  function updateAprilTagOverlayVisibility() {
    const want = isZoomCalibOverlayWanted();
    document.body.classList.toggle('zoom-calib-active', want);
    if (!want) clearAprilTagOverlay();
    else if (latestAprilTag) drawAprilTags(latestAprilTag);
    return want;
  }

'''
if old in s and "function isZoomCalibOverlayWanted()" not in s:
    s = s.replace(old, new, 1)
    print("OK: overlay visibility helpers inserted")
elif "function isZoomCalibOverlayWanted()" in s:
    print("SKIP: overlay helpers already exist")
else:
    raise SystemExit("ERROR: latestAprilTag anchor not found")

# 3) updateDebugImage should only update when overlay wanted
old = '''  function updateDebugImage() { if ($('apriltagDebugImage')) $('apriltagDebugImage').src = `/debug_apriltag_latest.jpg?ts=${Date.now()}`; }
'''
new = '''  function updateDebugImage() {
    if (!isZoomCalibOverlayWanted()) return;
    if ($('apriltagDebugImage')) $('apriltagDebugImage').src = `/debug_apriltag_latest.jpg?ts=${Date.now()}`;
  }
'''
if old in s:
    s = s.replace(old, new, 1)
    print("OK: updateDebugImage gated")
elif "if (!isZoomCalibOverlayWanted()) return;" in s:
    print("SKIP: updateDebugImage already gated")
else:
    raise SystemExit("ERROR: updateDebugImage anchor not found")

# 4) drawAprilTags should clear/return if not wanted
old = '''  function drawAprilTags(res) {
    const canvas = $('apriltagOverlay'); const wrap = $('streamWrap'); if (!canvas || !wrap) return;
'''
new = '''  function drawAprilTags(res) {
    if (!isZoomCalibOverlayWanted()) { clearAprilTagOverlay(); return; }
    const canvas = $('apriltagOverlay'); const wrap = $('streamWrap'); if (!canvas || !wrap) return;
'''
if old in s:
    s = s.replace(old, new, 1)
    print("OK: drawAprilTags gated")
elif "if (!isZoomCalibOverlayWanted()) { clearAprilTagOverlay(); return; }" in s:
    print("SKIP: drawAprilTags already gated")
else:
    raise SystemExit("ERROR: drawAprilTags anchor not found")

# 5) startPolling should activate overlay
old = '''  function startPolling() {
    stopPolling(false); running = true; setRunDisabled(true); status('calibration running...');
'''
new = '''  function startPolling() {
    stopPolling(false); running = true; setRunDisabled(true); updateAprilTagOverlayVisibility(); status('calibration running...');
'''
if old in s:
    s = s.replace(old, new, 1)
    print("OK: startPolling activates overlay")
elif "running = true; setRunDisabled(true); updateAprilTagOverlayVisibility();" in s:
    print("SKIP: startPolling already patched")
else:
    raise SystemExit("ERROR: startPolling anchor not found")

# 6) stopPolling should update/clear overlay
old = '''  function stopPolling(update = true) {
    if (pollTimer) clearInterval(pollTimer); if (testTimer) clearInterval(testTimer); pollTimer = null; testTimer = null; running = false; setRunDisabled(false); if (update) updateDebugImage();
  }
'''
new = '''  function stopPolling(update = true) {
    if (pollTimer) clearInterval(pollTimer);
    if (testTimer) clearInterval(testTimer);
    pollTimer = null;
    testTimer = null;
    running = false;
    setRunDisabled(false);
    updateAprilTagOverlayVisibility();
    if (update) updateDebugImage();
  }
'''
if old in s:
    s = s.replace(old, new, 1)
    print("OK: stopPolling clears overlay when needed")
elif "updateAprilTagOverlayVisibility();" in s and "function stopPolling(update = true)" in s:
    print("SKIP: stopPolling may already be patched")
else:
    raise SystemExit("ERROR: stopPolling anchor not found")

# 7) boot: bind details toggle + initial visibility
old = '''  function boot() {
    document.addEventListener('input', e => { if (isField(e.target)) scheduleSave(); }, true);
'''
new = '''  function boot() {
    updateAprilTagOverlayVisibility();

    const zoomMenu = $('zoomCalibMenu');
    if (zoomMenu && !zoomMenu.__apriltagOverlayToggleBound) {
      zoomMenu.__apriltagOverlayToggleBound = true;
      zoomMenu.addEventListener('toggle', () => {
        updateAprilTagOverlayVisibility();
        if (zoomMenu.open && latestAprilTag) drawAprilTags(latestAprilTag);
      });
    }

    document.addEventListener('input', e => { if (isField(e.target)) scheduleSave(); }, true);
'''
if old in s:
    s = s.replace(old, new, 1)
    print("OK: boot binds zoom menu toggle")
elif "__apriltagOverlayToggleBound" in s:
    print("SKIP: boot already has menu toggle")
else:
    raise SystemExit("ERROR: boot anchor not found")

p.write_text(s, encoding="utf-8")
print("DONE")
