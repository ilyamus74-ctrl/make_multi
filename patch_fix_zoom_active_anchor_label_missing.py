from pathlib import Path
import shutil, time, re

p = Path("web/index.html")
s = p.read_text(encoding="utf-8")

backup = p.with_suffix(f".html.bak_fix_zoom_active_anchor_label_{int(time.time())}")
shutil.copy2(p, backup)
print("Backup:", backup)

changed = False

decl_re = re.compile(r"\b(?:let|var|const)\s+zoomActiveAnchorLabel\b")

if not decl_re.search(s):
    marker = "  function updateZoomActiveAnchorSummary() {"
    insert = """  // Active ZOOM CALIB anchor state.
  // Required by autosave/load/settings code.
  let zoomActiveAnchorLabel = 'near';

"""
    if marker not in s:
        raise SystemExit("ERROR: updateZoomActiveAnchorSummary marker not found")
    s = s.replace(marker, insert + marker, 1)
    print("OK: inserted missing zoomActiveAnchorLabel declaration")
    changed = True
else:
    print("SKIP: zoomActiveAnchorLabel declaration already exists")

# Make the summary function more defensive.
old = """  function updateZoomActiveAnchorSummary() {
    const a = getActiveZoomAnchorFromUi();
    if ($('zoomActiveAnchorSummary') && a) {
      $('zoomActiveAnchorSummary').textContent =
        `active=${zoomActiveAnchorLabel} id=${a.tag_id} distance=${(a.distance_mm / 1000).toFixed(2)}m size=${a.tag_size_mm}mm`;
    }
    if ($('zoomCalibMenuSummary') && a) $('zoomCalibMenuSummary').textContent = `active ${zoomActiveAnchorLabel} id=${a.tag_id}`;
  }
"""

new = """  function updateZoomActiveAnchorSummary() {
    if (typeof zoomActiveAnchorLabel === 'undefined' || !zoomActiveAnchorLabel) {
      zoomActiveAnchorLabel = 'near';
    }

    const a = getActiveZoomAnchorFromUi();

    if ($('zoomActiveAnchorSummary') && a) {
      $('zoomActiveAnchorSummary').textContent =
        `active=${zoomActiveAnchorLabel} id=${a.tag_id} distance=${(a.distance_mm / 1000).toFixed(2)}m size=${a.tag_size_mm}mm`;
    }

    if ($('zoomCalibMenuSummary') && a) {
      const mode = $('zoomMoveMode')?.value || 'legacy_impulse';
      const samples = Number($('zoomSamples')?.value || 1);
      const full = Number($('zoomFullSweepMs')?.value || $('zoomWideHoldMs')?.value || 0);
      const step = full / Math.max(1, samples - 1);

      $('zoomCalibMenuSummary').textContent =
        mode === 'sweep_time_steps'
          ? `active ${zoomActiveAnchorLabel} id=${a.tag_id} mode=sweep step=${step.toFixed(1)}ms`
          : `active ${zoomActiveAnchorLabel} id=${a.tag_id} mode=impulse`;
    }
  }
"""

if old in s:
    s = s.replace(old, new, 1)
    print("OK: made updateZoomActiveAnchorSummary defensive and mode-visible")
    changed = True
else:
    print("WARN: exact updateZoomActiveAnchorSummary block not found; declaration fix still applied")

if changed:
    p.write_text(s, encoding="utf-8")
    print("DONE")
else:
    print("No changes")
