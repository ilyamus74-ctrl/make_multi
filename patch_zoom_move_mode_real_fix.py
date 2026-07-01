from pathlib import Path
import shutil, time

def backup(path: Path, suffix: str):
    b = path.with_suffix(path.suffix + f".bak_{suffix}_{int(time.time())}")
    shutil.copy2(path, b)
    print("Backup:", b)

# -----------------------------
# 1. Patch web/index.html
# -----------------------------
web = Path("web/index.html")
s = web.read_text(encoding="utf-8")
backup(web, "zoom_move_mode_real_fix")

changed = False

old = """      wide_hold_ms: num('zoomWideHoldMs', 1500),
      calibration_direction: $('zoomCalibDirection')?.value || 'wide_to_tele',
"""

new = """      wide_hold_ms: num('zoomWideHoldMs', 1500),

      // Real movement mode for backend calibration.
      zoom_move_mode: $('zoomMoveMode')?.value || 'legacy_impulse',
      full_sweep_ms: num('zoomFullSweepMs', num('zoomWideHoldMs', 1500)),

      calibration_direction: $('zoomCalibDirection')?.value || 'wide_to_tele',
"""

if "Real movement mode for backend calibration" not in s:
    if old in s:
        s = s.replace(old, new, 1)
        print("OK: RUN FULL CALIB payload now sends zoom_move_mode/full_sweep_ms")
        changed = True
    else:
        # If previous patches partially inserted these fields, force replace common variants.
        old2 = """      zoom_move_mode: 'sweep_time_steps',
      full_sweep_ms: num('zoomFullSweepMs', num('zoomWideHoldMs', 1500)),
"""
        new2 = """      // Real movement mode for backend calibration.
      zoom_move_mode: $('zoomMoveMode')?.value || 'legacy_impulse',
      full_sweep_ms: num('zoomFullSweepMs', num('zoomWideHoldMs', 1500)),
"""
        if old2 in s:
            s = s.replace(old2, new2, 1)
            print("OK: replaced forced sweep with real selector mode")
            changed = True
        else:
            print("WARN: payload block not found in web/index.html")
else:
    print("SKIP: web payload already patched")

old_status = """      const names = anchors.map(a => `${a.label}:id=${a.tag_id}@${(a.distance_mm / 1000).toFixed(2)}m`).join(', ');
      setZoomStatus(`FULL CALIB: anchors=${names}`);

      await clearOldProfilesOnce();
"""

new_status = """      const moveMode = $('zoomMoveMode')?.value || 'legacy_impulse';
      const samples = num('zoomSamples', 10);
      const fullSweepMs = num('zoomFullSweepMs', num('zoomWideHoldMs', 1500));
      const stepMs = fullSweepMs / Math.max(1, samples - 1);

      const names = anchors.map(a => `${a.label}:id=${a.tag_id}@${(a.distance_mm / 1000).toFixed(2)}m`).join(', ');
      setZoomStatus(
        moveMode === 'sweep_time_steps'
          ? `FULL CALIB: mode=sweep_time_steps samples=${samples} full=${fullSweepMs}ms step=${stepMs.toFixed(1)}ms anchors=${names}`
          : `FULL CALIB: mode=legacy_impulse impulse=${num('zoomImpulseMs', 170)}ms samples=${samples} anchors=${names}`
      );

      await clearOldProfilesOnce();
"""

if "FULL CALIB: mode=sweep_time_steps" not in s:
    if old_status in s:
        s = s.replace(old_status, new_status, 1)
        print("OK: RUN FULL CALIB status now shows selected movement mode")
        changed = True
    else:
        print("WARN: full calib status block not found")
else:
    print("SKIP: full calib status already patched")

if changed:
    web.write_text(s, encoding="utf-8")
    print("OK: web/index.html written")
else:
    print("web/index.html: no changes")


# -----------------------------
# 2. Patch mjpeg_gst_http.cpp
# -----------------------------
cpp = Path("mjpeg_gst_http.cpp")
s = cpp.read_text(encoding="utf-8")
backup(cpp, "zoom_move_mode_real_fix")

changed = False

# 2.1 /api/zoom_calibration POST should load latest settings from file before body overrides.
old = """        const std::string mode = extract_json_string_field(bodyReq, "mode");
        ZoomAprilTagCalibParams p = g_zoomCalibSettings;
        extract_json_double_field(bodyReq, "tag_size_mm", &p.tag_size_mm);
"""

new = """        const std::string mode = extract_json_string_field(bodyReq, "mode");
        ZoomAprilTagCalibParams p = g_zoomCalibSettings;
        // Always start from persisted ZOOM CALIB settings, so UI selector survives reloads
        // and RUN FULL CALIB cannot accidentally fall back to legacy_impulse.
        load_zoom_calib_settings(p, o.cmd_max_zoom);
        extract_json_double_field(bodyReq, "tag_size_mm", &p.tag_size_mm);
"""

if "Always start from persisted ZOOM CALIB settings" not in s:
    if old not in s:
        raise SystemExit("ERROR: /api/zoom_calibration POST init block not found")
    s = s.replace(old, new, 1)
    print("OK: /api/zoom_calibration now loads persisted zoom settings first")
    changed = True
else:
    print("SKIP: /api/zoom_calibration already loads persisted settings")

# 2.2 Do not overwrite calibration_direction/active_anchor_label with empty strings in /api/zoom_calibration.
old = """        p.calibration_direction = extract_json_string_field(bodyReq, "calibration_direction");
        p.active_anchor_label = extract_json_string_field(bodyReq, "active_anchor_label");
"""

new = """        { const std::string cd = extract_json_string_field(bodyReq, "calibration_direction"); if (!cd.empty()) p.calibration_direction = cd; }
        { const std::string aal = extract_json_string_field(bodyReq, "active_anchor_label"); if (!aal.empty()) p.active_anchor_label = aal; }
"""

if old in s:
    s = s.replace(old, new, 1)
    print("OK: /api/zoom_calibration no longer clears string settings when body omits them")
    changed = True
else:
    print("SKIP/WARN: /api/zoom_calibration string overwrite block not found or already patched")

# 2.3 Do same in /api/zoom_calibration/settings POST.
old = """      p.calibration_direction = extract_json_string_field(bodyReq, "calibration_direction");
      p.active_anchor_label = extract_json_string_field(bodyReq, "active_anchor_label");
"""

new = """      { const std::string cd = extract_json_string_field(bodyReq, "calibration_direction"); if (!cd.empty()) p.calibration_direction = cd; }
      { const std::string aal = extract_json_string_field(bodyReq, "active_anchor_label"); if (!aal.empty()) p.active_anchor_label = aal; }
"""

if old in s:
    s = s.replace(old, new, 1)
    print("OK: /api/zoom_calibration/settings no longer clears string settings when body omits them")
    changed = True
else:
    print("SKIP/WARN: settings string overwrite block not found or already patched")

# 2.4 Add explicit log at start of AprilTag calibration.
old = """  const bool sweepMode = (params.zoom_move_mode == "sweep_time_steps");
  const int sampleCount = std::max(1, params.samples);
"""

new = """  const bool sweepMode = (params.zoom_move_mode == "sweep_time_steps");
  const int sampleCount = std::max(1, params.samples);
  const double stepMs = static_cast<double>(params.full_sweep_ms) / static_cast<double>(std::max(1, params.samples - 1));
  std::cout << "zoom-apriltag: START mode=" << params.zoom_move_mode
            << " sweepMode=" << (sweepMode ? 1 : 0)
            << " samples=" << sampleCount
            << " full_sweep_ms=" << params.full_sweep_ms
            << " step_ms=" << std::fixed << std::setprecision(1) << stepMs
            << " impulse_ms=" << params.impulse_ms
            << " settle_ms=" << params.settle_ms
            << " direction=" << params.calibration_direction
            << "\\n";
"""

if "zoom-apriltag: START mode=" not in s:
    if old not in s:
        raise SystemExit("ERROR: sweepMode block not found")
    s = s.replace(old, new, 1)
    print("OK: added explicit backend start log with movement mode")
    changed = True
else:
    print("SKIP: backend start log already present")

if changed:
    cpp.write_text(s, encoding="utf-8")
    print("OK: mjpeg_gst_http.cpp written")
else:
    print("mjpeg_gst_http.cpp: no changes")
