from pathlib import Path
import shutil, time

p = Path("mjpeg_gst_http.cpp")
s = p.read_text(encoding="utf-8")

backup = p.with_suffix(f".cpp.bak_go_to_sample_sweep_{int(time.time())}")
shutil.copy2(p, backup)
print("Backup:", backup)

changed = False

old = """    const int from = g_zoomSampleIdx.load();
    const int delta = target - from;

    const int maxIdx = std::max(0, zoom_max_sample_idx());
    const bool edgeWide = (target == 0);
    const bool edgeTele = (target == maxIdx);

    if (ok && edgeWide) {
      ok = zoom_send_stream_cmd(zoom_wide_cmd(), zoom_wide_hold_ms(), &err);
      mode = "edge_wide";
    } else if (ok && edgeTele) {
      ok = zoom_send_stream_cmd(zoom_tele_cmd(), zoom_wide_hold_ms(), &err);
      mode = "edge_tele";
    } else if (ok && mode == "absolute_wide") {
      ok = zoom_send_stream_cmd(zoom_wide_cmd(), zoom_wide_hold_ms(), &err);
      if (ok && target > 0) ok = zoom_move_steps(target, &err);
    } else if (ok) {
      if (delta != 0) ok = zoom_move_steps(delta, &err);
    }
"""

new = """    const int from = g_zoomSampleIdx.load();
    const int delta = target - from;

    ZoomAprilTagCalibParams moveParams = g_zoomCalibSettings;
    load_zoom_calib_settings(moveParams, g_cmdMaxZoom);
    clamp_zoom_calib_params(moveParams, g_cmdMaxZoom);

    const bool sweepSampleMove =
      (moveParams.zoom_move_mode == "sweep_time_steps") &&
      (mode == "relative" || mode == "sample" || mode == "sweep_time_steps");

    int moveHoldMs = 0;
    double moveStepMs = static_cast<double>(moveParams.full_sweep_ms) /
      static_cast<double>(std::max(1, moveParams.samples - 1));

    const int maxIdx = std::max(0, zoom_max_sample_idx());
    const bool edgeWide = (target == 0);
    const bool edgeTele = (target == maxIdx);

    if (ok && sweepSampleMove) {
      bool moveOk = true;
      int holdMs = 0;

      const int newIdx = move_zoom_by_sample_delta_sweep_time(
        from,
        target,
        std::max(1, moveParams.samples),
        moveParams.full_sweep_ms,
        std::max(1, std::min(g_cmdMaxZoom, moveParams.cmd_abs)),
        moveParams.calibration_direction,
        moveParams.wide_cmd_sign,
        moveParams.settle_ms,
        &moveOk,
        &holdMs
      );

      moveHoldMs = holdMs;
      ok = moveOk && (newIdx == target);

      if (!ok) err = "sweep_time_move_failed";
      mode = "sweep_time_steps";

      std::cout << "zoom-go-to-sample: mode=sweep_time_steps from=" << from
                << " target=" << target
                << " delta=" << delta
                << " hold_ms=" << moveHoldMs
                << " step_ms=" << std::fixed << std::setprecision(1) << moveStepMs
                << " full_sweep_ms=" << moveParams.full_sweep_ms
                << " samples=" << moveParams.samples
                << " ok=" << (ok ? 1 : 0)
                << "\\n";
    } else if (ok && edgeWide) {
      ok = zoom_send_stream_cmd(zoom_wide_cmd(), zoom_wide_hold_ms(), &err);
      mode = "edge_wide";
      moveHoldMs = zoom_wide_hold_ms();
    } else if (ok && edgeTele) {
      ok = zoom_send_stream_cmd(zoom_tele_cmd(), zoom_wide_hold_ms(), &err);
      mode = "edge_tele";
      moveHoldMs = zoom_wide_hold_ms();
    } else if (ok && mode == "absolute_wide") {
      ok = zoom_send_stream_cmd(zoom_wide_cmd(), zoom_wide_hold_ms(), &err);
      moveHoldMs = zoom_wide_hold_ms();
      if (ok && target > 0) ok = zoom_move_steps(target, &err);
    } else if (ok) {
      if (delta != 0) ok = zoom_move_steps(delta, &err);
      mode = "legacy_impulse";
    }
"""

if "zoom-go-to-sample: mode=sweep_time_steps" not in s:
    if old not in s:
        raise SystemExit("ERROR: /api/zoom/go_to_sample movement block not found")
    s = s.replace(old, new, 1)
    print("OK: /api/zoom/go_to_sample now uses sweep_time_steps when configured")
    changed = True
else:
    print("SKIP: go_to_sample sweep patch already installed")

# Add response diagnostics.
old_resp = """       os << ",\\"zoom_ratio\\":" << point.zoom_ratio
          << ",\\"focal_px\\":" << point.focal_px
          << ",\\"steps_since_home\\":" << g_zoomStepsSinceHome.load();
"""

new_resp = """       os << ",\\"zoom_ratio\\":" << point.zoom_ratio
          << ",\\"focal_px\\":" << point.focal_px
          << ",\\"steps_since_home\\":" << g_zoomStepsSinceHome.load()
          << ",\\"move_hold_ms\\":" << moveHoldMs
          << ",\\"step_ms\\":" << std::fixed << std::setprecision(1) << moveStepMs
          << ",\\"zoom_move_mode\\":\\"" << json_escape(moveParams.zoom_move_mode) << "\\""
          << ",\\"full_sweep_ms\\":" << moveParams.full_sweep_ms;
"""

if '"move_hold_ms"' not in s[s.find('if (path == "/api/zoom/go_to_sample")'):s.find('if (path == "/api/zoom_calibration")') if 'if (path == "/api/zoom_calibration")' in s[s.find('if (path == "/api/zoom/go_to_sample")'):] else len(s)]:
    if old_resp not in s:
        print("WARN: response diagnostic block not found; movement patch still applied")
    else:
        s = s.replace(old_resp, new_resp, 1)
        print("OK: go_to_sample response now includes move_hold_ms/step_ms/zoom_move_mode")
        changed = True
else:
    print("SKIP: go_to_sample response diagnostics already present")

if changed:
    p.write_text(s, encoding="utf-8")
    print("DONE")
else:
    print("No changes")

