from pathlib import Path
import shutil, time

p = Path("mjpeg_gst_http.cpp")
s = p.read_text(encoding="utf-8")

backup = p.with_suffix(f".cpp.bak_startup_home_wide_{int(time.time())}")
shutil.copy2(p, backup)
print("Backup:", backup)

changed = False

helper_marker = "static void http_server_thread(const Opts& o) {"

helper = r'''
static bool zoom_home_wide_on_startup(const Opts& o) {
  ZoomAprilTagCalibParams p = g_zoomCalibSettings;
  load_zoom_calib_settings(p, o.cmd_max_zoom);
  clamp_zoom_calib_params(p, o.cmd_max_zoom);

  const int cmd = std::max(1, std::min(o.cmd_max_zoom, p.cmd_abs));
  const int wideCmd = p.wide_cmd_sign * cmd;

  // User requirement: on system startup, force the optical zoom to WIDE edge
  // using the full sweep duration from ZOOM CALIB.
  const int holdMs = std::max(300, std::min(20000, p.full_sweep_ms));

  std::cout << "zoom-startup-home-wide: cmd=" << wideCmd
            << " hold_ms=" << holdMs
            << " samples=" << p.samples
            << " mode=" << p.zoom_move_mode
            << "\n";

  const bool ok = send_zoom_stream_via_bridge_ws(wideCmd, holdMs);

  if (ok) {
    g_zoomSampleIdx.store(0);

    if (g_zoomSampleCount.load() <= 0) {
      g_zoomSampleCount.store(std::max(1, p.samples));
    }

    g_zoomRatio.store(0.0);
    g_zoomConfidence.store(1.0);
    g_zoomStepsSinceHome.store(0);

    const int64_t ts = now_ms();
    g_zoomLastHomeMs.store(ts);
    g_zoomLastMoveMs.store(ts);

    {
      std::lock_guard<std::mutex> lk(g_zoomMasterProfileMtx);
      g_zoomSource = "startup_wide_home";
    }

    save_zoom_runtime_state();

    std::cout << "zoom-startup-home-wide: ok sample_idx=0 ratio=0 source=startup_wide_home\n";
  } else {
    std::cerr << "zoom-startup-home-wide: failed\n";
  }

  return ok;
}

'''

if "zoom-startup-home-wide:" not in s:
    if helper_marker not in s:
        raise SystemExit("ERROR: helper insert marker not found")
    s = s.replace(helper_marker, helper + "\n" + helper_marker, 1)
    print("OK: inserted zoom_home_wide_on_startup helper")
    changed = True
else:
    print("SKIP: startup wide helper already present")

old_main = """  std::this_thread::sleep_for(std::chrono::milliseconds(600));
  run_zoom_image_time_calibration(o);
  std::thread t_zoom_recalib(zoom_background_recalibration_thread, o);
"""

new_main = """  std::this_thread::sleep_for(std::chrono::milliseconds(600));
  run_zoom_image_time_calibration(o);

  // After startup zoom calibration, force known WIDE home position and confirm sample 0.
  zoom_home_wide_on_startup(o);

  std::thread t_zoom_recalib(zoom_background_recalibration_thread, o);
"""

if "force known WIDE home position and confirm sample 0" not in s:
    if old_main not in s:
        raise SystemExit("ERROR: main startup block not found")
    s = s.replace(old_main, new_main, 1)
    print("OK: startup now homes zoom to WIDE after initial calibration")
    changed = True
else:
    print("SKIP: startup call already present")

if changed:
    p.write_text(s, encoding="utf-8")
    print("DONE")
else:
    print("No changes")
