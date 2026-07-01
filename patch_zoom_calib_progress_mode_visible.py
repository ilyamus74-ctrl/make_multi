from pathlib import Path
import shutil, time

p = Path("mjpeg_gst_http.cpp")
s = p.read_text(encoding="utf-8")

backup = p.with_suffix(f".cpp.bak_zoom_progress_mode_visible_{int(time.time())}")
shutil.copy2(p, backup)
print("Backup:", backup)

changed = False

# 1. Add progress globals.
old_globals = """static int g_zoomCalibProgressIdx = 0;
static int g_zoomCalibProgressSamples = 0;
static double g_zoomCalibProgressRatio = 0.0;
static int g_zoomCalibProgressTagsFound = 0;
"""

new_globals = """static int g_zoomCalibProgressIdx = 0;
static int g_zoomCalibProgressSamples = 0;
static double g_zoomCalibProgressRatio = 0.0;
static int g_zoomCalibProgressTagsFound = 0;
static std::string g_zoomCalibProgressMoveMode = "idle";
static int g_zoomCalibProgressMoveDelta = 0;
static int g_zoomCalibProgressMoveHoldMs = 0;
static int g_zoomCalibProgressCurrentIdx = 0;
static int g_zoomCalibProgressTargetIdx = 0;
static int g_zoomCalibProgressFullSweepMs = 0;
static double g_zoomCalibProgressStepMs = 0.0;
"""

if "g_zoomCalibProgressMoveMode" not in s:
    if old_globals not in s:
        raise SystemExit("ERROR: progress globals block not found")
    s = s.replace(old_globals, new_globals, 1)
    print("OK: added zoom movement progress globals")
    changed = True
else:
    print("SKIP: movement progress globals already present")

# 2. Expose progress fields in zoom_calibration_json().
old_json = """      << ",\"progress\":{\"idx\":" << g_zoomCalibProgressIdx
      << ",\"samples\":" << g_zoomCalibProgressSamples
      << ",\"zoom_ratio\":" << std::fixed << std::setprecision(3) << g_zoomCalibProgressRatio
      << ",\"tags_found\":" << g_zoomCalibProgressTagsFound << "},"
"""

new_json = """      << ",\"zoom_move_mode\":\"" << json_escape(g_zoomCalibSettings.zoom_move_mode) << "\""
      << ",\"full_sweep_ms\":" << g_zoomCalibSettings.full_sweep_ms
      << ",\"samples\":" << g_zoomCalibSettings.samples
      << ",\"progress\":{\"idx\":" << g_zoomCalibProgressIdx
      << ",\"samples\":" << g_zoomCalibProgressSamples
      << ",\"zoom_ratio\":" << std::fixed << std::setprecision(3) << g_zoomCalibProgressRatio
      << ",\"tags_found\":" << g_zoomCalibProgressTagsFound
      << ",\"mode\":\"" << json_escape(g_zoomCalibProgressMoveMode) << "\""
      << ",\"move_delta\":" << g_zoomCalibProgressMoveDelta
      << ",\"move_hold_ms\":" << g_zoomCalibProgressMoveHoldMs
      << ",\"current_idx\":" << g_zoomCalibProgressCurrentIdx
      << ",\"target_idx\":" << g_zoomCalibProgressTargetIdx
      << ",\"full_sweep_ms\":" << g_zoomCalibProgressFullSweepMs
      << ",\"step_ms\":" << std::fixed << std::setprecision(1) << g_zoomCalibProgressStepMs
      << "},"
"""

if "\"move_hold_ms\"" not in s[s.find("static std::string zoom_calibration_json"):s.find("static bool extract_json_double_field")]:
    if old_json not in s:
        raise SystemExit("ERROR: zoom_calibration_json progress block not found")
    s = s.replace(old_json, new_json, 1)
    print("OK: /api/zoom_calibration now exposes movement progress")
    changed = True
else:
    print("SKIP: movement progress already exposed")

# 3. Set progress fields inside sweep loop.
old_sweep_set = """      rows.push_back(m); moveHoldMs.push_back(holdMs); moveDelta.push_back(delta); moveCurrentIdx.push_back(beforeIdx); moveTargetIdx.push_back(i);
      g_zoomCalibProgressIdx = i; g_zoomCalibProgressSamples = sampleCount; g_zoomCalibProgressTagsFound = m.tags_found;
"""

new_sweep_set = """      rows.push_back(m); moveHoldMs.push_back(holdMs); moveDelta.push_back(delta); moveCurrentIdx.push_back(beforeIdx); moveTargetIdx.push_back(i);
      g_zoomCalibProgressIdx = i;
      g_zoomCalibProgressSamples = sampleCount;
      g_zoomCalibProgressTagsFound = m.tags_found;
      g_zoomCalibProgressMoveMode = "sweep_time_steps";
      g_zoomCalibProgressMoveDelta = delta;
      g_zoomCalibProgressMoveHoldMs = holdMs;
      g_zoomCalibProgressCurrentIdx = beforeIdx;
      g_zoomCalibProgressTargetIdx = i;
      g_zoomCalibProgressFullSweepMs = params.full_sweep_ms;
      g_zoomCalibProgressStepMs = static_cast<double>(params.full_sweep_ms) / static_cast<double>(std::max(1, params.samples - 1));
"""

if "g_zoomCalibProgressMoveMode = \"sweep_time_steps\";" not in s:
    if old_sweep_set not in s:
        raise SystemExit("ERROR: sweep progress set block not found")
    s = s.replace(old_sweep_set, new_sweep_set, 1)
    print("OK: sweep loop now sets movement progress")
    changed = True
else:
    print("SKIP: sweep progress already set")

# 4. Set progress fields inside legacy loop.
old_legacy_set = """      rows.push_back(m); moveHoldMs.push_back(params.impulse_ms); moveDelta.push_back(1); moveCurrentIdx.push_back(i-1); moveTargetIdx.push_back(i);
      g_zoomCalibProgressIdx = i; g_zoomCalibProgressSamples = sampleCount; g_zoomCalibProgressTagsFound = m.tags_found;
"""

new_legacy_set = """      rows.push_back(m); moveHoldMs.push_back(params.impulse_ms); moveDelta.push_back(1); moveCurrentIdx.push_back(i-1); moveTargetIdx.push_back(i);
      g_zoomCalibProgressIdx = i;
      g_zoomCalibProgressSamples = sampleCount;
      g_zoomCalibProgressTagsFound = m.tags_found;
      g_zoomCalibProgressMoveMode = "legacy_impulse";
      g_zoomCalibProgressMoveDelta = 1;
      g_zoomCalibProgressMoveHoldMs = params.impulse_ms;
      g_zoomCalibProgressCurrentIdx = i - 1;
      g_zoomCalibProgressTargetIdx = i;
      g_zoomCalibProgressFullSweepMs = params.full_sweep_ms;
      g_zoomCalibProgressStepMs = static_cast<double>(params.full_sweep_ms) / static_cast<double>(std::max(1, params.samples - 1));
"""

if "g_zoomCalibProgressMoveMode = \"legacy_impulse\";" not in s:
    if old_legacy_set not in s:
        raise SystemExit("ERROR: legacy progress set block not found")
    s = s.replace(old_legacy_set, new_legacy_set, 1)
    print("OK: legacy loop now sets movement progress")
    changed = True
else:
    print("SKIP: legacy progress already set")

if changed:
    p.write_text(s, encoding="utf-8")
    print("DONE")
else:
    print("No changes")

