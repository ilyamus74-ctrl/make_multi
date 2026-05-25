from pathlib import Path
import re
import shutil

p = Path("mjpeg_gst_http.cpp")
s = p.read_text(encoding="utf-8")

backup = p.with_suffix(".cpp.bak_fix_zoom_runtime_state_literal")
if not backup.exists():
    shutil.copy2(p, backup)
    print(f"backup: {backup}")

pattern = re.compile(
    r"static void save_zoom_runtime_state\(\)\s*\{.*?\}\s*static void load_zoom_runtime_state",
    re.S
)

replacement = r'''static void save_zoom_runtime_state() {
  std::lock_guard<std::mutex> lk(g_zoomRuntimeMtx);

  std::ofstream out(g_zoomRuntimeStateFile, std::ios::trunc);
  if (!out.good()) return;

  const double zr = g_zoomRatio.load();
  const double fp = zoom_profile_focal_for_ratio(zr);

  out << "{"
      << "\"version\":1"
      << ",\"zoom_sample_idx\":" << g_zoomSampleIdx.load()
      << ",\"zoom_ratio\":" << zr
      << ",\"focal_px\":" << fp
      << ",\"zoom_confidence\":" << g_zoomConfidence.load()
      << ",\"zoom_source\":\"" << json_escape(g_zoomSource) << "\""
      << ",\"steps_since_home\":" << g_zoomStepsSinceHome.load()
      << ",\"last_home_at_ms\":" << g_zoomLastHomeMs.load()
      << ",\"last_move_at_ms\":" << g_zoomLastMoveMs.load()
      << "}";
}

static void load_zoom_runtime_state'''

s2, n = pattern.subn(replacement, s, count=1)

if n != 1:
    raise SystemExit(f"ERROR: save_zoom_runtime_state block not replaced, replacements={n}")

p.write_text(s2, encoding="utf-8")
print("OK: fixed save_zoom_runtime_state()")
