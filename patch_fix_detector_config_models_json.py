from pathlib import Path
import shutil
import time

p = Path("mjpeg_gst_http.cpp")
s = p.read_text(encoding="utf-8")

backup = p.with_suffix(f".cpp.bak_fix_detector_config_models_json_{int(time.time())}")
shutil.copy2(p, backup)
print(f"backup: {backup}")

old = '''      os << "{\\"ok\\":true"
         << ",\\"detect_enabled\\":" << (g_detectEnabled.load() ? "true" : "false")
         << ",\\"current_model\\":\\"" << json_escape(g_currentModel) << "\\""
         << ",\\"selected_classes\\":[";
         << ",\\"models\\":[";
      if (!g_availableModels.empty()) {
        for (size_t i = 0; i < g_availableModels.size(); ++i) {
          if (i) os << ",";
          os << "\\"" << json_escape(g_availableModels[i]) << "\\"";
        }
      } else if (!g_currentModel.empty()) {
        os << "\\"" << json_escape(g_currentModel) << "\\"";
      }
      os
         << "]"
      bool first = true;
      for (int cls : g_selectedClasses) {
        if (!first) os << ",";
        os << cls;
        first = false;
      }
      os << "]}";'''

new = '''      os << "{\\"ok\\":true"
         << ",\\"detect_enabled\\":" << (g_detectEnabled.load() ? "true" : "false")
         << ",\\"current_model\\":\\"" << json_escape(g_currentModel) << "\\""
         << ",\\"models\\":[";

      if (!g_availableModels.empty()) {
        for (size_t i = 0; i < g_availableModels.size(); ++i) {
          if (i) os << ",";
          os << "\\"" << json_escape(g_availableModels[i]) << "\\"";
        }
      } else if (!g_currentModel.empty()) {
        os << "\\"" << json_escape(g_currentModel) << "\\"";
      }

      os << "],\\"selected_classes\\":[";

      bool first = true;
      for (int cls : g_selectedClasses) {
        if (!first) os << ",";
        os << cls;
        first = false;
      }

      os << "]}";'''

if old not in s:
    raise SystemExit("ERROR: broken detector config JSON block not found")

s = s.replace(old, new, 1)
p.write_text(s, encoding="utf-8")

print("OK: fixed /api/detector/config GET JSON: models[] before selected_classes[]")
