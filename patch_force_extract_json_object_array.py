from pathlib import Path
import shutil

p = Path("mjpeg_gst_http.cpp")
s = p.read_text(encoding="utf-8")

backup = p.with_suffix(".cpp.bak_force_extract_json_object_array")
if not backup.exists():
    shutil.copy2(p, backup)
    print(f"backup: {backup}")

signature = "static std::vector<std::string> extract_json_object_array("
anchor = "static bool load_zoom_master_profile() {\n"

helper = r'''
static std::vector<std::string> extract_json_object_array(const std::string& json, const std::string& key) {
  std::vector<std::string> out;

  const std::string pattern = "\"" + key + "\"";
  const size_t keyPos = json.find(pattern);
  if (keyPos == std::string::npos) return out;

  const size_t colonPos = json.find(':', keyPos + pattern.size());
  if (colonPos == std::string::npos) return out;

  const size_t arrStart = json.find('[', colonPos + 1);
  if (arrStart == std::string::npos) return out;

  bool inString = false;
  bool escaped = false;
  int depth = 0;
  size_t objStart = std::string::npos;

  for (size_t i = arrStart + 1; i < json.size(); ++i) {
    const char c = json[i];

    if (inString) {
      if (escaped) {
        escaped = false;
      } else if (c == '\\') {
        escaped = true;
      } else if (c == '"') {
        inString = false;
      }
      continue;
    }

    if (c == '"') {
      inString = true;
      continue;
    }

    if (c == '{') {
      if (depth == 0) objStart = i;
      depth++;
      continue;
    }

    if (c == '}') {
      if (depth > 0) {
        depth--;
        if (depth == 0 && objStart != std::string::npos) {
          out.push_back(json.substr(objStart, i - objStart + 1));
          objStart = std::string::npos;
        }
      }
      continue;
    }

    if (c == ']' && depth == 0) {
      break;
    }
  }

  return out;
}

'''

if signature in s:
    print("helper already exists, not inserting duplicate")
else:
    if anchor not in s:
        raise SystemExit("anchor not found: static bool load_zoom_master_profile()")
    s = s.replace(anchor, helper + anchor, 1)
    print("inserted helper before load_zoom_master_profile()")

p.write_text(s, encoding="utf-8")
