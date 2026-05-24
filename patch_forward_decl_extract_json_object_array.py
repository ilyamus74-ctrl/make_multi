from pathlib import Path
import shutil

p = Path("mjpeg_gst_http.cpp")
s = p.read_text(encoding="utf-8")

backup = p.with_suffix(".cpp.bak_forward_decl_extract_json_object_array")
if not backup.exists():
    shutil.copy2(p, backup)
    print(f"backup: {backup}")

anchor = "static bool load_zoom_master_profile() {\n"
decl = "static std::vector<std::string> extract_json_object_array(const std::string& json, const std::string& key);\n\n"

pos = s.find(anchor)
if pos < 0:
    raise SystemExit("anchor not found: load_zoom_master_profile")

before = s[:pos]

# Важно: ищем именно объявление с ; до load_zoom_master_profile,
# а не определение функции ниже.
if decl.strip() in before:
    print("forward declaration already exists before load_zoom_master_profile")
else:
    s = s[:pos] + decl + s[pos:]
    p.write_text(s, encoding="utf-8")
    print("inserted forward declaration before load_zoom_master_profile")
