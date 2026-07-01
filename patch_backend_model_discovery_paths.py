from pathlib import Path
import time

p = Path("mjpeg_gst_http.cpp")
s = p.read_text(encoding="utf-8")
bak = p.with_suffix(p.suffix + f".bak_model_discovery_{int(time.time())}")
bak.write_text(s, encoding="utf-8")

anchor = '''  dirs.push_back("new_yolo8/models");
  dirs.push_back("new_yolo8/model_rknn");
  dirs.push_back("models");
  dirs.push_back("model_rknn");
'''

insert = '''  dirs.push_back("new_yolo8/models");
  dirs.push_back("new_yolo8/model_rknn");
  dirs.push_back("models");
  dirs.push_back("model_rknn");
  dirs.push_back("/root/new_yolo8/models");
  dirs.push_back("/root/new_yolo8/model_rknn");
'''

if "/root/new_yolo8/model_rknn" not in s:
    if anchor not in s:
        raise SystemExit("ERROR: model discovery anchor not found")
    s = s.replace(anchor, insert, 1)
    p.write_text(s, encoding="utf-8")
    print(f"OK backend model discovery patched, backup={bak}")
else:
    print("OK backend model discovery already patched")
