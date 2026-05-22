#!/usr/bin/env python3
from pathlib import Path
import shutil

path = Path("ptz_autopilot.cpp")
s = path.read_text(encoding="utf-8")

backup = path.with_suffix(".cpp.bak_parse_box")
if not backup.exists():
    shutil.copy2(path, backup)
    print(f"backup: {backup}")

old = r'''static bool parse_tracker(const std::string&j,TrackerState&ts){ts.mode=json_str(j,"mode"); if(ts.mode.empty()) return false; json_int(j,"selected_track_id",ts.track_id); json_bool(j,"selected_box_valid",ts.valid); auto p=j.find("\"selected_box\""); if(p!=std::string::npos){auto sub=j.substr(p); json_int(sub,"left",ts.l); json_int(sub,"top",ts.t); json_int(sub,"right",ts.r); json_int(sub,"bottom",ts.b);} return true;}'''

new = r'''static bool parse_tracker(const std::string&j,TrackerState&ts){
  ts.mode=json_str(j,"mode");
  if(ts.mode.empty()) return false;

  json_int(j,"selected_track_id",ts.track_id);
  json_bool(j,"selected_box_valid",ts.valid);

  // /api/tracker/state returns: "box": {...}, not "selected_box".
  auto p=j.find("\"box\"");
  if(p!=std::string::npos){
    auto sub=j.substr(p);
    json_int(sub,"left",ts.l);
    json_int(sub,"top",ts.t);
    json_int(sub,"right",ts.r);
    json_int(sub,"bottom",ts.b);
  }

  // Safety: if tracker says valid but box was not parsed or is degenerate,
  // do not allow autopilot to move from fake 0,0,0,0 coordinates.
  if(ts.valid && (ts.r <= ts.l || ts.b <= ts.t)){
    ts.valid=false;
  }

  return true;
}'''

if old not in s:
    raise SystemExit("parse_tracker old block not found; patch not applied")

s = s.replace(old, new, 1)

path.write_text(s, encoding="utf-8")
print("patched parse_tracker()")
