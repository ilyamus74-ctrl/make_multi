#!/usr/bin/env python3
from pathlib import Path
import shutil

path = Path("ptz_autopilot.cpp")
s = path.read_text(encoding="utf-8")

backup = path.with_suffix(".cpp.bak_stop_once")
if not backup.exists():
    shutil.copy2(path, backup)
    print(f"backup: {backup}")

old = 'int pan=0,tilt=0; double ex=0,ey=0; bool active=g.enabled.load(); if(active && ok && ts.mode=="TRACKING" && ts.valid){'
new = 'int pan=0,tilt=0; double ex=0,ey=0; bool active=g.enabled.load(); bool needSend=false; if(active && ok && ts.mode=="TRACKING" && ts.valid){'
if old not in s:
    raise SystemExit("pattern 1 not found")
s = s.replace(old, new, 1)

old = 'prevp=pan; prevt=tilt; sent_stop=false;} else {ix=iy=pex=pey=0; prevp=prevt=0; if(!sent_stop){pan=0;tilt=0; sent_stop=true;} else {std::this_thread::sleep_for(std::chrono::milliseconds(10));}}'
new = 'prevp=pan; prevt=tilt; sent_stop=false; needSend=true;} else {ix=iy=pex=pey=0; prevp=prevt=0; if(!sent_stop){pan=0;tilt=0; sent_stop=true; needSend=true;} else {std::this_thread::sleep_for(std::chrono::milliseconds(10));}}'
if old not in s:
    raise SystemExit("pattern 2 not found")
s = s.replace(old, new, 1)

old = 'if((pan!=0||tilt!=0)||sent_stop){'
new = 'if(needSend){'
if old not in s:
    raise SystemExit("pattern 3 not found")
s = s.replace(old, new, 1)

path.write_text(s, encoding="utf-8")
print("patched ptz_autopilot.cpp")
