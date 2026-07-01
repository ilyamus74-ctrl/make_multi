from pathlib import Path
import shutil, time, re

p = Path("ptz_autopilot.cpp")
s = p.read_text(encoding="utf-8")

backup = p.with_suffix(f".cpp.bak_speed_profile_clear_{int(time.time())}")
shutil.copy2(p, backup)
print("Backup:", backup)

route = r'''
if((path=="/api/autopilot/speed_profile/clear"||path=="/api/autopilot/speed_profile/reset")&&method=="POST"){
  auto oldPts=load_speed_profile_points();
  const int oldCount=(int)oldPts.size();
  bool ok=save_speed_profile_points(std::vector<SpeedPoint>{});
  {
    std::lock_guard<std::mutex> lk(g_runtimeSpeedOverrideMtx);
    g_runtimeSpeedOverrideActive=false;
  }
  std::ostringstream os;
  os<<"{\"ok\":"<<(ok?"true":"false")<<",\"old_points\":"<<oldCount<<",\"points\":0}";
  std::string body=os.str();
  std::string hdr="HTTP/1.1 200 OK\r\nContent-Type: application/json\r\nCache-Control: no-store\r\nConnection: close\r\nContent-Length: "+std::to_string(body.size())+"\r\n\r\n";
  send_all(cfd,hdr.data(),hdr.size());
  send_all(cfd,body.data(),body.size());
  ::close(cfd);
  return;
}
'''

if "/api/autopilot/speed_profile/clear" in s:
    print("SKIP: clear endpoint already exists")
else:
    # Insert before the generic /api/autopilot/speed_profile handler.
    patterns = [
        'if(path=="/api/autopilot/speed_profile"',
        'if (path == "/api/autopilot/speed_profile"',
        'if(path=="/api/autopilot/speed_profile"&&method=="GET")',
        'if (path == "/api/autopilot/speed_profile" && method == "GET")',
    ]

    pos = -1
    for pat in patterns:
        pos = s.find(pat)
        if pos >= 0:
            break

    if pos < 0:
        # Fallback: place before config endpoint, still inside HTTP handler.
        for pat in ['if(path=="/api/autopilot/config"', 'if (path == "/api/autopilot/config"']:
            pos = s.find(pat)
            if pos >= 0:
                break

    if pos < 0:
        raise SystemExit("ERROR: could not find speed_profile/config route anchor in ptz_autopilot.cpp")

    s = s[:pos] + route + "\n" + s[pos:]
    p.write_text(s, encoding="utf-8")
    print("OK: inserted /api/autopilot/speed_profile/clear")
