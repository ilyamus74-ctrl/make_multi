from pathlib import Path
import shutil, time

p = Path("mjpeg_gst_http.cpp")
s = p.read_text(encoding="utf-8")

backup = p.with_suffix(f".cpp.bak_serve_apriltag_debug_jpg_{int(time.time())}")
shutil.copy2(p, backup)
print("Backup:", backup)

anchor = '''  std::string bodyReq = get_body(req);
'''

route = r'''
  if (path == "/debug_apriltag_latest.jpg" || path == "/api/apriltag/debug.jpg") {
    if (method != "GET") {
      const char* body = "Method Not Allowed";
      std::string hdr =
        "HTTP/1.1 405 Method Not Allowed\r\nAllow: GET\r\nContent-Type: text/plain\r\nConnection: close\r\n"
        "Content-Length: " + std::to_string(std::strlen(body)) + "\r\n\r\n";
      send_all(cfd, hdr.data(), hdr.size());
      send_all(cfd, body, std::strlen(body));
      ::close(cfd);
      return;
    }

    std::vector<std::string> candidates = {
      "debug_apriltag_latest.jpg",
      "./debug_apriltag_latest.jpg",
      "/root/new_yolo8/debug_apriltag_latest.jpg"
    };

    std::string bytes;
    bool found = false;

    for (const auto& fp : candidates) {
      std::ifstream in(fp, std::ios::binary);
      if (!in.is_open()) continue;
      std::ostringstream ss;
      ss << in.rdbuf();
      bytes = ss.str();
      found = !bytes.empty();
      if (found) break;
    }

    if (!found) {
      const char* body = "debug_apriltag_latest.jpg not found";
      std::string hdr =
        "HTTP/1.1 404 Not Found\r\nContent-Type: text/plain\r\nCache-Control: no-store\r\nConnection: close\r\n"
        "Content-Length: " + std::to_string(std::strlen(body)) + "\r\n\r\n";
      send_all(cfd, hdr.data(), hdr.size());
      send_all(cfd, body, std::strlen(body));
      ::close(cfd);
      return;
    }

    std::string hdr =
      "HTTP/1.1 200 OK\r\nContent-Type: image/jpeg\r\nCache-Control: no-store\r\nConnection: close\r\n"
      "Content-Length: " + std::to_string(bytes.size()) + "\r\n\r\n";
    send_all(cfd, hdr.data(), hdr.size());
    send_all(cfd, bytes.data(), bytes.size());
    ::close(cfd);
    return;
  }

'''

if "api/apriltag/debug.jpg" in s:
    print("SKIP: debug jpg route already exists")
else:
    if anchor not in s:
        raise SystemExit("ERROR: handle_client bodyReq anchor not found")
    s = s.replace(anchor, anchor + route, 1)
    p.write_text(s, encoding="utf-8")
    print("OK: route inserted")
