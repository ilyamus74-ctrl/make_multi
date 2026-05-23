from pathlib import Path
import shutil

p = Path("mjpeg_gst_http.cpp")
s = p.read_text(encoding="utf-8")

backup = p.with_suffix(".cpp.bak_zoom_test_reliable_stop")
if not backup.exists():
    shutil.copy2(p, backup)
    print(f"backup: {backup}")

# 1. Replace send_zoom_via_bridge_ws with a safer version:
#    - payload has newline
#    - send loop sends whole frame
#    - short delay before closing socket
start = s.find("static bool send_zoom_via_bridge_ws(int cmd) {")
if start < 0:
    raise SystemExit("send_zoom_via_bridge_ws start not found")

end = s.find("\n\nstatic bool fetch_latest_gray_frame", start)
if end < 0:
    raise SystemExit("send_zoom_via_bridge_ws end not found")

new_helper = r'''static bool send_zoom_via_bridge_ws(int cmd) {
  const std::string host = "127.0.0.1";
  const int port = 8765;

  int fd = tcp_connect_local(host, port);
  if (fd < 0) return false;

  auto close_fd = [&]() {
    ::shutdown(fd, SHUT_WR);
    std::this_thread::sleep_for(std::chrono::milliseconds(30));
    ::close(fd);
  };

  const std::string req =
    "GET /ws HTTP/1.1\r\n"
    "Host: " + host + ":" + std::to_string(port) + "\r\n"
    "Upgrade: websocket\r\n"
    "Connection: Upgrade\r\n"
    "Sec-WebSocket-Key: dGhlIHNhbXBsZSBub25jZQ==\r\n"
    "Sec-WebSocket-Version: 13\r\n"
    "\r\n";

  if (::send(fd, req.data(), req.size(), 0) < 0) {
    ::close(fd);
    return false;
  }

  char resp[1024];
  const int n = ::recv(fd, resp, sizeof(resp), 0);
  if (n <= 0 || std::string(resp, n).find("101") == std::string::npos) {
    ::close(fd);
    return false;
  }

  // Important: include newline. Bridge command parser is line-oriented.
  const std::string msg = "Z " + std::to_string(cmd) + "\n";
  if (msg.size() >= 126) {
    close_fd();
    return false;
  }

  std::vector<uint8_t> fr;
  fr.push_back(0x81); // text frame
  fr.push_back(static_cast<uint8_t>(0x80 | msg.size())); // masked payload
  uint8_t mk[4] = {0x12, 0x34, 0x56, 0x78};
  fr.insert(fr.end(), mk, mk + 4);
  for (size_t i = 0; i < msg.size(); ++i) {
    fr.push_back(static_cast<uint8_t>(msg[i]) ^ mk[i % 4]);
  }

  size_t sent = 0;
  while (sent < fr.size()) {
    const ssize_t k = ::send(fd, fr.data() + sent, fr.size() - sent, MSG_NOSIGNAL);
    if (k > 0) {
      sent += static_cast<size_t>(k);
      continue;
    }
    if (k < 0 && (errno == EINTR || errno == EAGAIN || errno == EWOULDBLOCK)) {
      std::this_thread::sleep_for(std::chrono::milliseconds(2));
      continue;
    }
    close_fd();
    return false;
  }

  // Give ws_uart_bridge time to read/process the frame before TCP close.
  std::this_thread::sleep_for(std::chrono::milliseconds(80));
  close_fd();
  return true;
}'''

s = s[:start] + new_helper + s[end:]

# 2. Replace /api/zoom_test block with unconditional repeated stop.
old = '''    int cmd = 0;
    int holdMs = 0;
    extract_json_int_field(bodyReq, "cmd", &cmd);
    extract_json_int_field(bodyReq, "hold_ms", &holdMs);
    cmd = std::max(-o.cmd_max_zoom, std::min(o.cmd_max_zoom, cmd));
    holdMs = std::max(20, std::min(8000, holdMs));
    bool ok = send_zoom_via_bridge_ws(cmd);
    if (cmd == 0) holdMs = 0;
    if (ok && cmd != 0) {
      std::this_thread::sleep_for(std::chrono::milliseconds(holdMs));
      ok = send_zoom_via_bridge_ws(0) && ok;
    }
    std::ostringstream os;
    os << "{\\"ok\\":" << (ok ? "true" : "false") << ",\\"cmd\\":" << cmd << ",\\"hold_ms\\":" << holdMs << "}";'''

new = '''    int cmd = 0;
    int holdMs = 0;
    extract_json_int_field(bodyReq, "cmd", &cmd);
    extract_json_int_field(bodyReq, "hold_ms", &holdMs);

    cmd = std::max(-o.cmd_max_zoom, std::min(o.cmd_max_zoom, cmd));
    holdMs = std::max(0, std::min(8000, holdMs));

    bool okStart = true;
    bool okStop = true;

    if (cmd != 0 && holdMs > 0) {
      okStart = send_zoom_via_bridge_ws(cmd);
      std::this_thread::sleep_for(std::chrono::milliseconds(holdMs));

      // Hard safety: always send Z 0 several times, even if start reported false.
      okStop = false;
      for (int i = 0; i < 3; ++i) {
        okStop = send_zoom_via_bridge_ws(0) || okStop;
        std::this_thread::sleep_for(std::chrono::milliseconds(80));
      }
    } else {
      cmd = 0;
      holdMs = 0;
      okStop = false;
      for (int i = 0; i < 3; ++i) {
        okStop = send_zoom_via_bridge_ws(0) || okStop;
        std::this_thread::sleep_for(std::chrono::milliseconds(80));
      }
    }

    const bool ok = okStart && okStop;

    std::ostringstream os;
    os << "{\\"ok\\":" << (ok ? "true" : "false")
       << ",\\"cmd\\":" << cmd
       << ",\\"hold_ms\\":" << holdMs
       << ",\\"ok_start\\":" << (okStart ? "true" : "false")
       << ",\\"ok_stop\\":" << (okStop ? "true" : "false")
       << "}";'''

if old not in s:
    raise SystemExit("/api/zoom_test command block not found")

s = s.replace(old, new, 1)

p.write_text(s, encoding="utf-8")
print("patched mjpeg_gst_http.cpp")
