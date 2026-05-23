from pathlib import Path
import shutil

p = Path("mjpeg_gst_http.cpp")
s = p.read_text(encoding="utf-8")

backup = p.with_suffix(".cpp.bak_zoom_test_direct_uart")
if not backup.exists():
    shutil.copy2(p, backup)
    print(f"backup: {backup}")

old = '''    int cmd = 0;
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

new = '''    int cmd = 0;
    int holdMs = 0;
    extract_json_int_field(bodyReq, "cmd", &cmd);
    extract_json_int_field(bodyReq, "hold_ms", &holdMs);

    cmd = std::max(-o.cmd_max_zoom, std::min(o.cmd_max_zoom, cmd));
    holdMs = std::max(0, std::min(8000, holdMs));

    const std::string uartDev = trim(o.zoom_calib_uart_dev).empty()
      ? std::string("/dev/ttyUSB0")
      : o.zoom_calib_uart_dev;
    const int uartBaud = o.zoom_calib_uart_baud > 0 ? o.zoom_calib_uart_baud : 115200;

    auto sendZoomDirect = [&](int c) -> bool {
      int fd = open_uart_for_zoom(uartDev, uartBaud);
      if (fd < 0) return false;

      const bool okWrite = write_uart_line(fd, std::string("Z ") + std::to_string(c));

      // Ensure bytes are flushed before closing the short-lived UART fd.
      tcdrain(fd);
      std::this_thread::sleep_for(std::chrono::milliseconds(30));
      ::close(fd);

      return okWrite;
    };

    bool okStart = true;
    bool okStop = false;

    if (cmd != 0 && holdMs > 0) {
      okStart = sendZoomDirect(cmd);
      std::this_thread::sleep_for(std::chrono::milliseconds(holdMs));
    } else {
      cmd = 0;
      holdMs = 0;
    }

    // Hard safety: always send Z 0 several times.
    for (int i = 0; i < 3; ++i) {
      okStop = sendZoomDirect(0) || okStop;
      std::this_thread::sleep_for(std::chrono::milliseconds(80));
    }

    const bool ok = okStart && okStop;

    std::ostringstream os;
    os << "{\\"ok\\":" << (ok ? "true" : "false")
       << ",\\"transport\\":\\"direct_uart\\""
       << ",\\"uart\\":\\"" << json_escape(uartDev) << "\\""
       << ",\\"baud\\":" << uartBaud
       << ",\\"cmd\\":" << cmd
       << ",\\"hold_ms\\":" << holdMs
       << ",\\"ok_start\\":" << (okStart ? "true" : "false")
       << ",\\"ok_stop\\":" << (okStop ? "true" : "false")
       << "}";'''

if old not in s:
    raise SystemExit("current /api/zoom_test block not found")

s = s.replace(old, new, 1)

p.write_text(s, encoding="utf-8")
print("patched mjpeg_gst_http.cpp")
