from pathlib import Path
import shutil

p = Path("web/ws_uart_bridge.cpp")
s = p.read_text(encoding="utf-8")

backup = p.with_suffix(".cpp.bak_no_crash")
if not backup.exists():
    shutil.copy2(p, backup)
    print(f"backup: {backup}")

start = s.find("void Session::run() {")
end_marker = "\n\nint main(int argc, char** argv)"
end = s.find(end_marker, start)

if start < 0 or end < 0:
    raise SystemExit("Session::run() block not found")

new_func = r'''void Session::run() {
  auto cleanup = [&]() {
    running.store(false);
    if (bridge_) {
      try { bridge_->unregister_session(this); } catch (...) {}
    }
    try { ws_.close(ws::close_code::normal); } catch (...) {}
  };

  try {
    ws_.set_option(ws::stream_base::timeout::suggested(beast::role_type::server));
    ws_.set_option(ws::stream_base::decorator([](ws::response_type& res) {
      res.set(http::field::server, std::string("ws-uart-bridge"));
    }));

    beast::flat_buffer rbuf;
    http::request<http::string_body> req;
    http::read(ws_.next_layer(), rbuf, req);

    if (!ws::is_upgrade(req) || req.target() != "/ws") {
      http::response<http::string_body> res{http::status::not_found, req.version()};
      res.set(http::field::content_type, "text/plain");
      res.body() = "Not Found\n";
      res.prepare_payload();
      http::write(ws_.next_layer(), res);
      cleanup();
      return;
    }

    ws_.accept(req);
    bridge_->register_session(shared_from_this());

    while (running.load()) {
      beast::flat_buffer b;
      beast::error_code ec;
      ws_.read(b, ec);

      if (ec) {
        // Normal browser/client disconnect. Do not crash the bridge.
        if (ec == ws::error::closed || ec == asio::error::eof ||
            ec == asio::error::connection_reset ||
            ec == asio::error::operation_aborted) {
          break;
        }

        std::cerr << "WS_READ_ERROR " << ec.message() << "\n";
        break;
      }

      std::string msg = beast::buffers_to_string(b.data());

      // Split into lines.
      // Priority commands (HOME, STOP, etc.) are sent immediately in order.
      // J lines: only the LAST one in this message batch is sent.
      // This prevents UART RX buffer overflow on STM32 (64-byte hardware buffer).

      std::string last_joy_line;

      size_t start = 0;
      while (start < msg.size()) {
        size_t end = msg.find('\n', start);
        if (end == std::string::npos) end = msg.size();

        std::string line = msg.substr(start, end - start);
        if (!line.empty() && line.back() == '\r') line.pop_back();

        if (!line.empty()) {
          if (is_joystick_line(line)) {
            last_joy_line = line;   // keep only the latest J command
          } else {
            std::cerr << "UART_TX " << line << "\n";
            bridge_->send_uart_line(line);  // priority: send immediately
          }
        }

        start = end + 1;
      }

      // Send the single latest joystick command (if any)
      if (!last_joy_line.empty()) {
        std::cerr << "UART_TX " << last_joy_line << "\n";
        bridge_->send_uart_line(last_joy_line);
      }
    }
  } catch (const boost::system::system_error& e) {
    // Important: end-of-stream during handshake/accept is normal when browser closes/reloads.
    const auto code = e.code();
    if (code == asio::error::eof ||
        code == asio::error::connection_reset ||
        code == asio::error::operation_aborted ||
        code == ws::error::closed) {
      // normal disconnect
    } else {
      std::cerr << "WS_SESSION_ERROR " << e.what() << "\n";
    }
  } catch (const std::exception& e) {
    std::cerr << "WS_SESSION_EXCEPTION " << e.what() << "\n";
  } catch (...) {
    std::cerr << "WS_SESSION_UNKNOWN_EXCEPTION\n";
  }

  cleanup();
}'''

s = s[:start] + new_func + s[end:]
p.write_text(s, encoding="utf-8")
print("patched web/ws_uart_bridge.cpp")
