from pathlib import Path
import shutil

p = Path("web/ws_uart_bridge.cpp")
s = p.read_text(encoding="utf-8")

backup = p.with_suffix(".cpp.bak_errorcode_ws")
if not backup.exists():
    shutil.copy2(p, backup)
    print(f"backup: {backup}")

# Patch ws_write_line: no throwing write.
old = '''  void ws_write_line(const std::string& line) {
    std::lock_guard<std::mutex> lk(ws_write_mtx);
    if (!running.load()) return;
    std::string msg = line;
    if (msg.empty() || msg.back() != '\\n') msg.push_back('\\n');
    ws_.text(true);
    ws_.write(asio::buffer(msg));
  }

  void stop() {
    running.store(false);
    try { ws_.close(ws::close_code::normal); } catch (...) {}
  }'''

new = '''  void ws_write_line(const std::string& line) {
    std::lock_guard<std::mutex> lk(ws_write_mtx);
    if (!running.load()) return;

    std::string msg = line;
    if (msg.empty() || msg.back() != '\\n') msg.push_back('\\n');

    beast::error_code ec;
    ws_.text(true, ec);
    if (ec) {
      running.store(false);
      return;
    }

    ws_.write(asio::buffer(msg), ec);
    if (ec) {
      running.store(false);
      return;
    }
  }

  void stop() {
    running.store(false);
    beast::error_code ec;
    ws_.close(ws::close_code::normal, ec);
  }'''

if old not in s:
    raise SystemExit("ws_write_line/stop block not found")
s = s.replace(old, new, 1)

# Replace whole Session::run with fully error_code-based handshake/read path.
start = s.find("void Session::run() {")
end_marker = "\n\nint main(int argc, char** argv)"
end = s.find(end_marker, start)

if start < 0 or end < 0:
    raise SystemExit("Session::run block not found")

new_run = r'''void Session::run() {
  auto cleanup = [&]() {
    running.store(false);
    if (bridge_) {
      try { bridge_->unregister_session(this); } catch (...) {}
    }
    beast::error_code ec;
    ws_.close(ws::close_code::normal, ec);
  };

  try {
    ws_.set_option(ws::stream_base::timeout::suggested(beast::role_type::server));
    ws_.set_option(ws::stream_base::decorator([](ws::response_type& res) {
      res.set(http::field::server, std::string("ws-uart-bridge"));
    }));

    beast::flat_buffer rbuf;
    http::request<http::string_body> req;
    beast::error_code ec;

    http::read(ws_.next_layer(), rbuf, req, ec);
    if (ec) {
      // Browser reload/close before WS upgrade. Normal.
      cleanup();
      return;
    }

    if (!ws::is_upgrade(req) || req.target() != "/ws") {
      http::response<http::string_body> res{http::status::not_found, req.version()};
      res.set(http::field::content_type, "text/plain");
      res.body() = "Not Found\n";
      res.prepare_payload();

      beast::error_code wec;
      http::write(ws_.next_layer(), res, wec);
      cleanup();
      return;
    }

    ws_.accept(req, ec);
    if (ec) {
      cleanup();
      return;
    }

    bridge_->register_session(shared_from_this());

    while (running.load()) {
      beast::flat_buffer b;
      ec.clear();
      ws_.read(b, ec);

      if (ec) {
        // Normal client disconnect/reload. Never abort process.
        if (ec != ws::error::closed &&
            ec != asio::error::eof &&
            ec != asio::error::connection_reset &&
            ec != asio::error::operation_aborted) {
          std::cerr << "WS_READ_ERROR " << ec.message() << "\n";
        }
        break;
      }

      std::string msg = beast::buffers_to_string(b.data());
      std::string last_joy_line;

      size_t start = 0;
      while (start < msg.size()) {
        size_t end = msg.find('\n', start);
        if (end == std::string::npos) end = msg.size();

        std::string line = msg.substr(start, end - start);
        if (!line.empty() && line.back() == '\r') line.pop_back();

        if (!line.empty()) {
          if (is_joystick_line(line)) {
            last_joy_line = line;
          } else {
            std::cerr << "UART_TX " << line << "\n";
            bridge_->send_uart_line(line);
          }
        }

        start = end + 1;
      }

      if (!last_joy_line.empty()) {
        std::cerr << "UART_TX " << last_joy_line << "\n";
        bridge_->send_uart_line(last_joy_line);
      }
    }
  } catch (const std::exception& e) {
    std::cerr << "WS_SESSION_EXCEPTION_CAUGHT " << e.what() << "\n";
  } catch (...) {
    std::cerr << "WS_SESSION_UNKNOWN_EXCEPTION_CAUGHT\n";
  }

  cleanup();
}'''

s = s[:start] + new_run + s[end:]

# Patch main accept loop to not throw on accept.
old = '''    for (;;) {
      tcp::socket sock{ioc};
      acc.accept(sock);
      sock.set_option(tcp::no_delay(true));

      auto s = std::make_shared<Session>(std::move(sock), bridge);
      std::thread([s]() { s->run(); }).detach();
    }'''

new = '''    for (;;) {
      tcp::socket sock{ioc};
      beast::error_code ec;
      acc.accept(sock, ec);
      if (ec) {
        std::cerr << "ACCEPT_ERROR " << ec.message() << "\\n";
        continue;
      }

      sock.set_option(tcp::no_delay(true), ec);

      auto s = std::make_shared<Session>(std::move(sock), bridge);
      std::thread([s]() {
        try {
          s->run();
        } catch (const std::exception& e) {
          std::cerr << "SESSION_THREAD_EXCEPTION " << e.what() << "\\n";
        } catch (...) {
          std::cerr << "SESSION_THREAD_UNKNOWN_EXCEPTION\\n";
        }
      }).detach();
    }'''

if old not in s:
    raise SystemExit("main accept loop block not found")
s = s.replace(old, new, 1)

p.write_text(s, encoding="utf-8")
print("patched web/ws_uart_bridge.cpp")
