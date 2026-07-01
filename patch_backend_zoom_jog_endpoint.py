from pathlib import Path
import shutil, time

p = Path("mjpeg_gst_http.cpp")
s = p.read_text(encoding="utf-8")

backup = p.with_suffix(f".cpp.bak_zoom_jog_endpoint_{int(time.time())}")
shutil.copy2(p, backup)
print("Backup:", backup)

changed = False

# 1. Add jog globals near zoom runtime globals.
old_globals = """static std::atomic<bool> g_zoomMoveBusy{false};
static std::atomic<int> g_zoomDriftIntervalSec{300};
"""

new_globals = """static std::atomic<bool> g_zoomMoveBusy{false};

// Manual zoom jog used by Q/A in SWEEP TIME STEPS.
// This is intentionally backend-driven because browser WS zoom is not reliable enough here.
static std::atomic<bool> g_zoomJogActive{false};
static std::atomic<int> g_zoomJogCmd{0};
static std::atomic<uint64_t> g_zoomJogGeneration{1};

static std::atomic<int> g_zoomDriftIntervalSec{300};
"""

if "g_zoomJogActive" not in s:
    if old_globals not in s:
        raise SystemExit("ERROR: zoom runtime globals marker not found")
    s = s.replace(old_globals, new_globals, 1)
    print("OK: added zoom jog globals")
    changed = True
else:
    print("SKIP: zoom jog globals already present")

# 2. Add jog helpers after send_zoom_stream_via_bridge_ws block.
marker = """static bool send_zoom_via_bridge_ws(int cmd) {
  return send_zoom_stream_via_bridge_ws(cmd, 0);
}


"""

helpers = r'''static bool send_zoom_stop_burst_via_bridge_ws() {
  return send_zoom_stream_via_bridge_ws(0, 0);
}

static void zoom_jog_stream_worker(uint64_t generation, int cmd, int maxHoldMs) {
  const std::string host = "127.0.0.1";
  const int port = 8765;

  int fd = tcp_connect_local(host, port);
  if (fd < 0) {
    std::cerr << "zoom-jog: connect failed\n";
    if (g_zoomJogGeneration.load() == generation) g_zoomJogActive.store(false);
    return;
  }

  bool handshakeOk = false;
  bool started = false;
  bool hadFatal = false;

  auto send_pair = [&](int seq, int zcmd) -> bool {
    return ws_send_text_frame(fd, "J " + std::to_string(seq) + " 0 0\n") &&
           ws_send_text_frame(fd, "Z " + std::to_string(zcmd) + "\n");
  };

  if (bridge_ws_handshake(fd, host, port)) {
    handshakeOk = true;
    static std::atomic<int> seqCounter{100000};

    const auto t0 = std::chrono::steady_clock::now();

    while (
      g_zoomJogActive.load() &&
      g_zoomJogGeneration.load() == generation &&
      g_zoomJogCmd.load() == cmd
    ) {
      const int seq = seqCounter.fetch_add(1);
      if (!send_pair(seq, cmd)) {
        hadFatal = true;
        break;
      }

      started = true;

      const auto now = std::chrono::steady_clock::now();
      const int elapsed = static_cast<int>(
        std::chrono::duration_cast<std::chrono::milliseconds>(now - t0).count()
      );

      if (elapsed >= maxHoldMs) break;

      std::this_thread::sleep_for(std::chrono::milliseconds(70));
    }

    // Always stop at the end of a jog worker.
    for (int i = 0; i < 10; ++i) {
      const int seq = seqCounter.fetch_add(1);
      if (!send_pair(seq, 0)) {
        hadFatal = true;
        break;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(35));
    }
  }

  ::shutdown(fd, SHUT_RDWR);
  ::close(fd);

  if (g_zoomJogGeneration.load() == generation) {
    g_zoomJogActive.store(false);
    g_zoomJogCmd.store(0);
  }

  std::cout << "zoom-jog: done generation=" << generation
            << " cmd=" << cmd
            << " started=" << (started ? 1 : 0)
            << " handshake=" << (handshakeOk ? 1 : 0)
            << " fatal=" << (hadFatal ? 1 : 0)
            << "\n";
}

static bool zoom_jog_start_backend(int cmd, int maxHoldMs) {
  cmd = std::max(-100, std::min(100, cmd));
  maxHoldMs = std::max(100, std::min(20000, maxHoldMs));

  if (cmd == 0) return false;

  const uint64_t generation = g_zoomJogGeneration.fetch_add(1) + 1;
  g_zoomJogCmd.store(cmd);
  g_zoomJogActive.store(true);

  std::thread([generation, cmd, maxHoldMs]() {
    zoom_jog_stream_worker(generation, cmd, maxHoldMs);
  }).detach();

  std::cout << "zoom-jog: start generation=" << generation
            << " cmd=" << cmd
            << " max_hold_ms=" << maxHoldMs
            << "\n";

  return true;
}

static bool zoom_jog_stop_backend() {
  const uint64_t generation = g_zoomJogGeneration.fetch_add(1) + 1;
  g_zoomJogActive.store(false);
  g_zoomJogCmd.store(0);

  // Immediate stop burst through a separate short connection.
  send_zoom_stop_burst_via_bridge_ws();

  std::cout << "zoom-jog: stop generation=" << generation << "\n";
  return true;
}

'''

if "zoom_jog_start_backend" not in s:
    if marker not in s:
        raise SystemExit("ERROR: send_zoom_via_bridge_ws marker not found")
    s = s.replace(marker, marker + helpers + "\n", 1)
    print("OK: added zoom jog backend helpers")
    changed = True
else:
    print("SKIP: zoom jog helpers already present")

# 3. Add endpoint before /api/zoom/go_to_sample.
endpoint_marker = """  if (path == "/api/zoom/go_to_sample") {
"""

endpoint = r'''  if (path == "/api/zoom/jog") {
    if (method != "POST") {
      const char* body = "Method Not Allowed";
      const std::string hdr =
        "HTTP/1.1 405 Method Not Allowed\r\nAllow: POST\r\nContent-Type: text/plain\r\nConnection: close\r\n"
        "Content-Length: " + std::to_string(std::strlen(body)) + "\r\n\r\n";
      send_all(cfd, hdr.data(), hdr.size());
      send_all(cfd, body, std::strlen(body));
      ::close(cfd);
      return;
    }

    std::string action = trim(extract_json_string_field(bodyReq, "action"));
    if (action.empty()) action = "start";

    ZoomAprilTagCalibParams p = g_zoomCalibSettings;
    load_zoom_calib_settings(p, o.cmd_max_zoom);
    clamp_zoom_calib_params(p, o.cmd_max_zoom);

    int cmd = 0;
    extract_json_int_field(bodyReq, "cmd", &cmd);

    int holdMs = p.full_sweep_ms;
    extract_json_int_field(bodyReq, "hold_ms", &holdMs);
    holdMs = std::max(100, std::min(20000, holdMs));

    bool ok = false;

    if (action == "stop" || cmd == 0) {
      ok = zoom_jog_stop_backend();
      action = "stop";
    } else {
      const int maxCmd = std::max(1, o.cmd_max_zoom);
      cmd = std::max(-maxCmd, std::min(maxCmd, cmd));
      ok = zoom_jog_start_backend(cmd, holdMs);
      action = "start";
    }

    std::ostringstream os;
    os << "{"
       << "\"ok\":" << (ok ? "true" : "false")
       << ",\"action\":\"" << json_escape(action) << "\""
       << ",\"cmd\":" << cmd
       << ",\"hold_ms\":" << holdMs
       << ",\"active\":" << (g_zoomJogActive.load() ? "true" : "false")
       << ",\"generation\":" << g_zoomJogGeneration.load()
       << ",\"zoom_move_mode\":\"" << json_escape(p.zoom_move_mode) << "\""
       << ",\"full_sweep_ms\":" << p.full_sweep_ms
       << ",\"samples\":" << p.samples
       << "}";

    const std::string body = os.str();
    const std::string hdr =
      "HTTP/1.1 200 OK\r\nContent-Type: application/json\r\nCache-Control: no-store\r\n"
      "Connection: close\r\nContent-Length: " + std::to_string(body.size()) + "\r\n\r\n";
    send_all(cfd, hdr.data(), hdr.size());
    send_all(cfd, body.data(), body.size());
    ::close(cfd);
    return;
  }

'''

if 'path == "/api/zoom/jog"' not in s:
    if endpoint_marker not in s:
        raise SystemExit("ERROR: /api/zoom/go_to_sample marker not found")
    s = s.replace(endpoint_marker, endpoint + endpoint_marker, 1)
    print("OK: added /api/zoom/jog endpoint")
    changed = True
else:
    print("SKIP: /api/zoom/jog endpoint already present")

if changed:
    p.write_text(s, encoding="utf-8")
    print("DONE")
else:
    print("No changes")
