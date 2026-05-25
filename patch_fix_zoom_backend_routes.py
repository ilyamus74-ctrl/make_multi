from pathlib import Path
import re
import shutil
import time

p = Path("mjpeg_gst_http.cpp")
s = p.read_text(encoding="utf-8")

backup = p.with_suffix(f".cpp.bak_fix_zoom_backend_routes_{int(time.time())}")
shutil.copy2(p, backup)
print(f"backup: {backup}")

# 1) Forward declaration for json_escape, because save_zoom_runtime_state() uses it before definition.
decl = "static std::string json_escape(const std::string& s);\n"
if decl not in s:
    marker = "static int64_t now_ms()"
    idx = s.find(marker)
    if idx < 0:
        raise SystemExit("ERROR: cannot find marker for json_escape forward declaration")
    s = s[:idx] + decl + s[idx:]
    print("inserted json_escape forward declaration")
else:
    print("json_escape forward declaration already exists")

# 2) Replace broken backend route block.
start = s.find('  if (path == "/api/zoom/go_to_sample"')
end = s.find('  if (path == "/api/zoom/master_profile"', start)

if start < 0 or end < 0:
    raise SystemExit(f"ERROR: cannot locate broken zoom route block start={start} end={end}")

new_block = r'''  if (path == "/api/zoom/go_to_sample") {
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

    if (g_zoomMoveBusy.exchange(true)) {
      const std::string body = "{\"ok\":false,\"error\":\"zoom_move_busy\"}";
      const std::string hdr =
        "HTTP/1.1 409 Conflict\r\nContent-Type: application/json\r\nCache-Control: no-store\r\n"
        "Connection: close\r\nContent-Length: " + std::to_string(body.size()) + "\r\n\r\n";
      send_all(cfd, hdr.data(), hdr.size());
      send_all(cfd, body.data(), body.size());
      ::close(cfd);
      return;
    }

    std::string err;
    bool ok = true;

    int target = 0;
    extract_json_int_field(bodyReq, "profile_idx", &target);

    std::string mode = extract_json_string_field(bodyReq, "mode");
    if (mode.empty()) mode = "relative";

    ZoomProfilePoint point;
    if (!zoom_get_profile_point(target, &point)) {
      ok = false;
      err = "sample_not_found";
    }

    const int from = g_zoomSampleIdx.load();
    const int delta = target - from;

    if (ok && mode == "absolute_wide") {
      ok = zoom_send_stream_cmd(zoom_wide_cmd(), zoom_wide_hold_ms(), &err);
      if (ok && target > 0) ok = zoom_move_steps(target, &err);
    } else if (ok) {
      if (delta != 0) ok = zoom_move_steps(delta, &err);
    }

    if (ok) {
      g_zoomSampleIdx.store(target);
      g_zoomRatio.store(point.zoom_ratio);
      g_zoomConfidence.store(1.0);

      {
        std::lock_guard<std::mutex> lk(g_zoomMasterProfileMtx);
        g_zoomSource = (mode == "absolute_wide")
          ? "go_to_sample_absolute_wide"
          : "go_to_sample_relative";
      }

      if (mode == "absolute_wide") {
        g_zoomStepsSinceHome.store(std::max(0, target));
        g_zoomLastHomeMs.store(now_ms());
      } else {
        g_zoomStepsSinceHome.store(g_zoomStepsSinceHome.load() + std::abs(delta));
      }

      g_zoomLastMoveMs.store(now_ms());
      save_zoom_runtime_state();
    }

    g_zoomMoveBusy.store(false);

    std::ostringstream os;
    os << "{"
       << "\"ok\":" << (ok ? "true" : "false")
       << ",\"from_sample\":" << from
       << ",\"to_sample\":" << target
       << ",\"delta\":" << delta
       << ",\"mode\":\"" << json_escape(mode) << "\"";

    if (ok) {
      os << ",\"zoom_ratio\":" << point.zoom_ratio
         << ",\"focal_px\":" << point.focal_px
         << ",\"steps_since_home\":" << g_zoomStepsSinceHome.load();
    } else {
      os << ",\"error\":\"" << json_escape(err) << "\"";
    }

    os << "}";

    const std::string body = os.str();
    const std::string hdr =
      "HTTP/1.1 200 OK\r\nContent-Type: application/json\r\nCache-Control: no-store\r\n"
      "Connection: close\r\nContent-Length: " + std::to_string(body.size()) + "\r\n\r\n";
    send_all(cfd, hdr.data(), hdr.size());
    send_all(cfd, body.data(), body.size());
    ::close(cfd);
    return;
  }

  if (path == "/api/zoom/rehome_current_sample") {
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

    if (g_zoomMoveBusy.exchange(true)) {
      const std::string body = "{\"ok\":false,\"error\":\"zoom_move_busy\"}";
      const std::string hdr =
        "HTTP/1.1 409 Conflict\r\nContent-Type: application/json\r\nCache-Control: no-store\r\n"
        "Connection: close\r\nContent-Length: " + std::to_string(body.size()) + "\r\n\r\n";
      send_all(cfd, hdr.data(), hdr.size());
      send_all(cfd, body.data(), body.size());
      ::close(cfd);
      return;
    }

    int target = g_zoomSampleIdx.load();
    extract_json_int_field(bodyReq, "profile_idx", &target);

    const int maxIdx = zoom_max_sample_idx();
    target = std::max(0, std::min(maxIdx, target));

    const double mid = maxIdx / 2.0;
    std::string edge = "wide";
    std::string err;
    bool ok = true;
    int backSteps = 0;

    if (target <= mid) {
      edge = "wide";
      ok = zoom_send_stream_cmd(zoom_wide_cmd(), zoom_wide_hold_ms(), &err);
      if (ok && target > 0) ok = zoom_move_steps(target, &err);
      backSteps = target;
    } else {
      edge = "tele";
      ok = zoom_send_stream_cmd(zoom_tele_cmd(), zoom_wide_hold_ms(), &err);
      backSteps = std::max(0, maxIdx - target);
      if (ok && backSteps > 0) ok = zoom_move_steps(-backSteps, &err);
    }

    ZoomProfilePoint point;
    if (!zoom_get_profile_point(target, &point)) {
      ok = false;
      if (err.empty()) err = "sample_not_found";
    }

    if (ok) {
      g_zoomSampleIdx.store(target);
      g_zoomRatio.store(point.zoom_ratio);
      g_zoomConfidence.store(1.0);

      {
        std::lock_guard<std::mutex> lk(g_zoomMasterProfileMtx);
        g_zoomSource = "rehome_current_sample_nearest_edge";
      }

      g_zoomStepsSinceHome.store(0);
      g_zoomLastHomeMs.store(now_ms());
      g_zoomLastMoveMs.store(now_ms());
      save_zoom_runtime_state();
    }

    g_zoomMoveBusy.store(false);

    std::ostringstream os;
    os << "{"
       << "\"ok\":" << (ok ? "true" : "false")
       << ",\"target_sample\":" << target
       << ",\"edge\":\"" << json_escape(edge) << "\""
       << ",\"back_steps\":" << backSteps
       << ",\"zoom_ratio\":" << point.zoom_ratio
       << ",\"focal_px\":" << point.focal_px;

    if (!ok) {
      os << ",\"error\":\"" << json_escape(err) << "\"";
    }

    os << "}";

    const std::string body = os.str();
    const std::string hdr =
      "HTTP/1.1 200 OK\r\nContent-Type: application/json\r\nCache-Control: no-store\r\n"
      "Connection: close\r\nContent-Length: " + std::to_string(body.size()) + "\r\n\r\n";
    send_all(cfd, hdr.data(), hdr.size());
    send_all(cfd, body.data(), body.size());
    ::close(cfd);
    return;
  }

  if (path == "/api/zoom/drift") {
    if (method == "GET") {
      std::ostringstream os;
      os << "{"
         << "\"ok\":true"
         << ",\"enabled\":" << (g_zoomDriftEnable.load() ? "true" : "false")
         << ",\"interval_sec\":" << g_zoomDriftIntervalSec.load()
         << ",\"min_steps\":" << g_zoomDriftMinSteps.load()
         << ",\"steps_since_home\":" << g_zoomStepsSinceHome.load()
         << ",\"move_busy\":" << (g_zoomMoveBusy.load() ? "true" : "false")
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

    if (method == "POST") {
      bool en = g_zoomDriftEnable.load();
      int interval = g_zoomDriftIntervalSec.load();
      int minSteps = g_zoomDriftMinSteps.load();

      extract_json_bool_field(bodyReq, "enabled", &en);
      extract_json_int_field(bodyReq, "interval_sec", &interval);
      extract_json_int_field(bodyReq, "min_steps", &minSteps);

      g_zoomDriftEnable.store(en);
      g_zoomDriftIntervalSec.store(std::max(30, std::min(3600, interval)));
      g_zoomDriftMinSteps.store(std::max(1, std::min(100, minSteps)));

      const std::string body = "{\"ok\":true}";
      const std::string hdr =
        "HTTP/1.1 200 OK\r\nContent-Type: application/json\r\nCache-Control: no-store\r\n"
        "Connection: close\r\nContent-Length: " + std::to_string(body.size()) + "\r\n\r\n";
      send_all(cfd, hdr.data(), hdr.size());
      send_all(cfd, body.data(), body.size());
      ::close(cfd);
      return;
    }

    const char* body = "Method Not Allowed";
    const std::string hdr =
      "HTTP/1.1 405 Method Not Allowed\r\nAllow: GET, POST\r\nContent-Type: text/plain\r\nConnection: close\r\n"
      "Content-Length: " + std::to_string(std::strlen(body)) + "\r\n\r\n";
    send_all(cfd, hdr.data(), hdr.size());
    send_all(cfd, body, std::strlen(body));
    ::close(cfd);
    return;
  }

'''

s = s[:start] + new_block + s[end:]
p.write_text(s, encoding="utf-8")
print("OK: replaced broken zoom backend route block")
