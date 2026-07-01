from pathlib import Path
import shutil, time

p = Path("mjpeg_gst_http.cpp")
s = p.read_text(encoding="utf-8")

backup = p.with_suffix(f".cpp.bak_zoom_clear_profiles_{int(time.time())}")
shutil.copy2(p, backup)
print("Backup:", backup)

# 1. Add headers if missing.
if "#include <filesystem>" not in s:
    s = s.replace("#include <fstream>", "#include <fstream>\n#include <filesystem>", 1)
    print("OK: added <filesystem>")

if "#include <ctime>" not in s:
    # not always needed, but harmless if absent in current file
    s = s.replace("#include <iomanip>", "#include <iomanip>\n#include <ctime>", 1)
    print("OK: added <ctime>")

# 2. Insert helper before handle_client.
anchor = '''static void handle_client(int cfd, const Opts& o) {
'''

helper = r'''
static std::string compact_timestamp_for_filename() {
  const auto now = std::chrono::system_clock::now();
  const std::time_t t = std::chrono::system_clock::to_time_t(now);
  std::tm tm{};
  gmtime_r(&t, &tm);
  std::ostringstream os;
  os << std::put_time(&tm, "%Y%m%d_%H%M%S");
  return os.str();
}

static std::string clear_zoom_apriltag_profiles_json(bool archive) {
  namespace fs = std::filesystem;

  const std::string stamp = compact_timestamp_for_filename();
  fs::path archiveDir = fs::path(".zoom_apriltag_old") / stamp;

  int moved = 0;
  int removed = 0;
  int skipped = 0;
  std::vector<std::string> affected;

  std::error_code ec;

  if (archive) {
    fs::create_directories(archiveDir, ec);
  }

  const std::vector<std::string> exactFiles = {
    "zoom_apriltag_profile.json",
    "zoom_apriltag_master_profile.json"
  };

  auto handle_file = [&](const fs::path& fp) {
    std::error_code lec;
    if (!fs::exists(fp, lec) || !fs::is_regular_file(fp, lec)) {
      skipped++;
      return;
    }

    affected.push_back(fp.string());

    if (archive) {
      fs::create_directories(archiveDir, lec);
      const fs::path dst = archiveDir / fp.filename();
      fs::rename(fp, dst, lec);
      if (lec) {
        fs::copy_file(fp, dst, fs::copy_options::overwrite_existing, lec);
        if (!lec) fs::remove(fp, lec);
      }
      if (!lec) moved++;
      else skipped++;
    } else {
      fs::remove(fp, lec);
      if (!lec) removed++;
      else skipped++;
    }
  };

  for (const auto& name : exactFiles) {
    handle_file(fs::path(name));
  }

  for (const auto& entry : fs::directory_iterator(".", ec)) {
    if (ec) break;
    const fs::path fp = entry.path();
    const std::string name = fp.filename().string();

    const bool isAnchorProfile =
      name.rfind("zoom_apriltag_profile_anchor_", 0) == 0 &&
      name.size() >= 5 &&
      name.substr(name.size() - 5) == ".json";

    if (isAnchorProfile) {
      handle_file(fp);
    }
  }

  std::ostringstream os;
  os << "{\"ok\":true"
     << ",\"archive\":" << (archive ? "true" : "false")
     << ",\"moved\":" << moved
     << ",\"removed\":" << removed
     << ",\"skipped\":" << skipped
     << ",\"archive_dir\":\"" << json_escape(archive ? archiveDir.string() : "") << "\""
     << ",\"files\":[";
  for (size_t i = 0; i < affected.size(); ++i) {
    if (i) os << ",";
    os << "\"" << json_escape(affected[i]) << "\"";
  }
  os << "]}";
  return os.str();
}

'''

if "clear_zoom_apriltag_profiles_json" not in s:
    if anchor not in s:
      raise SystemExit("ERROR: handle_client anchor not found")
    s = s.replace(anchor, helper + anchor, 1)
    print("OK: inserted clear helper")
else:
    print("SKIP: clear helper already exists")

# 3. Insert route after bodyReq is parsed.
route_anchor = '''  std::string bodyReq = get_body(req);

'''

route = r'''  if (path == "/api/zoom_calibration/clear_profiles") {
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

    if (g_zoomCalibInProgress.load()) {
      const std::string body = "{\"ok\":false,\"error\":\"calibration_running\"}";
      const std::string hdr =
        "HTTP/1.1 409 Conflict\r\nContent-Type: application/json\r\nCache-Control: no-store\r\n"
        "Connection: close\r\nContent-Length: " + std::to_string(body.size()) + "\r\n\r\n";
      send_all(cfd, hdr.data(), hdr.size());
      send_all(cfd, body.data(), body.size());
      ::close(cfd);
      return;
    }

    bool archive = true;
    extract_json_bool_field(bodyReq, "archive", &archive);

    const std::string body = clear_zoom_apriltag_profiles_json(archive);
    const std::string hdr =
      "HTTP/1.1 200 OK\r\nContent-Type: application/json\r\nCache-Control: no-store\r\n"
      "Connection: close\r\nContent-Length: " + std::to_string(body.size()) + "\r\n\r\n";
    send_all(cfd, hdr.data(), hdr.size());
    send_all(cfd, body.data(), body.size());
    ::close(cfd);
    return;
  }

'''

if "/api/zoom_calibration/clear_profiles" not in s:
    if route_anchor not in s:
        raise SystemExit("ERROR: bodyReq route anchor not found")
    s = s.replace(route_anchor, route_anchor + route, 1)
    print("OK: inserted clear_profiles route")
else:
    print("SKIP: clear_profiles route already exists")

p.write_text(s, encoding="utf-8")
print("DONE")
