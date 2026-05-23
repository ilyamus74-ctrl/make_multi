// mjpeg_gst_http.cpp
#include <gst/gst.h>
#include <gst/app/gstappsink.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <ctime>
#include <cctype>
#include <cstdlib>
#include <cstring>
#include <deque>
#include <filesystem>
#include <fstream>
#include <functional>
#include <iomanip>
#include <iostream>
#include <cmath>
#include <map>
#include <set>
#include <unordered_map>
#include <sstream>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <arpa/inet.h>
#include <cerrno>
#include <fcntl.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <termios.h>
#include <unistd.h>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include "../yolov8.h"
#include "../postprocess.h"


struct Opts {
  std::string dev = "/dev/video0";
  int port = 8080;
  int width = 1920;
  int height = 1080;
  int fps = 25;
  int jpegq = 80;
  bool deinterlace = false;

  std::string labels = "model/coco_80_labels_list.txt";
  std::string model_dir = "new_yolo8/model_rknn";
  std::string model = ""; // optional explicit model path
  int cmd_max_pan = 45;   // max absolute value for J pan command
  int cmd_max_tilt = 45;  // max absolute value for J tilt command
  int cmd_max_zoom = 24;  // max absolute value for Z zoom command
  bool zoom_calib_enable = false;
  std::string zoom_calib_uart_dev = "";
  int zoom_calib_uart_baud = 0;
};

static std::atomic<bool> g_running{true};
static std::atomic<bool> g_detectEnabled{false};
static std::mutex g_mtx;
static std::condition_variable g_cv;
static std::vector<uint8_t> g_lastJpeg;
static std::atomic<uint64_t> g_frameId{0};

static std::mutex g_modelMtx;
static rknn_app_context_t g_rknn{};
static bool g_modelReady = false;
static std::string g_currentModel;
static std::vector<std::string> g_availableModels;
static std::atomic<int> g_lastDetections{0};
static int g_cmdMaxPan = 45;
static int g_cmdMaxTilt = 45;
static int g_cmdMaxZoom = 24;

struct DetectionBox {
  uint64_t id = 0;
  int cls_id = -1;
  float prop = 0.f;
  int left = 0;
  int top = 0;
  int right = 0;
  int bottom = 0;
};

struct TrackState {
  uint64_t id = 0;
  DetectionBox box{};
  int missed = 0;
};



enum class TrackerMode {
  Idle,
  Tracking,
  Lost,
  Reacquire
};

struct TrackerRuntimeState {
  uint64_t selected_track_id = 0;
  bool has_selected_track = false;
  TrackerMode mode = TrackerMode::Idle;
  DetectionBox selected_box{};
  bool selected_box_valid = false;
  uint64_t last_frame_id = 0;
  int lost_frames = 0;
  int max_lost_frames = 12;
  std::chrono::steady_clock::time_point last_seen_ts{};
};

enum class DetectionMode {
  FullFrame,
  Roi,
  MultiRoi,
  Tiled,
  Hybrid
};

struct DetectionRoi {
  std::string id;
  bool enabled = true;
  int x = 0;
  int y = 0;
  int w = 0;
  int h = 0;
  int every_n_frames = 1;
  std::set<int> class_filter;
};

static std::mutex g_trackMtx;
static std::vector<TrackState> g_tracks;
static std::atomic<uint64_t> g_nextTrackId{1};
static std::atomic<int> g_trackRetentionFrames{30};
static std::mutex g_trackerStateMtx;
static TrackerRuntimeState g_trackerState;
static std::mutex g_roiMtx;
static DetectionMode g_detectionMode = DetectionMode::FullFrame;
static std::vector<DetectionRoi> g_detectionRois;
static std::atomic<int> g_lastRawBoxes{0};
static std::atomic<int> g_lastNmsBoxes{0};
static std::atomic<int> g_maxDetections{10};
static std::atomic<int> g_maxRawCandidates{50};
static std::string g_roiConfigFile = "detection_roi_config.json";
static std::string g_detectionLimitsFile = "detection_limits.json";

static float track_box_iou(const DetectionBox& a, const DetectionBox& b) {
  const int x1 = std::max(a.left, b.left);
  const int y1 = std::max(a.top, b.top);
  const int x2 = std::min(a.right, b.right);
  const int y2 = std::min(a.bottom, b.bottom);

  const int iw = std::max(0, x2 - x1);
  const int ih = std::max(0, y2 - y1);
  const float inter = static_cast<float>(iw * ih);

  const float areaA = static_cast<float>(
      std::max(0, a.right - a.left) * std::max(0, a.bottom - a.top));
  const float areaB = static_cast<float>(
      std::max(0, b.right - b.left) * std::max(0, b.bottom - b.top));

  const float denom = areaA + areaB - inter;
  if (denom <= 1e-6f) return 0.f;
  return inter / denom;
}

static float track_center_distance_norm(const DetectionBox& a, const DetectionBox& b, int frameW, int frameH) {
  const float acx = 0.5f * static_cast<float>(a.left + a.right);
  const float acy = 0.5f * static_cast<float>(a.top + a.bottom);
  const float bcx = 0.5f * static_cast<float>(b.left + b.right);
  const float bcy = 0.5f * static_cast<float>(b.top + b.bottom);

  const float dx = acx - bcx;
  const float dy = acy - bcy;
  const float diag = std::sqrt(static_cast<float>(frameW * frameW + frameH * frameH));
  if (diag <= 1.f) return 1.f;

  return std::sqrt(dx * dx + dy * dy) / diag;
}

static void assign_stable_track_ids(std::vector<DetectionBox>& boxes, int frameW, int frameH) {
  std::lock_guard<std::mutex> lk(g_trackMtx);

  std::vector<bool> trackUsed(g_tracks.size(), false);

  for (auto& box : boxes) {
    int bestIdx = -1;
    float bestScore = -999.f;

    for (size_t i = 0; i < g_tracks.size(); ++i) {
      if (trackUsed[i]) continue;

      const auto& tr = g_tracks[i];

      if (tr.box.cls_id != box.cls_id) {
        continue;
      }

      const float iou = track_box_iou(box, tr.box);
      const float dist = track_center_distance_norm(box, tr.box, frameW, frameH);

      const bool acceptable = (iou >= 0.18f) || (dist <= 0.08f);
      if (!acceptable) continue;

      const float score = iou * 2.0f - dist;

      if (score > bestScore) {
        bestScore = score;
        bestIdx = static_cast<int>(i);
      }
    }

    if (bestIdx >= 0) {
      auto& tr = g_tracks[bestIdx];
      box.id = tr.id;
      tr.box = box;
      tr.missed = 0;
      trackUsed[bestIdx] = true;
    } else {
      TrackState tr;
      tr.id = g_nextTrackId.fetch_add(1);
      tr.box = box;
      tr.box.id = tr.id;
      tr.missed = 0;

      box.id = tr.id;
      g_tracks.push_back(tr);
      trackUsed.push_back(true);
    }
  }

  for (size_t i = 0; i < g_tracks.size(); ++i) {
    if (i < trackUsed.size() && !trackUsed[i]) {
      g_tracks[i].missed++;
    }
  }

  g_tracks.erase(
      std::remove_if(g_tracks.begin(), g_tracks.end(), [](const TrackState& tr) {
        return tr.missed > g_trackRetentionFrames.load();
      }),
      g_tracks.end());
}

static std::mutex g_detMtx;
static std::vector<DetectionBox> g_lastDetectionBoxes;
static std::atomic<uint64_t> g_detSeq{1};
static std::atomic<int> g_lastFrameWidth{0};
static std::atomic<int> g_lastFrameHeight{0};
static std::mutex g_rangeMtx;
static std::deque<float> g_recentRangeM;
static int g_recentRangeClassId = -1;

struct TrackingConfidenceState {
  float l0_detector = 0.f;
  float l1_continuity = 0.f;
  float l2_motion = 0.f;
  float l3_model_agreement = 0.f;
  float fused_raw = 0.f;
  float fused = 0.f;
  bool cfg_l0_enabled = true;
  bool cfg_l1_enabled = true;
  bool cfg_l2_enabled = true;
  bool cfg_l3_enabled = true;
  float cfg_w0 = 0.50f;
  float cfg_w1 = 0.30f;
  float cfg_w2 = 0.20f;
  float cfg_w3 = 0.25f;
  uint64_t cfg_revision = 1;
  uint64_t frame_id = 0;
  bool has_track = false;
  bool has_prev = false;
  DetectionBox prev_best{};
  std::chrono::steady_clock::time_point prev_ts{};
};

struct TrackingConfidenceConfig {
  bool l0_enabled = true;
  bool l1_enabled = true;
  bool l2_enabled = true;
  bool l3_enabled = true;
  float w0 = 0.50f;
  float w1 = 0.30f;
  float w2 = 0.20f;
  float w3 = 0.25f;
};

static std::mutex g_confMtx;
static TrackingConfidenceState g_confState;
static TrackingConfidenceConfig g_confCfg;
static std::atomic<uint64_t> g_confCfgRevision{1};

static std::mutex g_classMtx;
static std::vector<std::string> g_labels;
static std::set<int> g_selectedClasses;
static std::string g_stateFile = "detector_state.cfg";
static std::string g_settingsFile = "ui_settings.json";
static std::string g_opsLogFile = "ops_events.log";

static std::string g_zoomCalibFile = "zoom_calibration.cfg";
static std::mutex g_zoomCalibMtx;
struct ZoomCalibProfile {
  int cmd_max_zoom = 24;
  int hold_ms = 4000;
  int recalibrate_interval_sec = 300;
  double measured_full_travel_sec = 4.0;
  double zoom_min_stop_sec = 0.0;
  double zoom_max_stop_sec = 0.0;
  std::string method = "time_only";
  std::string speed_mapping_mode = "nearest_step";
  std::string generated_at;
  std::string last_recalibrated_at;
  double repeatability_sec = 0.0;
  double drift_sec = 0.0;
  std::vector<int> cmd_abs = {6, 12, 18, 24};
  std::vector<std::pair<double, double>> time_to_ratio;
  std::vector<std::pair<double, double>> ratio_to_time;
};
static ZoomCalibProfile g_zoomCalib;
static std::atomic<bool> g_zoomRecalibRunning{false};
static std::atomic<bool> g_zoomCalibInProgress{false};
static std::mutex g_zoomCalibStatusMtx;
static std::string g_zoomCalibLastStatus = "idle";
static std::string g_zoomCalibLastMessage = "not_started";
static std::string g_zoomCalibLastRunAt;
static const std::chrono::steady_clock::time_point g_processStartedAt = std::chrono::steady_clock::now();
static std::chrono::steady_clock::time_point g_lastModelSwitchTs = std::chrono::steady_clock::time_point::min();

static std::string trim(const std::string& s) {
  auto b = s.find_first_not_of(" \t\r\n");
  if (b == std::string::npos) return "";
  auto e = s.find_last_not_of(" \t\r\n");
  return s.substr(b, e - b + 1);
}

static std::string now_utc_iso8601();

static std::string extract_json_string_field(const std::string& json, const std::string& key);
static bool extract_json_bool_field(const std::string& json, const std::string& key, bool* out);
static bool extract_json_int_field(const std::string& json, const std::string& key, int* out);
static std::set<int> extract_json_int_array_field(const std::string& json, const std::string& key);

static speed_t baud_to_flag(int baud) {
  switch (baud) {
    case 9600: return B9600;
    case 19200: return B19200;
    case 38400: return B38400;
    case 57600: return B57600;
    case 115200: return B115200;
    case 230400: return B230400;
    default: return B115200;
  }
}

static int open_uart_for_zoom(const std::string& dev, int baud) {
  int fd = ::open(dev.c_str(), O_RDWR | O_NOCTTY);
  if (fd < 0) return -1;
  termios tty{};
  if (tcgetattr(fd, &tty) != 0) {
    ::close(fd);
    return -1;
  }
  cfmakeraw(&tty);
  tty.c_cflag &= ~PARENB;
  tty.c_cflag &= ~CSTOPB;
  tty.c_cflag &= ~CSIZE;
  tty.c_cflag |= CS8;
  tty.c_cflag |= (CLOCAL | CREAD);
  tty.c_cflag &= ~CRTSCTS;
  const speed_t sp = baud_to_flag(baud);
  cfsetispeed(&tty, sp);
  cfsetospeed(&tty, sp);
  if (tcsetattr(fd, TCSANOW, &tty) != 0) {
    ::close(fd);
    return -1;
  }
  return fd;
}

static bool write_uart_line(int fd, const std::string& line) {
  std::string out = line;
  if (out.empty() || out.back() != '\n') out.push_back('\n');
  const char* p = out.data();
  size_t left = out.size();
  while (left > 0) {
    const ssize_t n = ::write(fd, p, left);
    if (n > 0) {
      p += n;
      left -= static_cast<size_t>(n);
      continue;
    }
    if (n < 0 && (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR)) {
      std::this_thread::sleep_for(std::chrono::milliseconds(2));
      continue;
    }
    return false;
  }
  return true;
}

static bool fetch_latest_gray_frame(uint64_t* outFid, cv::Mat* outGray) {
  std::vector<uint8_t> jpeg;
  uint64_t fid = 0;
  {
    std::lock_guard<std::mutex> lk(g_mtx);
    fid = g_frameId.load();
    if (fid == 0 || g_lastJpeg.empty()) return false;
    jpeg = g_lastJpeg;
  }
  cv::Mat bgr = cv::imdecode(jpeg, cv::IMREAD_COLOR);
  if (bgr.empty()) return false;
  cv::Mat gray;
  cv::cvtColor(bgr, gray, cv::COLOR_BGR2GRAY);
  *outFid = fid;
  *outGray = gray;
  return true;
}

static void rebuild_zoom_time_ratio_table_locked() {
  g_zoomCalib.time_to_ratio.clear();
  g_zoomCalib.ratio_to_time.clear();
  const int samples = 8;
  const double travel = std::max(0.5, g_zoomCalib.measured_full_travel_sec);
  for (int i = 0; i <= samples; ++i) {
    const double ratio = static_cast<double>(i) / samples;
    const double t = ratio * travel;
    g_zoomCalib.time_to_ratio.push_back({t, ratio});
    g_zoomCalib.ratio_to_time.push_back({ratio, t});
  }
}

static void generate_zoom_calibration_profile(int cmdMaxZoom, double fullTravelSec = 4.0, const std::string& method = "time_only") {
  std::lock_guard<std::mutex> lk(g_zoomCalibMtx);
  g_zoomCalib.cmd_max_zoom = std::max(1, std::min(100, cmdMaxZoom));
  g_zoomCalib.hold_ms = 4000;
  g_zoomCalib.recalibrate_interval_sec = 300;
  g_zoomCalib.measured_full_travel_sec = std::max(0.5, fullTravelSec);
  g_zoomCalib.zoom_min_stop_sec = g_zoomCalib.measured_full_travel_sec;
  g_zoomCalib.zoom_max_stop_sec = g_zoomCalib.measured_full_travel_sec;
  g_zoomCalib.method = method;
  g_zoomCalib.speed_mapping_mode = "nearest_step";
  g_zoomCalib.generated_at = now_utc_iso8601();
  g_zoomCalib.last_recalibrated_at = g_zoomCalib.generated_at;
  g_zoomCalib.repeatability_sec = 0.0;
  g_zoomCalib.drift_sec = 0.0;
  if (g_zoomCalib.cmd_max_zoom == 34) {
    // Week-3 W3.3: fixed 34/4 graduation (8.5°, 17°, 25°, 34°) with nearest-step mapping.
    g_zoomCalib.cmd_abs = {9, 17, 25, 34};
    rebuild_zoom_time_ratio_table_locked();
    return;
  }
  const double step = static_cast<double>(g_zoomCalib.cmd_max_zoom) / 4.0;
  g_zoomCalib.cmd_abs.clear();
  for (int level = 1; level <= 4; ++level) {
    int val = static_cast<int>(std::round(step * level));
    val = std::max(1, std::min(g_zoomCalib.cmd_max_zoom, val));
    g_zoomCalib.cmd_abs.push_back(val);
  }
  rebuild_zoom_time_ratio_table_locked();
}

static void save_zoom_calibration_profile() {
  std::lock_guard<std::mutex> lk(g_zoomCalibMtx);
  std::ofstream out(g_zoomCalibFile, std::ios::trunc);
  if (!out.is_open()) return;
  out << "generated_at=" << g_zoomCalib.generated_at << "\n";
  out << "cmd_max_zoom=" << g_zoomCalib.cmd_max_zoom << "\n";
  out << "hold_ms=" << g_zoomCalib.hold_ms << "\n";
  out << "recalibrate_interval_sec=" << g_zoomCalib.recalibrate_interval_sec << "\n";
  out << "measured_full_travel_sec=" << std::fixed << std::setprecision(3) << g_zoomCalib.measured_full_travel_sec << "\n";
  out << "zoom_min_stop_sec=" << std::fixed << std::setprecision(3) << g_zoomCalib.zoom_min_stop_sec << "\n";
  out << "zoom_max_stop_sec=" << std::fixed << std::setprecision(3) << g_zoomCalib.zoom_max_stop_sec << "\n";
  out << "method=" << g_zoomCalib.method << "\n";
  out << "speed_mapping_mode=" << g_zoomCalib.speed_mapping_mode << "\n";
  out << "last_recalibrated_at=" << g_zoomCalib.last_recalibrated_at << "\n";
  out << "repeatability_sec=" << std::fixed << std::setprecision(3) << g_zoomCalib.repeatability_sec << "\n";
  out << "drift_sec=" << std::fixed << std::setprecision(3) << g_zoomCalib.drift_sec << "\n";
  out << "cmd_abs=";
  for (size_t i = 0; i < g_zoomCalib.cmd_abs.size(); ++i) {
    if (i) out << ",";
    out << g_zoomCalib.cmd_abs[i];
  }
  out << "\n";
  out << "time_to_ratio=";
  for (size_t i = 0; i < g_zoomCalib.time_to_ratio.size(); ++i) {
    if (i) out << ",";
    out << std::fixed << std::setprecision(3) << g_zoomCalib.time_to_ratio[i].first
        << ":" << std::fixed << std::setprecision(3) << g_zoomCalib.time_to_ratio[i].second;
  }
  out << "\n";
  out << "ratio_to_time=";
  for (size_t i = 0; i < g_zoomCalib.ratio_to_time.size(); ++i) {
    if (i) out << ",";
    out << std::fixed << std::setprecision(3) << g_zoomCalib.ratio_to_time[i].first
        << ":" << std::fixed << std::setprecision(3) << g_zoomCalib.ratio_to_time[i].second;
  }
  out << "\n";
}

static std::string zoom_calibration_json() {
  std::string lastStatus = "idle";
  std::string lastMessage = "not_started";
  std::string lastRunAt;
  {
    std::lock_guard<std::mutex> lk(g_zoomCalibStatusMtx);
    lastStatus = g_zoomCalibLastStatus;
    lastMessage = g_zoomCalibLastMessage;
    lastRunAt = g_zoomCalibLastRunAt;
  }
  std::lock_guard<std::mutex> lk(g_zoomCalibMtx);
  std::ostringstream os;
  os << "{\"source\":\"startup_server\","
     << "\"in_progress\":" << (g_zoomCalibInProgress.load() ? "true" : "false") << ","
     << "\"last_status\":\"" << lastStatus << "\","
     << "\"last_message\":\"" << lastMessage << "\","
     << "\"last_run_at\":\"" << lastRunAt << "\","
     << "\"generated_at\":\"" << g_zoomCalib.generated_at << "\","
     << "\"config_path\":\"" << g_zoomCalibFile << "\","
     << "\"cmd_max_zoom\":" << g_zoomCalib.cmd_max_zoom << ","
     << "\"hold_ms\":" << g_zoomCalib.hold_ms << ","
     << "\"recalibrate_interval_sec\":" << g_zoomCalib.recalibrate_interval_sec << ","
     << "\"measured_full_travel_sec\":" << std::fixed << std::setprecision(3) << g_zoomCalib.measured_full_travel_sec << ","
     << "\"zoom_min_stop_sec\":" << std::fixed << std::setprecision(3) << g_zoomCalib.zoom_min_stop_sec << ","
     << "\"zoom_max_stop_sec\":" << std::fixed << std::setprecision(3) << g_zoomCalib.zoom_max_stop_sec << ","
     << "\"method\":\"" << g_zoomCalib.method << "\","
     << "\"speed_mapping_mode\":\"" << g_zoomCalib.speed_mapping_mode << "\","
     << "\"last_recalibrated_at\":\"" << g_zoomCalib.last_recalibrated_at << "\","
     << "\"repeatability_sec\":" << std::fixed << std::setprecision(3) << g_zoomCalib.repeatability_sec << ","
     << "\"drift_sec\":" << std::fixed << std::setprecision(3) << g_zoomCalib.drift_sec << ","
     << "\"speed_table\":[";
  for (size_t i = 0; i < g_zoomCalib.cmd_abs.size(); ++i) {
    const int level = static_cast<int>(i) + 1;
    const double ratioPerSec = (static_cast<double>(level) / 4.0) / g_zoomCalib.measured_full_travel_sec;
    if (i) os << ",";
    os << "{\"level\":" << level
       << ",\"cmd_abs\":" << g_zoomCalib.cmd_abs[i]
       << ",\"ratio_per_sec\":" << std::fixed << std::setprecision(5) << ratioPerSec
       << "}";
  }
  os << "],\"time_to_ratio\":[";
  for (size_t i = 0; i < g_zoomCalib.time_to_ratio.size(); ++i) {
    if (i) os << ",";
    os << "{\"time_sec\":" << std::fixed << std::setprecision(3) << g_zoomCalib.time_to_ratio[i].first
       << ",\"zoom_ratio\":" << std::fixed << std::setprecision(3) << g_zoomCalib.time_to_ratio[i].second
       << "}";
  }
  os << "],\"ratio_to_time\":[";
  for (size_t i = 0; i < g_zoomCalib.ratio_to_time.size(); ++i) {
    if (i) os << ",";
    os << "{\"zoom_ratio\":" << std::fixed << std::setprecision(3) << g_zoomCalib.ratio_to_time[i].first
       << ",\"time_sec\":" << std::fixed << std::setprecision(3) << g_zoomCalib.ratio_to_time[i].second
       << "}";
  }
  os << "]}";
  return os.str();
}

static bool run_zoom_image_time_calibration(const Opts& o) {
  if (g_zoomCalibInProgress.exchange(true)) {
    std::lock_guard<std::mutex> slk(g_zoomCalibStatusMtx);
    g_zoomCalibLastStatus = "busy";
    g_zoomCalibLastMessage = "calibration_already_running";
    g_zoomCalibLastRunAt = now_utc_iso8601();
    return false;
  }
  struct BusyGuard {
    ~BusyGuard() { g_zoomCalibInProgress = false; }
  } busyGuard;
  {
    std::lock_guard<std::mutex> slk(g_zoomCalibStatusMtx);
    g_zoomCalibLastStatus = "running";
    g_zoomCalibLastMessage = "calibration_started";
    g_zoomCalibLastRunAt = now_utc_iso8601();
  }

  if (!o.zoom_calib_enable) {
    std::lock_guard<std::mutex> slk(g_zoomCalibStatusMtx);
    g_zoomCalibLastStatus = "skipped";
    g_zoomCalibLastMessage = "zoom_calib_disabled";
    g_zoomCalibLastRunAt = now_utc_iso8601();
    return false;
  }
  if (trim(o.zoom_calib_uart_dev).empty()) {
    std::cerr << "zoom-calib: skipped, UART device is not set\n";
    std::lock_guard<std::mutex> slk(g_zoomCalibStatusMtx);
    g_zoomCalibLastStatus = "skipped";
    g_zoomCalibLastMessage = "uart_device_not_set";
    g_zoomCalibLastRunAt = now_utc_iso8601();
    return false;
  }
  if (o.zoom_calib_uart_baud <= 0) {
    std::cerr << "zoom-calib: skipped, UART baud is not set\n";
    std::lock_guard<std::mutex> slk(g_zoomCalibStatusMtx);
    g_zoomCalibLastStatus = "skipped";
    g_zoomCalibLastMessage = "uart_baud_not_set";
    g_zoomCalibLastRunAt = now_utc_iso8601();
    return false;
  }
  int uartFd = open_uart_for_zoom(o.zoom_calib_uart_dev, o.zoom_calib_uart_baud);
  if (uartFd < 0) {
    std::cerr << "zoom-calib: cannot open UART " << o.zoom_calib_uart_dev << "\n";
    std::lock_guard<std::mutex> slk(g_zoomCalibStatusMtx);
    g_zoomCalibLastStatus = "failed";
    g_zoomCalibLastMessage = "uart_open_failed";
    g_zoomCalibLastRunAt = now_utc_iso8601();
    return false;
  }

  auto hold_zoom_to_hard_stop = [&](int cmd, int holdMs) -> double {
    const auto t0 = std::chrono::steady_clock::now();
    write_uart_line(uartFd, std::string("Z ") + std::to_string(cmd));
    while (true) {
      const auto now = std::chrono::steady_clock::now();
      const int elapsedMs = static_cast<int>(std::chrono::duration_cast<std::chrono::milliseconds>(now - t0).count());
      if (elapsedMs >= holdMs) break;
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    write_uart_line(uartFd, "Z 0");
    std::this_thread::sleep_for(std::chrono::milliseconds(150));
    const auto t1 = std::chrono::steady_clock::now();
    return std::chrono::duration_cast<std::chrono::milliseconds>(t1 - t0).count() / 1000.0;
  };

  auto measure_until_visual_stop = [&](int cmd, int maxMs, double diffThreshold, int stableNeed) -> double {
    const auto t0 = std::chrono::steady_clock::now();
    write_uart_line(uartFd, std::string("Z ") + std::to_string(cmd));
    cv::Mat prevGray;
    uint64_t prevFid = 0;
    int stable = 0;
    while (true) {
      const auto now = std::chrono::steady_clock::now();
      const int elapsedMs = static_cast<int>(std::chrono::duration_cast<std::chrono::milliseconds>(now - t0).count());
      if (elapsedMs >= maxMs) break;
      uint64_t fid = 0;
      cv::Mat gray;
      if (!fetch_latest_gray_frame(&fid, &gray)) {
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
        continue;
      }
      if (fid == prevFid || gray.empty()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        continue;
      }
      if (!prevGray.empty() && prevGray.size() == gray.size()) {
        cv::Mat diff;
        cv::absdiff(gray, prevGray, diff);
        const double meanDiff = cv::mean(diff)[0];
        if (elapsedMs > 1200 && meanDiff < diffThreshold) stable++;
        else stable = 0;
        if (stable >= stableNeed) break;
      }
      prevGray = gray;
      prevFid = fid;
      std::this_thread::sleep_for(std::chrono::milliseconds(15));
    }
    write_uart_line(uartFd, "Z 0");
    const auto t1 = std::chrono::steady_clock::now();
    return std::chrono::duration_cast<std::chrono::milliseconds>(t1 - t0).count() / 1000.0;
  };

  // Week-3 W3.2 startup init: hard-stop sweep Z- then Z+ with cmd_max_zoom and 4.0 sec hold.
  const int cmdAbs = std::max(1, o.cmd_max_zoom);
  const double minStopSec = hold_zoom_to_hard_stop(-cmdAbs, 4000); // Z- => zoom_min_stop
  std::this_thread::sleep_for(std::chrono::milliseconds(250));
  const double maxStopSec = hold_zoom_to_hard_stop(cmdAbs, 4000);  // Z+ => zoom_max_stop
  std::this_thread::sleep_for(std::chrono::milliseconds(250));

  const double tWide = measure_until_visual_stop(cmdAbs, 8000, 1.5, 8);
  std::this_thread::sleep_for(std::chrono::milliseconds(250));
  const double tTele = measure_until_visual_stop(-cmdAbs, 8000, 1.5, 8);
  ::close(uartFd);

  const double measuredFullTravel = std::max(1.0, std::max(tWide, tTele));
  generate_zoom_calibration_profile(o.cmd_max_zoom, measuredFullTravel, "time+image");
  {
    std::lock_guard<std::mutex> lk(g_zoomCalibMtx);
    g_zoomCalib.zoom_min_stop_sec = minStopSec;
    g_zoomCalib.zoom_max_stop_sec = maxStopSec;
    g_zoomCalib.repeatability_sec = std::abs(tWide - tTele);
    g_zoomCalib.drift_sec = std::abs(maxStopSec - minStopSec);
    g_zoomCalib.last_recalibrated_at = now_utc_iso8601();
    rebuild_zoom_time_ratio_table_locked();
  }
  save_zoom_calibration_profile();
  {
    std::lock_guard<std::mutex> slk(g_zoomCalibStatusMtx);
    g_zoomCalibLastStatus = "ok";
    g_zoomCalibLastMessage = "calibration_done";
    g_zoomCalibLastRunAt = now_utc_iso8601();
  }
  std::cout << "zoom-calib: done (time+image), travel_sec=" << std::fixed << std::setprecision(3)
            << measuredFullTravel << " wide_sec=" << tWide << " tele_sec=" << tTele
            << " min_stop_sec=" << minStopSec << " max_stop_sec=" << maxStopSec << "\n";
  return true;
}

static void zoom_background_recalibration_thread(const Opts& o) {
  if (!o.zoom_calib_enable || trim(o.zoom_calib_uart_dev).empty() || o.zoom_calib_uart_baud <= 0) return;
  int uartFd = open_uart_for_zoom(o.zoom_calib_uart_dev, o.zoom_calib_uart_baud);
  if (uartFd < 0) {
    std::cerr << "zoom-recalib: skipped, cannot open UART " << o.zoom_calib_uart_dev << "\n";
    return;
  }
  g_zoomRecalibRunning = true;
  bool toTele = true;
  while (g_running.load()) {
    int waitSec = 300;
    {
      std::lock_guard<std::mutex> lk(g_zoomCalibMtx);
      waitSec = std::max(30, g_zoomCalib.recalibrate_interval_sec);
    }
    for (int i = 0; i < waitSec * 10 && g_running.load(); ++i) {
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    if (!g_running.load()) break;

    cv::Mat baseGray;
    uint64_t baseFid = 0;
    if (!fetch_latest_gray_frame(&baseFid, &baseGray) || baseGray.empty()) continue;

    const int cmdAbs = std::max(1, o.cmd_max_zoom / 2);
    const int cmd = toTele ? cmdAbs : -cmdAbs;
    toTele = !toTele;

    write_uart_line(uartFd, std::string("Z ") + std::to_string(cmd));
    std::this_thread::sleep_for(std::chrono::milliseconds(650));
    write_uart_line(uartFd, "Z 0");
    std::this_thread::sleep_for(std::chrono::milliseconds(180));

    cv::Mat afterGray;
    uint64_t afterFid = 0;
    if (!fetch_latest_gray_frame(&afterFid, &afterGray) || afterGray.empty() || afterGray.size() != baseGray.size()) continue;
    cv::Mat diff;
    cv::absdiff(afterGray, baseGray, diff);
    const double meanDiff = cv::mean(diff)[0];

    double updatedTravel = 0.0;
    {
      std::lock_guard<std::mutex> lk(g_zoomCalibMtx);
      const double prevTravel = std::max(0.5, g_zoomCalib.measured_full_travel_sec);
      const bool weakMotion = meanDiff < 1.0;
      const double sampleTravel = weakMotion ? (prevTravel * 1.02) : (prevTravel * 0.995);
      g_zoomCalib.measured_full_travel_sec = std::max(0.5, 0.9 * prevTravel + 0.1 * sampleTravel);
      g_zoomCalib.drift_sec = std::abs(g_zoomCalib.measured_full_travel_sec - prevTravel);
      g_zoomCalib.last_recalibrated_at = now_utc_iso8601();
      g_zoomCalib.method = "time+image+background";
      rebuild_zoom_time_ratio_table_locked();
      updatedTravel = g_zoomCalib.measured_full_travel_sec;
    }
    save_zoom_calibration_profile();
    std::cout << "zoom-recalib: drift update (mean_diff=" << std::fixed << std::setprecision(3)
              << meanDiff << ", travel_sec=" << updatedTravel << ")\n";
  }
  write_uart_line(uartFd, "Z 0");
  ::close(uartFd);
  g_zoomRecalibRunning = false;
}

static bool extract_json_bool_field(const std::string& json, const std::string& key, bool* out) {
  const std::string pattern = "\"" + key + "\"";
  const auto keyPos = json.find(pattern);
  if (keyPos == std::string::npos) return false;
  auto colonPos = json.find(':', keyPos + pattern.size());
  if (colonPos == std::string::npos) return false;
  auto valPos = json.find_first_not_of(" \t\r\n", colonPos + 1);
  if (valPos == std::string::npos) return false;
  if (json.compare(valPos, 4, "true") == 0) {
    *out = true;
    return true;
  }
  if (json.compare(valPos, 5, "false") == 0) {
    *out = false;
    return true;
  }
  return false;
}

static bool extract_json_float_field(const std::string& json, const std::string& key, float* out) {
  const std::string pattern = "\"" + key + "\"";
  const auto keyPos = json.find(pattern);
  if (keyPos == std::string::npos) return false;
  auto colonPos = json.find(':', keyPos + pattern.size());
  if (colonPos == std::string::npos) return false;
  auto valPos = json.find_first_not_of(" \t\r\n", colonPos + 1);
  if (valPos == std::string::npos) return false;
  const std::string allowed = "+-0123456789.eE";
  auto endPos = json.find_first_not_of(allowed, valPos);
  const std::string token = json.substr(valPos, endPos == std::string::npos ? std::string::npos : endPos - valPos);
  if (token.empty()) return false;
  try {
    *out = std::stof(token);
    return true;
  } catch (...) {
    return false;
  }
}

static std::set<int> extract_json_int_array_field(const std::string& json, const std::string& key) {
  std::set<int> out;
  const std::string pattern = "\"" + key + "\"";
  const auto keyPos = json.find(pattern);
  if (keyPos == std::string::npos) return out;
  auto colonPos = json.find(':', keyPos + pattern.size());
  if (colonPos == std::string::npos) return out;
  auto arrBegin = json.find('[', colonPos + 1);
  if (arrBegin == std::string::npos) return out;
  auto arrEnd = json.find(']', arrBegin + 1);
  if (arrEnd == std::string::npos || arrEnd <= arrBegin + 1) return out;
  std::stringstream ss(json.substr(arrBegin + 1, arrEnd - arrBegin - 1));
  std::string tok;
  while (std::getline(ss, tok, ',')) {
    tok = trim(tok);
    if (tok.empty()) continue;
    try {
      out.insert(std::stoi(tok));
    } catch (...) {
    }
  }
  return out;
}

static bool extract_json_int_field(const std::string& json, const std::string& key, int* out) {
  if (!out) return false;
  const std::string token = "\"" + key + "\"";
  size_t p = json.find(token);
  if (p == std::string::npos) return false;
  p = json.find(':', p + token.size());
  if (p == std::string::npos) return false;
  ++p;
  while (p < json.size() && std::isspace(static_cast<unsigned char>(json[p]))) ++p;
  size_t e = p;
  if (e < json.size() && (json[e] == '-' || json[e] == '+')) ++e;
  while (e < json.size() && std::isdigit(static_cast<unsigned char>(json[e]))) ++e;
  if (e <= p) return false;
  try {
    *out = std::stoi(json.substr(p, e - p));
    return true;
  } catch (...) {
    return false;
  }
}

static void save_state() {
  std::lock_guard<std::mutex> lk1(g_modelMtx);
  std::lock_guard<std::mutex> lk2(g_classMtx);
  std::ofstream out(g_stateFile, std::ios::trunc);
  if (!out.is_open()) return;
  out << "detect=" << (g_detectEnabled.load() ? "1" : "0") << "\n";
  out << "model=" << g_currentModel << "\n";
  out << "classes=";
  bool first = true;
  for (int cls : g_selectedClasses) {
    if (!first) out << ',';
    out << cls;
    first = false;
  }
  out << "\n";
}

static void load_state(std::string& modelFromState, bool* detectFromState) {
  std::ifstream in(g_stateFile);
  if (!in.is_open()) return;

  std::set<int> loadedClasses;
  bool hasClassesLine = false;
  std::string line;
  while (std::getline(in, line)) {
    line = trim(line);
    if (line.rfind("detect=", 0) == 0) {
      const std::string v = trim(line.substr(7));
      if (v == "1" || v == "true" || v == "on") *detectFromState = true;
      if (v == "0" || v == "false" || v == "off") *detectFromState = false;
    } else if (line.rfind("model=", 0) == 0) {
      modelFromState = trim(line.substr(6));
    } else if (line.rfind("classes=", 0) == 0) {
      hasClassesLine = true;
      std::string v = line.substr(8);
      std::stringstream ss(v);
      std::string item;
      while (std::getline(ss, item, ',')) {
        item = trim(item);
        if (item.empty()) continue;
        try {
          int cls = std::stoi(item);
          if (cls >= 0) loadedClasses.insert(cls);
        } catch (...) {
        }
      }
    }
  }

  {
    std::lock_guard<std::mutex> lk(g_classMtx);
    if (hasClassesLine) g_selectedClasses = std::move(loadedClasses);
  }
}

static std::vector<std::string> read_labels(const std::string& labelsPath) {
  std::vector<std::string> labels;
  std::ifstream in(labelsPath);
  if (!in.is_open()) return labels;
  std::string line;
  while (std::getline(in, line)) {
    line = trim(line);
    if (!line.empty()) labels.push_back(line);
  }
  return labels;
}

static bool send_all(int fd, const void* data, size_t len) {
  const uint8_t* p = (const uint8_t*)data;
  while (len) {
    ssize_t n = ::send(fd, p, len, MSG_NOSIGNAL);
    if (n <= 0) return false;
    p += (size_t)n;
    len -= (size_t)n;
  }
  return true;
}

static std::string read_http_request(int fd) {
  std::string req;
  char buf[2048];
  while (req.find("\r\n\r\n") == std::string::npos) {
    ssize_t n = ::recv(fd, buf, sizeof(buf), 0);
    if (n <= 0) break;
    req.append(buf, buf + n);
    if (req.size() > 8192) break;
  }

  auto hdrEnd = req.find("\r\n\r\n");
  if (hdrEnd == std::string::npos) return req;

  size_t contentLength = 0;
  const std::string key = "Content-Length:";
  auto pos = req.find(key);
  if (pos != std::string::npos && pos < hdrEnd) {
    pos += key.size();
    while (pos < hdrEnd && std::isspace(static_cast<unsigned char>(req[pos]))) ++pos;
    size_t end = pos;
    while (end < hdrEnd && std::isdigit(static_cast<unsigned char>(req[end]))) ++end;
    if (end > pos) {
      try {
        contentLength = static_cast<size_t>(std::stoul(req.substr(pos, end - pos)));
      } catch (...) {
        contentLength = 0;
      }
    }
  }

  const size_t bodyOffset = hdrEnd + 4;
  size_t haveBody = req.size() > bodyOffset ? (req.size() - bodyOffset) : 0;
  while (haveBody < contentLength && req.size() <= (1024 * 1024)) {
    ssize_t n = ::recv(fd, buf, sizeof(buf), 0);
    if (n <= 0) break;
    req.append(buf, buf + n);
    haveBody = req.size() > bodyOffset ? (req.size() - bodyOffset) : 0;
  }
  return req;
}


static std::string get_method(const std::string& req) {
  auto sp = req.find(' ');
  if (sp == std::string::npos) return "GET";
  return req.substr(0, sp);
}
static std::string get_target(const std::string& req) {
  auto p = req.find("GET ");
  if (p == std::string::npos) p = req.find("POST ");
  if (p == std::string::npos) return "/";

  p = req.find(' ', p);
  if (p == std::string::npos) return "/";
  ++p;
  auto sp = req.find(' ', p);
  if (sp == std::string::npos) return "/";
  return req.substr(p, sp - p);
}
static std::string get_body(const std::string& req) {
  auto p = req.find("\r\n\r\n");
  if (p == std::string::npos) return "";
  return req.substr(p + 4);
}
static std::string path_only(const std::string& target) {
  auto q = target.find('?');
  return q == std::string::npos ? target : target.substr(0, q);
}

static int hex_value(char c) {
  if (c >= '0' && c <= '9') return c - '0';
  if (c >= 'a' && c <= 'f') return 10 + (c - 'a');
  if (c >= 'A' && c <= 'F') return 10 + (c - 'A');
  return -1;
}

static std::string url_decode(const std::string& s) {
  std::string out;
  out.reserve(s.size());
  for (size_t i = 0; i < s.size(); ++i) {
    if (s[i] == '+' ) {
      out.push_back(' ');
      continue;
    }
    if (s[i] == '%' && i + 2 < s.size()) {
      int hi = hex_value(s[i + 1]);
      int lo = hex_value(s[i + 2]);
      if (hi >= 0 && lo >= 0) {
        out.push_back(static_cast<char>((hi << 4) | lo));
        i += 2;
        continue;
      }
    }
    out.push_back(s[i]);
  }
  return out;
}

static std::string query_param(const std::string& target, const std::string& key) {
  auto q = target.find('?');
  if (q == std::string::npos) return "";
  std::string query = target.substr(q + 1);
  std::string pat = key + "=";
  auto p = query.find(pat);
  if (p == std::string::npos) return "";
  p += pat.size();
  auto e = query.find('&', p);
  return url_decode(query.substr(p, e == std::string::npos ? std::string::npos : e - p));
}

static int query_param_int(const std::string& target, const std::string& key, int fallback, int minVal, int maxVal) {
  const std::string raw = query_param(target, key);
  if (raw.empty()) return fallback;
  char* end = nullptr;
  long v = std::strtol(raw.c_str(), &end, 10);
  if (end == raw.c_str() || *end != '\0') return fallback;
  if (v < static_cast<long>(minVal) || v > static_cast<long>(maxVal)) return fallback;
  return static_cast<int>(v);
}

static std::string utc_iso8601_seconds_ago(int secondsAgo) {
  if (secondsAgo <= 0) return "";
  const auto now = std::chrono::system_clock::now();
  const auto cutoff = now - std::chrono::seconds(secondsAgo);
  const std::time_t t = std::chrono::system_clock::to_time_t(cutoff);
  std::tm tm{};
  gmtime_r(&t, &tm);
  std::ostringstream os;
  os << std::put_time(&tm, "%Y-%m-%dT%H:%M:%SZ");
  return os.str();
}

static std::vector<std::string> discover_models(const Opts& o) {
  std::vector<std::string> out;
  std::vector<std::filesystem::path> dirs;


  auto has_rknn_extension = [](const std::filesystem::path& p) {
    std::string ext = p.extension().string();
    std::transform(ext.begin(), ext.end(), ext.begin(), [](unsigned char c) {
      return static_cast<char>(std::tolower(c));
    });
    return ext == ".rknn";
  };

  auto push_model_if_valid = [&](const std::filesystem::path& p) {
    if (has_rknn_extension(p)) out.push_back(p.string());
  };

  std::error_code ec;
  std::filesystem::path modelPath = o.model_dir;
  if (!modelPath.empty() && std::filesystem::exists(modelPath, ec) && std::filesystem::is_regular_file(modelPath, ec)) {
    push_model_if_valid(modelPath);
    if (modelPath.has_parent_path()) dirs.push_back(modelPath.parent_path());
  } else {
    dirs.push_back(modelPath);
  }

  dirs.push_back("new_yolo8/models");
  dirs.push_back("new_yolo8/model_rknn");
  dirs.push_back("models");
  dirs.push_back("model_rknn");


  for (const auto& d : dirs) {
    if (!std::filesystem::exists(d, ec)) continue;
    if (!std::filesystem::is_directory(d, ec)) continue;
    for (const auto& e : std::filesystem::directory_iterator(d, ec)) {
      if (ec) break;
      if (!e.is_regular_file()) continue;
      push_model_if_valid(e.path());
    }
  }

  std::sort(out.begin(), out.end());
  out.erase(std::unique(out.begin(), out.end()), out.end());
  return out;
}

static bool load_model_locked(const std::string& modelPath) {
  if (g_modelReady) {
    release_yolov8_model(&g_rknn);
    g_modelReady = false;
    g_currentModel.clear();
  }

  if (init_yolov8_model(modelPath.c_str(), &g_rknn) != 0) {
    std::cerr << "init_yolov8_model failed: " << modelPath << "\n";
    return false;
  }

  if (rknn_set_core_mask(g_rknn.rknn_ctx, RKNN_NPU_CORE_0_1_2) != RKNN_SUCC) {
    std::cerr << "warn: failed to set RKNN_NPU_CORE_0_1_2\n";
  }

  g_modelReady = true;
  g_currentModel = modelPath;
  std::cout << "Loaded model: " << g_currentModel << "\n";
  return true;
}

static bool load_model(const std::string& modelPath) {
  std::lock_guard<std::mutex> lk(g_modelMtx);
  return load_model_locked(modelPath);
}

static std::string json_escape(const std::string& s) {
  std::string out;
  out.reserve(s.size() + 8);
  for (char c : s) {
    if (c == '\\' || c == '"') out.push_back('\\');
    out.push_back(c);
  }
  return out;
}


static std::string now_utc_iso8601();

static void append_ops_event(const std::string& evType, const std::string& payloadJson) {
  if (evType.empty()) return;
  std::ofstream out(g_opsLogFile, std::ios::app | std::ios::binary);
  if (!out.is_open()) return;
  out << now_utc_iso8601() << " type=" << evType;
  if (!payloadJson.empty()) out << " payload=" << payloadJson;
  out << "\n";
}

static std::vector<std::string> read_label_file_lines(const std::filesystem::path& filePath) {
  std::vector<std::string> labels;
  std::ifstream in(filePath);
  if (!in.is_open()) return labels;
  std::string line;
  while (std::getline(in, line)) {
    line = trim(line);
    if (!line.empty()) labels.push_back(line);
  }
  return labels;
}

static std::string models_json() {
  std::lock_guard<std::mutex> lk1(g_modelMtx);
  std::lock_guard<std::mutex> lk2(g_classMtx);
  std::ostringstream os;
  os << "{\"current\":\"" << json_escape(g_currentModel) << "\",\"detect\":"
     << (g_detectEnabled.load() ? "true" : "false")
     << ",\"lastDetections\":" << g_lastDetections.load() << ",\"models\":[";
  for (size_t i = 0; i < g_availableModels.size(); ++i) {
    if (i) os << ',';
    os << '"' << json_escape(g_availableModels[i]) << '"';
  }

  os << "],\"labels\":[";
  for (size_t i = 0; i < g_labels.size(); ++i) {
    if (i) os << ',';
    os << '"' << json_escape(g_labels[i]) << '"';
  }
  os << "],\"selectedClasses\":[";
  bool first = true;
  for (int cls : g_selectedClasses) {
    if (!first) os << ',';
    os << cls;
    first = false;
  }
  os << "]}";
  return os.str();
}


static const char* tracker_mode_str(TrackerMode m) {
  switch (m) {
    case TrackerMode::Idle: return "IDLE";
    case TrackerMode::Tracking: return "TRACKING";
    case TrackerMode::Lost: return "LOST";
    case TrackerMode::Reacquire: return "REACQUIRE";
  }
  return "IDLE";
}

static const char* detection_mode_str(DetectionMode m) {
  switch (m) {
    case DetectionMode::FullFrame: return "full_frame";
    case DetectionMode::Roi: return "roi";
    case DetectionMode::MultiRoi: return "multi_roi";
    case DetectionMode::Tiled: return "tiled";
    case DetectionMode::Hybrid: return "hybrid";
  }
  return "full_frame";
}

static DetectionMode parse_detection_mode(const std::string& s) {
  if (s == "roi") return DetectionMode::Roi;
  if (s == "multi_roi") return DetectionMode::MultiRoi;
  if (s == "tiled") return DetectionMode::Tiled;
  if (s == "hybrid") return DetectionMode::Hybrid;
  return DetectionMode::FullFrame;
}

static std::string roi_config_json() {
  std::lock_guard<std::mutex> lk(g_roiMtx);
  std::ostringstream os;
  os << "{\"ok\":true,\"detection_mode\":\"" << detection_mode_str(g_detectionMode) << "\",\"rois\":[";
  for (size_t i = 0; i < g_detectionRois.size(); ++i) {
    if (i) os << ",";
    const auto& r = g_detectionRois[i];
    os << "{\"id\":\"" << json_escape(r.id) << "\""
       << ",\"enabled\":" << (r.enabled ? "true" : "false")
       << ",\"x\":" << r.x
       << ",\"y\":" << r.y
       << ",\"w\":" << r.w
       << ",\"h\":" << r.h
       << ",\"every_n_frames\":" << r.every_n_frames
       << ",\"classes\":[";
    bool first = true;
    for (int cls : r.class_filter) {
      if (!first) os << ",";
      os << cls;
      first = false;
    }
    os << "]}";
  }
  os << "]}";
  return os.str();
}

static size_t find_matching_json_bracket(const std::string& s, size_t openPos, char openCh, char closeCh) {
  if (openPos >= s.size() || s[openPos] != openCh) {
    return std::string::npos;
  }

  int depth = 0;
  bool inString = false;
  bool escape = false;

  for (size_t i = openPos; i < s.size(); ++i) {
    const char c = s[i];

    if (inString) {
      if (escape) {
        escape = false;
      } else if (c == '\\') {
        escape = true;
      } else if (c == '"') {
        inString = false;
      }
      continue;
    }

    if (c == '"') {
      inString = true;
      continue;
    }

    if (c == openCh) {
      ++depth;
    } else if (c == closeCh) {
      --depth;
      if (depth == 0) {
        return i;
      }
    }
  }

  return std::string::npos;
}

static bool parse_roi_config_json(const std::string& payload, int frameW, int frameH) {
  if (payload.empty() || payload.front() != '{' || payload.back() != '}') return false;
  const std::string modeStr = trim(extract_json_string_field(payload, "detection_mode"));
  DetectionMode mode = parse_detection_mode(modeStr);

  std::vector<DetectionRoi> parsedRois;
  size_t roisPos = payload.find("\"rois\"");
  if (roisPos != std::string::npos) {
    size_t arrBegin = payload.find('[', roisPos);
    if (arrBegin == std::string::npos) return false;
    size_t arrEnd = find_matching_json_bracket(payload, arrBegin, '[', ']');
    if (arrEnd == std::string::npos) return false;
    size_t p = arrBegin + 1;
    while (true) {
      size_t ob = payload.find('{', p);
      if (ob == std::string::npos || ob > arrEnd) break;
      size_t cb = payload.find('}', ob + 1);
      if (cb == std::string::npos || cb > arrEnd) return false;
      std::string obj = payload.substr(ob, cb - ob + 1);
      DetectionRoi r;
      r.id = trim(extract_json_string_field(obj, "id"));
      if (r.id.empty()) r.id = "roi_" + std::to_string(parsedRois.size());
      extract_json_bool_field(obj, "enabled", &r.enabled);
      extract_json_int_field(obj, "x", &r.x);
      extract_json_int_field(obj, "y", &r.y);
      extract_json_int_field(obj, "w", &r.w);
      extract_json_int_field(obj, "h", &r.h);
      extract_json_int_field(obj, "every_n_frames", &r.every_n_frames);
      r.class_filter = extract_json_int_array_field(obj, "classes");
      r.x = std::max(0, r.x);
      r.y = std::max(0, r.y);
      r.w = std::max(1, r.w);
      r.h = std::max(1, r.h);
      r.every_n_frames = std::max(1, std::min(300, r.every_n_frames));
      if (frameW > 0) {
        if (r.x >= frameW) r.x = frameW - 1;
        r.w = std::min(r.w, frameW - r.x);
      }
      if (frameH > 0) {
        if (r.y >= frameH) r.y = frameH - 1;
        r.h = std::min(r.h, frameH - r.y);
      }
      r.w = std::max(1, r.w);
      r.h = std::max(1, r.h);
      parsedRois.push_back(r);
      p = cb + 1;
    }
  }
  {
    std::lock_guard<std::mutex> lk(g_roiMtx);
    g_detectionMode = mode;
    g_detectionRois = std::move(parsedRois);
  }
  return true;
}

static void save_roi_config() {
  std::ofstream out(g_roiConfigFile, std::ios::trunc | std::ios::binary);
  if (!out) return;
  out << roi_config_json();
}

static void load_roi_config() {
  std::ifstream in(g_roiConfigFile, std::ios::binary);
  if (!in) return;
  std::ostringstream ss;
  ss << in.rdbuf();
  const std::string payload = trim(ss.str());
  if (payload.empty()) return;
  parse_roi_config_json(payload, g_lastFrameWidth.load(), g_lastFrameHeight.load());
}


static void limit_detection_boxes_by_confidence(std::vector<DetectionBox>& boxes, int maxBoxes) {
  if (maxBoxes <= 0) return;
  if (static_cast<int>(boxes.size()) <= maxBoxes) return;

  std::sort(boxes.begin(), boxes.end(), [](const DetectionBox& a, const DetectionBox& b) {
    return a.prop > b.prop;
  });

  boxes.resize(maxBoxes);
}

static void save_detection_limits() {
  std::ofstream out(g_detectionLimitsFile, std::ios::trunc | std::ios::binary);
  if (!out) return;
  out << "{\"ok\":true,\"max_detections\":" << g_maxDetections.load()
      << ",\"max_raw_candidates\":" << g_maxRawCandidates.load() << "}";
}

static void load_detection_limits() {
  std::ifstream in(g_detectionLimitsFile, std::ios::binary);
  if (!in) return;
  std::ostringstream ss;
  ss << in.rdbuf();
  const std::string payload = trim(ss.str());
  if (payload.empty()) return;

  int maxDet = g_maxDetections.load();
  int maxRaw = g_maxRawCandidates.load();
  extract_json_int_field(payload, "max_detections", &maxDet);
  extract_json_int_field(payload, "max_raw_candidates", &maxRaw);

  maxDet = std::max(1, std::min(100, maxDet));
  maxRaw = std::max(maxDet, std::min(300, std::max(1, maxRaw)));

  g_maxDetections = maxDet;
  g_maxRawCandidates = maxRaw;
}

static void update_tracker_runtime_state(const std::vector<DetectionBox>& boxes, uint64_t frameId) {
  std::lock_guard<std::mutex> lk(g_trackerStateMtx);
  g_trackerState.last_frame_id = frameId;
  if (g_trackerState.selected_track_id == 0 || !g_trackerState.has_selected_track) {
    g_trackerState.mode = TrackerMode::Idle;
    g_trackerState.selected_box_valid = false;
    return;
  }
  for (const auto& box : boxes) {
    if (box.id == g_trackerState.selected_track_id) {
      g_trackerState.mode = TrackerMode::Tracking;
      g_trackerState.selected_box = box;
      g_trackerState.selected_box_valid = true;
      g_trackerState.lost_frames = 0;
      g_trackerState.last_seen_ts = std::chrono::steady_clock::now();
      return;
    }
  }
  g_trackerState.lost_frames++;
  g_trackerState.selected_box_valid = false;
  g_trackerState.mode = (g_trackerState.lost_frames <= g_trackerState.max_lost_frames)
      ? TrackerMode::Lost : TrackerMode::Reacquire;
}

static std::vector<DetectionBox> nms_detection_boxes(const std::vector<DetectionBox>& boxes, float iou_threshold) {
  std::vector<DetectionBox> sorted = boxes, out;
  std::sort(sorted.begin(), sorted.end(), [](const DetectionBox& a, const DetectionBox& b) { return a.prop > b.prop; });
  std::vector<bool> sup(sorted.size(), false);
  for (size_t i = 0; i < sorted.size(); ++i) {
    if (sup[i]) continue;
    out.push_back(sorted[i]);
    for (size_t j = i + 1; j < sorted.size(); ++j) {
      if (sup[j] || sorted[j].cls_id != sorted[i].cls_id) continue;
      if (track_box_iou(sorted[i], sorted[j]) > iou_threshold) sup[j] = true;
    }
  }
  return out;
}

static std::string detections_json() {
  std::lock_guard<std::mutex> lk(g_detMtx);
  float l0 = 0.f, l1 = 0.f, l2 = 0.f, l3 = 0.f, fused = 0.f;
  uint64_t confFrameId = 0;
  bool hasTrack = false;
  TrackerRuntimeState tr{};
  DetectionMode dm = DetectionMode::FullFrame;
  size_t roiCount = 0;
  {
    std::lock_guard<std::mutex> lk2(g_confMtx);
    l0 = g_confState.l0_detector;
    l1 = g_confState.l1_continuity;
    l2 = g_confState.l2_motion;
    l3 = g_confState.l3_model_agreement;
    fused = g_confState.fused;
    confFrameId = g_confState.frame_id;
    hasTrack = g_confState.has_track;
  }
  {
    std::lock_guard<std::mutex> lk3(g_trackerStateMtx);
    tr = g_trackerState;
  }
  {
    std::lock_guard<std::mutex> lk4(g_roiMtx);
    dm = g_detectionMode;
    roiCount = g_detectionRois.size();
  }
  std::ostringstream os;
  os << "{\"frameId\":" << g_frameId.load()
     << ",\"width\":" << g_lastFrameWidth.load()
     << ",\"height\":" << g_lastFrameHeight.load()
     << ",\"detect\":" << (g_detectEnabled.load() ? "true" : "false")
     << ",\"tracker\":{\"mode\":\"" << tracker_mode_str(tr.mode) << "\""
     << ",\"selected_track_id\":" << tr.selected_track_id
     << ",\"selected_box_valid\":" << (tr.selected_box_valid ? "true" : "false")
     << ",\"lost_frames\":" << tr.lost_frames << "}"
     << ",\"confidence\":{\"frameId\":" << confFrameId
     << ",\"hasTrack\":" << (hasTrack ? "true" : "false")
     << ",\"l0_detector\":" << l0
     << ",\"l1_continuity\":" << l1
     << ",\"l2_motion\":" << l2
     << ",\"l3_model_agreement\":" << l3
     << ",\"fused\":" << fused << "}"
     << ",\"detection_debug\":{\"mode\":\"" << detection_mode_str(dm) << "\""
     << ",\"roi_count\":" << roiCount
     << ",\"raw_boxes\":" << g_lastRawBoxes.load()
     << ",\"raw_limit\":" << g_maxRawCandidates.load()
     << ",\"nms_boxes\":" << g_lastNmsBoxes.load()
     << ",\"max_detections\":" << g_maxDetections.load()
     << "}"
     << ",\"items\":[";
  for (size_t i = 0; i < g_lastDetectionBoxes.size(); ++i) {
    if (i) os << ',';
    const auto& d = g_lastDetectionBoxes[i];
    os << "{\"id\":" << d.id
       << ",\"cls\":" << d.cls_id
       << ",\"prop\":" << d.prop
       << ",\"source\":\"DET\""
       << ",\"left\":" << d.left
       << ",\"top\":" << d.top
       << ",\"right\":" << d.right
       << ",\"bottom\":" << d.bottom << '}';
  }
  os << "]}";
  return os.str();
}

static float clamp01(float v);
static float median_of(std::vector<float> v);

static std::string range_estimate_json() {
  DetectionBox best{};
  bool has = false;
  int frameH = g_lastFrameHeight.load();
  int preferredCls = -1;
  {
    std::lock_guard<std::mutex> lk(g_classMtx);
    if (!g_selectedClasses.empty()) preferredCls = *g_selectedClasses.begin();
  }
  {
    std::lock_guard<std::mutex> lk(g_detMtx);
    for (const auto& b : g_lastDetectionBoxes) {
      if (!has) {
        best = b;
        has = true;
        continue;
      }
      const bool curPreferred = preferredCls >= 0 && b.cls_id == preferredCls;
      const bool bestPreferred = preferredCls >= 0 && best.cls_id == preferredCls;
      if (curPreferred && !bestPreferred) {
        best = b;
        continue;
      }
      if (b.prop > best.prop) best = b;
    }
  }
  if (!has || frameH <= 0) {
    return "{\"ok\":false,\"reason\":\"no_detection\"}";
  }

  const int boxHpx = std::max(1, best.bottom - best.top);
  const float bboxHeightNorm = static_cast<float>(boxHpx) / static_cast<float>(frameH);
  if (bboxHeightNorm <= 0.001f) {
    return "{\"ok\":false,\"reason\":\"bbox_too_small\"}";
  }


  std::string className = "unknown";
  {
    std::lock_guard<std::mutex> lk(g_classMtx);
    if (best.cls_id >= 0 && best.cls_id < static_cast<int>(g_labels.size())) {
      className = g_labels[best.cls_id];
    }
  }
  std::string classKey = className;
  std::transform(classKey.begin(), classKey.end(), classKey.begin(), [](unsigned char c) {
    return static_cast<char>(std::tolower(c));
  });

  float objectHeightM = 1.70f;
  std::string heightSource = "default_person";
  static const std::unordered_map<std::string, float> kClassHeightM = {
      {"person", 1.70f},
      {"bicycle", 1.10f},
      {"motorbike", 1.30f},
      {"motorcycle", 1.30f},
      {"car", 1.50f},
      {"bus", 3.20f},
      {"truck", 3.40f},
  };
  auto it = kClassHeightM.find(classKey);
  if (it != kClassHeightM.end()) {
    objectHeightM = it->second;
    heightSource = "class_prior";
  }

  constexpr float kVerticalFovDeg = 58.0f;
  constexpr float kPi = 3.14159265358979323846f;
  const float halfFovRad = (kVerticalFovDeg * 0.5f) * kPi / 180.0f;
  const float viewHeightAt1m = 2.0f * std::tan(halfFovRad);
  const float distanceMRaw = objectHeightM / (bboxHeightNorm * viewHeightAt1m);
  float distanceM = distanceMRaw;
  float distanceSmoothDeltaM = 0.0f;
  {
    std::lock_guard<std::mutex> lk(g_rangeMtx);
    if (g_recentRangeClassId != best.cls_id) {
      g_recentRangeM.clear();
      g_recentRangeClassId = best.cls_id;
    }
    g_recentRangeM.push_back(distanceMRaw);
    constexpr size_t kRangeWindow = 5;
    while (g_recentRangeM.size() > kRangeWindow) g_recentRangeM.pop_front();
    std::vector<float> sorted(g_recentRangeM.begin(), g_recentRangeM.end());
    distanceM = median_of(sorted);
    distanceSmoothDeltaM = std::fabs(distanceMRaw - distanceM);
  }
  const float confidence = clamp01(0.7f * best.prop + 0.3f * std::min(1.0f, bboxHeightNorm * 4.0f));
  const float smoothPenalty = std::min(0.2f, distanceSmoothDeltaM / std::max(1.0f, distanceM));
  const float errRatio = 0.35f + (1.0f - confidence) * 0.45f + smoothPenalty;
  const float minM = std::max(0.2f, distanceM * (1.0f - errRatio));
  const float maxM = distanceM * (1.0f + errRatio);

  std::ostringstream ss;
  ss << std::fixed << std::setprecision(3)
     << "{\"ok\":true"
     << ",\"method\":\"bbox_height_fov\""
     << ",\"distance_m\":" << distanceM
     << ",\"distance_raw_m\":" << distanceMRaw
     << ",\"min_m\":" << minM
     << ",\"max_m\":" << maxM
     << ",\"confidence\":" << confidence
     << ",\"smooth_delta_m\":" << distanceSmoothDeltaM
     << ",\"object_height_m\":" << objectHeightM
     << ",\"height_source\":\"" << json_escape(heightSource) << "\""
     << ",\"vertical_fov_deg\":" << kVerticalFovDeg
     << ",\"bbox_height_norm\":" << bboxHeightNorm
     << ",\"bbox_height_px\":" << boxHpx
     << ",\"class_id\":" << best.cls_id
     << ",\"class_name\":\"" << json_escape(className) << "\""
     << ",\"score\":" << best.prop
     << "}";
  return ss.str();
}

static std::string event_type_from_log_line(const std::string& line) {
  const std::string key = "type=";
  const auto pos = line.find(key);
  if (pos == std::string::npos) return "";
  auto begin = pos + key.size();
  auto end = line.find_first_of(" \t\r\n", begin);
  if (end == std::string::npos) end = line.size();
  return line.substr(begin, end - begin);
}

static std::string events_summary_json(const std::string& typeFilter, size_t recentLimit = 0, int sinceSec = 0) {
  std::ifstream in(g_opsLogFile, std::ios::binary);
  std::deque<std::string> recentLines;
  std::map<std::string, uint64_t> counts;
  uint64_t total = 0;
  std::string lastType;
  std::string lastTs;
  const std::string cutoffTs = utc_iso8601_seconds_ago(sinceSec);
  std::string line;
  while (std::getline(in, line)) {
    if (!cutoffTs.empty() && line.size() >= cutoffTs.size() && line.compare(0, cutoffTs.size(), cutoffTs) < 0) continue;
    if (recentLimit > 0) {
      recentLines.push_back(line);
      while (recentLines.size() > recentLimit) recentLines.pop_front();
      continue;
    }
    const std::string evType = event_type_from_log_line(line);
    if (evType.empty()) continue;
    if (!typeFilter.empty() && evType != typeFilter) continue;
    total++;
    counts[evType]++;
    auto tsEnd = line.find(' ');
    if (tsEnd != std::string::npos && tsEnd > 0) {
      lastTs = line.substr(0, tsEnd);
    }
    lastType = evType;
  }

  if (recentLimit > 0) {
    for (const auto& ln : recentLines) {
      const std::string evType = event_type_from_log_line(ln);
      if (evType.empty()) continue;
      if (!typeFilter.empty() && evType != typeFilter) continue;
      total++;
      counts[evType]++;
      auto tsEnd = ln.find(' ');
      if (tsEnd != std::string::npos && tsEnd > 0) {
        lastTs = ln.substr(0, tsEnd);
      }
      lastType = evType;
    }
  }
  std::ostringstream os;
  os << "{\"ok\":true,\"total\":" << total
     << ",\"filter_type\":\"" << json_escape(typeFilter) << "\""
     << ",\"recent_limit\":" << recentLimit
     << ",\"since_sec\":" << sinceSec
     << ",\"last_event_type\":\"" << json_escape(lastType) << "\""
     << ",\"last_event_ts\":\"" << json_escape(lastTs) << "\""
     << ",\"counts\":{";
  bool first = true;
  for (const auto& kv : counts) {
    if (!first) os << ",";
    os << "\"" << json_escape(kv.first) << "\":" << kv.second;
    first = false;
  }
  os << "}}";
  return os.str();
}

static float clamp01(float v) {
  return std::max(0.f, std::min(1.f, v));
}

static float median_of(std::vector<float> v) {
  if (v.empty()) return 0.0f;
  std::sort(v.begin(), v.end());
  const size_t n = v.size();
  if ((n % 2) == 1) return v[n / 2];
  return 0.5f * (v[n / 2 - 1] + v[n / 2]);
}

static float box_iou(const DetectionBox& a, const DetectionBox& b) {
  const int x1 = std::max(a.left, b.left);
  const int y1 = std::max(a.top, b.top);
  const int x2 = std::min(a.right, b.right);
  const int y2 = std::min(a.bottom, b.bottom);
  const int iw = std::max(0, x2 - x1);
  const int ih = std::max(0, y2 - y1);
  const float inter = static_cast<float>(iw * ih);
  const float areaA = static_cast<float>(std::max(0, a.right - a.left) * std::max(0, a.bottom - a.top));
  const float areaB = static_cast<float>(std::max(0, b.right - b.left) * std::max(0, b.bottom - b.top));
  const float denom = areaA + areaB - inter;
  if (denom <= 1e-6f) return 0.f;
  return inter / denom;
}

static void update_confidence_layers(const std::vector<DetectionBox>& boxes, uint64_t frameId, int frameW, int frameH) {
  std::lock_guard<std::mutex> lk(g_confMtx);

  if (boxes.empty() || frameW <= 0 || frameH <= 0) {
    g_confState.l0_detector = 0.f;
    g_confState.l1_continuity = 0.f;
    g_confState.l2_motion = 0.f;
    g_confState.l3_model_agreement = 0.f;
    g_confState.fused_raw = 0.f;
    g_confState.fused = 0.f;
    g_confState.cfg_l0_enabled = g_confCfg.l0_enabled;
    g_confState.cfg_l1_enabled = g_confCfg.l1_enabled;
    g_confState.cfg_l2_enabled = g_confCfg.l2_enabled;
    g_confState.cfg_l3_enabled = g_confCfg.l3_enabled;
    g_confState.cfg_w0 = g_confCfg.w0;
    g_confState.cfg_w1 = g_confCfg.w1;
    g_confState.cfg_w2 = g_confCfg.w2;
    g_confState.cfg_w3 = g_confCfg.w3;
    g_confState.cfg_revision = g_confCfgRevision.load();
    g_confState.frame_id = frameId;
    g_confState.has_track = false;
    g_confState.has_prev = false;
    return;
  }

  const DetectionBox* best = &boxes.front();
  for (const auto& b : boxes) {
    if (b.prop > best->prop) best = &b;
  }

  const auto now = std::chrono::steady_clock::now();
  const float l0 = clamp01(best->prop);
  float l1 = 1.f;
  float l2 = 1.f;
  float l3 = 0.65f;

  if (g_confState.has_prev) {
    const float iou = box_iou(*best, g_confState.prev_best);
    const auto dtMs = std::chrono::duration_cast<std::chrono::milliseconds>(now - g_confState.prev_ts).count();
    const float dt = std::max(0.001f, static_cast<float>(dtMs) / 1000.f);
    const float clsBonus = (best->cls_id == g_confState.prev_best.cls_id) ? 1.f : 0.7f;
    l1 = clamp01(iou * clsBonus * std::exp(-0.15f * dt));

    const float cx = 0.5f * static_cast<float>(best->left + best->right);
    const float cy = 0.5f * static_cast<float>(best->top + best->bottom);
    const float px = 0.5f * static_cast<float>(g_confState.prev_best.left + g_confState.prev_best.right);
    const float py = 0.5f * static_cast<float>(g_confState.prev_best.top + g_confState.prev_best.bottom);
    const float dx = cx - px;
    const float dy = cy - py;
    const float frameDiag = std::sqrt(static_cast<float>(frameW * frameW + frameH * frameH));
    const float normShift = frameDiag > 1.f ? std::sqrt(dx * dx + dy * dy) / frameDiag : 0.f;
    const float normSpeed = normShift / dt;
    l2 = clamp01(1.f - (normSpeed / 1.8f));
  }

  if (boxes.size() >= 2) {
    const DetectionBox* second = nullptr;
    for (const auto& b : boxes) {
      if (&b == best) continue;
      if (!second || b.prop > second->prop) second = &b;
    }
    if (second) {
      const float iou2 = box_iou(*best, *second);
      const float clsAgreement = (best->cls_id == second->cls_id) ? 1.0f : 0.35f;
      const float scoreAgreement = clamp01(1.0f - std::fabs(best->prop - second->prop));
      l3 = clamp01(0.55f * clsAgreement + 0.30f * iou2 + 0.15f * scoreAgreement);
    }
  }

  const float cw0 = g_confCfg.l0_enabled ? std::max(0.f, g_confCfg.w0) : 0.f;
  const float cw1 = g_confCfg.l1_enabled ? std::max(0.f, g_confCfg.w1) : 0.f;
  const float cw2 = g_confCfg.l2_enabled ? std::max(0.f, g_confCfg.w2) : 0.f;
  const float cw3 = g_confCfg.l3_enabled ? std::max(0.f, g_confCfg.w3) : 0.f;
  const float weightSum = cw0 + cw1 + cw2 + cw3;
  const float fusedRaw = clamp01((l0 + l1 + l2 + l3) / 4.f);
  const float fused = weightSum > 1e-6f
    ? clamp01((cw0 * l0 + cw1 * l1 + cw2 * l2 + cw3 * l3) / weightSum)
    : 0.f;

  g_confState.l0_detector = l0;
  g_confState.l1_continuity = l1;
  g_confState.l2_motion = l2;
  g_confState.l3_model_agreement = l3;
  g_confState.fused_raw = fusedRaw;
  g_confState.fused = fused;
  g_confState.cfg_l0_enabled = g_confCfg.l0_enabled;
  g_confState.cfg_l1_enabled = g_confCfg.l1_enabled;
  g_confState.cfg_l2_enabled = g_confCfg.l2_enabled;
  g_confState.cfg_l3_enabled = g_confCfg.l3_enabled;
  g_confState.cfg_w0 = g_confCfg.w0;
  g_confState.cfg_w1 = g_confCfg.w1;
  g_confState.cfg_w2 = g_confCfg.w2;
  g_confState.cfg_w3 = g_confCfg.w3;
  g_confState.cfg_revision = g_confCfgRevision.load();
  g_confState.frame_id = frameId;
  g_confState.has_track = true;
  g_confState.has_prev = true;
  g_confState.prev_best = *best;
  g_confState.prev_ts = now;
}

static long process_rss_kb() {
  std::ifstream statm("/proc/self/statm");
  if (!statm.is_open()) return -1;
  long totalPages = 0;
  long residentPages = 0;
  statm >> totalPages >> residentPages;
  if (residentPages <= 0) return 0;
  const long pageSize = sysconf(_SC_PAGESIZE);
  if (pageSize <= 0) return -1;
  return (residentPages * pageSize) / 1024;
}

static double process_loadavg_1m() {
  double samples[3] = {0.0, 0.0, 0.0};
  const int got = getloadavg(samples, 3);
  if (got <= 0) return -1.0;
  return samples[0];
}

static std::string now_utc_iso8601() {
  const auto now = std::chrono::system_clock::now();
  const std::time_t t = std::chrono::system_clock::to_time_t(now);
  std::tm tm{};
  gmtime_r(&t, &tm);
  std::ostringstream os;
  os << std::put_time(&tm, "%Y-%m-%dT%H:%M:%SZ");
  return os.str();
}

static std::string extract_json_string_field(const std::string& json, const std::string& key) {
  const std::string pattern = "\"" + key + "\"";
  const auto keyPos = json.find(pattern);
  if (keyPos == std::string::npos) return "";
  auto colonPos = json.find(':', keyPos + pattern.size());
  if (colonPos == std::string::npos) return "";
  auto quotePos = json.find('"', colonPos + 1);
  if (quotePos == std::string::npos) return "";
  auto endQuotePos = json.find('"', quotePos + 1);
  if (endQuotePos == std::string::npos || endQuotePos <= quotePos + 1) return "";
  return json.substr(quotePos + 1, endQuotePos - quotePos - 1);
}

static void filter_detections(object_detect_result_list& od) {
  std::lock_guard<std::mutex> lk(g_classMtx);
  if (g_selectedClasses.empty()) return;
  int out = 0;
  for (int i = 0; i < od.count; ++i) {
    const auto& d = od.results[i];
    if (g_selectedClasses.count(d.cls_id)) {
      if (out != i) od.results[out] = od.results[i];
      ++out;
    }
  }
  od.count = out;
}

static void draw_detections(cv::Mat& bgr, const object_detect_result_list& od) {
  for (int i = 0; i < od.count; ++i) {
    const auto& d = od.results[i];
    int x = std::max(0, d.box.left);
    int y = std::max(0, d.box.top);
    int w = std::max(1, d.box.right - d.box.left);
    int h = std::max(1, d.box.bottom - d.box.top);
    cv::Rect r(x, y, w, h);
    r &= cv::Rect(0, 0, bgr.cols, bgr.rows);
    if (r.width <= 1 || r.height <= 1) continue;

    cv::rectangle(bgr, r, cv::Scalar(255, 0, 0), 2);
    char txt[128];
    snprintf(txt, sizeof(txt), "%s %.1f%%", coco_cls_to_name(d.cls_id), d.prop * 100.f);

    int base = 0;
    auto ts = cv::getTextSize(txt, cv::FONT_HERSHEY_SIMPLEX, 0.6, 1, &base);
    int tx = std::max(0, r.x);
    int ty = std::max(ts.height + 4, r.y - 4);
    cv::rectangle(bgr, cv::Rect(tx, ty - ts.height - 4, ts.width + 6, ts.height + 6),
                  cv::Scalar(255, 0, 0), cv::FILLED);
    cv::putText(bgr, txt, cv::Point(tx + 3, ty), cv::FONT_HERSHEY_SIMPLEX, 0.6,
                cv::Scalar(255, 255, 255), 1, cv::LINE_AA);
  }
}


static void draw_detection_boxes(cv::Mat& bgr, const std::vector<DetectionBox>& boxes) {
  for (const auto& d : boxes) {
    cv::rectangle(bgr, cv::Point(d.left, d.top), cv::Point(d.right, d.bottom),
                  cv::Scalar(0, 255, 0), 2);
    std::ostringstream lbl;
    lbl << d.cls_id << " " << std::fixed << std::setprecision(2) << d.prop << " id:" << d.id;
    cv::putText(bgr, lbl.str(), cv::Point(d.left, std::max(15, d.top - 6)),
                cv::FONT_HERSHEY_SIMPLEX, 0.55, cv::Scalar(0, 255, 0), 2);
  }
}

static bool run_inference_on_bgr(const cv::Mat& bgr, int offsetX, int offsetY, const std::set<int>* roiClassFilter, std::vector<DetectionBox>& outBoxes) {
  if (bgr.empty()) return false;
  cv::Mat rgb;
  cv::cvtColor(bgr, rgb, cv::COLOR_BGR2RGB);
  image_buffer_t frame{};
  frame.width = rgb.cols;
  frame.height = rgb.rows;
  frame.format = IMAGE_FORMAT_RGB888;
  frame.virt_addr = rgb.data;
  frame.size = static_cast<int>(rgb.total() * rgb.elemSize());
  object_detect_result_list od{};
  bool infer_ok = false;
  {
    std::lock_guard<std::mutex> lk(g_modelMtx);
    if (g_modelReady && inference_yolov8_model(&g_rknn, &frame, &od) == 0) infer_ok = true;
  }
  if (!infer_ok) return false;
  filter_detections(od);
  for (int i = 0; i < od.count; ++i) {
    const auto& d = od.results[i];
    if (roiClassFilter && !roiClassFilter->empty() &&
        roiClassFilter->count(d.cls_id) == 0) {
      continue;
    }
    DetectionBox box;
    box.id = 0;
    box.cls_id = d.cls_id;
    box.prop = d.prop;
    box.left = d.box.left + offsetX;
    box.top = d.box.top + offsetY;
    box.right = d.box.right + offsetX;
    box.bottom = d.box.bottom + offsetY;
    outBoxes.push_back(box);
  }
  return true;
}

static void handle_client(int cfd, const Opts& o) {
  std::string req = read_http_request(cfd);
  std::string method = get_method(req);
  std::string target = get_target(req);
  std::string path = path_only(target);
  std::string bodyReq = get_body(req);

  if (path == "/" || path == "/index.html") {

    std::string body;
    const std::vector<std::string> candidates = {
      "web/index.html",
      "./web/index.html",
      "../web/index.html"
    };
    for (const auto& candidate : candidates) {
      std::ifstream in(candidate, std::ios::binary);
      if (!in.is_open()) continue;
      std::ostringstream ss;
      ss << in.rdbuf();
      body = ss.str();
      if (!body.empty()) break;
    }

    if (body.empty()) {
      body = "<!doctype html><html><body><h1>web/index.html not found</h1></body></html>";
    }
    std::string hdr =
      "HTTP/1.1 200 OK\r\n"
      "Content-Type: text/html; charset=utf-8\r\n"
      "Cache-Control: no-store\r\n"
      "Connection: close\r\n"
      "Content-Length: " + std::to_string(body.size()) + "\r\n\r\n";
    send_all(cfd, hdr.data(), hdr.size());
    send_all(cfd, body.data(), body.size());
    ::close(cfd);
    return;
  }

  if (path == "/api/models") {
    std::string body = models_json();
    std::string hdr =
      "HTTP/1.1 200 OK\r\nContent-Type: application/json\r\nCache-Control: no-store\r\n"
      "Connection: close\r\nContent-Length: " + std::to_string(body.size()) + "\r\n\r\n";
    send_all(cfd, hdr.data(), hdr.size());

    send_all(cfd, body.data(), body.size());
    ::close(cfd);
    return;
  }



  if (path == "/api/settings") {
    if (method == "GET") {
      std::string body = "{\"config_version\":2}";
      std::ifstream in(g_settingsFile, std::ios::binary);
      if (in.is_open()) {
        std::ostringstream ss;
        ss << in.rdbuf();
        const std::string loaded = trim(ss.str());
        if (!loaded.empty()) body = loaded;
      }
      std::string hdr =
        "HTTP/1.1 200 OK\r\nContent-Type: application/json\r\nCache-Control: no-store\r\n"
        "Connection: close\r\nContent-Length: " + std::to_string(body.size()) + "\r\n\r\n";
      send_all(cfd, hdr.data(), hdr.size());
      send_all(cfd, body.data(), body.size());
      ::close(cfd);
      return;
    }
    if (method == "POST") {
      std::string payload = trim(bodyReq);
      bool ok = false;
      if (!payload.empty() && payload.size() <= 65536 && payload.front() == '{' && payload.back() == '}') {
        std::ofstream out(g_settingsFile, std::ios::trunc | std::ios::binary);
        if (out.is_open()) {
          out << payload;
          ok = out.good();
        }
      }
      std::string body = std::string("{\"ok\":") + (ok ? "true" : "false") + "}";
      std::string hdr =
        "HTTP/1.1 200 OK\r\nContent-Type: application/json\r\nCache-Control: no-store\r\n"
        "Connection: close\r\nContent-Length: " + std::to_string(body.size()) + "\r\n\r\n";
      send_all(cfd, hdr.data(), hdr.size());
      send_all(cfd, body.data(), body.size());
      ::close(cfd);
      return;
    }
    const char* body = "Method Not Allowed";
    std::string hdr =
      "HTTP/1.1 405 Method Not Allowed\r\nAllow: GET, POST\r\nContent-Type: text/plain\r\nConnection: close\r\n"
      "Content-Length: " + std::to_string(std::strlen(body)) + "\r\n\r\n";
    send_all(cfd, hdr.data(), hdr.size());
    send_all(cfd, body, std::strlen(body));
    ::close(cfd);
    return;
  }

  if (path == "/api/events") {
    if (method == "POST") {
      std::string payload = trim(bodyReq);
      const std::string evType = extract_json_string_field(payload, "type");
      bool ok = false;
      if (!payload.empty() && payload.size() <= 65536 && payload.front() == '{' && payload.back() == '}' && !evType.empty()) {
        std::ofstream out(g_opsLogFile, std::ios::app | std::ios::binary);
        if (out.is_open()) {
          out << now_utc_iso8601() << " type=" << evType << " payload=" << payload << "\n";
          ok = out.good();
        }
      }
      std::string body = std::string("{\"ok\":") + (ok ? "true" : "false") + "}";
      std::string hdr =
        "HTTP/1.1 200 OK\r\nContent-Type: application/json\r\nCache-Control: no-store\r\n"
        "Connection: close\r\nContent-Length: " + std::to_string(body.size()) + "\r\n\r\n";
      send_all(cfd, hdr.data(), hdr.size());
      send_all(cfd, body.data(), body.size());
      ::close(cfd);
      return;
    }
    if (method == "GET") {
      std::string filterType = query_param(target, "type");
      const int recent = query_param_int(target, "recent", 0, 0, 10000);
      const int sinceSec = query_param_int(target, "since_sec", 0, 0, 604800);
      const std::string cutoffTs = utc_iso8601_seconds_ago(sinceSec);
      std::ifstream in(g_opsLogFile, std::ios::binary);
      std::string body;
      if (in.is_open()) {
        std::deque<std::string> lastLines;
        std::string line;
        while (std::getline(in, line)) {
          if (!cutoffTs.empty() && line.size() >= cutoffTs.size() && line.compare(0, cutoffTs.size(), cutoffTs) < 0) continue;
          if (recent > 0) {
            lastLines.push_back(line);
            while (lastLines.size() > static_cast<size_t>(recent)) lastLines.pop_front();
            continue;
          }
          if (!filterType.empty()) {
            const std::string token = "type=" + filterType;
            if (line.find(token) == std::string::npos) continue;
          }
          lastLines.push_back(line);
        }
        std::ostringstream lines;
        for (const auto& ln : lastLines) {
          if (recent > 0 && !filterType.empty()) {
            const std::string token = "type=" + filterType;
            if (ln.find(token) == std::string::npos) continue;
          }
          lines << ln << "\n";
        }
        body = lines.str();
      }
      std::string hdr =
        "HTTP/1.1 200 OK\r\nContent-Type: text/plain; charset=utf-8\r\nCache-Control: no-store\r\n"
        "Connection: close\r\nContent-Length: " + std::to_string(body.size()) + "\r\n\r\n";
      send_all(cfd, hdr.data(), hdr.size());
      if (!body.empty()) send_all(cfd, body.data(), body.size());
      ::close(cfd);
      return;
    }
    const char* body = "Method Not Allowed";
    std::string hdr =
      "HTTP/1.1 405 Method Not Allowed\r\nAllow: GET, POST\r\nContent-Type: text/plain\r\nConnection: close\r\n"
      "Content-Length: " + std::to_string(std::strlen(body)) + "\r\n\r\n";
    send_all(cfd, hdr.data(), hdr.size());
    send_all(cfd, body, std::strlen(body));
    ::close(cfd);
    return;
  }

  if (path == "/api/events/summary") {
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
    const std::string filterType = query_param(target, "type");
    const int recent = query_param_int(target, "recent", 0, 0, 10000);
    const int sinceSec = query_param_int(target, "since_sec", 0, 0, 604800);
    const std::string body = events_summary_json(filterType, static_cast<size_t>(recent), sinceSec);
    const std::string hdr =
      "HTTP/1.1 200 OK\r\nContent-Type: application/json\r\nCache-Control: no-store\r\n"
      "Connection: close\r\nContent-Length: " + std::to_string(body.size()) + "\r\n\r\n";
    send_all(cfd, hdr.data(), hdr.size());
    send_all(cfd, body.data(), body.size());
    ::close(cfd);
    return;
  }

  if (path == "/api/detections") {
    std::string body = detections_json();
    std::string hdr =
      "HTTP/1.1 200 OK\r\nContent-Type: application/json\r\nCache-Control: no-store\r\n"
      "Connection: close\r\nContent-Length: " + std::to_string(body.size()) + "\r\n\r\n";
    send_all(cfd, hdr.data(), hdr.size());
    send_all(cfd, body.data(), body.size());
    ::close(cfd);
    return;
  }

  if (path == "/api/tracker/state") {
    TrackerRuntimeState tr{};
    {
      std::lock_guard<std::mutex> lk(g_trackerStateMtx);
      tr = g_trackerState;
    }
    std::ostringstream os;
    os << "{\"ok\":true,\"mode\":\"" << tracker_mode_str(tr.mode) << "\""
       << ",\"selected_track_id\":" << tr.selected_track_id
       << ",\"has_selected_track\":" << (tr.has_selected_track ? "true" : "false")
       << ",\"selected_box_valid\":" << (tr.selected_box_valid ? "true" : "false")
       << ",\"lost_frames\":" << tr.lost_frames
       << ",\"max_lost_frames\":" << tr.max_lost_frames
       << ",\"frameId\":" << tr.last_frame_id
       << ",\"box\":";
    if (tr.selected_box_valid) {
      os << "{\"id\":" << tr.selected_box.id << ",\"cls\":" << tr.selected_box.cls_id
         << ",\"prop\":" << tr.selected_box.prop << ",\"left\":" << tr.selected_box.left
         << ",\"top\":" << tr.selected_box.top << ",\"right\":" << tr.selected_box.right
         << ",\"bottom\":" << tr.selected_box.bottom << "}";
    } else os << "null";
    os << "}";
    std::string body = os.str();
    std::string hdr = "HTTP/1.1 200 OK\r\nContent-Type: application/json\r\nCache-Control: no-store\r\nConnection: close\r\nContent-Length: " + std::to_string(body.size()) + "\r\n\r\n";
    send_all(cfd, hdr.data(), hdr.size()); send_all(cfd, body.data(), body.size()); ::close(cfd); return;
  }
  if (path == "/api/tracker/select" && method == "POST") {
    const std::string payload = trim(bodyReq);
    int tid = 0;
    if (!extract_json_int_field(payload, "track_id", &tid)) extract_json_int_field(payload, "selected_track_id", &tid);
    {
      std::lock_guard<std::mutex> lk(g_trackerStateMtx);
      g_trackerState.selected_track_id = tid > 0 ? static_cast<uint64_t>(tid) : 0;
      g_trackerState.has_selected_track = g_trackerState.selected_track_id > 0;
      g_trackerState.lost_frames = 0;
      g_trackerState.selected_box_valid = false;
      g_trackerState.mode = g_trackerState.has_selected_track ? TrackerMode::Lost : TrackerMode::Idle;
    }
    std::string body = std::string("{\"ok\":true,\"selected_track_id\":") + std::to_string(std::max(0, tid)) + "}";
    std::string hdr = "HTTP/1.1 200 OK\r\nContent-Type: application/json\r\nCache-Control: no-store\r\nConnection: close\r\nContent-Length: " + std::to_string(body.size()) + "\r\n\r\n";
    send_all(cfd, hdr.data(), hdr.size()); send_all(cfd, body.data(), body.size()); ::close(cfd); return;
  }
  if (path == "/api/tracker/clear" && method == "POST") {
    std::lock_guard<std::mutex> lk(g_trackerStateMtx);
    g_trackerState.selected_track_id = 0; g_trackerState.has_selected_track = false; g_trackerState.selected_box_valid = false; g_trackerState.lost_frames = 0; g_trackerState.mode = TrackerMode::Idle;
    std::string body = "{\"ok\":true}";
    std::string hdr = "HTTP/1.1 200 OK\r\nContent-Type: application/json\r\nCache-Control: no-store\r\nConnection: close\r\nContent-Length: " + std::to_string(body.size()) + "\r\n\r\n";
    send_all(cfd, hdr.data(), hdr.size()); send_all(cfd, body.data(), body.size()); ::close(cfd); return;
  }
  if (path == "/api/tracker/config" && method == "POST") {
    int maxLost = 12;
    extract_json_int_field(trim(bodyReq), "max_lost_frames", &maxLost);
    maxLost = std::max(1, std::min(120, maxLost));
    {
      std::lock_guard<std::mutex> lk(g_trackerStateMtx);
      g_trackerState.max_lost_frames = maxLost;
    }
    std::string body = std::string("{\"ok\":true,\"max_lost_frames\":") + std::to_string(maxLost) + "}";
    std::string hdr = "HTTP/1.1 200 OK\r\nContent-Type: application/json\r\nCache-Control: no-store\r\nConnection: close\r\nContent-Length: " + std::to_string(body.size()) + "\r\n\r\n";
    send_all(cfd, hdr.data(), hdr.size()); send_all(cfd, body.data(), body.size()); ::close(cfd); return;
  }

  if (path == "/api/ping") {
    const auto uptimeSec = std::chrono::duration_cast<std::chrono::seconds>(
      std::chrono::steady_clock::now() - g_processStartedAt
    ).count();
    const long rssKb = process_rss_kb();
    const double load1m = process_loadavg_1m();
    std::ostringstream os;
    os << "{\"ok\":true,\"server_time\":\"" << now_utc_iso8601()
       << "\",\"uptime_s\":" << uptimeSec
       << ",\"mem_rss_kb\":" << rssKb
       << ",\"loadavg_1m\":" << std::fixed << std::setprecision(2) << load1m
       << ",\"rtt_ms\":0}";
    const std::string body = os.str();
    const std::string hdr =
      "HTTP/1.1 200 OK\r\nContent-Type: application/json\r\nCache-Control: no-store\r\n"
      "Connection: close\r\nContent-Length: " + std::to_string(body.size()) + "\r\n\r\n";
    send_all(cfd, hdr.data(), hdr.size());
    send_all(cfd, body.data(), body.size());
    ::close(cfd);
    return;
  }

  if (path == "/api/command_limits") {
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
    std::ostringstream os;
    os << "{\"cmd_max_pan\":" << g_cmdMaxPan
       << ",\"cmd_max_tilt\":" << g_cmdMaxTilt
       << ",\"cmd_max_zoom\":" << g_cmdMaxZoom
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


  if (path == "/api/zoom_calibration") {
    if (method == "POST") {
      bool started = false;
      if (!g_zoomCalibInProgress.load()) {
        std::thread([o]() {
          const bool ok = run_zoom_image_time_calibration(o);
          if (!ok) std::cerr << "zoom-calib: requested via API but failed/skipped\n";
        }).detach();
        started = true;
      }
      const std::string body = std::string("{\"ok\":true,\"started\":") + (started ? "true" : "false") + "}";
      const std::string hdr =
        "HTTP/1.1 200 OK\r\nContent-Type: application/json\r\nCache-Control: no-store\r\n"
        "Connection: close\r\nContent-Length: " + std::to_string(body.size()) + "\r\n\r\n";
      send_all(cfd, hdr.data(), hdr.size());
      send_all(cfd, body.data(), body.size());
      ::close(cfd);
      return;
    }
    if (method != "GET") {
      const char* body = "Method Not Allowed";
      std::string hdr =
        "HTTP/1.1 405 Method Not Allowed\r\nAllow: GET, POST\r\nContent-Type: text/plain\r\nConnection: close\r\n"
        "Content-Length: " + std::to_string(std::strlen(body)) + "\r\n\r\n";
      send_all(cfd, hdr.data(), hdr.size());
      send_all(cfd, body, std::strlen(body));
      ::close(cfd);
      return;
    }
    const std::string body = zoom_calibration_json();
    const std::string hdr =
      "HTTP/1.1 200 OK\r\nContent-Type: application/json\r\nCache-Control: no-store\r\n"
      "Connection: close\r\nContent-Length: " + std::to_string(body.size()) + "\r\n\r\n";
    send_all(cfd, hdr.data(), hdr.size());
    send_all(cfd, body.data(), body.size());
    ::close(cfd);
    return;
  }

  if (path == "/api/range_estimate") {
    std::string body = range_estimate_json();
    std::string hdr =
      "HTTP/1.1 200 OK\r\nContent-Type: application/json\r\nCache-Control: no-store\r\n"
      "Connection: close\r\nContent-Length: " + std::to_string(body.size()) + "\r\n\r\n";
    send_all(cfd, hdr.data(), hdr.size());
    send_all(cfd, body.data(), body.size());
    ::close(cfd);
    return;
  }

  if (path == "/api/tracking/confidence") {
    float l0 = 0.f, l1 = 0.f, l2 = 0.f, l3 = 0.f, fusedRaw = 0.f, fused = 0.f;
    bool l0e = true, l1e = true, l2e = true, l3e = true;
    float w0 = 0.f, w1 = 0.f, w2 = 0.f, w3 = 0.f;
    uint64_t cfgRev = 1;
    uint64_t frameId = 0;
    bool hasTrack = false;
    {
      std::lock_guard<std::mutex> lk(g_confMtx);
      l0 = g_confState.l0_detector;
      l1 = g_confState.l1_continuity;
      l2 = g_confState.l2_motion;
      l3 = g_confState.l3_model_agreement;
      fusedRaw = g_confState.fused_raw;
      fused = g_confState.fused;
      l0e = g_confState.cfg_l0_enabled;
      l1e = g_confState.cfg_l1_enabled;
      l2e = g_confState.cfg_l2_enabled;
      l3e = g_confState.cfg_l3_enabled;
      w0 = g_confState.cfg_w0;
      w1 = g_confState.cfg_w1;
      w2 = g_confState.cfg_w2;
      w3 = g_confState.cfg_w3;
      cfgRev = g_confState.cfg_revision;
      frameId = g_confState.frame_id;
      hasTrack = g_confState.has_track;
    }
    std::ostringstream os;
    os << "{\"frameId\":" << frameId
       << ",\"hasTrack\":" << (hasTrack ? "true" : "false")
       << ",\"layers\":["
       << "{\"id\":\"L0\",\"name\":\"detector_confidence\",\"score\":" << l0 << "},"
       << "{\"id\":\"L1\",\"name\":\"track_continuity\",\"score\":" << l1 << "},"
       << "{\"id\":\"L2\",\"name\":\"motion_plausibility\",\"score\":" << l2 << "},"
       << "{\"id\":\"L3\",\"name\":\"model_agreement\",\"score\":" << l3 << "}"
       << "],\"fused_raw\":" << fusedRaw
       << ",\"fused\":" << fused
       << ",\"applied_config\":{\"revision\":" << cfgRev
       << ",\"l0_enabled\":" << (l0e ? "true" : "false")
       << ",\"l1_enabled\":" << (l1e ? "true" : "false")
       << ",\"l2_enabled\":" << (l2e ? "true" : "false")
       << ",\"l3_enabled\":" << (l3e ? "true" : "false")
       << ",\"w0\":" << w0
       << ",\"w1\":" << w1
       << ",\"w2\":" << w2
       << ",\"w3\":" << w3
       << "}}";
    std::string body = os.str();
    std::string hdr =
      "HTTP/1.1 200 OK\r\nContent-Type: application/json\r\nCache-Control: no-store\r\n"
      "Connection: close\r\nContent-Length: " + std::to_string(body.size()) + "\r\n\r\n";
    send_all(cfd, hdr.data(), hdr.size());
    send_all(cfd, body.data(), body.size());
    ::close(cfd);
    return;
  }

  if (path == "/api/tracking/confidence/config") {
    if (method == "GET") {
      std::ostringstream os;
      {
        std::lock_guard<std::mutex> lk(g_confMtx);
        const uint64_t cfgRev = g_confCfgRevision.load();
        os << "{\"ok\":true"
           << ",\"revision\":" << cfgRev
           << ",\"l0_enabled\":" << (g_confCfg.l0_enabled ? "true" : "false")
           << ",\"l1_enabled\":" << (g_confCfg.l1_enabled ? "true" : "false")
           << ",\"l2_enabled\":" << (g_confCfg.l2_enabled ? "true" : "false")
           << ",\"l3_enabled\":" << (g_confCfg.l3_enabled ? "true" : "false")
           << ",\"w0\":" << g_confCfg.w0
           << ",\"w1\":" << g_confCfg.w1
           << ",\"w2\":" << g_confCfg.w2
           << ",\"w3\":" << g_confCfg.w3
           << "}";
      }
      std::string body = os.str();
      std::string hdr =
        "HTTP/1.1 200 OK\r\nContent-Type: application/json\r\nCache-Control: no-store\r\n"
        "Connection: close\r\nContent-Length: " + std::to_string(body.size()) + "\r\n\r\n";
      send_all(cfd, hdr.data(), hdr.size());
      send_all(cfd, body.data(), body.size());
      ::close(cfd);
      return;
    }
    if (method == "POST") {
      std::string payload = trim(bodyReq);
      bool ok = false;
      uint64_t cfgRev = g_confCfgRevision.load();
      if (!payload.empty() && payload.front() == '{' && payload.back() == '}') {
        bool l0e = g_confCfg.l0_enabled;
        bool l1e = g_confCfg.l1_enabled;
        bool l2e = g_confCfg.l2_enabled;
        bool l3e = g_confCfg.l3_enabled;
        float w0 = g_confCfg.w0;
        float w1 = g_confCfg.w1;
        float w2 = g_confCfg.w2;
        float w3 = g_confCfg.w3;
        extract_json_bool_field(payload, "l0_enabled", &l0e);
        extract_json_bool_field(payload, "l1_enabled", &l1e);
        extract_json_bool_field(payload, "l2_enabled", &l2e);
        extract_json_bool_field(payload, "l3_enabled", &l3e);
        extract_json_float_field(payload, "w0", &w0);
        extract_json_float_field(payload, "w1", &w1);
        extract_json_float_field(payload, "w2", &w2);
        extract_json_float_field(payload, "w3", &w3);
        {
          std::lock_guard<std::mutex> lk(g_confMtx);
          g_confCfg.l0_enabled = l0e;
          g_confCfg.l1_enabled = l1e;
          g_confCfg.l2_enabled = l2e;
          g_confCfg.l3_enabled = l3e;
          g_confCfg.w0 = clamp01(w0);
          g_confCfg.w1 = clamp01(w1);
          g_confCfg.w2 = clamp01(w2);
          g_confCfg.w3 = clamp01(w3);
          cfgRev = g_confCfgRevision.fetch_add(1) + 1;
        }
        ok = true;
      }
      std::ostringstream os;
      os << "{\"ok\":" << (ok ? "true" : "false")
         << ",\"revision\":" << cfgRev
         << "}";
      const std::string body = os.str();
      std::string hdr =
        "HTTP/1.1 200 OK\r\nContent-Type: application/json\r\nCache-Control: no-store\r\n"
        "Connection: close\r\nContent-Length: " + std::to_string(body.size()) + "\r\n\r\n";
      send_all(cfd, hdr.data(), hdr.size());
      send_all(cfd, body.data(), body.size());
      ::close(cfd);
      return;
    }
    const char* body = "Method Not Allowed";
    std::string hdr =
      "HTTP/1.1 405 Method Not Allowed\r\nAllow: GET, POST\r\nContent-Type: text/plain\r\nConnection: close\r\n"
      "Content-Length: " + std::to_string(std::strlen(body)) + "\r\n\r\n";
    send_all(cfd, hdr.data(), hdr.size());
    send_all(cfd, body, std::strlen(body));
    ::close(cfd);
    return;
  }

  if (path == "/api/detector/config") {
    if (method == "GET") {
      std::ostringstream os;
      std::lock_guard<std::mutex> lk(g_classMtx);
      os << "{\"ok\":true"
         << ",\"detect_enabled\":" << (g_detectEnabled.load() ? "true" : "false")
         << ",\"current_model\":\"" << json_escape(g_currentModel) << "\""
         << ",\"selected_classes\":[";
      bool first = true;
      for (int cls : g_selectedClasses) {
        if (!first) os << ",";
        os << cls;
        first = false;
      }
      os << "]}";
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
      std::string payload = trim(bodyReq);
      bool ok = false;
      std::string modelSwitchError;
      if (!payload.empty() && payload.front() == '{' && payload.back() == '}') {
        bool detectEnabled = g_detectEnabled.load();
        extract_json_bool_field(payload, "detect_enabled", &detectEnabled);
        extract_json_bool_field(payload, "detect", &detectEnabled);
        g_detectEnabled = detectEnabled;

        const std::string model = trim(extract_json_string_field(payload, "current_model"));
        if (!model.empty()) {
          const bool deferSwitch = extract_json_string_field(payload, "defer_reason") == "auto_critical";
          auto it = std::find(g_availableModels.begin(), g_availableModels.end(), model);
          if (it != g_availableModels.end()) {

            if (deferSwitch) {
              modelSwitchError = "deferred_auto_critical";
              append_ops_event("detector_model_switch_fail",
                               std::string("{\"type\":\"detector_model_switch_fail\",\"model\":\"") +
                                 json_escape(model) + "\",\"reason\":\"deferred_auto_critical\"}");
            } else {
              const auto now = std::chrono::steady_clock::now();
              const auto cooldown = std::chrono::seconds(10);
              if (g_lastModelSwitchTs != std::chrono::steady_clock::time_point::min() &&
                  now - g_lastModelSwitchTs < cooldown) {
                modelSwitchError = "cooldown";
                append_ops_event("detector_model_switch_fail",
                                 std::string("{\"type\":\"detector_model_switch_fail\",\"model\":\"") +
                                   json_escape(model) + "\",\"reason\":\"cooldown\"}");
              } else {
                append_ops_event("detector_model_switch_start",
                                 std::string("{\"type\":\"detector_model_switch_start\",\"model\":\"") +
                                   json_escape(model) + "\"}");
                if (load_model(model)) {
                  g_lastModelSwitchTs = now;
                  append_ops_event("detector_model_switch_ok",
                                   std::string("{\"type\":\"detector_model_switch_ok\",\"model\":\"") +
                                     json_escape(model) + "\"}");
                } else {
                  modelSwitchError = "load_failed";
                  append_ops_event("detector_model_switch_fail",
                                   std::string("{\"type\":\"detector_model_switch_fail\",\"model\":\"") +
                                     json_escape(model) + "\",\"reason\":\"load_failed\"}");
                }
              }
            }
          }
        }

        const std::set<int> requestedClasses = extract_json_int_array_field(payload, "selected_classes");
        if (payload.find("\"selected_classes\"") != std::string::npos) {
          std::set<int> validClasses;
          std::lock_guard<std::mutex> lk(g_classMtx);
          for (int cls : requestedClasses) {
            if (cls >= 0 && cls < static_cast<int>(g_labels.size())) validClasses.insert(cls);
          }
          g_selectedClasses = std::move(validClasses);
        }

        save_state();
        ok = true;
      }
      std::ostringstream body;
      body << "{\"ok\":" << (ok ? "true" : "false")
           << ",\"detect_enabled\":" << (g_detectEnabled.load() ? "true" : "false")
           << ",\"model_switch_error\":\"" << json_escape(modelSwitchError) << "\""
           << "}";
      const std::string hdr =
        "HTTP/1.1 200 OK\r\nContent-Type: application/json\r\nCache-Control: no-store\r\n"
        "Connection: close\r\nContent-Length: " + std::to_string(body.str().size()) + "\r\n\r\n";
      send_all(cfd, hdr.data(), hdr.size());
      const std::string bodyStr = body.str();
      send_all(cfd, bodyStr.data(), bodyStr.size());
      ::close(cfd);
      return;
    }
    const char* body = "Method Not Allowed";
    std::string hdr =
      "HTTP/1.1 405 Method Not Allowed\r\nAllow: GET, POST\r\nContent-Type: text/plain\r\nConnection: close\r\n"
      "Content-Length: " + std::to_string(std::strlen(body)) + "\r\n\r\n";
    send_all(cfd, hdr.data(), hdr.size());
    send_all(cfd, body, std::strlen(body));
    ::close(cfd);
    return;
  }



  if (path == "/api/detection/limits") {
    if (method == "GET") {
      std::ostringstream os;
      os << "{\"ok\":true,\"max_detections\":" << g_maxDetections.load()
         << ",\"max_raw_candidates\":" << g_maxRawCandidates.load() << "}";
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
      int maxDet = g_maxDetections.load();
      int maxRaw = g_maxRawCandidates.load();
      const std::string payload = trim(bodyReq);
      if (!payload.empty()) {
        extract_json_int_field(payload, "max_detections", &maxDet);
        extract_json_int_field(payload, "max_raw_candidates", &maxRaw);
      }
      maxDet = std::max(1, std::min(100, maxDet));
      maxRaw = std::max(maxDet, std::min(300, std::max(1, maxRaw)));
      g_maxDetections = maxDet;
      g_maxRawCandidates = maxRaw;
      save_detection_limits();
      std::ostringstream os;
      os << "{\"ok\":true,\"max_detections\":" << maxDet
         << ",\"max_raw_candidates\":" << maxRaw << "}";
      const std::string body = os.str();
      const std::string hdr =
        "HTTP/1.1 200 OK\r\nContent-Type: application/json\r\nCache-Control: no-store\r\n"
        "Connection: close\r\nContent-Length: " + std::to_string(body.size()) + "\r\n\r\n";
      send_all(cfd, hdr.data(), hdr.size());
      send_all(cfd, body.data(), body.size());
      ::close(cfd);
      return;
    }
    const char* body = "Method Not Allowed";
    std::string hdr =
      "HTTP/1.1 405 Method Not Allowed\r\nAllow: GET, POST\r\nContent-Type: text/plain\r\nConnection: close\r\n"
      "Content-Length: " + std::to_string(std::strlen(body)) + "\r\n\r\n";
    send_all(cfd, hdr.data(), hdr.size());
    send_all(cfd, body, std::strlen(body));
    ::close(cfd);
    return;
  }


  if (path == "/api/detection/roi_config") {
    if (method == "GET") {
      const std::string body = roi_config_json();
      const std::string hdr =
        "HTTP/1.1 200 OK\r\nContent-Type: application/json\r\nCache-Control: no-store\r\n"
        "Connection: close\r\nContent-Length: " + std::to_string(body.size()) + "\r\n\r\n";
      send_all(cfd, hdr.data(), hdr.size());
      send_all(cfd, body.data(), body.size());
      ::close(cfd);
      return;
    }
    if (method == "POST") {
      const int frameW = std::max(1, g_lastFrameWidth.load() > 0 ? g_lastFrameWidth.load() : o.width);
      const int frameH = std::max(1, g_lastFrameHeight.load() > 0 ? g_lastFrameHeight.load() : o.height);
      const bool ok = parse_roi_config_json(trim(bodyReq), frameW, frameH);
      if (ok) save_roi_config();
      const std::string body = ok ? roi_config_json() : "{\"ok\":false,\"error\":\"invalid_payload\"}";
      const std::string hdr =
        "HTTP/1.1 200 OK\r\nContent-Type: application/json\r\nCache-Control: no-store\r\n"
        "Connection: close\r\nContent-Length: " + std::to_string(body.size()) + "\r\n\r\n";
      send_all(cfd, hdr.data(), hdr.size());
      send_all(cfd, body.data(), body.size());
      ::close(cfd);
      return;
    }
    const char* body = "Method Not Allowed";
    std::string hdr =
      "HTTP/1.1 405 Method Not Allowed\r\nAllow: GET, POST\r\nContent-Type: text/plain\r\nConnection: close\r\n"
      "Content-Length: " + std::to_string(std::strlen(body)) + "\r\n\r\n";
    send_all(cfd, hdr.data(), hdr.size());
    send_all(cfd, body, std::strlen(body));
    ::close(cfd);
    return;
  }

  if (path == "/api/detector/models") {
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
    std::ostringstream os;
    std::lock_guard<std::mutex> lk(g_modelMtx);
    os << "{\"ok\":true,\"current_model\":\"" << json_escape(g_currentModel) << "\",\"models\":[";
    for (size_t i = 0; i < g_availableModels.size(); ++i) {
      if (i) os << ",";
      os << "\"" << json_escape(g_availableModels[i]) << "\"";
    }
    os << "]}";
    const std::string body = os.str();
    const std::string hdr =
      "HTTP/1.1 200 OK\r\nContent-Type: application/json\r\nCache-Control: no-store\r\n"
      "Connection: close\r\nContent-Length: " + std::to_string(body.size()) + "\r\n\r\n";
    send_all(cfd, hdr.data(), hdr.size());
    send_all(cfd, body.data(), body.size());
    ::close(cfd);
    return;
  }

  if (path == "/api/detector/classes") {
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
    std::string source = query_param(target, "source");
    if (source.empty()) source = "coco.names";
    static const std::set<std::string> kAllowedSources = {
      "coco.names", "coco_80_labels_list.txt", "coco_person_lebels_list.txt"
    };
    if (kAllowedSources.find(source) == kAllowedSources.end()) {
      const std::string body = "{\"ok\":false,\"error\":\"invalid_source\"}";
      const std::string hdr =
        "HTTP/1.1 400 Bad Request\r\nContent-Type: application/json\r\nCache-Control: no-store\r\n"
        "Connection: close\r\nContent-Length: " + std::to_string(body.size()) + "\r\n\r\n";
      send_all(cfd, hdr.data(), hdr.size());
      send_all(cfd, body.data(), body.size());
      ::close(cfd);
      return;
    }
    const auto labels = read_label_file_lines(std::filesystem::path("models") / source);
    std::ostringstream os;
    os << "{\"ok\":true,\"source\":\"" << json_escape(source) << "\",\"classes\":[";
    for (size_t i = 0; i < labels.size(); ++i) {
      if (i) os << ",";
      os << "\"" << json_escape(labels[i]) << "\"";
    }
    os << "]}";
    const std::string body = os.str();
    const std::string hdr =
      "HTTP/1.1 200 OK\r\nContent-Type: application/json\r\nCache-Control: no-store\r\n"
      "Connection: close\r\nContent-Length: " + std::to_string(body.size()) + "\r\n\r\n";
    send_all(cfd, hdr.data(), hdr.size());
    send_all(cfd, body.data(), body.size());
    ::close(cfd);
    return;
  }

  if (path == "/model/select") {
    std::string model = query_param(target, "name");
    bool ok = false;
    if (!model.empty()) {
      auto it = std::find(g_availableModels.begin(), g_availableModels.end(), model);
      if (it != g_availableModels.end()) ok = load_model(model);
    }
    std::string body = std::string("{\"ok\":") + (ok ? "true" : "false") + "}";

    if (ok) save_state();
    std::string hdr =
      "HTTP/1.1 200 OK\r\nContent-Type: application/json\r\nCache-Control: no-store\r\n"
      "Connection: close\r\nContent-Length: " + std::to_string(body.size()) + "\r\n\r\n";
    send_all(cfd, hdr.data(), hdr.size());
    send_all(cfd, body.data(), body.size());
    ::close(cfd);
    return;
  }

  if (path == "/classes/select") {
    std::set<int> selected;
    std::string ids = query_param(target, "ids");
    if (!ids.empty()) {
      std::stringstream ss(ids);
      std::string tok;
      while (std::getline(ss, tok, ',')) {
        tok = trim(tok);
        if (tok.empty()) continue;
        try {
          int id = std::stoi(tok);
          if (id >= 0 && id < static_cast<int>(g_labels.size())) selected.insert(id);
        } catch (...) {
        }
      }
    }
    {
      std::lock_guard<std::mutex> lk(g_classMtx);
      g_selectedClasses = std::move(selected);
    }
    save_state();
    std::string body = "{\"ok\":true}";
    std::string hdr =
      "HTTP/1.1 200 OK\r\nContent-Type: application/json\r\nCache-Control: no-store\r\n"
      "Connection: close\r\nContent-Length: " + std::to_string(body.size()) + "\r\n\r\n";
    send_all(cfd, hdr.data(), hdr.size());
    send_all(cfd, body.data(), body.size());
    ::close(cfd);
    return;
  }

  if (path == "/detect/toggle") {
    g_detectEnabled = !g_detectEnabled.load();
    std::string body = std::string("{\"detect\":") + (g_detectEnabled.load() ? "true" : "false") + "}";
    std::string hdr =
      "HTTP/1.1 200 OK\r\nContent-Type: application/json\r\nCache-Control: no-store\r\n"
      "Connection: close\r\nContent-Length: " + std::to_string(body.size()) + "\r\n\r\n";
    send_all(cfd, hdr.data(), hdr.size());
    send_all(cfd, body.data(), body.size());
    ::close(cfd);
    return;
  }

  if (path == "/stream") {
    const std::string boundary = "frame";
    std::string hdr =
      "HTTP/1.1 200 OK\r\n"
      "Cache-Control: no-store\r\nPragma: no-cache\r\nConnection: close\r\n"
      "Content-Type: multipart/x-mixed-replace; boundary=" + boundary + "\r\n\r\n";
    if (!send_all(cfd, hdr.data(), hdr.size())) { ::close(cfd); return; }

    uint64_t last = 0;
    while (g_running.load()) {
      std::vector<uint8_t> jpeg;
      uint64_t fid = 0;

      {
        std::unique_lock<std::mutex> lk(g_mtx);
        g_cv.wait_for(lk, std::chrono::milliseconds(500), [&]{
          return !g_running.load() || g_frameId.load() != last;
        });
        fid = g_frameId.load();
        if (fid == last) continue;
        jpeg = g_lastJpeg;
      }

      last = fid;
      if (jpeg.empty()) continue;

      std::string part = "--" + boundary + "\r\nContent-Type: image/jpeg\r\nContent-Length: " +
                         std::to_string(jpeg.size()) + "\r\n\r\n";
      if (!send_all(cfd, part.data(), part.size())) break;
      if (!send_all(cfd, jpeg.data(), jpeg.size())) break;
      if (!send_all(cfd, "\r\n", 2)) break;
    }

    ::close(cfd);
    return;
  }

  const char* body = "Not Found";
  std::string hdr =
    "HTTP/1.1 404 Not Found\r\nContent-Type: text/plain\r\nConnection: close\r\n"
    "Content-Length: " + std::to_string(std::strlen(body)) + "\r\n\r\n";
  send_all(cfd, hdr.data(), hdr.size());
  send_all(cfd, body, std::strlen(body));
  ::close(cfd);
}

static void http_server_thread(const Opts& o) {
  int sfd = ::socket(AF_INET, SOCK_STREAM, 0);
  if (sfd < 0) { perror("socket"); g_running = false; return; }

  int yes = 1;
  setsockopt(sfd, SOL_SOCKET, SO_REUSEADDR, &yes, sizeof(yes));

  sockaddr_in addr{};
  addr.sin_family = AF_INET;
  addr.sin_addr.s_addr = INADDR_ANY;
  addr.sin_port = htons((uint16_t)o.port);

  if (::bind(sfd, (sockaddr*)&addr, sizeof(addr)) < 0) {
    perror("bind");
    ::close(sfd);
    g_running = false;
    return;
  }
  if (::listen(sfd, 16) < 0) {
    perror("listen");
    ::close(sfd);
    g_running = false;
    return;
  }

  std::cout << "HTTP MJPEG on 0.0.0.0:" << o.port << "  ( / , /stream, /api/models, /model/select )\n";
  while (g_running.load()) {
    int cfd = ::accept(sfd, nullptr, nullptr);
    if (cfd < 0) continue;
    std::thread([cfd, &o]() { handle_client(cfd, o); }).detach();
  }
  ::close(sfd);
}

static void gst_capture_thread(const Opts& o) {
  gst_init(nullptr, nullptr);


  std::string deint = o.deinterlace ? "deinterlace mode=auto ! " : "";
  std::string pipe =
    "v4l2src device=" + o.dev + " io-mode=2 do-timestamp=true ! "
    + deint +
    "video/x-raw,format=NV24,width=" + std::to_string(o.width) +
    ",height=" + std::to_string(o.height) +
    ",framerate=" + std::to_string(o.fps) + "/1 ! "
    "videoconvert ! video/x-raw,format=I420 ! "
    "jpegenc quality=" + std::to_string(o.jpegq) + " ! "
    "queue leaky=downstream max-size-buffers=1 ! "
    "appsink name=sink emit-signals=false sync=false max-buffers=1 drop=true";

  GError* err = nullptr;
  GstElement* pipeline = gst_parse_launch(pipe.c_str(), &err);
  if (!pipeline) {
    std::cerr << "GST parse failed: " << (err ? err->message : "unknown") << "\n";
    if (err) g_error_free(err);
    g_running = false;
    return;
  }

  GstElement* sink = gst_bin_get_by_name(GST_BIN(pipeline), "sink");
  if (!sink) {
    std::cerr << "GST: appsink not found\n";
    gst_object_unref(pipeline);
    g_running = false;
    return;
  }

  GstBus* bus = gst_element_get_bus(pipeline);
  gst_element_set_state(pipeline, GST_STATE_PLAYING);

  while (g_running.load()) {
    if (bus) {
      if (GstMessage* msg = gst_bus_pop_filtered(bus, (GstMessageType)(GST_MESSAGE_ERROR | GST_MESSAGE_EOS))) {
        if (GST_MESSAGE_TYPE(msg) == GST_MESSAGE_ERROR) {
          GError* e = nullptr;
          gchar* dbg = nullptr;
          gst_message_parse_error(msg, &e, &dbg);
          std::cerr << "GST ERROR: " << (e ? e->message : "unknown") << "\n";
          if (dbg) std::cerr << "GST DEBUG: " << dbg << "\n";
          if (e) g_error_free(e);
          if (dbg) g_free(dbg);
        }
        gst_message_unref(msg);
        g_running = false;
        break;
      }
    }

    GstSample* sample = gst_app_sink_try_pull_sample(GST_APP_SINK(sink), 500 * GST_MSECOND);
    if (!sample) continue;

    GstBuffer* buf = gst_sample_get_buffer(sample);
    if (!buf) { gst_sample_unref(sample); continue; }

    GstMapInfo map{};
    if (gst_buffer_map(buf, &map, GST_MAP_READ)) {

      std::vector<uint8_t> outJpeg;

      if (g_detectEnabled.load()) {
        std::vector<uint8_t> inJpeg(map.data, map.data + map.size);
        cv::Mat bgr = cv::imdecode(inJpeg, cv::IMREAD_COLOR);

        if (!bgr.empty()) {
          g_lastFrameWidth = bgr.cols;
          g_lastFrameHeight = bgr.rows;
          std::vector<DetectionBox> collectedBoxes;
          DetectionMode dm = DetectionMode::FullFrame;
          std::vector<DetectionRoi> rois;
          {
            std::lock_guard<std::mutex> lk(g_roiMtx);
            dm = g_detectionMode;
            rois = g_detectionRois;
          }
          const uint64_t frameId = g_frameId.load();
          bool inferenceAttempted = false;
          bool anyInfer = false;
          if (dm == DetectionMode::FullFrame) {
            inferenceAttempted = true;
            anyInfer = run_inference_on_bgr(bgr, 0, 0, nullptr, collectedBoxes) || anyInfer;
          } else if (dm == DetectionMode::Tiled) {
            const int tw = std::max(1, bgr.cols / 2);
            const int th = std::max(1, bgr.rows / 2);
            for (int ty = 0; ty < 2; ++ty) for (int tx = 0; tx < 2; ++tx) {
              int x = tx * tw;
              int y = ty * th;
              int w = (tx == 1) ? (bgr.cols - x) : tw;
              int h = (ty == 1) ? (bgr.rows - y) : th;
              cv::Rect rc(x, y, std::max(1, w), std::max(1, h));
              inferenceAttempted = true;
              anyInfer = run_inference_on_bgr(bgr(rc), x, y, nullptr, collectedBoxes) || anyInfer;
            }
          } else {
            for (const auto& roi : rois) {
              if (!roi.enabled) continue;
              const int nth = std::max(1, roi.every_n_frames);
              if ((frameId % static_cast<uint64_t>(nth)) != 0) continue;
              int x = std::max(0, roi.x), y = std::max(0, roi.y);
              int w = std::max(1, roi.w), h = std::max(1, roi.h);
              if (x >= bgr.cols || y >= bgr.rows) continue;
              w = std::min(w, bgr.cols - x);
              h = std::min(h, bgr.rows - y);
              if (w <= 0 || h <= 0) continue;
              if (dm == DetectionMode::Hybrid && x == 0 && y == 0 && w == bgr.cols && h == bgr.rows) {
                inferenceAttempted = true;
                anyInfer = run_inference_on_bgr(bgr, 0, 0, &roi.class_filter, collectedBoxes) || anyInfer;
              } else {
                cv::Rect rc(x, y, w, h);
                inferenceAttempted = true;
                anyInfer = run_inference_on_bgr(bgr(rc), x, y, &roi.class_filter, collectedBoxes) || anyInfer;
              }
            }
          }

          if (anyInfer) {
            const int rawBeforeLimit = static_cast<int>(collectedBoxes.size());
            g_lastRawBoxes = rawBeforeLimit;
            limit_detection_boxes_by_confidence(collectedBoxes, g_maxRawCandidates.load());
            auto boxes = nms_detection_boxes(collectedBoxes, 0.45f);
            // TODO(adaptive): preserve selected track candidate while trimming by confidence.
            limit_detection_boxes_by_confidence(boxes, g_maxDetections.load());
            g_lastNmsBoxes = static_cast<int>(boxes.size());
            assign_stable_track_ids(boxes, bgr.cols, bgr.rows);
            update_tracker_runtime_state(boxes, frameId);
            update_confidence_layers(boxes, frameId, bgr.cols, bgr.rows);
            {
              std::lock_guard<std::mutex> lk(g_detMtx);
              g_lastDetectionBoxes = boxes;
              g_lastDetections = static_cast<int>(boxes.size());
            }
            draw_detection_boxes(bgr, boxes);
          } else if (inferenceAttempted) {
            g_lastDetections = 0;
            std::lock_guard<std::mutex> lk(g_detMtx);
            g_lastDetectionBoxes.clear();
            g_lastRawBoxes = 0;
            g_lastNmsBoxes = 0;
            update_tracker_runtime_state(g_lastDetectionBoxes, g_frameId.load());
            update_confidence_layers(g_lastDetectionBoxes, g_frameId.load(), bgr.cols, bgr.rows);
          }

          std::vector<int> p = {cv::IMWRITE_JPEG_QUALITY, o.jpegq};
          cv::imencode(".jpg", bgr, outJpeg, p);
        }
      }

      {
        std::lock_guard<std::mutex> lk(g_mtx);
        if (!outJpeg.empty()) g_lastJpeg.swap(outJpeg);
        else g_lastJpeg.assign(map.data, map.data + map.size);
        if (!g_detectEnabled.load()) {
          std::lock_guard<std::mutex> lk2(g_detMtx);
          g_lastDetectionBoxes.clear();
          g_lastRawBoxes = 0;
          g_lastNmsBoxes = 0;
          update_tracker_runtime_state(g_lastDetectionBoxes, g_frameId.load());
          update_confidence_layers(g_lastDetectionBoxes, g_frameId.load(),
                                   g_lastFrameWidth.load(), g_lastFrameHeight.load());
        }
        g_frameId.fetch_add(1);
      }
      g_cv.notify_all();
      gst_buffer_unmap(buf, &map);
    }

    gst_sample_unref(sample);
  }

  gst_element_set_state(pipeline, GST_STATE_NULL);
  if (bus) gst_object_unref(bus);
  gst_object_unref(sink);
  gst_object_unref(pipeline);
}

static void usage() {
  std::cout
    << "mjpeg_gst_http --dev /dev/video0 --port 8080 --width 1920 --height 1080 --fps 25 --jpeg 80 [--deinterlace]\n"
    << "              [--labels model/coco_80_labels_list.txt]\n"
    << "              [--model-dir new_yolo8/model_rknn] [--model <path/to/model.rknn>]\n"
    << "              [--cmd-max-pan 45] [--cmd-max-tilt 45] [--cmd-max-zoom 24]\n"
    << "              [--max-detections 10] [--max-raw-candidates 50]\n"
    << "              [--zoom-calib-enable] [--zoom-calib-uart /dev/ttyUSB0] [--zoom-calib-baud 115200] [--no-zoom-calib]\n";
}

static bool arg_eq(const char* a, const char* b) { return std::strcmp(a, b) == 0; }
int main(int argc, char** argv) {
  Opts o;
  load_detection_limits();
  for (int i = 1; i < argc; ++i) {
    if (arg_eq(argv[i], "--dev") && i + 1 < argc) o.dev = argv[++i];
    else if (arg_eq(argv[i], "--port") && i + 1 < argc) o.port = std::stoi(argv[++i]);
    else if (arg_eq(argv[i], "--width") && i + 1 < argc) o.width = std::stoi(argv[++i]);
    else if (arg_eq(argv[i], "--height") && i + 1 < argc) o.height = std::stoi(argv[++i]);
    else if (arg_eq(argv[i], "--fps") && i + 1 < argc) o.fps = std::stoi(argv[++i]);
    else if (arg_eq(argv[i], "--jpeg") && i + 1 < argc) o.jpegq = std::stoi(argv[++i]);
    else if (arg_eq(argv[i], "--labels") && i + 1 < argc) o.labels = argv[++i];
    else if (arg_eq(argv[i], "--model-dir") && i + 1 < argc) o.model_dir = argv[++i];
    else if (arg_eq(argv[i], "--model") && i + 1 < argc) o.model = argv[++i];
    else if (arg_eq(argv[i], "--cmd-max-pan") && i + 1 < argc) o.cmd_max_pan = std::stoi(argv[++i]);
    else if (arg_eq(argv[i], "--cmd-max-tilt") && i + 1 < argc) o.cmd_max_tilt = std::stoi(argv[++i]);
    else if (arg_eq(argv[i], "--cmd-max-zoom") && i + 1 < argc) o.cmd_max_zoom = std::stoi(argv[++i]);
    else if (arg_eq(argv[i], "--zoom-calib-enable")) o.zoom_calib_enable = true;
    else if (arg_eq(argv[i], "--zoom-calib-uart") && i + 1 < argc) o.zoom_calib_uart_dev = argv[++i];
    else if (arg_eq(argv[i], "--zoom-calib-baud") && i + 1 < argc) o.zoom_calib_uart_baud = std::stoi(argv[++i]);
    else if (arg_eq(argv[i], "--no-zoom-calib")) o.zoom_calib_enable = false;
    else if (arg_eq(argv[i], "--deinterlace")) o.deinterlace = true;
    else if (arg_eq(argv[i], "--max-detections") && i + 1 < argc) g_maxDetections = std::stoi(argv[++i]);
    else if (arg_eq(argv[i], "--max-raw-candidates") && i + 1 < argc) g_maxRawCandidates = std::stoi(argv[++i]);
    else if (arg_eq(argv[i], "-h") || arg_eq(argv[i], "--help")) { usage(); return 0; }
  }
  o.cmd_max_pan = std::max(0, std::min(100, o.cmd_max_pan));
  o.cmd_max_tilt = std::max(0, std::min(100, o.cmd_max_tilt));
  o.cmd_max_zoom = std::max(0, std::min(100, o.cmd_max_zoom));
  g_maxDetections = std::max(1, std::min(100, g_maxDetections.load()));
  g_maxRawCandidates = std::max(g_maxDetections.load(), std::min(300, std::max(1, g_maxRawCandidates.load())));
  g_cmdMaxPan = o.cmd_max_pan;
  g_cmdMaxTilt = o.cmd_max_tilt;
  g_cmdMaxZoom = o.cmd_max_zoom;
  std::cout << "zoom-calib: startup mode="
            << (o.zoom_calib_enable ? "enabled" : "disabled")
            << ", uart=" << (trim(o.zoom_calib_uart_dev).empty() ? "<unset>" : o.zoom_calib_uart_dev)
            << ", baud=" << o.zoom_calib_uart_baud << "\n";
  generate_zoom_calibration_profile(g_cmdMaxZoom);
  save_zoom_calibration_profile();
  std::cout << "Zoom calibration profile (fallback) generated at startup: " << g_zoomCalibFile
            << " (cmd_max_zoom=" << g_cmdMaxZoom << ", hold_ms=4000)\n";
  if (init_post_process(o.labels.c_str()) != 0) {
    std::cerr << "init_post_process failed: " << o.labels << "\n";
    return 1;
  }

  g_availableModels = discover_models(o);

  g_labels = read_labels(o.labels);
  if (g_labels.empty()) {
    std::cerr << "warn: labels list is empty: " << o.labels << "\n";
  }
  std::string modelFromState;
  bool detectFromState = false;
  load_state(modelFromState, &detectFromState);
  g_detectEnabled = detectFromState;
  load_roi_config();
  if (g_availableModels.empty()) {
    std::cerr << "No .rknn models found. Detection can be toggled but no model is loaded.\n";
  }

  std::error_code ec;
  if (o.model.empty() && !o.model_dir.empty()) {
    std::filesystem::path modelDir = o.model_dir;
    if (std::filesystem::exists(modelDir, ec) && std::filesystem::is_regular_file(modelDir, ec) && modelDir.extension() == ".rknn") {
      o.model = modelDir.string();
      std::cout << "--model-dir points to .rknn file, using it as startup model: " << o.model << "\n";
    }
  }

  if (o.model.empty() && !modelFromState.empty()) {
    o.model = modelFromState;
  }

  if (!o.model.empty()) {
    auto it = std::find(g_availableModels.begin(), g_availableModels.end(), o.model);
    if (it == g_availableModels.end()) g_availableModels.push_back(o.model);
    load_model(o.model);
  } else if (!g_availableModels.empty()) {
    load_model(g_availableModels.front());
  }

  std::thread t_gst(gst_capture_thread, o);
  std::this_thread::sleep_for(std::chrono::milliseconds(600));
  run_zoom_image_time_calibration(o);
  std::thread t_zoom_recalib(zoom_background_recalibration_thread, o);
  std::thread t_http(http_server_thread, o);

  t_http.join();
  g_running = false;
  if (t_zoom_recalib.joinable()) t_zoom_recalib.join();
  t_gst.join();
{
    std::lock_guard<std::mutex> lk(g_modelMtx);
    if (g_modelReady) release_yolov8_model(&g_rknn);
  }
  deinit_post_process();
  return 0;

}
