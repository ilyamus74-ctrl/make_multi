
#pragma once
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <map>
#include <mutex>
#include <set>
#include <string>
#include <thread>
#include <deque>
#include <vector>
#include <sys/types.h>
#include <cstdint>
#include <opencv2/calib3d.hpp>

// Simple camera manager that reads configuration from JSON file
// and monitors /dev/v4l/by-id for matching devices. When a camera is
// connected or disconnected, a message is printed. This is a skeleton
// for future integration with detection pipelines.
class CameraManager {
public:
  CameraManager();
  ~CameraManager();
  using Clock = std::chrono::steady_clock;

  enum class PixelFormat { RGB, GRAY };

  struct StereoPair {
    std::string cam0;
    std::string cam1;
    cv::Mat Q;
    cv::Ptr<cv::StereoSGBM> matcher;
  };

  struct Frame {
    uint64_t t_mono_ns{0};
    int w{0};
    int h{0};
    std::vector<uint8_t> jpeg;
  };
  struct CamConfig {
    std::string id;           // logical camera id
    std::string match_substr; // substring to match in /dev/v4l/by-id path
    std::string match_path_substr; // substring to match in /dev/v4l/by-path path
    std::string device_path;  // last known /dev/videoX path
    std::string expected_bus_info; // unique hardware bus identifier
    std::string expected_card;     // human readable card name reported by V4L2
    enum class Mode { Preview, Detect, Calibration };
    Mode mode{Mode::Preview}; // current operating mode
    struct VideoMode {
      int w{1280};
      int h{720};
      std::string pixfmt{"MJPG"};
      int fps{30};
    };
    VideoMode preferred;             // current capture parameters
    VideoMode def_preferred{};       // default capture parameters
    int npu_worker{0};               // assigned NPU worker index
    int def_npu_worker{0};
    bool auto_profiles{true};
    bool def_auto_profiles{true};
    std::string profile{"auto"};   // current control profile
    std::string def_profile{"auto"};
    int det_port{0};                 // port of detection server
    int def_det_port{0};
    std::string model_path{"model_rknn/yolov8.rknn"};
    std::string labels_path{"model/coco_80_labels_list.txt"};
    std::string def_model_path{"model_rknn/yolov8.rknn"};
    std::string def_labels_path{"model/coco_80_labels_list.txt"};
    std::vector<std::string> det_args; // additional detector args
    struct Position { double x{0}; double y{0}; double z{0}; } position;
    Position def_position{};
    int cap_fps{30};                 // --cap-fps
    int def_cap_fps{30};
    int buffers{3};                  // --buffers
    int def_buffers{3};
    // --buffer-type: one of "auto", "single", "mplane"
    std::string buffer_type{"auto"};
    std::string def_buffer_type{"auto"};
    int jpeg_quality{60};            // --jpeg-quality
    int def_jpeg_quality{60};
    int http_fps_limit{20};          // --http-fps-limit
    int def_http_fps_limit{20};
    bool show_det_fps{false};        // --fps
    bool def_show_det_fps{false};
    std::string npu_core{"auto"};   // --npu-core
    std::string def_npu_core{"auto"};
    std::string log_file;            // --log-file
    std::string def_log_file;
    Mode def_mode{Mode::Preview};
    double fps{0.0};
    std::chrono::steady_clock::time_point last_frame{};
    // User-selected role of the camera within a multi-camera setup
    std::string role{"wide_angle_primary"};
    std::string def_role{"wide_angle_primary"};
    PixelFormat pixel_format{PixelFormat::RGB};
  };

  // Load configuration from JSON file. Returns true on success.
  bool loadConfig(const std::string &path);

  // Save current scheme type and camera roles to configuration file.
  bool saveConfig(const std::string &scheme_type,
                  const std::map<std::string, std::string> &roles);

  // Return current scheme type selected by user
  const std::string &schemeType() const { return scheme_type_; }


  // Start monitoring threads. If enable_monitoring is false, monitoring
  // threads are not launched, allowing the manager to be used without
  // spawning detector processes.
  void start(bool enable_monitoring = true);

  // Stop monitoring thread.
  void stop();

  bool isRunning() const { return running_.load(); }

  // Notify monitoring thread to re-check state immediately.
  void notify();

  // Schedule configuration reload from disk (safe to call from signal
  // handlers).
  void requestConfigReload();

  struct ConfiguredInfo {
    std::string id;
    bool present;
    CamConfig::Mode mode;
    CamConfig::VideoMode preferred;
    int npu_worker;
    bool auto_profiles;
    std::string profile;
    int det_port;
    bool det_running;
    CamConfig::Position position;
    double fps;
    std::string model_path;
    std::string labels_path;
    int cap_fps;
    int buffers;
    std::string buffer_type;
    int jpeg_quality;
    int http_fps_limit;
    bool show_det_fps;
    std::string npu_core;
    std::string log_file;
    std::string role;
  };

  // Thread-safe snapshot of configured cameras with presence flag
  std::vector<ConfiguredInfo> configuredCameras();

  // Thread-safe snapshot of newly discovered camera paths
  struct DiscoveredCamera {
    struct Identifier {
      std::string type;  // e.g. "by-id", "by-path", "device"
      std::string value; // identifier value for the given type
    };

    std::string device_path; // canonical /dev/videoX path
    std::string bus_info;    // value from v4l2_capability::bus_info
    std::string card;        // value from v4l2_capability::card
    std::vector<Identifier> identifiers;
  };

  std::vector<DiscoveredCamera> unconfiguredCameras();

  // Append new camera definition to config and start monitoring it
  bool addCamera(const std::string &id, const std::string &match_value,
                 const std::string &match_type = "by-id",
                 const std::string &device_path_hint = "");

  bool reassignCamera(const std::string &id, const std::string &match_value,
                      const std::string &match_type = "by-id",
                      const std::string &device_path_hint = "");
  // Set operating mode for camera id
  bool setMode(const std::string &id, CamConfig::Mode mode);

  // Set role for camera id
  bool setRole(const std::string &id, const std::string &role);


  // Update advanced settings for camera id
  bool updateSettings(const std::string &id, const CamConfig::VideoMode &pref,
                      int npu_worker, bool auto_profiles,
                      const std::string &profile,
                      const std::string &model_path,
                      const std::string &labels_path,
                      int cap_fps, int buffers, const std::string &buffer_type,
                      int jpeg_quality,
                      int http_fps_limit, bool show_det_fps,
                      const std::string &npu_core,
                      const std::string &log_file);
  // Reset settings to defaults for camera id
  bool resetSettings(const std::string &id);

  // Report captured frame for FPS calculation
  void reportFrame(const std::string &id);

 // Store a captured JPEG frame in short-term buffer
  void pushFrame(const std::string &id, int w, int h,
                 const std::vector<uint8_t> &jpeg,
                 uint64_t t_mono_ns);

  // Retrieve frame nearest to given timestamp from buffer
  bool getFrame(const std::string &id, uint64_t t_mono_ns, Frame &out);

  // Save JPEG and metadata to disk
  bool saveFrameWithMeta(const std::string &path, const Frame &f,
                         const std::string &cam_id);

  // Return current monotonic time in nanoseconds since manager start
  uint64_t nowMonoNs() const;

  std::vector<StereoPair> getActivePairs();
  void setActivePairs(const std::vector<StereoPair> &pairs);


  // Remove camera from config and stop monitoring it
  bool removeCamera(const std::string &id);

  // Return device path for active camera id ("/dev/v4l/by-id/..."), empty if
  // not active
  std::string devicePath(const std::string &id);


  bool setPixelFormat(const std::string &id, PixelFormat fmt);
  PixelFormat getPixelFormat(const std::string &id);

private:
  void monitorLoop();
  void configWatchLoop();
  bool applyProfile(CamConfig &cfg);
  void persistMatchMetadata(const CamConfig &cfg);
  std::string config_path_;
  std::map<std::string, CamConfig> configs_;
  std::set<std::string> active_;
  std::map<std::string, std::string> active_paths_; // id -> /dev/videoX
  std::vector<DiscoveredCamera> unconfigured_;
  std::map<std::string, pid_t> det_pids_;
  // Snapshot of arguments used to launch each detection process to detect
  // configuration drift and restart processes when settings change.
  std::map<std::string, CamConfig> last_det_configs_;
  std::map<std::string, Clock::time_point> last_det_restart_;
  std::mutex mutex_;
  std::condition_variable cv_;
  std::thread monitor_thread_;
  std::thread config_thread_;
  std::atomic<bool> running_{false};
  int wake_fd_{-1};
  int config_reload_event_fd_{-1};
  std::atomic<bool> config_reload_pending_{false};

  Clock::time_point start_time_;
  std::string scheme_type_{"hemisphere_single"};
  std::map<std::string, std::deque<Frame>> frame_buffers_;
  const std::chrono::seconds buffer_keep_{3};
  std::vector<StereoPair> active_pairs_;
};


extern CameraManager g_camera_manager;
