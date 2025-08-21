
#pragma once
#include <atomic>
#include <chrono>
#include <map>
#include <mutex>
#include <set>
#include <string>
#include <thread>
#include <vector>

// Simple camera manager that reads configuration from JSON file
// and monitors /dev/v4l/by-id for matching devices. When a camera is
// connected or disconnected, a message is printed. This is a skeleton
// for future integration with detection pipelines.
class CameraManager {
public:
  struct CamConfig {
    std::string id;           // logical camera id
    std::string match_substr; // substring to match in /dev/v4l/by-id path
    std::string device_path;  // last known /dev/videoX path
    bool preview{true};       // preview enabled flag
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
    struct Position { double x{0}; double y{0}; double z{0}; } position;
    Position def_position{};
    double fps{0.0};
    std::chrono::steady_clock::time_point last_frame{};
  };


  // Load configuration from JSON file. Returns true on success.
  bool loadConfig(const std::string &path);

  // Start monitoring thread.
  void start();

  // Stop monitoring thread.
  void stop();

  struct ConfiguredInfo {
    std::string id;
    bool present;
    bool preview;
    CamConfig::VideoMode preferred;
    int npu_worker;
    bool auto_profiles;
    std::string profile;
    int det_port;
    CamConfig::Position position;
    double fps;
  };

  // Thread-safe snapshot of configured cameras with presence flag
  std::vector<ConfiguredInfo> configuredCameras();

  // Thread-safe snapshot of newly discovered camera paths
  std::vector<std::string> unconfiguredCameras();

  // Append new camera definition to config and start monitoring it
  bool addCamera(const std::string &id, const std::string &by_id_path);

  // Enable or disable preview for camera id
  bool setPreview(const std::string &id, bool enable);

  // Update advanced settings for camera id
  bool updateSettings(const std::string &id, const CamConfig::VideoMode &pref,
                      int npu_worker, bool auto_profiles,
                      const std::string &profile);
  // Reset settings to defaults for camera id
  bool resetSettings(const std::string &id);

  // Report captured frame for FPS calculation
  void reportFrame(const std::string &id);

  // Remove camera from config and stop monitoring it
  bool removeCamera(const std::string &id);

  // Return device path for active camera id ("/dev/v4l/by-id/..."), empty if
  // not active
  std::string devicePath(const std::string &id);

private:
  void monitorLoop();
  bool applyProfile(CamConfig &cfg);
  std::string config_path_;
  std::map<std::string, CamConfig> configs_;
  std::set<std::string> active_;
  std::map<std::string, std::string> active_paths_; // id -> /dev/videoX
  std::set<std::string> unconfigured_;
  std::mutex mutex_;
  std::thread monitor_thread_;
  std::atomic<bool> running_{false};
};
