#pragma once
#include <string>
#include <map>
#include <set>
#include <thread>
#include <atomic>
#include <vector>
#include <mutex>

// Simple camera manager that reads configuration from JSON file
// and monitors /dev/v4l/by-id for matching devices. When a camera is
// connected or disconnected, a message is printed. This is a skeleton
// for future integration with detection pipelines.
class CameraManager {
public:
    struct CamConfig {
        std::string id;            // logical camera id
        std::string match_substr;  // substring to match in /dev/v4l/by-id path
    };

    // Load configuration from JSON file. Returns true on success.
    bool loadConfig(const std::string& path);

    // Start monitoring thread.
    void start();

    // Stop monitoring thread.
    void stop();

    struct ConfiguredInfo {
        std::string id;
        bool present;
    };

    // Thread-safe snapshot of configured cameras with presence flag
    std::vector<ConfiguredInfo> configuredCameras();

    // Thread-safe snapshot of newly discovered camera paths
    std::vector<std::string> unconfiguredCameras();

    // Append new camera definition to config and start monitoring it
    bool addCamera(const std::string& id, const std::string& by_id_path);

    // Remove camera from config and stop monitoring it
    bool removeCamera(const std::string& id);

    // Return device path for active camera id ("/dev/v4l/by-id/..."), empty if not active
    std::string devicePath(const std::string& id);

private:
    void monitorLoop();

    std::string config_path_;
    std::map<std::string, CamConfig> configs_;
    std::set<std::string> active_;
    std::map<std::string, std::string> active_paths_;
    std::set<std::string> unconfigured_;
    std::mutex mutex_;
    std::thread monitor_thread_;
    std::atomic<bool> running_{false};
};