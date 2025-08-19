#pragma once
#include <string>
#include <map>
#include <set>
#include <thread>
#include <atomic>

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

private:
    void monitorLoop();
    bool isPresent(const CamConfig& cfg);

    std::map<std::string, CamConfig> configs_;
    std::set<std::string> active_;
    std::thread monitor_thread_;
    std::atomic<bool> running_{false};
};