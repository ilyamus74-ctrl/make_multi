#include "camera_manager.h"

#include <filesystem>
#include <fstream>
#include <iostream>
#include <chrono>
#include "nlohmann/json.hpp"

using json = nlohmann::json;

bool CameraManager::loadConfig(const std::string& path) {
    std::ifstream f(path);
    if (!f.is_open()) {
        std::cerr << "CameraManager: failed to open config " << path << std::endl;
        return false;
    }
    json j;
    try {
        f >> j;
    } catch (const std::exception& e) {
        std::cerr << "CameraManager: failed to parse json: " << e.what() << std::endl;
        return false;
    }
    if (!j.contains("cameras")) {
        std::cerr << "CameraManager: no cameras section" << std::endl;
        return false;
    }
    for (auto& c : j["cameras"]) {
        CamConfig cfg;
        cfg.id = c.value("id", "");
        if (c.contains("match") && c["match"].contains("by_id_contains"))
            cfg.match_substr = c["match"]["by_id_contains"].get<std::string>();
        if (!cfg.id.empty())
            configs_[cfg.id] = cfg;
    }
    return true;
}

void CameraManager::start() {
    if (running_) return;
    running_ = true;
    monitor_thread_ = std::thread(&CameraManager::monitorLoop, this);
}

void CameraManager::stop() {
    running_ = false;
    if (monitor_thread_.joinable()) monitor_thread_.join();
}

bool CameraManager::isPresent(const CamConfig& cfg) {
    namespace fs = std::filesystem;
    const std::string base = "/dev/v4l/by-id";
    if (!fs::exists(base)) return false;
    for (auto& p : fs::directory_iterator(base)) {
        std::string path = p.path().string();
        if (path.find(cfg.match_substr) != std::string::npos) {
            return true;
        }
    }
    return false;
}

void CameraManager::monitorLoop() {
    using namespace std::chrono_literals;
    while (running_) {
        for (auto& kv : configs_) {
            const auto& id = kv.first;
            const CamConfig& cfg = kv.second;
            bool present = isPresent(cfg);
            bool active = active_.count(id) > 0;
            if (present && !active) {
                std::cout << "Camera " << id << " connected" << std::endl;
                active_.insert(id);
            } else if (!present && active) {
                std::cout << "Camera " << id << " disconnected" << std::endl;
                active_.erase(id);
            }
        }
        std::this_thread::sleep_for(1s);
    }
}