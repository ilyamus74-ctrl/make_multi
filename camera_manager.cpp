
#include "camera_manager.h"

#include "nlohmann/json.hpp"
#include <chrono>
#include <filesystem>
#include <fstream>
#include <cstdio>
#include <cstdlib>
#include <iostream>
#include <cstring>
#include <cerrno>
#include <vector>
#include <sys/wait.h>
#include <unistd.h>
#include <signal.h>

using json = nlohmann::json;

bool CameraManager::loadConfig(const std::string &path) {
  config_path_ = path;
  std::ifstream f(path);
  json j;
  if (!f.is_open()) {
    namespace fs = std::filesystem;
    fs::path p(path);
    fs::path dir = p.parent_path();
    if (!dir.empty()) {
      std::error_code ec;
      fs::create_directories(dir, ec);
      if (ec) {
        std::cerr << "CameraManager: failed to create config directory " << dir
                  << ": " << ec.message() << std::endl;
        return false;
      }
    }
    j = json::object();
    j["cameras"] = json::array();
    std::ofstream out(path);
    if (!out.is_open()) {
      std::cerr << "CameraManager: failed to create config " << path
                << std::endl;
      return false;
    }
    out << j.dump(2);
  } else {
    try {
      f >> j;
    } catch (const std::exception &e) {
      std::cerr << "CameraManager: failed to parse json: " << e.what()
                << std::endl;
      return false;
    }
  }
  if (j.contains("cameras")) {
    bool need_save = false;
    std::set<int> used_ports;
    for (auto &c : j["cameras"]) {
      int p = c.value("det_port", 0);
      if (p > 0)
        used_ports.insert(p);
    }
    int next_port = 8000;
    for (auto &c : j["cameras"]) {
      if (!c.contains("det_port") || c.value("det_port", 0) == 0) {
        while (used_ports.count(next_port))
          ++next_port;
        c["det_port"] = next_port;
        used_ports.insert(next_port);
        std::cout << "CameraManager: auto-assigned det_port " << next_port
                  << " for camera " << c.value("id", "") << std::endl;
        need_save = true;
      }
      if (!c.contains("model_path")) {
        c["model_path"] = "model_rknn/yolov8.rknn";
        need_save = true;
      }
      if (!c.contains("labels_path")) {
        c["labels_path"] = "model/coco_80_labels_list.txt";
        need_save = true;
      }
      if (!c.contains("det_args")) {
        c["det_args"] = json::array();
        need_save = true;
      }
      if (!c.contains("cap_fps")) {
        c["cap_fps"] = 30;
        need_save = true;
      }
      if (!c.contains("buffers")) {
        c["buffers"] = 3;
        need_save = true;
      }
      if (!c.contains("jpeg_quality")) {
        c["jpeg_quality"] = 60;
        need_save = true;
      }
      if (!c.contains("http_fps_limit")) {
        c["http_fps_limit"] = 20;
        need_save = true;
      }
      if (!c.contains("show_fps")) {
        c["show_fps"] = false;
        need_save = true;
      }
      if (!c.contains("npu_core")) {
        c["npu_core"] = "auto";
        need_save = true;
      }
      if (!c.contains("log_file")) {
        c["log_file"] = "";
        need_save = true;
      }
    }
    if (need_save) {
      std::ofstream out(path, std::ios::trunc);
      if (out.is_open())
        out << j.dump(2);
    }
    for (auto &c : j["cameras"]) {
      CamConfig cfg;
      cfg.id = c.value("id", "");
      if (c.contains("match") && c["match"].contains("by_id_contains"))
        cfg.match_substr = c["match"]["by_id_contains"].get<std::string>();
      cfg.device_path = c.value("device", "");
      cfg.preview = c.value("preview", true);
      if (c.contains("preferred")) {
        auto &p = c["preferred"];
        cfg.preferred.w = p.value("w", 1280);
        cfg.preferred.h = p.value("h", 720);
        cfg.preferred.pixfmt = p.value("pixfmt", std::string("MJPG"));
        cfg.preferred.fps = p.value("fps", 30);
      }
      cfg.npu_worker = c.value("npu_worker", 0);
      cfg.auto_profiles = c.value("auto_profiles", true);
      cfg.profile = c.value("profile", std::string("auto"));
      cfg.det_port = c.value("det_port", 0);
      cfg.model_path = c.value("model_path", std::string("model_rknn/yolov8.rknn"));
      cfg.labels_path = c.value("labels_path", std::string("model/coco_80_labels_list.txt"));
      if (c.contains("det_args") && c["det_args"].is_array())
        cfg.det_args = c["det_args"].get<std::vector<std::string>>();
      if (!c.contains("det_port")) {
        std::cerr << "CameraManager: camera " << cfg.id
                  << " missing det_port; detection disabled" << std::endl;
      }
      if (c.contains("position")) {
        auto &p = c["position"];
        cfg.position.x = p.value("x", 0.0);
        cfg.position.y = p.value("y", 0.0);
        cfg.position.z = p.value("z", 0.0);
      }
      cfg.cap_fps = c.value("cap_fps", 30);
      cfg.buffers = c.value("buffers", 3);
      cfg.jpeg_quality = c.value("jpeg_quality", 60);
      cfg.http_fps_limit = c.value("http_fps_limit", 20);
      cfg.show_det_fps = c.value("show_fps", false);
      cfg.npu_core = c.value("npu_core", std::string("auto"));
      cfg.log_file = c.value("log_file", std::string(""));
      cfg.def_preferred = cfg.preferred;
      cfg.def_npu_worker = cfg.npu_worker;
      cfg.def_auto_profiles = cfg.auto_profiles;
      cfg.def_profile = cfg.profile;
      cfg.def_det_port = cfg.det_port;
      cfg.def_position = cfg.position;
      cfg.def_model_path = cfg.model_path;
      cfg.def_labels_path = cfg.labels_path;
      cfg.def_cap_fps = cfg.cap_fps;
      cfg.def_buffers = cfg.buffers;
      cfg.def_jpeg_quality = cfg.jpeg_quality;
      cfg.def_http_fps_limit = cfg.http_fps_limit;
      cfg.def_show_det_fps = cfg.show_det_fps;
      cfg.def_npu_core = cfg.npu_core;
      cfg.def_log_file = cfg.log_file;
      if (!cfg.id.empty())
        configs_[cfg.id] = cfg;
    }
  }
  return true;
}

void CameraManager::start() {
  if (running_)
    return;
  running_ = true;
  monitor_thread_ = std::thread(&CameraManager::monitorLoop, this);
}

void CameraManager::stop() {
  running_ = false;
  if (monitor_thread_.joinable())
    monitor_thread_.join();
}

void CameraManager::monitorLoop() {
  using namespace std::chrono_literals;
  while (running_) {
    namespace fs = std::filesystem;
    std::set<std::string> current;
    const std::string base = "/dev/v4l/by-id";
    if (fs::exists(base)) {
      for (auto &p : fs::directory_iterator(base)) {
        std::string byid = p.path().string();
        std::string cmd = "udevadm info -q property -n " + byid;
        FILE *fp = popen(cmd.c_str(), "r");
        bool has_capture = false;
        if (fp) {
          char *line = nullptr;
          size_t len = 0;
          while (getline(&line, &len, fp) != -1) {
            std::string s(line);
            const std::string prefix = "ID_V4L_CAPABILITIES=";
            if (s.rfind(prefix, 0) == 0) {
              if (s.find(":capture:") != std::string::npos)
                has_capture = true;
              break;
            }
          }
          if (line)
            free(line);
          pclose(fp);
        }
        if (has_capture)
          current.insert(p.path().filename().string());
      }
    }

    std::set<std::string> new_unconfigured = current;

    for (auto &kv : configs_) {
      const auto &id = kv.first;
      CamConfig &cfg = kv.second;
      bool present = false;
      std::string matched;
      for (const auto &p : current) {
        if (p.find(cfg.match_substr) != std::string::npos) {
          present = true;
          matched = p;
          break;
        }
      }
      bool active;
      {
        std::lock_guard<std::mutex> lk(mutex_);
        active = active_.count(id) > 0;
        if (present) {
          std::error_code ec;
          auto dev = fs::canonical(base + "/" + matched, ec);
          std::string devpath = ec ? std::string{} : dev.string();
          active_paths_[id] = devpath;
          cfg.device_path = devpath;
          if (!active) {
            std::cout << "Camera " << id << " connected" << std::endl;
            active_.insert(id);
            if (cfg.det_port == 0)
              std::cerr << "CameraManager: detection disabled for camera "
                        << id << std::endl;
          }
        } else if (active) {
          std::cout << "Camera " << id << " disconnected" << std::endl;
          active_.erase(id);
          active_paths_.erase(id);
          auto itp = det_pids_.find(id);
          if (itp != det_pids_.end()) {
            kill(itp->second, SIGTERM);
            waitpid(itp->second, nullptr, 0);
            det_pids_.erase(itp);
          }
        }
      }
      for (auto it = new_unconfigured.begin(); it != new_unconfigured.end();) {
        if (it->find(cfg.match_substr) != std::string::npos)
          it = new_unconfigured.erase(it);
        else
          ++it;
      }

          if (present && cfg.det_port > 0) {
        pid_t pid = 0;
        {
          std::lock_guard<std::mutex> lk(mutex_);
          auto itp = det_pids_.find(id);
          if (itp != det_pids_.end())
            pid = itp->second;
        }
        if (pid > 0) {
          int status;
          pid_t rc = waitpid(pid, &status, WNOHANG);
          if (rc == pid) {
            if (WIFEXITED(status)) {
              std::cerr << "CameraManager: detection process for camera " << id
                        << " exited with status " << WEXITSTATUS(status)
                        << std::endl;
            } else if (WIFSIGNALED(status)) {
              std::cerr << "CameraManager: detection process for camera " << id
                        << " terminated by signal " << WTERMSIG(status)
                        << std::endl;
            }
            std::lock_guard<std::mutex> lk(mutex_);
            det_pids_.erase(id);
            pid = 0;
          }
        }
        if (pid == 0) {
          pid_t child = fork();
          if (child == 0) {
            std::string port = std::to_string(cfg.det_port);
            std::vector<std::string> args;
            args.push_back("yolov8_web_server");
            args.push_back(cfg.model_path);
            args.push_back("--dev");
            args.push_back(cfg.device_path);
            args.push_back("--port");
            args.push_back(port);
            if (!cfg.labels_path.empty()) {
              args.push_back("--labels");
              args.push_back(cfg.labels_path);
            }
            args.push_back("--cap-fps");
            args.push_back(std::to_string(cfg.cap_fps));
            args.push_back("--buffers");
            args.push_back(std::to_string(cfg.buffers));
            args.push_back("--jpeg-quality");
            args.push_back(std::to_string(cfg.jpeg_quality));
            args.push_back("--http-fps-limit");
            args.push_back(std::to_string(cfg.http_fps_limit));
            if (cfg.show_det_fps)
              args.push_back("--fps");
            args.push_back("--npu-core");
            args.push_back(cfg.npu_core);
            if (!cfg.log_file.empty()) {
              args.push_back("--log-file");
              args.push_back(cfg.log_file);
            }
            for (const auto &a : cfg.det_args)
              args.push_back(a);
            std::vector<char *> argv;
            for (auto &a : args)
              argv.push_back(const_cast<char *>(a.c_str()));
            argv.push_back(nullptr);
            execv("./yolov8_web_server", argv.data());
            std::cerr << "CameraManager: execv failed for camera " << id
                      << ": " << std::strerror(errno) << std::endl;
            _exit(1);
          } else if (child > 0) {
            {
              std::lock_guard<std::mutex> lk(mutex_);
              det_pids_[id] = child;
            }
            std::cout << "CameraManager: forked detection pid " << child
                      << " for device " << cfg.device_path << " port "
                      << cfg.det_port << std::endl;
          } else {
            std::cerr << "CameraManager: fork failed for camera " << id
                      << ": " << std::strerror(errno) << std::endl;
          }
        }
      }
    }

    {
      std::lock_guard<std::mutex> lk(mutex_);
      unconfigured_ = std::move(new_unconfigured);
    }

    std::this_thread::sleep_for(1s);
  }
}

bool CameraManager::removeCamera(const std::string &id) {
  std::lock_guard<std::mutex> lk(mutex_);
  if (!configs_.erase(id))
    return false;
  active_.erase(id);
  active_paths_.erase(id);
  json j;
  {
    std::ifstream f(config_path_);
    if (f.is_open()) {
      try {
        f >> j;
      } catch (...) {
      }
    }
  }
  if (!j.is_object())
    j = json::object();
  if (j.contains("cameras")) {
    auto &arr = j["cameras"];
    for (auto it = arr.begin(); it != arr.end(); ++it) {
      if (it->value("id", "") == id) {
        arr.erase(it);
        break;
      }
    }
  }
  std::ofstream out(config_path_, std::ios::trunc);
  if (!out.is_open())
    return false;
  out << j.dump(2);
  return true;
}

std::string CameraManager::devicePath(const std::string &id) {
  std::lock_guard<std::mutex> lk(mutex_);
  auto it = active_paths_.find(id);
  if (it == active_paths_.end())
    return {};
  return it->second;
}

bool CameraManager::applyProfile(CamConfig &cfg) {
  if (cfg.device_path.empty())
    return false;
  std::string ctrl;
  if (cfg.profile == "bright")
    ctrl = "--set-ctrl=auto_exposure=1,exposure_time_absolute=3,gain=0,gamma=120,saturation=128";
  else if (cfg.profile == "indoor")
    ctrl = "--set-ctrl=auto_exposure=1,exposure_time_absolute=6,gain=16,gamma=133,saturation=128";
  else if (cfg.profile == "dark")
    ctrl = "--set-ctrl=auto_exposure=1,exposure_time_absolute=12,gain=32,gamma=150,saturation=0";
  else
    ctrl = "--set-ctrl=auto_exposure=3"; // auto
  std::string cmd = "v4l2-ctl -d " + cfg.device_path + " " + ctrl;
  int rc = std::system(cmd.c_str());
  if (rc != 0) {
    std::cerr << "CameraManager: failed to apply profile '" << cfg.profile
              << "' for camera " << cfg.id << std::endl;
    return false;
  }
  return true;
}

std::vector<CameraManager::ConfiguredInfo> CameraManager::configuredCameras() {
  std::lock_guard<std::mutex> lk(mutex_);
  std::vector<ConfiguredInfo> out;
  for (auto &kv : configs_) {
    ConfiguredInfo ci{};
    ci.id = kv.first;
    ci.present = active_.count(kv.first) > 0;
    ci.preview = kv.second.preview;
    ci.preferred = kv.second.preferred;
    ci.npu_worker = kv.second.npu_worker;
    ci.auto_profiles = kv.second.auto_profiles;
    ci.profile = kv.second.profile;
    ci.det_port = kv.second.det_port;
    ci.position = kv.second.position;
    ci.fps = kv.second.fps;
    ci.model_path = kv.second.model_path;
    ci.labels_path = kv.second.labels_path;
    ci.cap_fps = kv.second.cap_fps;
    ci.buffers = kv.second.buffers;
    ci.jpeg_quality = kv.second.jpeg_quality;
    ci.http_fps_limit = kv.second.http_fps_limit;
    ci.show_det_fps = kv.second.show_det_fps;
    ci.npu_core = kv.second.npu_core;
    ci.log_file = kv.second.log_file;
    out.push_back(ci);
  } 
  return out;
}

std::vector<std::string> CameraManager::unconfiguredCameras() {
  std::lock_guard<std::mutex> lk(mutex_);
  return std::vector<std::string>(unconfigured_.begin(), unconfigured_.end());
}

bool CameraManager::addCamera(const std::string &id,
                              const std::string &by_id_path) {
  CamConfig cfg;
  cfg.id = id;
  cfg.match_substr = by_id_path;
  cfg.preview = true;
  cfg.profile = "auto";
  cfg.det_port = 0;
  cfg.position = {};
  std::error_code ec;
  auto dev = std::filesystem::canonical(
      std::string("/dev/v4l/by-id/") + by_id_path, ec);
  if (!ec)
    cfg.device_path = dev.string();
 {
    std::set<int> used;
    for (auto &kv : configs_)
      used.insert(kv.second.det_port);
    int port = 8000;
    while (used.count(port))
      ++port;
    cfg.det_port = port;
  }
  cfg.def_preferred = cfg.preferred;
  cfg.def_det_port = cfg.det_port;
  cfg.def_position = cfg.position;
  cfg.def_npu_worker = cfg.npu_worker;
  cfg.def_auto_profiles = cfg.auto_profiles;
  cfg.def_profile = cfg.profile;
  cfg.def_model_path = cfg.model_path;
  cfg.def_labels_path = cfg.labels_path;
  cfg.def_cap_fps = cfg.cap_fps;
  cfg.def_buffers = cfg.buffers;
  cfg.def_jpeg_quality = cfg.jpeg_quality;
  cfg.def_http_fps_limit = cfg.http_fps_limit;
  cfg.def_show_det_fps = cfg.show_det_fps;
  cfg.def_npu_core = cfg.npu_core;
  cfg.def_log_file = cfg.log_file;

  {
    std::lock_guard<std::mutex> lk(mutex_);
    if (configs_.count(id))
      return false;
    configs_[id] = cfg;
  }
  json j;
  {
    std::ifstream f(config_path_);
    if (f.is_open()) {
      try {
        f >> j;
      } catch (...) {
      }
    }
  }
  if (!j.is_object())
    j = json::object();
  if (!j.contains("cameras"))
    j["cameras"] = json::array();
  json cam;
  cam["id"] = id;
  cam["match"] = json{{"by_id_contains", by_id_path}};
  if (!cfg.device_path.empty())
    cam["device"] = cfg.device_path;
  cam["preview"] = true;
  cam["preferred"] =
      json{{"w", cfg.preferred.w},
           {"h", cfg.preferred.h},
           {"pixfmt", cfg.preferred.pixfmt},
           {"fps", cfg.preferred.fps}};
  cam["det_port"] = cfg.det_port;
  cam["model_path"] = cfg.model_path;
  cam["labels_path"] = cfg.labels_path;
  cam["det_args"] = json::array();
  cam["position"] = json{{"x", cfg.position.x}, {"y", cfg.position.y}, {"z", cfg.position.z}};
  cam["npu_worker"] = cfg.npu_worker;
  cam["auto_profiles"] = cfg.auto_profiles;
  cam["profile"] = cfg.profile;
  cam["cap_fps"] = cfg.cap_fps;
  cam["buffers"] = cfg.buffers;
  cam["jpeg_quality"] = cfg.jpeg_quality;
  cam["http_fps_limit"] = cfg.http_fps_limit;
  cam["show_fps"] = cfg.show_det_fps;
  cam["npu_core"] = cfg.npu_core;
  cam["log_file"] = cfg.log_file;
  j["cameras"].push_back(cam);
  std::ofstream out(config_path_, std::ios::trunc);
  if (!out.is_open())
    return false;
  out << j.dump(2);
  return true;
}

bool CameraManager::setPreview(const std::string &id, bool enable) {
  std::lock_guard<std::mutex> lk(mutex_);
  auto it = configs_.find(id);
  if (it == configs_.end())
    return false;
  if (it->second.preview == enable)
    return true;
  it->second.preview = enable;
  json j;
  {
    std::ifstream f(config_path_);
    if (f.is_open()) {
      try {
        f >> j;
      } catch (...) {
      }
    }
  }
  if (!j.is_object())
    j = json::object();
  if (!j.contains("cameras"))
    j["cameras"] = json::array();
  for (auto &c : j["cameras"]) {
    if (c.value("id", "") == id) {
      c["preview"] = enable;
      if (it->second.device_path.size())
        c["device"] = it->second.device_path;
    }
  }
  std::ofstream out(config_path_, std::ios::trunc);
  if (!out.is_open())
    return false;
  out << j.dump(2);
  return true;
}

bool CameraManager::updateSettings(const std::string &id,
                                   const CamConfig::VideoMode &pref,
                                   int npu_worker, bool auto_profiles,
                                   const std::string &profile,
                                   const std::string &model_path,
                                   const std::string &labels_path,
                                   int cap_fps, int buffers, int jpeg_quality,
                                   int http_fps_limit, bool show_det_fps,
                                   const std::string &npu_core,
                                   const std::string &log_file) {
  std::lock_guard<std::mutex> lk(mutex_);
  auto it = configs_.find(id);
  if (it == configs_.end())
    return false;
  it->second.preferred = pref;
  it->second.npu_worker = npu_worker;
  it->second.auto_profiles = auto_profiles;
  if (!profile.empty())
    it->second.profile = profile;
  if (!model_path.empty())
    it->second.model_path = model_path;
  if (!labels_path.empty())
    it->second.labels_path = labels_path;
  it->second.cap_fps = cap_fps;
  it->second.buffers = buffers;
  it->second.jpeg_quality = jpeg_quality;
  it->second.http_fps_limit = http_fps_limit;
  it->second.show_det_fps = show_det_fps;
  if (!npu_core.empty())
    it->second.npu_core = npu_core;
  it->second.log_file = log_file;
  applyProfile(it->second);
  json j;
  {
    std::ifstream f(config_path_);
    if (f.is_open()) {
      try {
        f >> j;
      } catch (...) {
      }
    }
  }
  if (!j.is_object())
    j = json::object();
  if (!j.contains("cameras"))
    j["cameras"] = json::array();
  for (auto &c : j["cameras"]) {
    if (c.value("id", "") == id) {
      c["preferred"] = {
          {"w", pref.w},
          {"h", pref.h},
          {"pixfmt", pref.pixfmt},
          {"fps", pref.fps}};
      c["npu_worker"] = npu_worker;
      c["auto_profiles"] = auto_profiles;
      c["profile"] = it->second.profile;
      c["model_path"] = it->second.model_path;
      c["labels_path"] = it->second.labels_path;
      c["cap_fps"] = cap_fps;
      c["buffers"] = buffers;
      c["jpeg_quality"] = jpeg_quality;
      c["http_fps_limit"] = http_fps_limit;
      c["show_fps"] = show_det_fps;
      c["npu_core"] = it->second.npu_core;
      c["log_file"] = it->second.log_file;
      if (it->second.device_path.size())
        c["device"] = it->second.device_path;
    }
  }
  std::ofstream out(config_path_, std::ios::trunc);
  if (!out.is_open())
    return false;
  out << j.dump(2);
  return true;
}

bool CameraManager::resetSettings(const std::string &id) {
  std::lock_guard<std::mutex> lk(mutex_);
  auto it = configs_.find(id);
  if (it == configs_.end())
    return false;
  CamConfig &cfg = it->second;
  cfg.preferred = cfg.def_preferred;
  cfg.npu_worker = cfg.def_npu_worker;
  cfg.auto_profiles = cfg.def_auto_profiles;
  cfg.profile = cfg.def_profile;
  cfg.det_port = cfg.def_det_port;
  cfg.position = cfg.def_position;
  cfg.model_path = cfg.def_model_path;
  cfg.labels_path = cfg.def_labels_path;
  cfg.cap_fps = cfg.def_cap_fps;
  cfg.buffers = cfg.def_buffers;
  cfg.jpeg_quality = cfg.def_jpeg_quality;
  cfg.http_fps_limit = cfg.def_http_fps_limit;
  cfg.show_det_fps = cfg.def_show_det_fps;
  cfg.npu_core = cfg.def_npu_core;
  cfg.log_file = cfg.def_log_file;
  applyProfile(cfg);

  json j;
  {
    std::ifstream f(config_path_);
    if (f.is_open()) {
      try {
        f >> j;
      } catch (...) {
      }
    }
  }
  if (!j.is_object())
    j = json::object();
  if (!j.contains("cameras"))
    j["cameras"] = json::array();
  for (auto &c : j["cameras"]) {
    if (c.value("id", "") == id) {
      c["preferred"] = {{"w", cfg.preferred.w},
                         {"h", cfg.preferred.h},
                         {"pixfmt", cfg.preferred.pixfmt},
                         {"fps", cfg.preferred.fps}};
      c["npu_worker"] = cfg.npu_worker;
      c["auto_profiles"] = cfg.auto_profiles;
      c["profile"] = cfg.profile;
      c["det_port"] = cfg.det_port;
      c["position"] = {{"x", cfg.position.x}, {"y", cfg.position.y}, {"z", cfg.position.z}};
      c["model_path"] = cfg.model_path;
      c["labels_path"] = cfg.labels_path;
      c["cap_fps"] = cfg.cap_fps;
      c["buffers"] = cfg.buffers;
      c["jpeg_quality"] = cfg.jpeg_quality;
      c["http_fps_limit"] = cfg.http_fps_limit;
      c["show_fps"] = cfg.show_det_fps;
      c["npu_core"] = cfg.npu_core;
      c["log_file"] = cfg.log_file;
      if (!cfg.device_path.empty())
        c["device"] = cfg.device_path;
    }
  }
  std::ofstream out(config_path_, std::ios::trunc);
  if (!out.is_open())
    return false;
  out << j.dump(2);
  return true;
}

void CameraManager::reportFrame(const std::string &id) {
  std::lock_guard<std::mutex> lk(mutex_);
  auto it = configs_.find(id);
  if (it == configs_.end())
    return;
  auto now = std::chrono::steady_clock::now();
  CamConfig &cfg = it->second;
  if (cfg.last_frame.time_since_epoch().count() != 0) {
    double dt =
        std::chrono::duration<double>(now - cfg.last_frame).count();
    if (dt > 0) {
      double fps_inst = 1.0 / dt;
      cfg.fps = (cfg.fps == 0.0) ? fps_inst : (0.8 * cfg.fps + 0.2 * fps_inst);
    }
  }
  cfg.last_frame = now;
}
