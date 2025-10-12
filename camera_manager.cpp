
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
#include <limits>
#include <sys/wait.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/videodev2.h>
#include <signal.h>
#include <opencv2/calib3d.hpp>
#include <sys/inotify.h>

// Информация о стереопарах теперь хранится внутри CameraManager.

using json = nlohmann::json;

CameraManager g_camera_manager;

CameraManager::CameraManager() { start_time_ = Clock::now(); }

bool CameraManager::loadConfig(const std::string &path) {
  std::lock_guard<std::mutex> lk(mutex_);
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
//    j["cameras"] = json::array();
    j["scheme_type"] = "hemisphere_single";
    json cam;
    cam["id"] = "";
    cam["device"] = "";
    cam["role"] = "wide_angle_primary";
    j["cameras"] = json::array({cam});
    std::ofstream out(path);
    if (!out.is_open()) {
      std::cerr << "CameraManager: failed to create config " << path
                << std::endl;
      return false;
    }
    out << j.dump(2);
    scheme_type_ = j["scheme_type"].get<std::string>();
    return true;
  } else {
    try {
      f >> j;
    } catch (const std::exception &e) {
      std::cerr << "CameraManager: failed to parse json: " << e.what()
                << std::endl;
      return false;
    }
  }
  bool need_save = false;
  if (!j.contains("scheme_type")) {
    j["scheme_type"] = "hemisphere_single";
    need_save = true;
  }
  scheme_type_ = j.value("scheme_type", std::string("hemisphere_single"));
  if (j.contains("cameras")) {
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
      if (!c.contains("buffer_type")) {
        c["buffer_type"] = "auto";
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
      if (!c.contains("role")) {
        c["role"] = "wide_angle_primary";
        need_save = true;
      }
    }
  }
  if (need_save) {
    std::ofstream out(path, std::ios::trunc);
    if (out.is_open())
      out << j.dump(2);
  }
  if (j.contains("cameras")) {
    for (auto &c : j["cameras"]) {
      CamConfig cfg;
      cfg.id = c.value("id", "");
      if (c.contains("match") && c["match"].is_object()) {
        auto &match = c["match"];
        if (match.contains("by_id_contains"))
          cfg.match_substr = match["by_id_contains"].get<std::string>();
        if (match.contains("by_path_contains"))
          cfg.match_path_substr = match["by_path_contains"].get<std::string>();
        if (cfg.device_path.empty() && match.contains("device_path"))
          cfg.device_path = match["device_path"].get<std::string>();
      }
      std::string device_from_json = c.value("device", std::string());
      if (!device_from_json.empty())
        cfg.device_path = device_from_json;
      cfg.mode = CamConfig::Mode::Preview;
      if (c.contains("mode") && c["mode"].is_string()) {
        std::string m = c["mode"].get<std::string>();
        if (m == "detect")
          cfg.mode = CamConfig::Mode::Detect;
        else if (m == "calibration")
          cfg.mode = CamConfig::Mode::Calibration;
      } else if (c.contains("preview")) {
        if (c["preview"].is_string() && c["preview"] == "calibration") {
          cfg.mode = CamConfig::Mode::Calibration;
        } else if (c["preview"].is_boolean()) {
          cfg.mode = c["preview"].get<bool>() ? CamConfig::Mode::Preview
                                               : CamConfig::Mode::Detect;
        }
      }
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
      cfg.buffer_type = c.value("buffer_type", std::string("auto"));
      cfg.jpeg_quality = c.value("jpeg_quality", 60);
      cfg.http_fps_limit = c.value("http_fps_limit", 20);
      cfg.show_det_fps = c.value("show_fps", false);
      cfg.npu_core = c.value("npu_core", std::string("auto"));
      cfg.log_file = c.value("log_file", std::string(""));
      cfg.role = c.value("role", std::string("wide_angle_primary"));
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
      cfg.def_buffer_type = cfg.buffer_type;
      cfg.def_jpeg_quality = cfg.jpeg_quality;
      cfg.def_http_fps_limit = cfg.http_fps_limit;
      cfg.def_mode = cfg.mode;
      cfg.def_show_det_fps = cfg.show_det_fps;
      cfg.def_npu_core = cfg.npu_core;
      cfg.def_log_file = cfg.log_file;
      cfg.def_role = cfg.role;
      if (!cfg.id.empty())
        configs_[cfg.id] = cfg;
    }
  }

 // Загрузка описаний стереопар из конфигурации. Для каждой пары создаётся
  // объект StereoSGBM с параметрами, указанными в файле калибровки. Это
  // позволяет в дальнейшем рассчитывать карту диспаритета.
  if (j.contains("stereo_pairs") && j["stereo_pairs"].is_array()) {
    active_pairs_.clear();
    for (auto &p : j["stereo_pairs"]) {
      StereoPair sp;
      sp.cam0 = p.value("cam0", std::string());
      sp.cam1 = p.value("cam1", std::string());
      if (p.contains("Q") && p["Q"].is_array()) {
        std::vector<double> qv = p["Q"].get<std::vector<double>>();
        if (qv.size() == 16)
          sp.Q = cv::Mat(4, 4, CV_64F, qv.data()).clone();
      }
      int min_disp = p.value("min_disparity", 0);
      int num_disp = p.value("num_disparities", 64);
      int block_size = p.value("block_size", 5);
      sp.matcher = cv::StereoSGBM::create(min_disp, num_disp, block_size);
      active_pairs_.push_back(sp);
    }
  }


  return true;
}

void CameraManager::start(bool enable_monitoring) {
  if (!enable_monitoring)
    return;
  if (running_)
    return;
  running_ = true;
  monitor_thread_ = std::thread(&CameraManager::monitorLoop, this);
  config_thread_ = std::thread(&CameraManager::configWatchLoop, this);
}

void CameraManager::stop() {
  running_ = false;
  cv_.notify_all();
  if (config_thread_.joinable())
    config_thread_.join();
  if (monitor_thread_.joinable())
    monitor_thread_.join();

  // Ensure all detection subprocesses are terminated when stopping. Without
  // this, leftover yolov8_web_server instances may continue running after the
  // manager exits.
  std::lock_guard<std::mutex> lk(mutex_);
  for (auto &kv : det_pids_) {
    pid_t pid = kv.second;
    if (pid <= 0)
      continue;
    // Try to terminate gracefully first.
    kill(pid, SIGTERM);
    waitpid(pid, nullptr, 0);
  }
  det_pids_.clear();
}

void CameraManager::notify() { cv_.notify_all(); }

void CameraManager::monitorLoop() {
  namespace fs = std::filesystem;
  using namespace std::chrono_literals;
 
  while (running_) {


    {
      std::lock_guard<std::mutex> lk(mutex_);
      for (auto it = active_.begin(); it != active_.end();) {
        if (!configs_.count(*it)) {
          auto id = *it;
          it = active_.erase(it);
          active_paths_.erase(id);
          auto itp = det_pids_.find(id);
          if (itp != det_pids_.end()) {
            kill(itp->second, SIGTERM);
            waitpid(itp->second, nullptr, 0);
            det_pids_.erase(itp);

          }
     } else {
          ++it;
        }
      }
    }

    struct AvailableCamera {
      std::string canonical;
      std::vector<DiscoveredCamera::Identifier> identifiers;
      bool checked{false};
      bool has_capture{false};
      std::string bus_info;
      std::string card;
    };

    std::map<std::string, AvailableCamera> available;
    auto register_entry = [&](const fs::path &dir_entry,
                              const std::string &type) {
      std::error_code ec;
      if (!dir_entry.is_symlink(ec) || ec)
        return;
      auto canonical = fs::canonical(dir_entry, ec);
      if (ec)
        return;
      auto &info = available[canonical.string()];
      info.canonical = canonical.string();
      DiscoveredCamera::Identifier ident{type,
                                         dir_entry.filename().string()};
      info.identifiers.push_back(std::move(ident));
    };

    std::error_code ec;

    const fs::path by_id_base{"/dev/v4l/by-id"};
    if (fs::exists(by_id_base, ec)) {
      for (auto it = fs::directory_iterator(by_id_base, ec);
           it != fs::directory_iterator(); ++it) {
        register_entry(it->path(), "by-id");
      }
    }
    ec.clear();
    const fs::path by_path_base{"/dev/v4l/by-path"};
    if (fs::exists(by_path_base, ec)) {
      for (auto it = fs::directory_iterator(by_path_base, ec);
           it != fs::directory_iterator(); ++it) {
        register_entry(it->path(), "by-path");
      }
    }

    ec.clear();
    const fs::path video_base{"/dev"};
    if (fs::exists(video_base, ec)) {
      for (auto it = fs::directory_iterator(video_base, ec);
           it != fs::directory_iterator(); ++it) {
        if (ec)
          break;
        auto name = it->path().filename().string();
        if (name.rfind("video", 0) != 0)
          continue;
        std::error_code sec;
        auto canonical = fs::canonical(it->path(), sec);
        if (sec)
          canonical = it->path();
        auto &info = available[canonical.string()];
        if (info.canonical.empty())
          info.canonical = canonical.string();
      }
    }

    auto ensure_capture = [&](AvailableCamera &info) {
      if (info.checked)
        return info.has_capture;
      info.checked = true;
      int fd = open(info.canonical.c_str(), O_RDONLY | O_NONBLOCK);
      if (fd >= 0) {
        v4l2_capability cap{};
        if (ioctl(fd, VIDIOC_QUERYCAP, &cap) == 0) {
          uint32_t caps = cap.device_caps ? cap.device_caps : cap.capabilities;
          info.has_capture = (caps & V4L2_CAP_VIDEO_CAPTURE) ||
                             (caps & V4L2_CAP_VIDEO_CAPTURE_MPLANE);
          if (info.has_capture) {
            info.bus_info = reinterpret_cast<const char *>(cap.bus_info);
            info.card = reinterpret_cast<const char *>(cap.card);
          }
        }
        close(fd);
      }
      if (!info.has_capture) {
        info.bus_info.clear();
        info.card.clear();
      }
      return info.has_capture;
    };

    std::vector<DiscoveredCamera> discovered;
    discovered.reserve(available.size());
    for (auto it = available.begin(); it != available.end();) {
      if (!ensure_capture(it->second)) {
        it = available.erase(it);
        continue;
      }
      DiscoveredCamera dc;
      dc.device_path = it->second.canonical;
      dc.bus_info = it->second.bus_info;
      dc.card = it->second.card;
      dc.identifiers = it->second.identifiers;
      if (dc.identifiers.empty())
        dc.identifiers.push_back({"device", it->second.canonical});
      discovered.push_back(std::move(dc));
      ++it;
    }

    std::vector<bool> matched_for_unconfigured(discovered.size(), false);
    for (auto &kv : configs_) {
      const auto &id = kv.first;
      CamConfig &cfg = kv.second;
      bool present = false;
      size_t matched_index = discovered.size();
      bool active;
      {
        std::lock_guard<std::mutex> lk(mutex_);
        active = active_.count(id) > 0;
      }
      for (size_t i = 0; i < discovered.size(); ++i) {
        const auto &dc = discovered[i];
        bool matched = false;
        if (!cfg.match_substr.empty()) {
          for (const auto &ident : dc.identifiers) {
            if (ident.type == "by-id" &&
                ident.value.find(cfg.match_substr) != std::string::npos) {
              matched = true;
              break;
            }
          }
        }
        if (!matched && !cfg.match_path_substr.empty()) {
          for (const auto &ident : dc.identifiers) {
            if (ident.type == "by-path" &&
                ident.value.find(cfg.match_path_substr) != std::string::npos) {
              matched = true;
              break;
            }
          }
        }
        if (!matched && !cfg.device_path.empty()) {
          if (dc.device_path == cfg.device_path)
            matched = true;
        }
        if (matched) {
          present = true;
          matched_index = i;
          break;
        }
      }

      {
        std::lock_guard<std::mutex> lk(mutex_);
        if (present && matched_index < discovered.size()) {
          const auto &dc = discovered[matched_index];
          active_paths_[id] = dc.device_path;
          cfg.device_path = dc.device_path;
          if (!active) {
            std::cout << "Camera " << id << " connected" << std::endl;
            active_.insert(id);
            if (cfg.det_port == 0)
              std::cerr << "CameraManager: detection disabled for camera "
                        << id << std::endl;
          }
          matched_for_unconfigured[matched_index] = true;
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


      pid_t pid = 0;
      {
        std::lock_guard<std::mutex> lk(mutex_);
        auto itp = det_pids_.find(id);
        if (itp != det_pids_.end())
          pid = itp->second;
      }


     if (!present || cfg.det_port <= 0) {
         if (pid > 0) {
             kill(pid, SIGTERM);
             waitpid(pid, nullptr, 0);
             {
                 std::lock_guard<std::mutex> lk(mutex_);
                 det_pids_.erase(id);
             }
         }
         continue;
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
          args.push_back(cfg.mode == CamConfig::Mode::Calibration || cfg.mode == CamConfig::Mode::Preview
                             ? "yolov8_web_server_calibration"
                             : "yolov8_web_server");
          args.push_back(cfg.model_path);
          args.push_back("--dev");
          args.push_back(cfg.device_path);
          args.push_back("--port");
          args.push_back(port);
          args.push_back("--size");
          args.push_back(std::to_string(cfg.preferred.w) + "x" + std::to_string(cfg.preferred.h));
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
          // *** ДОБАВЬТЕ ЭТИ СТРОКИ ***
          // Для режима Preview отключаем детекцию
          if (cfg.mode == CamConfig::Mode::Preview) {
          args.push_back("--no-draw");
          }
          if (cfg.mode == CamConfig::Mode::Calibration) {
          args.push_back("--no-draw");
          }
          // *** КОНЕЦ ДОБАВЛЕНИЯ ***
          for (const auto &a : cfg.det_args)
            args.push_back(a);
          if (!config_path_.empty()) {
            args.push_back("--config");
            args.push_back(config_path_);
          }
          std::vector<char *> argv;
          for (auto &a : args)
            argv.push_back(const_cast<char *>(a.c_str()));
          argv.push_back(nullptr);
          execv((cfg.mode == CamConfig::Mode::Calibration || cfg.mode == CamConfig::Mode::Preview)
                    ? "./yolov8_web_server_calibration"
                    : "./yolov8_web_server",
                argv.data());
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

    std::vector<DiscoveredCamera> new_unconfigured;
    for (size_t i = 0; i < discovered.size(); ++i) {
      if (!matched_for_unconfigured[i])
        new_unconfigured.push_back(discovered[i]);
    }

    {
      std::lock_guard<std::mutex> lk(mutex_);
      unconfigured_ = std::move(new_unconfigured);
    }

    {
      std::unique_lock<std::mutex> lk(mutex_);
      cv_.wait_for(lk, 1s);
    }
  }


}


void CameraManager::configWatchLoop() {
  if (config_path_.empty())
    return;
  int fd = inotify_init1(IN_NONBLOCK);
  if (fd < 0) {
    std::cerr << "CameraManager: inotify_init failed: "
              << std::strerror(errno) << std::endl;
    return;
  }
  int wd = inotify_add_watch(fd, config_path_.c_str(), IN_MODIFY);
  if (wd < 0) {
    std::cerr << "CameraManager: inotify_add_watch failed for "
              << config_path_ << ": " << std::strerror(errno) << std::endl;
    close(fd);
    return;
  }
  std::vector<char> buf(sizeof(struct inotify_event) + 512);
  while (running_) {
    ssize_t len = read(fd, buf.data(), buf.size());
    if (len <= 0) {
      std::this_thread::sleep_for(std::chrono::milliseconds(200));
      continue;
    }
    for (char *ptr = buf.data(); ptr < buf.data() + len;
         ptr += sizeof(struct inotify_event) +
                ((struct inotify_event *)ptr)->len) {
      auto *ev = reinterpret_cast<struct inotify_event *>(ptr);
      if (ev->mask & IN_MODIFY) {
        if (loadConfig(config_path_)) {
          std::lock_guard<std::mutex> lk(mutex_);
          for (auto it = det_pids_.begin(); it != det_pids_.end();) {
            if (!configs_.count(it->first)) {
              kill(it->second, SIGTERM);
              waitpid(it->second, nullptr, 0);
              it = det_pids_.erase(it);
            } else {
              kill(it->second, SIGHUP);
              ++it;
            }
          }
        }
      }
    }
  }
  inotify_rm_watch(fd, wd);
  close(fd);
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
  if (!j.contains("scheme_type"))
    j["scheme_type"] = scheme_type_;
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


bool CameraManager::setPixelFormat(const std::string &id, PixelFormat fmt) {
  std::lock_guard<std::mutex> lk(mutex_);
  auto it = configs_.find(id);
  if (it == configs_.end())
    return false;
  it->second.pixel_format = fmt;
  return true;
}

CameraManager::PixelFormat CameraManager::getPixelFormat(const std::string &id) {
  std::lock_guard<std::mutex> lk(mutex_);
  auto it = configs_.find(id);
  if (it == configs_.end())
    return PixelFormat::RGB;
  return it->second.pixel_format;
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
    ci.mode = kv.second.mode;
    ci.preferred = kv.second.preferred;
    ci.npu_worker = kv.second.npu_worker;
    ci.auto_profiles = kv.second.auto_profiles;
    ci.profile = kv.second.profile;
    ci.det_port = kv.second.det_port;
    ci.det_running =
        kv.second.mode == CamConfig::Mode::Detect &&
        det_pids_.count(kv.first) > 0;
    ci.position = kv.second.position;
    ci.fps = kv.second.fps;
    ci.model_path = kv.second.model_path;
    ci.labels_path = kv.second.labels_path;
    ci.cap_fps = kv.second.cap_fps;
    ci.buffers = kv.second.buffers;
    ci.buffer_type = kv.second.buffer_type;
    ci.jpeg_quality = kv.second.jpeg_quality;
    ci.http_fps_limit = kv.second.http_fps_limit;
    ci.show_det_fps = kv.second.show_det_fps;
    ci.npu_core = kv.second.npu_core;
    ci.log_file = kv.second.log_file;
    ci.role = kv.second.role;
    out.push_back(ci);
  }
  return out;
}

std::vector<CameraManager::DiscoveredCamera>
CameraManager::unconfiguredCameras() {
  std::lock_guard<std::mutex> lk(mutex_);
  return unconfigured_;
}

bool CameraManager::addCamera(const std::string &id,
                              const std::string &match_value,
                              const std::string &match_type) {
  CamConfig cfg;
  cfg.id = id;
  if (match_type == "by-path")
    cfg.match_path_substr = match_value;
  else if (match_type == "device")
    cfg.device_path = match_value;
  else
    cfg.match_substr = match_value;
  cfg.mode = CamConfig::Mode::Preview;
  cfg.profile = "auto";
  cfg.det_port = 0;
  cfg.position = {};
  cfg.role = "wide_angle_primary";
  std::error_code ec;
  if (match_type != "device") {
    std::string base = match_type == "by-path" ? "/dev/v4l/by-path/"
                                                : "/dev/v4l/by-id/";
    auto dev = std::filesystem::canonical(base + match_value, ec);
    if (!ec)
      cfg.device_path = dev.string();
  }

  if (!cfg.device_path.empty()) {
    const std::filesystem::path canonical(cfg.device_path);
    auto populate_match = [&](const std::filesystem::path &dir,
                              const std::string &type) {
      std::error_code sec;
      if (!std::filesystem::exists(dir, sec) || sec)
        return;
      for (auto it = std::filesystem::directory_iterator(dir, sec);
           it != std::filesystem::directory_iterator(); ++it) {
        if (sec)
          break;
        if (!it->is_symlink(sec) || sec)
          continue;
        auto target = std::filesystem::canonical(it->path(), sec);
        if (sec)
          continue;
        if (target == canonical) {
          auto name = it->path().filename().string();
          if (type == "by-id" && cfg.match_substr.empty())
            cfg.match_substr = name;
          if (type == "by-path" && cfg.match_path_substr.empty())
            cfg.match_path_substr = name;
        }
      }
    };
    populate_match("/dev/v4l/by-id", "by-id");
    populate_match("/dev/v4l/by-path", "by-path");
  }

  // Detect whether the device prefers single or multi-plane buffers.
  cfg.buffer_type = "auto";
  if (!cfg.device_path.empty()) {
    int fd = open(cfg.device_path.c_str(), O_RDWR);
    if (fd >= 0) {
      v4l2_capability cap{};
      if (ioctl(fd, VIDIOC_QUERYCAP, &cap) == 0) {
        uint32_t caps = cap.device_caps ? cap.device_caps : cap.capabilities;
        bool determined = false;
        v4l2_format fmt{};
        if (caps & V4L2_CAP_VIDEO_CAPTURE_MPLANE) {
          std::memset(&fmt, 0, sizeof(fmt));
          fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
          fmt.fmt.pix_mp.width = 640;
          fmt.fmt.pix_mp.height = 480;
          fmt.fmt.pix_mp.pixelformat =
              cfg.pixel_format == PixelFormat::GRAY ? V4L2_PIX_FMT_GREY
                                                    : V4L2_PIX_FMT_RGB24;
          fmt.fmt.pix_mp.field = V4L2_FIELD_NONE;
          fmt.fmt.pix_mp.num_planes = 1;
          if (ioctl(fd, VIDIOC_TRY_FMT, &fmt) == 0) {
            cfg.buffer_type = "mplane";
            determined = true;
          }
        }
        if (!determined && (caps & V4L2_CAP_VIDEO_CAPTURE)) {
          std::memset(&fmt, 0, sizeof(fmt));
          fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
          fmt.fmt.pix.width = 640;
          fmt.fmt.pix.height = 480;
          fmt.fmt.pix.pixelformat =
              cfg.pixel_format == PixelFormat::GRAY ? V4L2_PIX_FMT_GREY
                                                    : V4L2_PIX_FMT_RGB24;
          fmt.fmt.pix.field = V4L2_FIELD_NONE;
          if (ioctl(fd, VIDIOC_TRY_FMT, &fmt) == 0) {
            cfg.buffer_type = "single";
            determined = true;
          }
        }
      }
      close(fd);
    }
  }
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
  cfg.def_buffer_type = cfg.buffer_type;
  cfg.def_jpeg_quality = cfg.jpeg_quality;
  cfg.def_http_fps_limit = cfg.http_fps_limit;
  cfg.def_show_det_fps = cfg.show_det_fps;
  cfg.def_npu_core = cfg.npu_core;
  cfg.def_log_file = cfg.log_file;
  cfg.def_role = cfg.role;

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
  if (!j.contains("scheme_type"))
    j["scheme_type"] = scheme_type_;
  json cam;
  cam["id"] = id;
  json match = json::object();
  if (!cfg.match_substr.empty())
    match["by_id_contains"] = cfg.match_substr;
  if (!cfg.match_path_substr.empty())
    match["by_path_contains"] = cfg.match_path_substr;
  if (!cfg.device_path.empty() && match.empty())
    match["device_path"] = cfg.device_path;
  cam["match"] = match;
  if (!cfg.device_path.empty())
    cam["device"] = cfg.device_path;
  cam["mode"] = "preview";
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
  cam["buffer_type"] = cfg.buffer_type;
  cam["jpeg_quality"] = cfg.jpeg_quality;
  cam["http_fps_limit"] = cfg.http_fps_limit;
  cam["show_fps"] = cfg.show_det_fps;
  cam["npu_core"] = cfg.npu_core;
  cam["log_file"] = cfg.log_file;
  cam["role"] = cfg.role;
  j["cameras"].push_back(cam);
  std::ofstream out(config_path_, std::ios::trunc);
  if (!out.is_open())
    return false;
  out << j.dump(2);
  return true;
}

bool CameraManager::setMode(const std::string &id, CamConfig::Mode mode) {
  bool changed = false;
  {
std::lock_guard<std::mutex> lk(mutex_);
    auto it = configs_.find(id);
    if (it == configs_.end())
      return false;
    if (it->second.mode == mode)
      return true;
    it->second.mode = mode;
    changed = true;
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
    if (!j.contains("scheme_type"))
      j["scheme_type"] = scheme_type_;
    for (auto &c : j["cameras"]) {
      if (c.value("id", "") == id) {
        std::string m = "preview";
        if (mode == CamConfig::Mode::Detect)
          m = "detect";
        else if (mode == CamConfig::Mode::Calibration)
          m = "calibration";
        c["mode"] = m;
        if (it->second.device_path.size())
          c["device"] = it->second.device_path;
      }
    }
    std::ofstream out(config_path_, std::ios::trunc);
    if (!out.is_open())
      return false;
    out << j.dump(2);
  }
  if (changed)
    cv_.notify_all();
  return true;
}

bool CameraManager::setRole(const std::string &id, const std::string &role) {
  std::lock_guard<std::mutex> lk(mutex_);
  auto it = configs_.find(id);
  if (it == configs_.end())
    return false;
  it->second.role = role;
  return true;
}


bool CameraManager::updateSettings(const std::string &id,
                                   const CamConfig::VideoMode &pref,
                                   int npu_worker, bool auto_profiles,
                                   const std::string &profile,
                                   const std::string &model_path,
                                   const std::string &labels_path,
                                   int cap_fps, int buffers,
                                   const std::string &buffer_type,
                                   int jpeg_quality,
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
  if (!buffer_type.empty()) {
    if (buffer_type != "auto" && buffer_type != "single" &&
        buffer_type != "mplane")
      return false;
    it->second.buffer_type = buffer_type;
  }
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
  if (!j.contains("scheme_type"))
    j["scheme_type"] = scheme_type_;
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
      c["buffer_type"] = it->second.buffer_type;
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
  // Restore default buffer type when resetting settings.
  cfg.buffer_type = cfg.def_buffer_type;
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
  if (!j.contains("scheme_type"))
    j["scheme_type"] = scheme_type_;
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
      c["buffer_type"] = cfg.buffer_type;
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
  auto now = Clock::now();
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

void CameraManager::pushFrame(const std::string &id, int w, int h,
                              const std::vector<uint8_t> &jpeg,
                              uint64_t t_mono_ns) {
  std::lock_guard<std::mutex> lk(mutex_);
  auto &buf = frame_buffers_[id];
  buf.push_back(Frame{t_mono_ns, w, h, jpeg});
  // drop old frames
  while (!buf.empty()) {
    if (buf.back().t_mono_ns - buf.front().t_mono_ns >
        std::chrono::duration_cast<std::chrono::nanoseconds>(buffer_keep_).count())
      buf.pop_front();
    else
      break;
  }
}

bool CameraManager::getFrame(const std::string &id, uint64_t t_mono_ns,
                             Frame &out) {
  std::lock_guard<std::mutex> lk(mutex_);
  auto it = frame_buffers_.find(id);
  if (it == frame_buffers_.end() || it->second.empty())
    return false;
  auto &buf = it->second;
  auto best = buf.begin();
  uint64_t best_diff = std::numeric_limits<uint64_t>::max();
  for (auto itf = buf.begin(); itf != buf.end(); ++itf) {
    uint64_t diff = (itf->t_mono_ns > t_mono_ns)
                        ? itf->t_mono_ns - t_mono_ns
                        : t_mono_ns - itf->t_mono_ns;
    if (diff < best_diff) {
      best_diff = diff;
      best = itf;
    }
  }
  out = *best;
  return true;
}

bool CameraManager::saveFrameWithMeta(const std::string &path, const Frame &f,
                                      const std::string &cam_id) {
  try {
    std::filesystem::path p(path);
    std::filesystem::create_directories(p.parent_path());
    std::ofstream jf(path, std::ios::binary);
    if (!jf.is_open())
      return false;
    jf.write(reinterpret_cast<const char *>(f.jpeg.data()), f.jpeg.size());
    jf.close();
    json meta;
    meta["t_mono_ns"] = f.t_mono_ns;
    meta["w"] = f.w;
    meta["h"] = f.h;
    meta["cam_id"] = cam_id;
    std::ofstream mf(path + ".json");
    if (!mf.is_open())
      return false;
    mf << meta.dump(2);
    return true;
  } catch (...) {
    return false;
  }
}

uint64_t CameraManager::nowMonoNs() const {
  return std::chrono::duration_cast<std::chrono::nanoseconds>(Clock::now() -
                                                             start_time_)
      .count();
}

std::vector<CameraManager::StereoPair> CameraManager::getActivePairs() {
  std::lock_guard<std::mutex> lk(mutex_);
  return active_pairs_;
}

void CameraManager::setActivePairs(const std::vector<CameraManager::StereoPair> &pairs) {
  std::lock_guard<std::mutex> lk(mutex_);
  active_pairs_ = pairs;
}



bool CameraManager::saveConfig(const std::string &scheme_type,
                               const std::map<std::string, std::string> &roles) {
  std::lock_guard<std::mutex> lk(mutex_);
  if (config_path_.empty())
    return false;

  json j;
  {
    std::ifstream f(config_path_);
    if (f.is_open()) {
      try {
        f >> j;
      } catch (...) {
        j = json::object();
      }
    }
  }
  if (!j.is_object())
    j = json::object();

  j["scheme_type"] = scheme_type;

  if (j.contains("cameras") && j["cameras"].is_array()) {
    for (auto &cam : j["cameras"]) {
      std::string id = cam.value("id", "");
      auto it = roles.find(id);
      if (it != roles.end())
        cam["role"] = it->second;
    }
  }

  std::ofstream out(config_path_, std::ios::trunc);
  if (!out.is_open())
    return false;
  out << j.dump(2);
  return true;
}
