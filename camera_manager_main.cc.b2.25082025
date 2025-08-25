#include "camera_manager.h"
#include <csignal>
#include <fcntl.h>
#include <fstream>
#include <iostream>
#include <linux/videodev2.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <unistd.h>
#include <vector>
#include <cerrno>
#include <cstring>
#include <filesystem>
#include <atomic>
#include <sys/stat.h>

#include "httplib.h"
#include "nlohmann/json.hpp"
#include <opencv2/opencv.hpp>

// Описание стереопары синхронизировано с camera_manager.cpp. Для доступа к
// активным парам используется внешний вектор g_active_pairs.
struct StereoPair {
  std::string cam0;
  std::string cam1;
  cv::Mat Q;
  cv::Ptr<cv::StereoSGBM> matcher;
};

extern std::vector<StereoPair> g_active_pairs;


static CameraManager g_mgr;
static httplib::Server g_server;
static bool g_preview_enabled = true;


static bool fileExists(const std::string &p) {
  struct stat st {
  };
  return stat(p.c_str(), &st) == 0;
}

static bool dirExists(const std::string &p) {
  struct stat st {
  };
  return stat(p.c_str(), &st) == 0 && S_ISDIR(st.st_mode);
}

static nlohmann::json readMainConfig() {
  nlohmann::json cfg = nlohmann::json::object();
  auto p = std::filesystem::absolute("config/config.json");
  printf("readMainConfig path: %s\n", p.c_str());
  std::ifstream f(p);
  if (f) {
    try {
      f >> cfg;
    } catch (...) {
    }
  }
  return cfg;
}

static bool writeMainConfig(const nlohmann::json &j) {
  auto dir = std::filesystem::absolute("config");
  auto file = dir / "config.json";
  printf("writeMainConfig path: %s\n", file.c_str());
  if (mkdir(dir.c_str(), 0755) != 0 && errno != EEXIST)
    return false;
  std::ofstream f(file);
  if (!f)
    return false;
  f << j.dump(2);
  return f.good();
}


static void sigint(int) {
  g_mgr.stop();
  g_server.stop();
}

static bool capture_jpeg(const std::string &dev,
                         std::vector<unsigned char> &out) {
  int fd = open(dev.c_str(), O_RDWR);
  if (fd < 0) {
    std::cerr << "Failed to open " << dev << ": "
              << std::strerror(errno) << std::endl;
    return false;
  }
  v4l2_format fmt{};
  fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  fmt.fmt.pix.width = 320;
  fmt.fmt.pix.height = 240;
  fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_MJPEG;
  fmt.fmt.pix.field = V4L2_FIELD_NONE;
  if (ioctl(fd, VIDIOC_S_FMT, &fmt) < 0) {

    int err = errno;
    std::cerr << "VIDIOC_S_FMT failed for " << dev << ": "
              << std::strerror(err) << std::endl;
    close(fd);
    return false;
  }
  v4l2_requestbuffers req{};
  req.count = 1;
  req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  req.memory = V4L2_MEMORY_MMAP;
  if (ioctl(fd, VIDIOC_REQBUFS, &req) < 0) {
    int err = errno;
    std::cerr << "VIDIOC_REQBUFS failed for " << dev << ": "
              << std::strerror(err) << std::endl;
    close(fd);
    return false;
  }
  v4l2_buffer buf{};
  buf.type = req.type;
  buf.memory = V4L2_MEMORY_MMAP;
  buf.index = 0;
  if (ioctl(fd, VIDIOC_QUERYBUF, &buf) < 0) {
    int err = errno;
    std::cerr << "VIDIOC_QUERYBUF failed for " << dev << ": "
              << std::strerror(err) << std::endl;
    close(fd);
    return false;
  }
  void *mem = mmap(NULL, buf.length, PROT_READ | PROT_WRITE, MAP_SHARED, fd,
                   buf.m.offset);
  if (mem == MAP_FAILED) {
 int err = errno;
    std::cerr << "mmap failed for " << dev << ": " << std::strerror(err)
              << std::endl;
    close(fd);
    return false;
  }
  if (ioctl(fd, VIDIOC_QBUF, &buf) < 0) {
 int err = errno;
    std::cerr << "VIDIOC_QBUF failed for " << dev << ": "
              << std::strerror(err) << std::endl;
    munmap(mem, buf.length);
    close(fd);
    return false;
  }
  v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  if (ioctl(fd, VIDIOC_STREAMON, &type) < 0) {
    int err = errno;
    std::cerr << "VIDIOC_STREAMON failed for " << dev << ": "
              << std::strerror(err) << std::endl;
    munmap(mem, buf.length);
    close(fd);
    return false;
  }
  if (ioctl(fd, VIDIOC_DQBUF, &buf) < 0) {
    int err = errno;
    std::cerr << "VIDIOC_DQBUF failed for " << dev << ": "
              << std::strerror(err) << std::endl;
    ioctl(fd, VIDIOC_STREAMOFF, &type);
    munmap(mem, buf.length);
    close(fd);
    return false;
  }
  out.assign(static_cast<unsigned char *>(mem),
             static_cast<unsigned char *>(mem) + buf.bytesused);
  ioctl(fd, VIDIOC_STREAMOFF, &type);
  munmap(mem, buf.length);
  close(fd);
  return true;
}


// Захват кадра в формате cv::Mat. Используется существующая функция
// capture_jpeg, после чего изображение декодируется в градации серого.
static cv::Mat capture_mat(const std::string &dev) {
  std::vector<unsigned char> buf;
  if (!capture_jpeg(dev, buf))
    return cv::Mat();
  return cv::imdecode(buf, cv::IMREAD_GRAYSCALE);
}

// Флаг работы стерео-потока.
static std::atomic<bool> g_stereo_running{true};

// Основной цикл обработки стереопар. Для каждой активной пары вычисляется
// карта диспаритета, точки переводятся в систему cam0, после чего карты глубин
// объединяются. Также выполняется простой KLT-трекер для оценки движения
// между кадрами.
static void stereo_loop() {
  cv::Mat prev_gray;
  std::vector<cv::Point2f> prev_pts;
  while (g_stereo_running) {
    cv::Mat merged;
    for (auto &pair : g_active_pairs) {
      std::string dev0 = g_mgr.devicePath(pair.cam0);
      std::string dev1 = g_mgr.devicePath(pair.cam1);
      if (dev0.empty() || dev1.empty())
        continue;
      cv::Mat left = capture_mat(dev0);
      cv::Mat right = capture_mat(dev1);
      if (left.empty() || right.empty())
        continue;
      cv::Mat disp;
      pair.matcher->compute(left, right, disp);
      cv::Mat pts3d;
      cv::reprojectImageTo3D(disp, pts3d, pair.Q);
      cv::Mat zmap;
      cv::extractChannel(pts3d, zmap, 2);
      if (merged.empty())
        merged = zmap;
      else
        cv::min(merged, zmap, merged);
    }

    if (!merged.empty()) {
      cv::Mat gray;
      merged.convertTo(gray, CV_8U, 255.0 / 10.0);
      if (prev_pts.empty()) {
        cv::goodFeaturesToTrack(gray, prev_pts, 200, 0.01, 3);
      } else {
        std::vector<cv::Point2f> next_pts;
        std::vector<uchar> status;
        std::vector<float> err;
        cv::calcOpticalFlowPyrLK(prev_gray, gray, prev_pts, next_pts, status,
                                 err);
        prev_pts.clear();
        for (size_t i = 0; i < status.size(); ++i) {
          if (status[i])
            prev_pts.push_back(next_pts[i]);
        }
      }
      prev_gray = gray;
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(30));
  }
}



int main(int argc, char **argv) {
  std::string cfg = "config.json";
  if (argc > 1)
    cfg = argv[1];
  if (!g_mgr.loadConfig(cfg))
    return 1;
  std::ifstream jf(cfg);
  nlohmann::json j;
  jf >> j;
  g_preview_enabled = j.value("preview_enabled", true);
  int port = j.value("http", nlohmann::json::object()).value("port", 8080);

  std::signal(SIGINT, sigint);
  g_mgr.start();


  // Отдельный поток обработки стереопар.
  std::thread stereo_thread(stereo_loop);


  g_server.set_mount_point("/", "./web");
  g_server.Get("/api/config", [](const httplib::Request &, httplib::Response &res) {
    nlohmann::json out{{"preview_enabled", g_preview_enabled}};
    res.set_content(out.dump(), "application/json");
  });


  g_server.Get(
      "/api/configured", [](const httplib::Request &, httplib::Response &res) {
        auto cams = g_mgr.configuredCameras();
        nlohmann::json out = nlohmann::json::array();
        for (auto &c : cams)
          out.push_back({{"id", c.id},
                         {"present", c.present},
                         {"preview", c.preview},
                         {"preferred",
                         {{"w", c.preferred.w},
                          {"h", c.preferred.h},
                          {"pixfmt", c.preferred.pixfmt},
                          {"fps", c.preferred.fps}}},
                         {"npu_worker", c.npu_worker},
                         {"auto_profiles", c.auto_profiles},
                         {"profile", c.profile},
                         {"det_port", c.det_port},
                         {"det_running", c.det_running},
                         {"position", {{"x", c.position.x}, {"y", c.position.y}, {"z", c.position.z}}},
                         {"fps", c.fps},
                         {"model_path", c.model_path},
                         {"labels_path", c.labels_path},
                         {"cap_fps", c.cap_fps},
                         {"buffers", c.buffers},
                         {"jpeg_quality", c.jpeg_quality},
                         {"http_fps_limit", c.http_fps_limit},
                         {"show_fps", c.show_det_fps},
                         {"npu_core", c.npu_core},
                         {"log_file", c.log_file}});
        res.set_content(out.dump(), "application/json");
      });

  g_server.Get("/api/models",
               [](const httplib::Request &, httplib::Response &res) {
                 nlohmann::json out;
                 out["rknn"] = nlohmann::json::array();
                 out["labels"] = nlohmann::json::array();
                 namespace fs = std::filesystem;
                 try {
                   for (auto &p : fs::directory_iterator("model_rknn"))
                     if (p.is_regular_file())
                       out["rknn"].push_back(std::string("model_rknn/") +
                                             p.path().filename().string());
                 } catch (...) {
                 }
                 try {
                   for (auto &p : fs::directory_iterator("model"))
                     if (p.is_regular_file())
                       out["labels"].push_back(std::string("model/") +
                                               p.path().filename().string());
                 } catch (...) {
                 }
                 res.set_content(out.dump(), "application/json");
               });

  g_server.Get("/api/new",
               [](const httplib::Request &, httplib::Response &res) {
                 nlohmann::json out = g_mgr.unconfiguredCameras();
                 res.set_content(out.dump(), "application/json");
               });

  g_server.Post("/api/add",
                [](const httplib::Request &req, httplib::Response &res) {
                  try {
                    auto j = nlohmann::json::parse(req.body);
                    std::string id = j.at("id").get<std::string>();
                    std::string by = j.at("by_id").get<std::string>();
                    if (!g_mgr.addCamera(id, by))
                      res.status = 400;
                  } catch (...) {
                    res.status = 400;
                  }
                });

  g_server.Post("/api/delete",
                [](const httplib::Request &req, httplib::Response &res) {
                  try {
                    auto j = nlohmann::json::parse(req.body);
                    std::string id = j.at("id").get<std::string>();
                    if (!g_mgr.removeCamera(id))
                      res.status = 400;
                  } catch (...) {
                    res.status = 400;
                  }
                });

  g_server.Post("/api/calib/setup",
                [](const httplib::Request &req, httplib::Response &res) {
                  try {
                    auto j = nlohmann::json::parse(req.body);
                    std::string cam = j.value("camera", "");
                    if (cam.empty()) {
                      res.status = 400;
                      res.set_content("{\"error\":\"missing camera\"}",
                                      "application/json");
                      return;
                    }
                    auto cfg = readMainConfig();
                    cfg["calib_camera"] = cam;
                    if (!writeMainConfig(cfg)) {
                      res.status = 500;
                      res.set_content("{\"error\":\"write failure\"}",
                                      "application/json");
                      return;
                    }
                    std::string dir = "CalibCam" + cam;
                    auto absDir = std::filesystem::absolute(dir);
                    printf("calibration dir: %s\n", absDir.c_str());
                    if (mkdir(absDir.c_str(), 0755) != 0 && errno != EEXIST) {
                      res.status = 500;
                      res.set_content("{\"error\":\"mkdir failure\"}",
                                      "application/json");
                      return;
                    }
                    res.set_content("{\"status\":\"ok\"}",
                                    "application/json");
                  } catch (...) {
                    res.status = 400;
                    res.set_content("{\"error\":\"invalid json\"}",
                                    "application/json");
                  }
                });

  g_server.Get("/api/calib/status",
               [](const httplib::Request &req, httplib::Response &res) {
                 std::string cam;
                 if (req.has_param("camera")) {
                   cam = req.get_param_value("camera");
                 } else {
                   auto cfg = readMainConfig();
                   cam = cfg.value("calib_camera", "");
                 }
                 nlohmann::json resp;
                 resp["camera"] = cam;
                 std::string dir = cam.empty() ? std::string() : "CalibCam" + cam;
                 bool folder = !cam.empty() && dirExists(dir);
                 bool mono_done =
                     !cam.empty() && fileExists("out/cam" + cam + ".yml");
                 bool stereo_ready = mono_done && fileExists("out/cam0.yml") &&
                                     !fileExists("out/stereo_0" + cam + ".yml");
                 resp["folder_exists"] = folder;
                 resp["mono_done"] = mono_done;
                 resp["stereo_ready"] = stereo_ready;
                 res.set_content(resp.dump(), "application/json");
               });


  g_server.Post("/api/preview",
                [](const httplib::Request &req, httplib::Response &res) {
                    if (!g_preview_enabled) {
                    res.status = 403;
                    return;
                  }
                  try {
                    auto j = nlohmann::json::parse(req.body);
                    std::string id = j.at("id").get<std::string>();
                    bool enable = j.at("enable").get<bool>();
                    if (!g_mgr.setPreview(id, enable))
                      res.status = 400;
                    else
                      g_mgr.notify();
                  } catch (...) {
                    res.status = 400;
                  }
                });

  g_server.Post("/api/settings",
                [](const httplib::Request &req, httplib::Response &res) {
                  try {
                    auto j = nlohmann::json::parse(req.body);
                    std::string id = j.at("id").get<std::string>();
                    auto pref = j.at("preferred");
                    CameraManager::CamConfig::VideoMode vm;
                    vm.w = pref.value("w", 1280);
                    vm.h = pref.value("h", 720);
                    vm.pixfmt = pref.value("pixfmt", std::string("MJPG"));
                    vm.fps = pref.value("fps", 30);
                    int worker = j.value("npu_worker", 0);
                    bool auto_profiles = j.value("auto_profiles", true);
                    std::string profile =
                        j.value("profile", std::string("auto"));
                    std::string model_path =
                        j.value("model_path", std::string(""));
                    std::string labels_path =
                        j.value("labels_path", std::string(""));
                    int cap_fps = j.value("cap_fps", 30);
                    int buffers = j.value("buffers", 3);
                    int jpeg_quality = j.value("jpeg_quality", 60);
                    int http_fps_limit = j.value("http_fps_limit", 20);
                    bool show_fps = j.value("show_fps", false);
                    std::string npu_core =
                        j.value("npu_core", std::string("auto"));
                    std::string log_file =
                        j.value("log_file", std::string(""));
                    if (!g_mgr.updateSettings(id, vm, worker, auto_profiles,
                                              profile, model_path, labels_path,
                                              cap_fps, buffers, jpeg_quality,
                                              http_fps_limit, show_fps,
                                              npu_core, log_file))
                      res.status = 400;
                  } catch (...) {
                    res.status = 400;
                  }
                });

 g_server.Post("/api/settings/reset",
                [](const httplib::Request &req, httplib::Response &res) {
                  try {
                    auto j = nlohmann::json::parse(req.body);
                    std::string id = j.at("id").get<std::string>();
                    if (!g_mgr.resetSettings(id))
                      res.status = 400;
                  } catch (...) {
                    res.status = 400;
                  }
                });
 
 g_server.Get(
      "/api/preview", [](const httplib::Request &req, httplib::Response &res) {
	 if (!g_preview_enabled) {
          res.status = 403;
          return;
        }
        std::string dev;
        if (req.has_param("id"))
          dev = g_mgr.devicePath(req.get_param_value("id"));
        else if (req.has_param("by"))
          dev = std::string("/dev/v4l/by-id/") + req.get_param_value("by");
        std::vector<unsigned char> jpg;
	 if (dev.empty()) {
          res.status = 404;
          return;
        }
        if (!capture_jpeg(dev, jpg)) {
          std::cerr << "capture_jpeg failed for device '" << dev << "'" << std::endl;
          res.status = 404;
          return;
        }
        if (req.has_param("id"))
          g_mgr.reportFrame(req.get_param_value("id"));
        res.set_content(reinterpret_cast<const char *>(jpg.data()), jpg.size(),
                        "image/jpeg");
      });

  std::thread http_thr([&] { g_server.listen("0.0.0.0", port); });
  std::cout << "CameraManager running. Press Ctrl+C to exit." << std::endl;
  while (g_server.is_running()) {
    std::this_thread::sleep_for(std::chrono::seconds(1));
  }
  if (http_thr.joinable())
    http_thr.join();
  g_stereo_running = false;
  if (stereo_thread.joinable())
    stereo_thread.join();
  return 0;
}
