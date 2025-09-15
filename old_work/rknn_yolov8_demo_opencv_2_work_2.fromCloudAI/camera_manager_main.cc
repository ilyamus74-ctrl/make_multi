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
#include <thread>
#include <chrono>
#include <cstdio>
#include <unordered_map>
#include <mutex>

#include "httplib.h"
#include "nlohmann/json.hpp"
#include <opencv2/opencv.hpp>
#include <memory>
#include "calibration/session.h"



static CameraManager g_mgr;
static httplib::Server g_server;
static bool g_preview_enabled = true;
static std::unique_ptr<CalibrationSession> g_calib;
static std::filesystem::path g_config_path;
static std::filesystem::path g_exe_dir;
static nlohmann::json readMainConfig();


// Cached v4l2 formats per device. Avoids reconfiguring a device that already
// runs with the requested format.
static std::unordered_map<std::string, v4l2_format> g_format_cache;
static std::mutex g_format_cache_mutex;

// Backup of global preview flag and per-camera preview states while
// calibration is in progress. This lets us restore the exact state when the
// user finishes calibration.
static bool g_prev_preview_enabled = true;
struct CamPreviewState {
  std::string id;
  bool preview;
};
static std::vector<CamPreviewState> g_prev_cam_states;

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
  auto p = std::filesystem::absolute(g_config_path);
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
  auto file = std::filesystem::absolute(g_config_path);
  auto dir = file.parent_path();
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

static bool formatsEqual(const v4l2_format &a, const v4l2_format &b) {
  if (a.type != b.type)
    return false;
  if (a.type == V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE) {
    return a.fmt.pix_mp.width == b.fmt.pix_mp.width &&
           a.fmt.pix_mp.height == b.fmt.pix_mp.height &&
           a.fmt.pix_mp.pixelformat == b.fmt.pix_mp.pixelformat;
  }
  return a.fmt.pix.width == b.fmt.pix.width &&
         a.fmt.pix.height == b.fmt.pix.height &&
         a.fmt.pix.pixelformat == b.fmt.pix.pixelformat;
}

// Populate cached formats for all currently active devices.
static void populate_format_cache() {
  std::lock_guard<std::mutex> lk(g_format_cache_mutex);
  g_format_cache.clear();
  auto cams = g_mgr.configuredCameras();
  for (auto &c : cams) {
    std::string dev = g_mgr.devicePath(c.id);
    if (dev.empty())
      continue;
    int fd = open(dev.c_str(), O_RDWR);
    if (fd < 0)
      continue;
    v4l2_format f{};
    v4l2_buf_type types[] = {V4L2_BUF_TYPE_VIDEO_CAPTURE,
                             V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE};
    for (auto t : types) {
      std::memset(&f, 0, sizeof(f));
      f.type = t;
      if (ioctl(fd, VIDIOC_G_FMT, &f) == 0) {
        g_format_cache[dev] = f;
        break;
      }
    }
    close(fd);
  }
}


static bool capture_jpeg(const std::string &dev,
                         std::vector<unsigned char> &out,
                         const std::string &cam_id = std::string(),
                         v4l2_buf_type req_type = static_cast<v4l2_buf_type>(0)) {

  // If cam_id is provided, the camera is already streaming and frames are
  // available via the CameraManager buffer. Avoid touching the device again
  // and fetch the latest frame directly from the running stream.
  if (!cam_id.empty()) {
    CameraManager::Frame fr;
    if (!g_mgr.getFrame(cam_id, g_mgr.nowMonoNs(), fr))
      return false;
    out = fr.jpeg;
    return true;
  }

  int fd = open(dev.c_str(), O_RDWR);
  if (fd < 0) {
    std::cerr << "Failed to open " << dev << ": "
              << std::strerror(errno) << std::endl;
    return false;
  }

 v4l2_buf_type type = req_type;
  if (type != V4L2_BUF_TYPE_VIDEO_CAPTURE &&
      type != V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE) {
    v4l2_capability cap{};
    if (ioctl(fd, VIDIOC_QUERYCAP, &cap) == 0) {
      if (cap.device_caps & V4L2_CAP_VIDEO_CAPTURE_MPLANE)
        type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
      else
        type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    }
  }

  v4l2_format fmt{};
  bool fmt_ok = false;
  std::vector<v4l2_buf_type> try_types;
  if (type == V4L2_BUF_TYPE_VIDEO_CAPTURE ||
      type == V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE)
    try_types.push_back(type);
  else {
    try_types.push_back(V4L2_BUF_TYPE_VIDEO_CAPTURE);
    try_types.push_back(V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE);
  }


  for (auto t : try_types) {
    v4l2_format desired{};
    std::memset(&desired, 0, sizeof(desired));
    desired.type = t;
    if (t == V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE) {
      desired.fmt.pix_mp.width = 320;
      desired.fmt.pix_mp.height = 240;
      desired.fmt.pix_mp.pixelformat = V4L2_PIX_FMT_MJPEG;
      desired.fmt.pix_mp.field = V4L2_FIELD_NONE;
      desired.fmt.pix_mp.num_planes = 1;
    } else {
      desired.fmt.pix.width = 320;
      desired.fmt.pix.height = 240;
      desired.fmt.pix.pixelformat = V4L2_PIX_FMT_MJPEG;
      desired.fmt.pix.field = V4L2_FIELD_NONE;
    }

    v4l2_format cur{};
    std::memset(&cur, 0, sizeof(cur));
    cur.type = t;
    bool have_cur = (ioctl(fd, VIDIOC_G_FMT, &cur) == 0);
    if (have_cur) {
      {
        std::lock_guard<std::mutex> lk(g_format_cache_mutex);
        g_format_cache[dev] = cur;
      }
      if (formatsEqual(cur, desired)) {
        fmt = cur;
        type = t;
        fmt_ok = true;
        break;
      }
    }

    if (ioctl(fd, VIDIOC_S_FMT, &desired) == 0) {
      fmt = desired;
      type = t;
      fmt_ok = true;
      {
        std::lock_guard<std::mutex> lk(g_format_cache_mutex);
        g_format_cache[dev] = fmt;
      }
      break;
    }
  }
  if (!fmt_ok) {
    int err = errno;
    std::cerr << "VIDIOC_S_FMT failed for " << dev << ": "
              << std::strerror(err) << std::endl;
    close(fd);
    return false;
  }

  v4l2_requestbuffers req{};
  req.count = 1;
  req.type = type;
  req.memory = V4L2_MEMORY_MMAP;
  if (ioctl(fd, VIDIOC_REQBUFS, &req) < 0) {
    int err = errno;
    std::cerr << "VIDIOC_REQBUFS failed for " << dev << ": "
              << std::strerror(err) << std::endl;
    close(fd);
    return false;
  }
  v4l2_buffer buf{};
  buf.type = type;
  buf.memory = V4L2_MEMORY_MMAP;
  buf.index = 0;
  v4l2_plane planes[VIDEO_MAX_PLANES];
  if (type == V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE) {
    std::memset(planes, 0, sizeof(planes));
    buf.length = 1;
    buf.m.planes = planes;
  }
  if (ioctl(fd, VIDIOC_QUERYBUF, &buf) < 0) {
    int err = errno;
    std::cerr << "VIDIOC_QUERYBUF failed for " << dev << ": "
              << std::strerror(err) << std::endl;
    close(fd);
    return false;
  }

  void *mem = nullptr;
  size_t map_len = 0;
  if (type == V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE) {
    mem = mmap(NULL, buf.m.planes[0].length, PROT_READ | PROT_WRITE,
               MAP_SHARED, fd, buf.m.planes[0].m.mem_offset);
    map_len = buf.m.planes[0].length;
  } else {
    mem = mmap(NULL, buf.length, PROT_READ | PROT_WRITE, MAP_SHARED, fd,
               buf.m.offset);
    map_len = buf.length;
  }
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
    munmap(mem, map_len);
    close(fd);
    return false;
  }

  if (ioctl(fd, VIDIOC_STREAMON, &type) < 0) {
    int err = errno;
    std::cerr << "VIDIOC_STREAMON failed for " << dev << ": "
              << std::strerror(err) << std::endl;
    munmap(mem, map_len);
    close(fd);
    return false;
  }
  if (ioctl(fd, VIDIOC_DQBUF, &buf) < 0) {
    int err = errno;
    std::cerr << "VIDIOC_DQBUF failed for " << dev << ": "
              << std::strerror(err) << std::endl;
    ioctl(fd, VIDIOC_STREAMOFF, &type);
    munmap(mem, map_len);
    close(fd);
    return false;
  }

  size_t used = (type == V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE)
                    ? buf.m.planes[0].bytesused
                    : buf.bytesused;

  out.assign(static_cast<unsigned char *>(mem),
             static_cast<unsigned char *>(mem) + used);
  ioctl(fd, VIDIOC_STREAMOFF, &type);
  munmap(mem, map_len);
  close(fd);
  if (!cam_id.empty()) {
    uint64_t t_ns = g_mgr.nowMonoNs();
    int w = (type == V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE)
                ? fmt.fmt.pix_mp.width
                : fmt.fmt.pix.width;
    int h = (type == V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE)
                ? fmt.fmt.pix_mp.height
                : fmt.fmt.pix.height;
    g_mgr.pushFrame(cam_id, w, h, out, t_ns);
  }
  return true;
}


// Захват кадра в формате cv::Mat. Используется существующая функция
// capture_jpeg, после чего изображение декодируется в градации серого.
static cv::Mat capture_mat(const std::string &dev) {
  std::vector<unsigned char> buf;
  if (!capture_jpeg(dev, buf, std::string(), V4L2_BUF_TYPE_VIDEO_CAPTURE))
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
    auto pairs = g_mgr.getActivePairs();
    for (auto &pair : pairs) {
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

  std::filesystem::path exe_dir = std::filesystem::canonical(argv[0]).parent_path();
  g_exe_dir = exe_dir;
  g_config_path = argc > 1 ? std::filesystem::path(argv[1]) : exe_dir / "config.json";
  if (!g_mgr.loadConfig(g_config_path.string()))
    return 1;
  std::ifstream jf(g_config_path);
  nlohmann::json j;
  jf >> j;
  g_preview_enabled = j.value("preview_enabled", true);
  int port = j.value("http", nlohmann::json::object()).value("port", 8080);

  g_calib = std::make_unique<CalibrationSession>(
      g_mgr, g_preview_enabled,
      std::filesystem::absolute(readMainConfig().value("calib_root", ".")));

  std::signal(SIGINT, sigint);
  g_mgr.start();

  // Seed format cache with current formats of active devices.
  populate_format_cache();



  // Отдельный поток обработки стереопар.
  std::thread stereo_thread(stereo_loop);


  g_server.set_mount_point("/", "./web");

  g_server.Get("/api/status",
               [](const httplib::Request &, httplib::Response &res) {
                 bool detect = false;
                 for (auto &c : g_mgr.configuredCameras()) {
                   if (c.det_running) {
                     detect = true;
                     break;
                   }
                 }
                 nlohmann::json out{{"preview", g_preview_enabled},
                                      {"detect", detect}};
                 res.set_content(out.dump(), "application/json");
               });

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
                         {"mode",
                          (c.mode == CameraManager::CamConfig::Mode::Detect)
                              ? "detect"
                              : (c.mode == CameraManager::CamConfig::Mode::Calibration)
                                    ? "calibration"
                                    : "preview"},
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
                         {"buffer_type", c.buffer_type},
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
                    std::filesystem::path dir = std::filesystem::current_path() /
                                              "calibration" /
                                              ("cam_" + cam) / "images";
                    std::error_code ec;
                    std::filesystem::create_directories(dir, ec);
                    auto absDir = std::filesystem::absolute(dir);
                    printf("calibration dir: %s\n", absDir.c_str());
                    if (ec) {
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
                 std::string dir = cam.empty()
                                        ? std::string()
                                        : "calibration/cam_" + cam + "/images";
                 bool folder = !cam.empty() && dirExists(dir);
                 bool mono_done =
                     !cam.empty() &&
                     fileExists("calibration/results/cam_" + cam + ".yml");
                 bool stereo_ready =
                     mono_done &&
                     fileExists("calibration/results/cam_0.yml") &&
                     !fileExists("calibration/results/stereo_0_" + cam +
                                 ".yml");
                 resp["folder_exists"] = folder;
                 resp["mono_done"] = mono_done;
                 resp["stereo_ready"] = stereo_ready;
                 res.set_content(resp.dump(), "application/json");
               });
  g_server.Post("/api/calib/start",
                [](const httplib::Request &, httplib::Response &res) {
                  g_calib->start();
                  res.set_content("{\"status\":\"ok\"}", "application/json");
                });

  g_server.Post("/api/calib/stop",
                [](const httplib::Request &, httplib::Response &res) {
                  // Restore per-camera preview states before restarting.
                  g_calib->stop();
                  res.set_content("{\"status\":\"ok\"}", "application/json");
                });

  g_server.Post("/api/calibration/start",
                [](const httplib::Request &, httplib::Response &res) {
                  g_calib->start();
                  nlohmann::json j; j["status"] = "ok";
                  res.set_content(j.dump(), "application/json");
                });

  g_server.Post("/api/calibration/stop",
                [](const httplib::Request &, httplib::Response &res) {
                  g_calib->stop();
                  nlohmann::json j; j["status"] = "ok";
                  res.set_content(j.dump(), "application/json");
                });

  g_server.Post("/api/calibration/run",
                [](const httplib::Request &req, httplib::Response &res) {
                  nlohmann::json resp;
                  bool started = false;
                  try {
                    auto j = nlohmann::json::parse(req.body);
                    auto ids = j.at("ids").get<std::vector<std::string>>();
                    int duration = j.value("duration", 30);
                    int bw = j.value("board_w", 0);
                    int bh = j.value("board_h", 0);
                    g_calib->start();
                    started = true;
                    char q = '"';
                    std::string cmd = std::string(1, q) + (g_exe_dir / "calibration_cli").string() + std::string("\" ");
                    if (ids.size() == 1) {
                      cmd += "mono " + ids[0];
                    } else {
                      cmd += "stereo";
                      for (auto &id : ids) cmd += " " + id;
                    }
                    cmd += " " + std::to_string(duration);
                    if (bw > 0 && bh > 0) {
                      cmd += " --board " + std::to_string(bw) + "x" + std::to_string(bh);
                    }
                    cmd += " --config " + std::string(1, q) + g_config_path.string() + std::string(1, q);
                    FILE *pipe = popen(cmd.c_str(), "r");
                    if (!pipe) {
                      resp["error"] = "popen failed";
                      res.status = 500;
                    } else {
                      std::string output; char buffer[256];
                      while (fgets(buffer, sizeof(buffer), pipe)) output += buffer;
                      int rc = pclose(pipe);
                      if (rc == 0) {
                        try { resp["paths"] = nlohmann::json::parse(output); }
                        catch (...) { resp["paths"] = nlohmann::json::array(); }
                      } else {
                        resp["error"] = rc;
                        res.status = 500;
                      }
                    }
                  } catch (const std::exception &e) {
                    resp["error"] = e.what();
                    res.status = 400;
                  }
                  if (started) g_calib->stop();
                  res.set_content(resp.dump(), "application/json");
                });


  g_server.Post(
      "/api/calib/stereo-capture",
      [](const httplib::Request &req, httplib::Response &res) {
        try {
          auto j = nlohmann::json::parse(req.body);
          if (!j.contains("cameras") || !j["cameras"].is_array()) {
            res.status = 400;
            res.set_content("{\"error\":\"missing cameras\"}",
                            "application/json");

            return;
          }
          std::vector<std::string> cams =
              j["cameras"].get<std::vector<std::string>>();
          int frames = j.value("frames", 0);
          int interval = j.value("interval", 0);
          auto r = g_calib->captureStereo(cams, frames, interval);
          res.status = r.status;
          res.set_content(r.body.dump(), "application/json");
        } catch (...) {
          res.status = 400;
          res.set_content("{\"error\":\"invalid json\"}",
                          "application/json");
        }
      });


  g_server.Post(
      "/api/calib/mono",
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
          auto r = g_calib->captureMono(cam);
          res.status = r.status;
          res.set_content(r.body.dump(), "application/json");
        } catch (...) {
          res.status = 400;
          res.set_content("{\"error\":\"invalid json\"}",
                          "application/json");
        }
      });

 g_server.Post(
      "/api/calib/mono/start",
      [](const httplib::Request &req, httplib::Response &res) {
        try {
          auto j = nlohmann::json::parse(req.body);
          std::string cam = j.value("camera", "");
          int bw = j.value("board_w", 0);
          int bh = j.value("board_h", 0);
          if (cam.empty()) {
            res.status = 400;
            res.set_content("{\"error\":\"missing camera\"}",
                            "application/json");
            return;
          }
          int job = g_calib->startMonoJob(cam, bw, bh);
          nlohmann::json out; out["job_id"] = job;
          res.set_content(out.dump(), "application/json");
        } catch (...) {
          res.status = 400;
          res.set_content("{\"error\":\"invalid json\"}",
                          "application/json");
        }
      });

  g_server.Get(
      "/api/calib/mono/progress",
      [](const httplib::Request &req, httplib::Response &res) {
        if (!req.has_param("job_id")) {
          res.status = 400;
          res.set_content("{\"error\":\"missing job_id\"}",
                          "application/json");
          return;
        }
        int job = std::stoi(req.get_param_value("job_id"));
        auto r = g_calib->monoProgress(job);
        res.status = r.status;
        res.set_content(r.body.dump(), "application/json");
      });


  g_server.Post("/api/calib/prepare",
                [](const httplib::Request &, httplib::Response &res) {
                  g_mgr.stop();
                  // Keep preview so the web UI can show frames during setup.
                  g_preview_enabled = true;
                  res.set_content("{\"status\":\"ok\"}", "application/json");
                });

  g_server.Post(
      "/api/preview/enable",
      [](const httplib::Request &req, httplib::Response &res) {
        try {
          auto j = nlohmann::json::parse(req.body);
          g_preview_enabled = j.at("enable").get<bool>();
          res.set_content("{\"status\":\"ok\"}", "application/json");
        } catch (...) {
          res.status = 400;
        }
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
                  std::string mode = j.at("mode").get<std::string>();
                  bool ok = false;
                  if (mode == "preview")
                    ok = g_mgr.setMode(id, CameraManager::CamConfig::Mode::Preview);
                  else if (mode == "detect")
                    ok = g_mgr.setMode(id, CameraManager::CamConfig::Mode::Detect);
                  else if (mode == "calibration")
                    ok = g_mgr.setMode(id, CameraManager::CamConfig::Mode::Calibration);
                  else {
                    res.status = 400;
                    return;
                  }
                  if (!ok)
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
                    std::string buffer_type =
                        j.value("buffer_type", std::string("auto"));
                    if (buffer_type != "auto" && buffer_type != "single" &&
                        buffer_type != "mplane") {
                      res.status = 400;
                      return;
                    }
                    int jpeg_quality = j.value("jpeg_quality", 60);
                    int http_fps_limit = j.value("http_fps_limit", 20);
                    bool show_fps = j.value("show_fps", false);
                    std::string npu_core =
                        j.value("npu_core", std::string("auto"));
                    std::string log_file =
                        j.value("log_file", std::string(""));
                    if (!g_mgr.updateSettings(id, vm, worker, auto_profiles,
                                              profile, model_path, labels_path,
                                              cap_fps, buffers, buffer_type,
                                              jpeg_quality, http_fps_limit,
                                              show_fps,
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

        if (req.has_param("id")) {
          std::string id = req.get_param_value("id");
          CameraManager::Frame fr;
          if (!g_mgr.getFrame(id, g_mgr.nowMonoNs(), fr)) {
            res.status = 404;
            return;
          }
          g_mgr.reportFrame(id);
          res.set_content(reinterpret_cast<const char *>(fr.jpeg.data()),
                          fr.jpeg.size(), "image/jpeg");
          return;
        }


        std::string dev;
        if (req.has_param("by"))
          dev = std::string("/dev/v4l/by-id/") + req.get_param_value("by");
        if (dev.empty()) {
          res.status = 404;
          return;
        }

        std::vector<unsigned char> jpg;
        if (!capture_jpeg(dev, jpg, std::string(), V4L2_BUF_TYPE_VIDEO_CAPTURE)) {
          std::cerr << "capture_jpeg failed for device '" << dev << "'" << std::endl;
          res.status = 404;
          return;
        }
        res.set_content(reinterpret_cast<const char *>(jpg.data()), jpg.size(),
                        "image/jpeg");
      });

 g_server.Get(
      "/api/preview.mjpg",
      [](const httplib::Request &req, httplib::Response &res) {
        if (!g_preview_enabled) {
          res.status = 403;
          return;
        }
        std::string dev;
        std::string id;
        int http_fps_limit = 0;
        if (req.has_param("id")) {
          id = req.get_param_value("id");
          for (auto &ci : g_mgr.configuredCameras()) {
            if (ci.id == id) {
              http_fps_limit = ci.http_fps_limit;
              break;
            }
          }
        } else if (req.has_param("by")) {
          dev = std::string("/dev/v4l/by-id/") + req.get_param_value("by");
        } else {
          res.status = 404;
          return;
        }

        res.set_header("Cache-Control",
                       "no-store, no-cache, must-revalidate");
        res.set_header("Pragma", "no-cache");
        res.set_header("Connection", "keep-alive");
        auto last_push = std::chrono::steady_clock::now();
        res.set_chunked_content_provider(
            "multipart/x-mixed-replace; boundary=frame",
            [dev, id, http_fps_limit, last_push](size_t,
                                                httplib::DataSink &sink) mutable {
              while (true) {
                std::vector<unsigned char> jpg;
                if (!id.empty()) {
                  CameraManager::Frame fr;
                  if (!g_mgr.getFrame(id, g_mgr.nowMonoNs(), fr))
                    break;
                  jpg = std::move(fr.jpeg);
                  g_mgr.reportFrame(id);
                } else {
                  if (!capture_jpeg(dev, jpg, std::string(),
                                    V4L2_BUF_TYPE_VIDEO_CAPTURE))
                    break;
                }
                if (http_fps_limit > 0) {
                  auto now = std::chrono::steady_clock::now();
                  double elapsed =
                      std::chrono::duration<double>(now - last_push).count();
                  double min_dt = 1.0 / http_fps_limit;
                  if (elapsed < min_dt) {
                    auto sleep_d =
                        std::chrono::duration<double>(min_dt - elapsed);
                    std::this_thread::sleep_for(
                        std::chrono::duration_cast<std::chrono::milliseconds>(
                            sleep_d));
                  }
                  last_push = std::chrono::steady_clock::now();
                }

                std::string header =
                    "--frame\r\nContent-Type: image/jpeg\r\nContent-Length: " +
                    std::to_string(jpg.size()) + "\r\n\r\n";
                if (!sink.write(header.data(), header.size()))
                  break;
                if (!sink.write(reinterpret_cast<const char *>(jpg.data()),
                                jpg.size()))
                  break;
                if (!sink.write("\r\n", 2))
                  break;
              }
              return true;
            },
            [](bool) {});
      });

 g_server.Get(
      "/api/capture", [](const httplib::Request &req, httplib::Response &res) {
        if (!req.has_param("id")) {
          res.status = 400;
          return;
        }
        std::string id = req.get_param_value("id");
        uint64_t t_ns = g_mgr.nowMonoNs();
        if (req.has_param("t")) {
          try {
            t_ns = std::stoull(req.get_param_value("t"));
          } catch (...) {
          }
        }
        CameraManager::Frame fr;
        if (!g_mgr.getFrame(id, t_ns, fr)) {
          res.status = 404;
          return;
        }
        std::string dir = "captures";
        std::filesystem::create_directories(dir);
        std::string file = dir + "/" + id + "_" + std::to_string(fr.t_mono_ns) + ".jpg";
        if (!g_mgr.saveFrameWithMeta(file, fr, id)) {
          res.status = 500;
          return;
        }
        nlohmann::json j;
        j["path"] = file;
        j["t_mono_ns"] = fr.t_mono_ns;
        res.set_content(j.dump(), "application/json");
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
