
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

#include "httplib.h"
#include "nlohmann/json.hpp"

static CameraManager g_mgr;
static httplib::Server g_server;

static void sigint(int) {
  g_mgr.stop();
  g_server.stop();
}

static bool capture_jpeg(const std::string &dev,
                         std::vector<unsigned char> &out) {
  int fd = open(dev.c_str(), O_RDWR);
  if (fd < 0)
    return false;
  v4l2_format fmt{};
  fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  fmt.fmt.pix.width = 320;
  fmt.fmt.pix.height = 240;
  fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_MJPEG;
  fmt.fmt.pix.field = V4L2_FIELD_NONE;
  if (ioctl(fd, VIDIOC_S_FMT, &fmt) < 0) {
    close(fd);
    return false;
  }
  v4l2_requestbuffers req{};
  req.count = 1;
  req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  req.memory = V4L2_MEMORY_MMAP;
  if (ioctl(fd, VIDIOC_REQBUFS, &req) < 0) {
    close(fd);
    return false;
  }
  v4l2_buffer buf{};
  buf.type = req.type;
  buf.memory = V4L2_MEMORY_MMAP;
  buf.index = 0;
  if (ioctl(fd, VIDIOC_QUERYBUF, &buf) < 0) {
    close(fd);
    return false;
  }
  void *mem = mmap(NULL, buf.length, PROT_READ | PROT_WRITE, MAP_SHARED, fd,
                   buf.m.offset);
  if (mem == MAP_FAILED) {
    close(fd);
    return false;
  }
  if (ioctl(fd, VIDIOC_QBUF, &buf) < 0) {
    munmap(mem, buf.length);
    close(fd);
    return false;
  }
  v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  if (ioctl(fd, VIDIOC_STREAMON, &type) < 0) {
    munmap(mem, buf.length);
    close(fd);
    return false;
  }
  if (ioctl(fd, VIDIOC_DQBUF, &buf) < 0) {
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

int main(int argc, char **argv) {
  std::string cfg = "config.json";
  if (argc > 1)
    cfg = argv[1];
  if (!g_mgr.loadConfig(cfg))
    return 1;
  std::ifstream jf(cfg);
  nlohmann::json j;
  jf >> j;
  int port = j.value("http", nlohmann::json::object()).value("port", 8080);

  std::signal(SIGINT, sigint);
  g_mgr.start();

  g_server.set_mount_point("/", "./web");

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
                         {"auto_profiles", c.auto_profiles}});
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

  g_server.Post("/api/preview",
                [](const httplib::Request &req, httplib::Response &res) {
                  try {
                    auto j = nlohmann::json::parse(req.body);
                    std::string id = j.at("id").get<std::string>();
                    bool enable = j.at("enable").get<bool>();
                    if (!g_mgr.setPreview(id, enable))
                      res.status = 400;
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
                    if (!g_mgr.updateSettings(id, vm, worker, auto_profiles))
                      res.status = 400;
                  } catch (...) {
                    res.status = 400;
                  }
                });

  g_server.Get(
      "/api/preview", [](const httplib::Request &req, httplib::Response &res) {
        std::string dev;
        if (req.has_param("id"))
          dev = g_mgr.devicePath(req.get_param_value("id"));
        else if (req.has_param("by"))
          dev = std::string("/dev/v4l/by-id/") + req.get_param_value("by");
        std::vector<unsigned char> jpg;
        if (dev.empty() || !capture_jpeg(dev, jpg)) {
          res.status = 404;
          return;
        }
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
  return 0;
}
