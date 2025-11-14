#include "httplib.h"
#include "nlohmann/json.hpp"
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <map>
#include <mutex>
#include <optional>
#include <random>
#include <sstream>
#include <string>
#include <unordered_map>
#include <vector>

namespace fs = std::filesystem;
using json = nlohmann::json;

struct CameraCalibrationConfig {
  cv::Vec3f position{0.0f, 0.0f, 2.0f};
  float meters_per_pixel{0.02f};
  float floor_z{0.0f};
};

struct DemoCamera {
  std::string id;
  cv::Mat frame;
  CameraCalibrationConfig calibration;
};

struct DemoObject {
  int id{0};
  cv::Rect2f box{150.0f, 150.0f, 180.0f, 260.0f};
  cv::Point2f velocity{1.5f, 1.0f};
};

struct DetectionOverlay {
  cv::Rect box;
  int object_id{0};
  cv::Point3f world;
};

struct GlobalTrackState {
  int id{0};
  cv::Point3f world;
  uint64_t last_frame{0};
};

class SimpleTrackingEngine {
public:
  bool loadConfig(const std::string &path, std::string &error_message) {
    config_path_ = path;
    fs::path config_abs = fs::absolute(path);
    config_dir_ = config_abs.parent_path();

    std::ifstream input(config_abs);
    if (!input.is_open()) {
      error_message = "Не удается открыть файл конфигурации " + config_abs.string();
      return false;
    }

    json root;
    try {
      input >> root;
    } catch (const std::exception &e) {
      error_message = std::string("Ошибка парсинга JSON: ") + e.what();
      return false;
    }

    http_port_ = root.value("http_port", http_port_);

    if (!root.contains("cameras") || !root["cameras"].is_array() || root["cameras"].empty()) {
      error_message = "В конфигурации отсутствует список камер (cameras[])";
      return false;
    }

    cameras_.clear();
    camera_index_.clear();

    for (const auto &cam_json : root["cameras"]) {
      DemoCamera cam;
      cam.id = cam_json.value("id", std::string());
      std::string frame_path = cam_json.value("frame", std::string());
      if (cam.id.empty() || frame_path.empty()) {
        error_message = "Каждая камера должна содержать поля id и frame";
        cameras_.clear();
        camera_index_.clear();
        return false;
      }

      fs::path resolved = frame_path;
      if (resolved.is_relative()) {
        resolved = config_dir_ / resolved;
      }
      cam.frame = cv::imread(resolved.string(), cv::IMREAD_COLOR);
      if (cam.frame.empty()) {
        error_message = "Не удалось загрузить тестовое изображение " + resolved.string();
        cameras_.clear();
        camera_index_.clear();
        return false;
      }

      if (cam_json.contains("calibration")) {
        const auto &calib_json = cam_json["calibration"];
        if (calib_json.contains("position") && calib_json["position"].is_array() &&
            calib_json["position"].size() == 3) {
          cam.calibration.position[0] = calib_json["position"][0].get<float>();
          cam.calibration.position[1] = calib_json["position"][1].get<float>();
          cam.calibration.position[2] = calib_json["position"][2].get<float>();
        }
        cam.calibration.meters_per_pixel =
            calib_json.value("meters_per_pixel", cam.calibration.meters_per_pixel);
        cam.calibration.floor_z = calib_json.value("floor_z", cam.calibration.floor_z);
      }

      camera_index_[cam.id] = cameras_.size();
      cameras_.push_back(std::move(cam));
    }

    objects_.clear();
    initial_objects_.clear();
    if (root.contains("objects") && root["objects"].is_array() && !root["objects"].empty()) {
      for (const auto &obj_json : root["objects"]) {
        DemoObject obj;
        obj.id = obj_json.value("id", static_cast<int>(objects_.size() + 1));
        if (obj_json.contains("box") && obj_json["box"].is_array() && obj_json["box"].size() == 4) {
          obj.box.x = obj_json["box"][0].get<float>();
          obj.box.y = obj_json["box"][1].get<float>();
          obj.box.width = std::max(10.0f, obj_json["box"][2].get<float>());
          obj.box.height = std::max(10.0f, obj_json["box"][3].get<float>());
        }
        if (obj_json.contains("velocity") && obj_json["velocity"].is_array() &&
            obj_json["velocity"].size() == 2) {
          obj.velocity.x = obj_json["velocity"][0].get<float>();
          obj.velocity.y = obj_json["velocity"][1].get<float>();
        }
        objects_.push_back(obj);
      }
    } else {
      // default synthetic objects
      DemoObject a;
      a.id = 1;
      a.box = cv::Rect2f(220.0f, 160.0f, 200.0f, 280.0f);
      a.velocity = cv::Point2f(1.7f, 1.1f);
      objects_.push_back(a);
      DemoObject b;
      b.id = 2;
      b.box = cv::Rect2f(420.0f, 180.0f, 160.0f, 260.0f);
      b.velocity = cv::Point2f(-1.3f, 1.4f);
      objects_.push_back(b);
    }

    initial_objects_ = objects_;
    resetInternalState();
    return true;
  }

  int httpPort() const { return http_port_; }

  void overridePort(int port) {
    if (port > 0) {
      http_port_ = port;
    }
  }

  bool renderCamera(const std::string &camera_id, std::vector<uint8_t> &jpeg,
                    std::string &error_message) {
    std::lock_guard<std::mutex> lock(mutex_);
    auto idx_it = camera_index_.find(camera_id);
    if (idx_it == camera_index_.end()) {
      error_message = "Неизвестная камера: " + camera_id;
      return false;
    }

    stepSimulation();

    const auto &camera = cameras_[idx_it->second];
    auto overlay_it = last_overlays_.find(camera_id);
    if (overlay_it == last_overlays_.end()) {
      error_message = "Нет данных детекции для камеры " + camera_id;
      return false;
    }

    cv::Mat frame = camera.frame.clone();
    const auto &overlays = overlay_it->second;
    for (const auto &overlay : overlays) {
      cv::rectangle(frame, overlay.box, cv::Scalar(32, 201, 255), 2);
      std::ostringstream label;
      label.setf(std::ios::fixed);
      label.precision(2);
      label << "cam:" << camera_id << " obj:" << overlay.object_id << " xyz:("
            << overlay.world.x << ',' << overlay.world.y << ',' << overlay.world.z << ")";
      auto anchor = overlay.box.tl();
      anchor.y = std::max(20, anchor.y - 5);
      cv::putText(frame, label.str(), anchor, cv::FONT_HERSHEY_SIMPLEX, 0.5,
                  cv::Scalar(255, 255, 255), 1, cv::LINE_AA);
    }

    std::vector<int> params{cv::IMWRITE_JPEG_QUALITY, 85};
    if (!cv::imencode(".jpg", frame, jpeg, params)) {
      error_message = "Не удалось кодировать JPEG";
      return false;
    }
    return true;
  }

  json statusJson() const {
    std::lock_guard<std::mutex> lock(mutex_);
    json cameras = json::array();
    for (const auto &cam : cameras_) {
      cameras.push_back({
          {"id", cam.id},
          {"size", {cam.frame.cols, cam.frame.rows}},
          {"calibration",
           {{"position", {cam.calibration.position[0], cam.calibration.position[1],
                            cam.calibration.position[2]}},
            {"meters_per_pixel", cam.calibration.meters_per_pixel},
            {"floor_z", cam.calibration.floor_z}}}});
    }
    return json{{"config", config_path_},
                {"http_port", http_port_},
                {"frame", frame_index_},
                {"cameras", cameras},
                {"objects", objects_.size()},
                {"active_tracks", global_tracks_.size()}};
  }

  json tracksJson() {
    std::lock_guard<std::mutex> lock(mutex_);
    cleanupExpiredTracks();
    json tracks = json::array();
    for (const auto &[id, track] : global_tracks_) {
      tracks.push_back({{"global_id", id},
                        {"world", {track.world.x, track.world.y, track.world.z}},
                        {"last_frame", track.last_frame}});
    }
    return json{{"frame", frame_index_}, {"tracks", tracks}};
  }

  void resetSimulation() {
    std::lock_guard<std::mutex> lock(mutex_);
    objects_ = initial_objects_;
    resetInternalState();
  }

private:
  void resetInternalState() {
    frame_index_ = 0;
    last_overlays_.clear();
    global_tracks_.clear();
  }

  void stepSimulation() {
    ++frame_index_;
    const cv::Size reference = cameras_.front().frame.size();
    for (auto &obj : objects_) {
      obj.box.x += obj.velocity.x;
      obj.box.y += obj.velocity.y;
      if (obj.box.x <= 0 || obj.box.x + obj.box.width >= reference.width) {
        obj.velocity.x *= -1.0f;
        obj.box.x = std::clamp(obj.box.x, 0.0f,
                               static_cast<float>(reference.width - obj.box.width - 1));
      }
      if (obj.box.y <= 0 || obj.box.y + obj.box.height >= reference.height) {
        obj.velocity.y *= -1.0f;
        obj.box.y = std::clamp(obj.box.y, 0.0f,
                               static_cast<float>(reference.height - obj.box.height - 1));
      }
    }

    last_overlays_.clear();
    for (size_t idx = 0; idx < cameras_.size(); ++idx) {
      const auto &cam = cameras_[idx];
      const float offset = static_cast<float>(idx) * 15.0f;
      std::vector<DetectionOverlay> overlays;
      overlays.reserve(objects_.size());
      for (const auto &obj : objects_) {
        auto rect = makeRectForCamera(obj, cam.frame.size(), offset);
        auto world = estimateWorld(cam, rect);
        overlays.push_back({rect, obj.id, world});
        updateGlobalTrack(obj.id, world);
      }
      last_overlays_[cam.id] = std::move(overlays);
    }
    cleanupExpiredTracks();
  }

  static cv::Rect makeRectForCamera(const DemoObject &obj, const cv::Size &frame_size,
                                    float offset) {
    cv::Rect2f shifted = obj.box;
    shifted.x += offset;
    shifted.y += offset * 0.25f;
    shifted.x = std::clamp(shifted.x, 0.0f,
                           static_cast<float>(frame_size.width - shifted.width - 1));
    shifted.y = std::clamp(shifted.y, 0.0f,
                           static_cast<float>(frame_size.height - shifted.height - 1));
    return cv::Rect(static_cast<int>(shifted.x), static_cast<int>(shifted.y),
                    static_cast<int>(shifted.width), static_cast<int>(shifted.height));
  }

  static cv::Point3f estimateWorld(const DemoCamera &camera, const cv::Rect &rect) {
    cv::Point2f center(rect.x + rect.width * 0.5f, rect.y + rect.height * 0.5f);
    float nx = center.x / static_cast<float>(camera.frame.cols) - 0.5f;
    float ny = center.y / static_cast<float>(camera.frame.rows) - 0.5f;

    cv::Point3f world;
    world.x = camera.calibration.position[0] +
              nx * static_cast<float>(camera.frame.cols) * camera.calibration.meters_per_pixel;
    world.y = camera.calibration.position[1] +
              ny * static_cast<float>(camera.frame.rows) * camera.calibration.meters_per_pixel;
    world.z = camera.calibration.floor_z;
    return world;
  }

  void updateGlobalTrack(int object_id, const cv::Point3f &world) {
    auto &track = global_tracks_[object_id];
    track.id = object_id;
    track.world = world;
    track.last_frame = frame_index_;
  }

  void cleanupExpiredTracks() {
    const uint64_t ttl = 240; // roughly 8 секунд при 30 FPS
    for (auto it = global_tracks_.begin(); it != global_tracks_.end();) {
      if (frame_index_ > it->second.last_frame &&
          frame_index_ - it->second.last_frame > ttl) {
        it = global_tracks_.erase(it);
      } else {
        ++it;
      }
    }
  }

  std::string config_path_;
  fs::path config_dir_;
  int http_port_{8095};

  std::vector<DemoCamera> cameras_;
  std::unordered_map<std::string, size_t> camera_index_;
  std::vector<DemoObject> objects_;
  std::vector<DemoObject> initial_objects_;

  std::map<std::string, std::vector<DetectionOverlay>> last_overlays_;
  std::map<int, GlobalTrackState> global_tracks_;

  uint64_t frame_index_{0};
  mutable std::mutex mutex_;
};

static void printUsage(const char *argv0) {
  std::cout << "Usage: " << argv0 << " [--config path/to/simple_tracking_demo.json] [--port 8095]\n";
}

int main(int argc, char **argv) {
  std::string config_path = "web/simple_tracking_demo.json";
  std::optional<int> override_port;

  for (int i = 1; i < argc; ++i) {
    std::string arg = argv[i];
    if (arg == "--config" && i + 1 < argc) {
      config_path = argv[++i];
    } else if (arg == "--port" && i + 1 < argc) {
      override_port = std::stoi(argv[++i]);
    } else if (arg == "--help" || arg == "-h") {
      printUsage(argv[0]);
      return 0;
    } else {
      std::cerr << "Неизвестный аргумент: " << arg << "\n";
      printUsage(argv[0]);
      return 1;
    }
  }

  SimpleTrackingEngine engine;
  std::string error;
  if (!engine.loadConfig(config_path, error)) {
    std::cerr << error << std::endl;
    return 1;
  }
  if (override_port.has_value()) {
    engine.overridePort(*override_port);
  }

  std::cout << "🚀 Старт тестового сервера слежения. Камеры: "
            << engine.statusJson()["cameras"].size() << ", порт " << engine.httpPort()
            << std::endl;

  httplib::Server server;

  server.Get("/", [&engine](const httplib::Request &, httplib::Response &res) {
    std::ostringstream html;
    auto status = engine.statusJson();
    html << "<html><body><h1>Simple Tracking Demo</h1>";
    html << "<p>Config: " << status["config"].get<std::string>() << "</p>";
    html << "<p>Frame: " << status["frame"].get<uint64_t>() << "</p>";
    html << "<h2>Cameras</h2><ul>";
    for (const auto &cam : status["cameras"]) {
      html << "<li>" << cam["id"].get<std::string>() << " - <a href=\"/api/frame/"
           << cam["id"].get<std::string>() << "\">preview</a></li>";
    }
    html << "</ul><p><a href=\"/api/tracks\">JSON tracks</a></p>";
    html << "<p>POST /api/reset to restart simulation</p>";
    html << "</body></html>";
    res.set_content(html.str(), "text/html; charset=UTF-8");
  });

  server.Get(R"(/api/frame/(.+))", [&engine](const httplib::Request &req, httplib::Response &res) {
    std::string camera_id;
    if (!req.matches.empty()) {
      camera_id = req.matches[1];
    }
    std::vector<uint8_t> jpeg;
    std::string error_message;
    if (!engine.renderCamera(camera_id, jpeg, error_message)) {
      res.status = 404;
      res.set_content(error_message, "text/plain; charset=UTF-8");
      return;
    }
    res.set_content(reinterpret_cast<const char *>(jpeg.data()), jpeg.size(), "image/jpeg");
  });

  server.Get("/api/status", [&engine](const httplib::Request &, httplib::Response &res) {
    res.set_content(engine.statusJson().dump(2), "application/json");
  });

  server.Get("/api/tracks", [&engine](const httplib::Request &, httplib::Response &res) {
    res.set_content(engine.tracksJson().dump(2), "application/json");
  });

  server.Post("/api/reset", [&engine](const httplib::Request &, httplib::Response &res) {
    engine.resetSimulation();
    res.set_content("reset", "text/plain; charset=UTF-8");
  });

  server.set_exception_handler([](const httplib::Request &, httplib::Response &res, std::exception_ptr ep) {
    std::string what = "internal error";
    try {
      if (ep) {
        std::rethrow_exception(ep);
      }
    } catch (const std::exception &e) {
      what = e.what();
    }
    res.status = 500;
    res.set_content(what, "text/plain; charset=UTF-8");
  });

  server.listen("0.0.0.0", engine.httpPort());
  return 0;
}