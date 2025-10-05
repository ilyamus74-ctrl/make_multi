#include "camera_manager.h"
#include "calibration/session.h"
#include <nlohmann/json.hpp>
#include <opencv2/opencv.hpp>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <map>
#include <vector>
#include <chrono>
#include <thread>

using json = nlohmann::json;

// Check if frame is suitable for calibration
static bool frame_ok(const cv::Mat &frame,
                     cv::Size board = cv::Size(9,6),
                     double sharp_thr = 100.0,
                     double bright_min = 50.0,
                     double bright_max = 200.0,
                     double sat_min = 25.0) {
    cv::Mat gray; cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
    cv::Mat lap; cv::Laplacian(gray, lap, CV_64F);
    cv::Scalar mean, stddev; cv::meanStdDev(lap, mean, stddev);
    double sharpness = stddev[0]*stddev[0];
    if (sharpness < sharp_thr) return false;
    double brightness = cv::mean(gray)[0];
    if (brightness < bright_min || brightness > bright_max) return false;
    cv::Mat hsv; cv::cvtColor(frame, hsv, cv::COLOR_BGR2HSV);
    double saturation = cv::mean(hsv)[1];
    if (saturation < sat_min) return false;
    bool found = cv::findChessboardCorners(gray, board, cv::noArray());
    return found;
}

static void capture_mono_images(const json &cam_cfg,
                                int duration,
                                cv::Size board,
                                const std::filesystem::path &root = "calibration/mono") {
    std::string id = cam_cfg.value("id", "");
    std::string device = cam_cfg.value("device", "");
    auto pref = cam_cfg["preferred"];
    int w = pref.value("w",1280);
    int h = pref.value("h",720);
    int fps = pref.value("fps",30);

    std::filesystem::path dir = root / id;
    std::filesystem::create_directories(dir);

    cv::VideoCapture cap(device);
    cap.set(cv::CAP_PROP_FRAME_WIDTH, w);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, h);
    cap.set(cv::CAP_PROP_FPS, fps);
    int idx = 0;
    auto start = std::chrono::steady_clock::now();
    while (std::chrono::steady_clock::now() - start < std::chrono::seconds(duration)) {
        cv::Mat frame; cap >> frame;
        if (!frame.empty() && frame_ok(frame, board)) {
            char buf[64]; snprintf(buf, sizeof(buf), "frame_%06d.png", idx++);
            cv::imwrite((dir / buf).string(), frame);
        }
        std::this_thread::sleep_for(std::chrono::seconds(2));
    }
    cap.release();
}

static void capture_stereo_images(const std::vector<json> &cams_cfg,
                                  int duration,
                                  cv::Size board,
                                  const std::filesystem::path &root = "calibration/stereo") {
    std::map<std::string, cv::VideoCapture> caps;
    std::map<std::string, std::filesystem::path> dirs;
    for (auto &cfg : cams_cfg) {
        std::string id = cfg.value("id", "");
        std::string device = cfg.value("device", "");
        auto pref = cfg["preferred"];
        int w = pref.value("w",1280);
        int h = pref.value("h",720);
        int fps = pref.value("fps",30);
        cv::VideoCapture cap(device);
        cap.set(cv::CAP_PROP_FRAME_WIDTH, w);
        cap.set(cv::CAP_PROP_FRAME_HEIGHT, h);
        cap.set(cv::CAP_PROP_FPS, fps);
        caps[id] = std::move(cap);
        std::filesystem::path dir = root / id;
        std::filesystem::create_directories(dir);
        dirs[id] = dir;
       }

    int idx = 0;
    auto start = std::chrono::steady_clock::now();
    while (std::chrono::steady_clock::now() - start < std::chrono::seconds(duration)) {

        std::map<std::string, cv::Mat> frames;
        bool ok = true;
        for (auto &kv : caps) {
            cv::Mat f; kv.second >> f;
            if (f.empty() || !frame_ok(f, board)) { ok = false; break; }
            frames[kv.first] = f;
        }
        if (ok) {
            for (auto &kv : frames) {
                char buf[64]; snprintf(buf, sizeof(buf), "frame_%06d.png", idx);
                cv::imwrite((dirs[kv.first] / buf).string(), kv.second);
            }
            ++idx;
        }
        std::this_thread::sleep_for(std::chrono::seconds(2));
    }
    for (auto &kv : caps) kv.second.release();
}

int main(int argc, char **argv) {
    std::filesystem::path exe_dir = std::filesystem::canonical(argv[0]).parent_path();
    std::filesystem::path config_path = exe_dir / "config.json";
    cv::Size board(9,6);
    int argi = 1;
    while (argi + 1 < argc) {
        std::string opt = argv[argi];
        if (opt == "--config") {
            config_path = argv[argi + 1];
            argi += 2;
        } else if (opt == "--board") {
            std::string val = argv[argi + 1];
            auto x = val.find('x');
            if (x != std::string::npos) {
                board.width = std::atoi(val.substr(0, x).c_str());
                board.height = std::atoi(val.substr(x + 1).c_str());
            }
            argi += 2;
        } else {
            break;
        }
    }

    if (argc - argi < 2) {
        std::cout << "Usage: " << argv[0]
                  << " [--config <path>] [--board WxH] mono <id> [duration] | stereo <id1> <id2> ... [duration]" << std::endl;
        return 0;
    }

    std::ifstream f(config_path);
    if (!f.is_open()) {
        std::cerr << "Failed to open " << config_path << std::endl;
        return 1;
    }
    json cfg; f >> cfg;

    CameraManager mgr;
    mgr.loadConfig(config_path.string());
    mgr.start();
    bool preview = true;
    CalibrationSession session(mgr, preview, std::filesystem::current_path());

    std::vector<std::string> out_dirs;
    std::string mode = argv[argi];
    if (mode == "mono") {
        std::string id = argv[argi + 1];
        int duration = 30;
        if (argc - argi > 2) duration = std::atoi(argv[argi + 2]);
        json cam_cfg;
        bool found = false;
        for (auto &c : cfg["cameras"]) if (c["id"] == id) { cam_cfg = c; found = true; break; }
        if (!found) { std::cerr << "Camera id " << id << " not in config" << std::endl; return 1; }
        capture_mono_images(cam_cfg, duration, board);
        out_dirs.push_back((std::filesystem::path("calibration/mono") / id).string());
    } else if (mode == "stereo") {
        std::vector<std::string> ids;
        for (int i = argi + 1; i < argc; i++) ids.push_back(argv[i]);
        int duration = 30;
        if (ids.size() >= 3) {
            try {
                duration = std::stoi(ids.back());
                ids.pop_back();
            } catch (...) {
                // last argument was not a duration
            }
        }
        if (ids.size() < 2) { std::cerr << "Need at least two camera ids" << std::endl; return 1; }
        std::vector<json> cams;
        for (auto &id : ids) {
            bool found = false;
            for (auto &c : cfg["cameras"]) if (c["id"] == id) { cams.push_back(c); found = true; break; }
            if (!found) { std::cerr << "Camera id " << id << " not in config" << std::endl; return 1; }
        }
        capture_stereo_images(cams, duration, board);
        for (auto &id : ids) out_dirs.push_back((std::filesystem::path("calibration/stereo") / id).string());
    } else {
        std::cerr << "Unknown mode: " << mode << std::endl; mgr.stop(); return 1;
    }
    mgr.stop();
    json j = out_dirs;
    std::cout << j.dump() << std::endl;
    return 0;
}
