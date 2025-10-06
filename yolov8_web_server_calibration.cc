// Calibration-only HTTP server based on the YOLOv8 web server.
// This variant is intended for camera calibration previews and keeps
// the calibration preview state active while the process runs.
//
// Flags:
//   --dev /dev/videoX
//   --port N
//   --size WxH
//   --cap-fps N
//   --buffers N
//   --jpeg-quality 30..95
//   --http-fps-limit N
//   --fps
//   --npu-core auto|0|1|2|01|012
//   --log-file FILE NAME

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <signal.h>
#include <iostream>
#include <vector>
#include <chrono>
#include <thread>
#include <atomic>
#include <mutex>
#include <condition_variable>
#include <memory>
#include <optional>
#include <algorithm>
#include <cmath>
#include <ctime>
#include <fstream>
#include <set>
#include <map>
#include <variant>
#include <dirent.h>
#include <filesystem>
#include <errno.h>

#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/select.h>
#include <sys/stat.h>
#include <linux/videodev2.h>

#include "turbojpeg.h"
#include "image_drawing.h"
#include "yolov8.h"
#include "common.h"
#include "image_utils.h"
#include "file_utils.h"
#include "httplib.h"
#include "nlohmann/json.hpp"
#include <opencv2/opencv.hpp>
#include "camera_manager.h"
#include "calibration/live_calibrator.h"

//debug start

struct StageAcc {
  double cap=0, prep=0, infer=0, draw=0, enc=0, loop=0;
  int n=0;
  void add_print_and_reset(int every=30) {
    n++;
    if (n % every == 0) {
      printf("TIMES(avg %d): cap=%.1fms prep=%.1fms infer=%.1fms draw=%.1fms enc=%.1fms | loop=%.1fms\n",
             every, cap/every, prep/every, infer/every, draw/every, enc/every, loop/every);
      cap=prep=infer=draw=enc=loop=0; n=0;
    }
  }
};

#define TICK(x) auto x##_t0 = std::chrono::steady_clock::now()
#define TOCK(x,acc_field) acc_field += std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now() - x##_t0).count()
//debug end


using json = nlohmann::json;
using namespace httplib;
using Clock = std::chrono::high_resolution_clock;

// -----------------------------------------------------------------------------
// Camera compatibility analysis helpers

struct CameraCharacteristics {
    std::string id;
    float estimated_fov = 0.0f;
    float avg_corner_distance = 0.0f;
    cv::Size resolution;
    int quality_score = 0;
    bool is_wide_angle = false;
};

class CameraCompatibilityAnalyzer {
    std::map<std::string, CameraCharacteristics> camera_chars_;

    int evaluateImageQuality(const cv::Mat& frame, const std::vector<cv::Point2f>& corners) {
        cv::Mat gray;
        if(frame.channels() > 1) {
            cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
        } else {
            gray = frame;
        }

        cv::Mat grad_x, grad_y, grad;
        cv::Sobel(gray, grad_x, CV_32F, 1, 0, 3);
        cv::Sobel(gray, grad_y, CV_32F, 0, 1, 3);
        cv::magnitude(grad_x, grad_y, grad);

        double mean_gradient = cv::mean(grad)[0];
        return static_cast<int>(mean_gradient);
    }

public:
    void analyzeCameraFromFrame(const std::string& cam_id, const cv::Mat& frame,
                                const std::vector<cv::Point2f>& corners, cv::Size board_size) {
        CameraCharacteristics& chars = camera_chars_[cam_id];
        chars.id = cam_id;
        chars.resolution = frame.size();

        if(!corners.empty()) {
            float board_width_pixels = cv::norm(corners[board_size.width-1] - corners[0]);
            float board_height_pixels = cv::norm(corners[corners.size()-1] - corners[corners.size()-board_size.width]);

            float frame_diagonal = std::sqrt(frame.rows*frame.rows + frame.cols*frame.cols);
            float board_diagonal = std::sqrt(board_width_pixels*board_width_pixels + board_height_pixels*board_height_pixels);

            chars.estimated_fov = (board_diagonal / frame_diagonal) * 60.0f;

            float total_distance = 0;
            int count = 0;
            for(int i = 0; i < board_size.height; i++) {
                for(int j = 0; j < board_size.width-1; j++) {
                    int idx1 = i * board_size.width + j;
                    int idx2 = i * board_size.width + j + 1;
                    total_distance += cv::norm(corners[idx1] - corners[idx2]);
                    count++;
                }
            }
            chars.avg_corner_distance = count > 0 ? total_distance / count : 0;
            chars.is_wide_angle = chars.estimated_fov > 80.0f;
            chars.quality_score = evaluateImageQuality(frame, corners);
        }
    }

    CameraCharacteristics getCameraCharacteristics(const std::string& cam_id) const {
        auto it = camera_chars_.find(cam_id);
        if(it != camera_chars_.end()) return it->second;
        return CameraCharacteristics{};
    }

    std::vector<std::vector<std::string>> getCompatibleGroups(float fov_tolerance = 15.0f,
                                                              float distance_tolerance = 10.0f) {
        std::vector<std::vector<std::string>> groups;
        std::set<std::string> processed;

        for(auto& [id1, chars1] : camera_chars_) {
            if(processed.count(id1)) continue;

            std::vector<std::string> group;
            group.push_back(id1);
            processed.insert(id1);

            for(auto& [id2, chars2] : camera_chars_) {
                if(processed.count(id2) || id1 == id2) continue;

                float fov_diff = std::fabs(chars1.estimated_fov - chars2.estimated_fov);
                float dist_diff = std::fabs(chars1.avg_corner_distance - chars2.avg_corner_distance);
                bool same_wide_angle = chars1.is_wide_angle == chars2.is_wide_angle;

                if(fov_diff <= fov_tolerance && dist_diff <= distance_tolerance && same_wide_angle) {
                    group.push_back(id2);
                    processed.insert(id2);
                }
            }

            if(group.size() >= 2) {
                groups.push_back(group);
            }
    }

        return groups;
    }
};


// -----------------------------------------------------------------------------
// Watcher that processes recorded videos and runs calibration extraction.
// Scans the provided directory for completed .avi files and invokes the
// standalone calibration utility on each of them.
/*
class CalibrationWatcher {
public:
    // Returns true on success, false on failure and fills `err` with a message
    // describing the problem.
    static bool processAll(const std::string &dir, std::string &err) {
        namespace fs = std::filesystem;
        try {
            if (!fs::exists(dir)) return true; // nothing to do

            for (auto &entry : fs::directory_iterator(dir)) {
                if (!entry.is_regular_file()) continue;
                auto path = entry.path();
                if (path.extension() != ".avi") continue;

                // Consider file complete if it hasn't been modified recently
                auto now = fs::file_time_type::clock::now();
                auto age = now - fs::last_write_time(path);
                if (age < std::chrono::seconds(1)) continue;

                fs::path outdir = fs::path("calibration") / "mono" / path.stem();
                fs::create_directories(outdir);

                std::string cmd = "calibration_main extract_mono \"" +
                                   path.string() + "\" \"" +
                                   outdir.string() + "\"";
                int rc = std::system(cmd.c_str());
                if (rc != 0) {
                    err = "calibration command failed";
                    return false;
                }
            }
        } catch (const std::exception &e) {
            err = e.what();
            return false;
        }
        return true;
    }
};
*/

static std::filesystem::path g_exe_dir;
static std::filesystem::path g_config_path;
static bool fileExists(const std::string& p){ struct stat st{}; return stat(p.c_str(), &st)==0; }
static bool dirExists(const std::string& p){ struct stat st{}; return stat(p.c_str(), &st)==0 && S_ISDIR(st.st_mode); }
static json readMainConfig(){
    json cfg=json::object();
    auto p=std::filesystem::absolute(g_config_path);
    printf("readMainConfig path: %s\n",p.c_str());
    std::ifstream f(p);
    if(f){ try{f>>cfg;}catch(...){} }
    return cfg;
}
static bool writeMainConfig(const json& j){
    auto file=std::filesystem::absolute(g_config_path);
    auto dir=file.parent_path();
    printf("writeMainConfig path: %s\n",file.c_str());
    if(mkdir(dir.c_str(),0755)!=0 && errno!=EEXIST) return false;
    std::ofstream f(file);
    if(!f) return false;
    f<<j.dump(2);
    return f.good();
}
static std::string deviceForCam(const std::string& id){ auto cfg=readMainConfig(); if(cfg.contains("cameras")) for(auto& c:cfg["cameras"]) if(c.value("id","")==id) return c.value("device",""); return ""; }

static std::string camIdForDevice(const std::string& dev){
    auto cfg = readMainConfig();
    if(cfg.contains("cameras"))
        for(auto& c:cfg["cameras"])
            if(c.value("device","") == dev)
                return c.value("id","");
    return "";
}

// Simple centroid-based tracker to provide stable IDs across frames
// Simple tracker with basic re-identification using color similarity
struct Track {
//    int id;
//    image_rect_t box;
//    int misses;
    int id;              // unique identifier
    image_rect_t box;    // last known bounding box
    int misses;          // number of consecutive misses while active
    float color[3];      // average RGB color inside the box
    int cls;             // object class id
};


class SimpleTracker {
    int next_id = 0;
//    std::vector<Track> tracks;
//    float max_dist = 50.0f; // pixels
    std::vector<Track> tracks;      // active tracks
    std::vector<Track> lost;        // recently lost tracks that may reappear
    float max_dist = 100.0f;        // max distance for active match (pixels)
    float reid_dist = 120.0f;       // max distance for re-id
    float color_thresh = 40.0f;     // max avg color difference for re-id
    int max_misses = 30;            // frames before track becomes "lost"
    int max_lost_age = 150;         // how long to keep lost track for re-id

    static void avgColor(const image_buffer_t* img, const image_rect_t& b, float out[3]) {
        int x1 = std::max(0, b.left);
        int y1 = std::max(0, b.top);
        int x2 = std::min(img->width - 1, b.right - 1);
        int y2 = std::min(img->height - 1, b.bottom - 1);
        long r = 0, g = 0, bsum = 0; int cnt = 0;
        for (int y = y1; y <= y2; ++y) {
            unsigned char* row = img->virt_addr + y * img->width * 3;
            for (int x = x1; x <= x2; ++x) {
                unsigned char* px = row + x * 3;
                r += px[0]; g += px[1]; bsum += px[2];
                cnt++;
            }
        }
        if (cnt == 0) cnt = 1;
        out[0] = r / (float)cnt; out[1] = g / (float)cnt; out[2] = bsum / (float)cnt;
    }

    static float colorDiff(const float a[3], const float b[3]) {
        return std::fabs(a[0]-b[0]) + std::fabs(a[1]-b[1]) + std::fabs(a[2]-b[2]);
    }

public:
//    void update(object_detect_result_list* dets) {
    // Update tracks using current detections and image for color features
    void update(object_detect_result_list* dets, const image_buffer_t* img) {
        std::vector<bool> assigned(dets->count, false);
        // reset ids
        for (int i = 0; i < dets->count; ++i) dets->results[i].track_id = -1;
        // match existing tracks

        // match with active tracks
        for (auto& t : tracks) {
            int best = -1; float best_d = max_dist;
            float tcx = (t.box.left + t.box.right) / 2.0f;
            float tcy = (t.box.top + t.box.bottom) / 2.0f;
            for (int i = 0; i < dets->count; ++i) if (!assigned[i]) {
                auto& b = dets->results[i].box;
                float dcx = (b.left + b.right) / 2.0f;
                float dcy = (b.top + b.bottom) / 2.0f;
                float d = std::hypot(tcx - dcx, tcy - dcy);
                if (d < best_d) { best_d = d; best = i; }
            }
            if (best != -1) {
//                t.box = dets->results[best].box;
                auto& det = dets->results[best];
                t.box = det.box;
                t.misses = 0;
//                dets->results[best].track_id = t.id;
                t.cls = det.cls_id;
                avgColor(img, det.box, t.color);
                det.track_id = t.id;
                assigned[best] = true;
            } else {
                t.misses++;
            }
        }
        // remove lost tracks
//        tracks.erase(std::remove_if(tracks.begin(), tracks.end(),
//                    [](const Track& t){ return t.misses > 30; }), tracks.end());
        // add new tracks for unmatched detections

        // move expired active tracks to lost list
        auto it = tracks.begin();
        while (it != tracks.end()) {
            if (it->misses > max_misses) {
                it->misses = 0; // reuse as age in lost list
                lost.push_back(*it);
                it = tracks.erase(it);
            } else {
                ++it;
            }
        }

        // attempt to re-id lost tracks
        for (int i = 0; i < dets->count; ++i) if (!assigned[i]) {
            auto& det = dets->results[i];
            float col[3];
            avgColor(img, det.box, col);
            int best = -1; float best_d = reid_dist; float best_c = color_thresh;
            float dcx = (det.box.left + det.box.right) / 2.0f;
            float dcy = (det.box.top + det.box.bottom) / 2.0f;
            for (size_t j = 0; j < lost.size(); ++j) {
                auto& lt = lost[j];
                if (lt.cls != det.cls_id) continue;
                float tcx = (lt.box.left + lt.box.right) / 2.0f;
                float tcy = (lt.box.top + lt.box.bottom) / 2.0f;
                float dist = std::hypot(tcx - dcx, tcy - dcy);
                float cdist = colorDiff(lt.color, col);
                if (dist < best_d && cdist < best_c) { best_d = dist; best_c = cdist; best = j; }
            }
            if (best != -1) {
                // reactivate track
                Track t = lost[best];
                t.box = det.box;
                t.cls = det.cls_id;
                std::copy(col, col+3, t.color);
                t.misses = 0;
                det.track_id = t.id;
                tracks.push_back(t);
                lost.erase(lost.begin() + best);
                assigned[i] = true;
            }
        }

        // add new tracks for remaining detections
        for (int i = 0; i < dets->count; ++i) if (!assigned[i]) {
//            Track t{next_id++, dets->results[i].box, 0};
//            dets->results[i].track_id = t.id;
            auto& det = dets->results[i];
            Track t{};
            t.id = next_id++;
            t.box = det.box;
            t.misses = 0;
            t.cls = det.cls_id;
            avgColor(img, det.box, t.color);
            det.track_id = t.id;
            tracks.push_back(t);
        }

        // age lost tracks and drop old ones
        auto lit = lost.begin();
        while (lit != lost.end()) {
            lit->misses++;
            if (lit->misses > max_lost_age) lit = lost.erase(lit);
            else ++lit;
        }
    }
    bool getColor(int id, float out[3]) const {
        for (auto& t : tracks) if (t.id == id) { out[0]=t.color[0]; out[1]=t.color[1]; out[2]=t.color[2]; return true; }
        for (auto& t : lost)   if (t.id == id) { out[0]=t.color[0]; out[1]=t.color[1]; out[2]=t.color[2]; return true; }
        return false;
    }
};


// ---------- CLI ----------
struct Args {
    std::string model;
    std::string labels = "model/coco_80_labels_list.txt";
    std::string dev = "/dev/video0";
    int port = 8080;
    int cap_w = 640, cap_h = 480;
    int cap_fps = 30;
    int buffers = 3;
    int jpeg_q = 70;
    int http_fps_limit = 0;
    bool show_fps = false;
    bool draw = true;
    std::string npu_core = "auto"; // auto|0|1|2|01|012
    std::string log_file;
    std::string config;

    // Video recording options
    std::string record_dir = "./rec";
    int record_seconds = 180;  // 3 minutes default
    bool no_record = false;
};

static bool parseSize(const std::string& s, int& w, int& h) {
    auto x = s.find('x');
    if (x == std::string::npos) return false;
    try { w = std::stoi(s.substr(0, x)); h = std::stoi(s.substr(x+1)); return w>0 && h>0; }
    catch (...) { return false; }
}

static bool isNumber(const char* s) {
    if (!s || !*s) return false;
    for (const char* p = s; *p; ++p) if (*p < '0' || *p > '9') return false;
    return true;
}

static Args parseArgs(int argc, char** argv) {
    Args a;
    if (argc >= 2) a.model = argv[1];

    // совместимость: если 2-й позиционный — число, это порт; если /dev/video*, это dev
    if (argc >= 3 && argv[2][0] != '-') {
        std::string s2 = argv[2];
        if (s2.rfind("/dev/video", 0) == 0) a.dev = s2;
        else if (isNumber(argv[2])) a.port = atoi(argv[2]);
    }
    // остальные — только именованные
    for (int i = 2; i < argc; ++i) {
        std::string k = argv[i];
        auto need = [&](int more){ return i+more < argc; };
        if (k == "--dev" && need(1)) a.dev = argv[++i];
        else if (k == "--port" && need(1) && isNumber(argv[i+1])) a.port = atoi(argv[++i]);
        else if (k == "--size" && need(1)) { int w=0,h=0; if (parseSize(argv[++i], w, h)){ a.cap_w=w; a.cap_h=h; } }
        else if (k == "--cap-fps" && need(1)) a.cap_fps = atoi(argv[++i]);
        else if (k == "--buffers" && need(1)) a.buffers = std::max(1, atoi(argv[++i]));
        else if (k == "--jpeg-quality" && need(1)) a.jpeg_q = std::max(30, std::min(95, atoi(argv[++i])));
        else if (k == "--http-fps-limit" && need(1)) a.http_fps_limit = std::max(0, atoi(argv[++i]));
        else if (k == "--fps") a.show_fps = true;
        else if (k == "--no-draw") a.draw = false;
        else if (k == "--npu-core" && need(1)) a.npu_core = argv[++i]; // auto|0|1|2|01|012
        else if (k == "--log-file" && need(1)) a.log_file = argv[++i];
        else if (k == "--labels" && need(1)) a.labels = argv[++i];
        else if (k == "--config" && need(1)) a.config = argv[++i];
        else if (k == "--record-dir" && need(1)) a.record_dir = argv[++i];
        else if (k == "--record-seconds" && need(1)) a.record_seconds = std::max(1, atoi(argv[++i]));
        else if (k == "--no-record") a.no_record = true;
    }
    return a;
}

// ---------- utils ----------
static inline const char* fourcc_to_str(__u32 f, char s[5]) {
    s[0] = (char)(f & 0xFF);
    s[1] = (char)((f >> 8) & 0xFF);
    s[2] = (char)((f >> 16) & 0xFF);
    s[3] = (char)((f >> 24) & 0xFF);
    s[4] = 0;
    return s;
}

static rknn_core_mask npu_mask_from_string(const std::string& s) {
    if (s == "auto") return RKNN_NPU_CORE_AUTO;
    if (s == "0")    return RKNN_NPU_CORE_0;
    if (s == "1")    return RKNN_NPU_CORE_1;
    if (s == "2")    return RKNN_NPU_CORE_2;
    if (s == "01" || s == "0_1")                 return RKNN_NPU_CORE_0_1;
    if (s == "012" || s == "0_1_2" || s == "all") return RKNN_NPU_CORE_0_1_2;
    return RKNN_NPU_CORE_AUTO;
}

// ---------- server ----------
class YOLOWebServer {
private:
    rknn_app_context_t rknn_app_ctx{};
    Server server;
    bool model_initialized = false;

    // cfg
    std::string model_path;
    std::string labels_path;
    int server_port = 8080;
    std::string cam_dev = "/dev/video0";
    int cam_w_req=640, cam_h_req=480, cam_fps_req=30, cam_buffers=3;
    int jpeg_q = 70;
    int http_fps_limit = 0;
    bool show_fps = false;
    bool draw = true;
    rknn_core_mask npu_mask = RKNN_NPU_CORE_AUTO;

    // camera
    struct CamBuffer { void* start=nullptr; size_t length=0; };
    int cam_fd = -1;
    std::vector<CamBuffer> cam_bufs;
    std::thread cam_thread{};
    std::atomic<bool> cam_running{false};
    int cam_w=0, cam_h=0, cam_fps=0;
    std::string cam_id_;

    // shared
    std::mutex infer_mtx;
    std::mutex frame_mtx;
    std::condition_variable frame_cv;
    std::vector<uint8_t> last_jpeg;
    json last_meta;
    SimpleTracker tracker; // maintains unique object IDs

    // video recording
    std::unique_ptr<cv::VideoWriter> video_writer_;
    std::string record_dir_ = "./rec";
    int record_seconds_ = 180;
    bool record_enabled_ = false;  // ИЗМЕНЕНО: теперь false по умолчанию
    std::chrono::steady_clock::time_point record_start_time_;
    bool recording_active_ = false;
    std::string record_filename_prefix_;

    // video recording control
    std::atomic<bool> should_start_recording_{false};
    std::atomic<bool> should_stop_recording_{false};
    int recording_duration_seconds_ = 30;

    // preview timing
    std::chrono::steady_clock::time_point last_preview_update_;
    const std::chrono::milliseconds preview_interval_{300};


    // stereo config
    struct StereoPairCfg { int a=0; int b=0; std::string file; };
    std::vector<StereoPairCfg> stereo_pairs;
    json stereo_cfg;

   // calibration manager
    CameraManager cam_mgr_{};
    bool preview_flag_{true};
    std::filesystem::path calib_root_;

    class CameraPauseGuard {
    public:
        explicit CameraPauseGuard(YOLOWebServer &server)
            : server_(server) {
            was_running_ = server_.pauseForCalibration();
        }

        CameraPauseGuard(const CameraPauseGuard&) = delete;
        CameraPauseGuard& operator=(const CameraPauseGuard&) = delete;

        bool wasRunning() const { return was_running_; }

        ~CameraPauseGuard() {
            if (was_running_) {
                server_.resumeAfterCalibration();
            }
        }

    private:
        YOLOWebServer &server_;
        bool was_running_{false};
    };

    enum class LiveCalibrationMode { None, Mono, Stereo };

    class ActiveCalibratorManager {
    public:
        using MonoPtr = std::unique_ptr<calibration::MonoCalibrator>;
        using StereoPtr = std::unique_ptr<calibration::StereoCalibrator>;

        bool isActive() const { return mode_ != LiveCalibrationMode::None; }
        LiveCalibrationMode mode() const { return mode_; }
        const calibration::CalibConfig &config() const { return config_; }
        const std::string &cameraPrimary() const { return camera_primary_; }
        const std::string &cameraSecondary() const { return camera_secondary_; }

        void setMono(MonoPtr mono, const calibration::CalibConfig &config,
                     std::unique_ptr<CameraPauseGuard> guard, std::string camera_id) {
            reset();
            calibrator_.template emplace<MonoPtr>(std::move(mono));
            mode_ = LiveCalibrationMode::Mono;
            config_ = config;
            camera_primary_ = std::move(camera_id);
            camera_secondary_.clear();
            guard_ = std::move(guard);
            progress_current_ = 0;
            progress_max_ = config_.max_frames;
            pattern_visible_ = false;
        }

        void setStereo(StereoPtr stereo, const calibration::CalibConfig &config,
                      std::unique_ptr<CameraPauseGuard> guard,
                      std::string camera_a, std::string camera_b) {
            reset();
            calibrator_.template emplace<StereoPtr>(std::move(stereo));
            mode_ = LiveCalibrationMode::Stereo;
            config_ = config;
            camera_primary_ = std::move(camera_a);
            camera_secondary_ = std::move(camera_b);
            guard_ = std::move(guard);
            progress_current_ = 0;
            progress_max_ = config_.max_frames;
            pattern_visible_ = false;
        }


        calibration::MonoCalibrator *mono() {
            if (auto ptr = std::get_if<MonoPtr>(&calibrator_)) {
                return ptr->get();
            }
            return nullptr;
        }

        calibration::StereoCalibrator *stereo() {
            if (auto ptr = std::get_if<StereoPtr>(&calibrator_)) {
                return ptr->get();
            }
            return nullptr;
        }

// В yolov8_web_server_calibration.cc, внутри класса ActiveCalibratorManager
void updateFromFrame(const cv::Mat& frame) {
    if (!isActive()) return;
    
    std::string hint;
    int progress_current = 0;
    int progress_max = 0;
    bool pattern_visible = false;
    
    if (auto* mono = this->mono()) {
        mono->getFrame(const_cast<cv::Mat&>(frame), hint, 
                      progress_current, progress_max, pattern_visible);
        setProgress(progress_current, progress_max, pattern_visible);
        setHint(hint);
    } else if (auto* stereo = this->stereo()) {
        // Для стерео нужны два кадра, пока пропускаем
    }
}

        void setHint(std::string hint) { last_hint_ = std::move(hint); }
        const std::string &lastHint() const { return last_hint_; }

        void setProgress(int current, int max, bool pattern_visible) {
            progress_current_ = current;
            progress_max_ = max;
            pattern_visible_ = pattern_visible;
        }

        void refreshProgressFromCalibrator() {
            if (auto *mono_ptr = mono()) {
                progress_current_ = mono_ptr->framesCollected();
                progress_max_ = config_.max_frames;
                pattern_visible_ =
                    progress_max_ > 0 && progress_current_ >= progress_max_;
            } else if (auto *stereo_ptr = stereo()) {
                progress_current_ = stereo_ptr->framesCollected();
                progress_max_ = config_.max_frames;
                pattern_visible_ =
                    progress_max_ > 0 && progress_current_ >= progress_max_;
            }
        }


        int progressCurrent() const { return progress_current_; }
        int progressMax() const { return progress_max_; }
        bool patternVisible() const { return pattern_visible_; }

        void setVideoActive(bool active) { video_active_ = active; }
        bool videoActive() const { return video_active_; }

        void setComputeInProgress(bool value) { compute_in_progress_ = value; }
        bool computeInProgress() const { return compute_in_progress_; }

        void setLastError(std::string err) { last_error_ = std::move(err); }
        const std::string &lastError() const { return last_error_; }

        void setResults(std::vector<calibration::MonoCalibrationSummary> mono,
                        std::vector<calibration::StereoCalibrationSummary> stereo) {
            mono_results_ = std::move(mono);
            stereo_results_ = std::move(stereo);
        }

        const std::vector<calibration::MonoCalibrationSummary> &monoResults() const {
            return mono_results_;
        }

        const std::vector<calibration::StereoCalibrationSummary> &stereoResults() const {
            return stereo_results_;
        }

        struct ReleasedCalibrators {
            MonoPtr mono;
            StereoPtr stereo;
            std::unique_ptr<CameraPauseGuard> guard;
            LiveCalibrationMode mode{LiveCalibrationMode::None};
        };

        ReleasedCalibrators releaseCalibrators() {
            ReleasedCalibrators released;
            released.mode = mode_;
            if (auto ptr = std::get_if<MonoPtr>(&calibrator_)) {
                released.mono = std::move(*ptr);
            } else if (auto ptr = std::get_if<StereoPtr>(&calibrator_)) {
                released.stereo = std::move(*ptr);
            }
            released.guard = std::move(guard_);
            reset();
            return released;
        }

        void reset() {
            calibrator_.template emplace<std::monostate>();
            guard_.reset();
            mode_ = LiveCalibrationMode::None;
            camera_primary_.clear();
            camera_secondary_.clear();
            last_hint_.clear();
            progress_current_ = 0;
            progress_max_ = 0;
            pattern_visible_ = false;
            video_active_ = false;
            compute_in_progress_ = false;
            last_error_.clear();
            mono_results_.clear();
            stereo_results_.clear();
        }

    private:
        std::variant<std::monostate, MonoPtr, StereoPtr> calibrator_{};
        LiveCalibrationMode mode_{LiveCalibrationMode::None};
        calibration::CalibConfig config_{};
        std::string camera_primary_;
        std::string camera_secondary_;
        std::unique_ptr<CameraPauseGuard> guard_{};
        std::string last_hint_;
        int progress_current_{0};
        int progress_max_{0};
        bool pattern_visible_{false};
        bool video_active_{false};
        bool compute_in_progress_{false};
        std::string last_error_;
        std::vector<calibration::MonoCalibrationSummary> mono_results_;
        std::vector<calibration::StereoCalibrationSummary> stereo_results_;
    };

    class CalibrationResultsManager {
    public:
        void setResultsDirectory(std::filesystem::path dir) {
            results_dir_ = std::move(dir);
        }

        void reload() {
            mono_.clear();
            stereo_.clear();
            if (results_dir_.empty()) {
                return;
            }
            std::error_code ec;
            if (!std::filesystem::exists(results_dir_, ec)) {
                return;
            }
            for (const auto &entry : std::filesystem::directory_iterator(results_dir_, ec)) {
                if (ec) break;
                if (!entry.is_regular_file()) continue;
                auto ext = entry.path().extension().string();
                if (ext != ".yml" && ext != ".yaml") continue;
                auto stem = entry.path().stem().string();
                if (stem.rfind("cam_", 0) == 0) {
                    loadMonoFile(entry.path());
                } else if (stem.rfind("stereo_", 0) == 0) {
                    loadStereoFile(entry.path());
                }
            }
        }

        bool hasMonoCalibration(const std::string &camera_id) const {
            return mono_.count(camera_id) > 0;
        }

        float measureMonoDistance(const std::string &camera_id, const cv::Rect &bbox,
                                  float real_height_mm = 1700.0f) const {
            if (bbox.height <= 0) {
                return -1.0f;
            }
            auto it = mono_.find(camera_id);
            if (it == mono_.end()) {
                return -1.0f;
            }
            const cv::Mat &camera_matrix = it->second.camera_matrix;
            if (camera_matrix.empty()) {
                return -1.0f;
            }
            double fy = 0.0;
            if (camera_matrix.type() == CV_64F || camera_matrix.type() == CV_64FC1) {
                fy = camera_matrix.at<double>(1, 1);
            } else {
                fy = camera_matrix.at<float>(1, 1);
            }
            if (fy <= 0.0) {
                return -1.0f;
            }
            return static_cast<float>((real_height_mm * fy) / static_cast<double>(bbox.height));
        }

    private:
        struct MonoData {
            calibration::MonoCalibrationSummary summary;
            cv::Mat camera_matrix;
            cv::Mat dist_coeffs;
        };

        struct StereoData {
            calibration::StereoCalibrationSummary summary;
        };

        void loadMonoFile(const std::filesystem::path &path) {
            cv::FileStorage fs(path.string(), cv::FileStorage::READ);
            if (!fs.isOpened()) {
                return;
            }

            MonoData data;
            data.summary.output_file = path;
            auto stem = path.stem().string();
            if (stem.rfind("cam_", 0) == 0) {
                data.summary.camera_id = stem.substr(4);
            }
            fs["camera_matrix"] >> data.camera_matrix;
            if (!data.camera_matrix.empty() && data.camera_matrix.type() != CV_64F) {
                data.camera_matrix.convertTo(data.camera_matrix, CV_64F);
            }
            fs["distortion_coefficients"] >> data.dist_coeffs;
            if (!data.dist_coeffs.empty() && data.dist_coeffs.type() != CV_64F) {
                data.dist_coeffs.convertTo(data.dist_coeffs, CV_64F);
            }
            data.summary.image_size.width = static_cast<int>(fs["image_width"]);
            data.summary.image_size.height = static_cast<int>(fs["image_height"]);
            data.summary.reprojection_error = static_cast<double>(fs["reprojection_error"]);
            data.summary.frames_used = static_cast<int>(fs["frames_used"]);
            data.summary.calibration_time = static_cast<std::string>(fs["calibration_time"]);

            auto per_view = fs["per_view_errors"];
            if (!per_view.empty() && per_view.isSeq()) {
                for (const auto &node : per_view) {
                    data.summary.per_view_errors.push_back(static_cast<double>(node));
                }
            }

            mono_[data.summary.camera_id] = std::move(data);
        }

        void loadStereoFile(const std::filesystem::path &path) {
            cv::FileStorage fs(path.string(), cv::FileStorage::READ);
            if (!fs.isOpened()) {
                return;
            }

            StereoData data;
            data.summary.output_file = path;
            auto stem = path.stem().string();
            if (stem.rfind("stereo_", 0) == 0) {
                auto rest = stem.substr(7);
                auto pos = rest.find('_');
                if (pos != std::string::npos) {
                    data.summary.camera_a = rest.substr(0, pos);
                    data.summary.camera_b = rest.substr(pos + 1);
                }
            }

            fs["R"] >> data.summary.R;
            fs["T"] >> data.summary.T;
            fs["E"] >> data.summary.E;
            fs["F"] >> data.summary.F;
            fs["R1"] >> data.summary.R1;
            fs["R2"] >> data.summary.R2;
            fs["P1"] >> data.summary.P1;
            fs["P2"] >> data.summary.P2;
            fs["Q"] >> data.summary.Q;
            data.summary.reprojection_error = static_cast<double>(fs["stereo_rms_error"]);
            data.summary.frames_used = static_cast<int>(fs["frames_used"]);
            data.summary.calibration_time = static_cast<std::string>(fs["calibration_time"]);

            if (!data.summary.R.empty() && data.summary.R.type() != CV_64F) data.summary.R.convertTo(data.summary.R, CV_64F);
            if (!data.summary.T.empty() && data.summary.T.type() != CV_64F) data.summary.T.convertTo(data.summary.T, CV_64F);
            if (!data.summary.E.empty() && data.summary.E.type() != CV_64F) data.summary.E.convertTo(data.summary.E, CV_64F);
            if (!data.summary.F.empty() && data.summary.F.type() != CV_64F) data.summary.F.convertTo(data.summary.F, CV_64F);
            if (!data.summary.R1.empty() && data.summary.R1.type() != CV_64F) data.summary.R1.convertTo(data.summary.R1, CV_64F);
            if (!data.summary.R2.empty() && data.summary.R2.type() != CV_64F) data.summary.R2.convertTo(data.summary.R2, CV_64F);
            if (!data.summary.P1.empty() && data.summary.P1.type() != CV_64F) data.summary.P1.convertTo(data.summary.P1, CV_64F);
            if (!data.summary.P2.empty() && data.summary.P2.type() != CV_64F) data.summary.P2.convertTo(data.summary.P2, CV_64F);
            if (!data.summary.Q.empty() && data.summary.Q.type() != CV_64F) data.summary.Q.convertTo(data.summary.Q, CV_64F);

            stereo_[data.summary.camera_a + "_" + data.summary.camera_b] = std::move(data);
        }

        std::filesystem::path results_dir_{};
        std::map<std::string, MonoData> mono_;
        std::map<std::string, StereoData> stereo_;
    };

    mutable std::mutex live_calib_mutex_;
    ActiveCalibratorManager live_calib_manager_{};
    CalibrationResultsManager calibration_results_manager_{};

  // logging
    bool log_enabled = false;
    std::string log_base;
    std::set<int> logged_ids;
    std::map<int,int> minute_counts; // class_id -> count
    std::chrono::system_clock::time_point minute_start{std::chrono::system_clock::now()};
    int event_counter = 0;

    void rotateLogs() {
        DIR* dir = opendir("/tmp");
        if (!dir) return;
        auto now = std::time(nullptr);
        struct dirent* ent;
        while ((ent = readdir(dir)) != nullptr) {
            std::string name = ent->d_name;
            if (name.rfind("npudet.", 0) == 0) {
                std::string path = std::string("/tmp/") + name;
                struct stat st{};
                if (stat(path.c_str(), &st) == 0) {
                    if (now - st.st_mtime > 24*3600*10) unlink(path.c_str());
                }
            }
        }
        closedir(dir);
    }

    std::string currentLogPath() const {
        auto t = std::time(nullptr);
        char datebuf[16];
        std::strftime(datebuf, sizeof(datebuf), "%d-%m-%Y", std::localtime(&t));
        std::string base = log_base;
        auto pos = base.find_last_of('/');
        if (pos != std::string::npos) base = base.substr(pos+1);
        return std::string("/tmp/npudet.") + datebuf + "." + base;
    }

    void appendLog(const std::string& line) {
        if (!log_enabled) return;
        std::ofstream ofs(currentLogPath(), std::ios::app);
        if (ofs) ofs << line << '\n';
    }

    void logMinuteSummary(const std::chrono::system_clock::time_point& now) {
        if (minute_counts.empty()) return;
        auto start_t = std::chrono::system_clock::to_time_t(minute_start);
        auto end_t = std::chrono::system_clock::to_time_t(now);
        char datebuf[16], sbuf[16], ebuf[16];
        std::strftime(datebuf, sizeof(datebuf), "%d-%m-%Y", std::localtime(&start_t));
        std::strftime(sbuf, sizeof(sbuf), "%H:%M:%S", std::localtime(&start_t));
        std::strftime(ebuf, sizeof(ebuf), "%H:%M:%S", std::localtime(&end_t));
        for (auto& kv : minute_counts) {
            std::string line = std::to_string(kv.second) + " " + coco_cls_to_name(kv.first) + ", " +
                               datebuf + ", " + sbuf + " - " + ebuf;
            appendLog(line);
        }
        minute_counts.clear();
        minute_start = now;
    }


    void parseStereoConfig() {
        stereo_pairs.clear();
        if (stereo_cfg.contains("pairs") && stereo_cfg["pairs"].is_array()) {
            for (auto &p : stereo_cfg["pairs"]) {
                StereoPairCfg sp;
                sp.a = p.value("a", 0);
                sp.b = p.value("b", 0);
                sp.file = p.value("file", std::string());
                stereo_pairs.push_back(sp);
            }
        }
    }

    void loadStereoConfig() {
        auto file = g_config_path.parent_path() / "stereo_config.json";
        std::ifstream f(file);
        if (f) {
            try { f >> stereo_cfg; } catch (...) { stereo_cfg = json::object(); }
        } else {
            stereo_cfg = json::object();
        }
        parseStereoConfig();
    }

    void saveStereoConfig() {
        auto file = g_config_path.parent_path() / "stereo_config.json";
        std::error_code ec;
        std::filesystem::create_directories(file.parent_path(), ec);
        std::ofstream f(file);
        if (f) f << stereo_cfg.dump(2);
    }

    bool getFrameFromCamera(const std::string& cam_id, std::vector<uint8_t>& jpeg_out) {
        printf("Attempting to get frame from camera: %s\n", cam_id.c_str());

        CameraManager::Frame frame;
        uint64_t timestamp = cam_mgr_.nowMonoNs();

    if (!cam_mgr_.getFrame(cam_id, timestamp, frame)) {
        printf("Failed to get frame from camera: %s\n", cam_id.c_str());
        return false;
        }
        jpeg_out = frame.jpeg;
        printf("Successfully got frame from camera %s: %zu bytes\n", cam_id.c_str(), jpeg_out.size());
    return true;
    }

    void createStereoPairs(const std::vector<std::string>& camera_ids,
                           const std::string& stereo_mode,
                           int capture_index,
                           const std::string& timestamp,
                           const std::filesystem::path& calib_root) {
        std::vector<std::pair<std::string,std::string>> pairs;

        if(stereo_mode == "adjacent") {
            for(size_t i=0;i+1<camera_ids.size();++i)
                pairs.emplace_back(camera_ids[i], camera_ids[i+1]);
        } else {
            for(size_t i=0;i<camera_ids.size();++i)
                for(size_t j=i+1;j<camera_ids.size();++j)
                    pairs.emplace_back(camera_ids[i], camera_ids[j]);
        }

        for(const auto& pr : pairs) {
            std::string pair_name = "cam_" + pr.first + "_" + pr.second;
            std::filesystem::path stereo_dir = calib_root / "stereo" / pair_name;
            std::filesystem::create_directories(stereo_dir);

            char src_filename[256], dst_filename[256];
            snprintf(src_filename, sizeof(src_filename), "calib_%03d_%s.jpg",
                     capture_index, timestamp.c_str());

            snprintf(dst_filename, sizeof(dst_filename), "left_%03d_%s.jpg",
                     capture_index, timestamp.c_str());
            std::filesystem::path src_left = calib_root / "mono" / ("cam_" + pr.first) / src_filename;
            std::filesystem::path dst_left = stereo_dir / dst_filename;
            std::filesystem::copy_file(src_left, dst_left,
                    std::filesystem::copy_options::overwrite_existing);

            snprintf(dst_filename, sizeof(dst_filename), "right_%03d_%s.jpg",
                     capture_index, timestamp.c_str());
            std::filesystem::path src_right = calib_root / "mono" / ("cam_" + pr.second) / src_filename;
            std::filesystem::path dst_right = stereo_dir / dst_filename;
            std::filesystem::copy_file(src_right, dst_right,
                    std::filesystem::copy_options::overwrite_existing);
        }
    }


public:
    YOLOWebServer(const Args& a)
        : model_path(a.model), labels_path(a.labels), server_port(a.port),
          cam_dev(a.dev), cam_w_req(a.cap_w), cam_h_req(a.cap_h),
          cam_fps_req(a.cap_fps), cam_buffers(a.buffers),
          jpeg_q(a.jpeg_q), http_fps_limit(a.http_fps_limit),
          show_fps(a.show_fps), draw(a.draw),
          npu_mask(npu_mask_from_string(a.npu_core)),
          log_enabled(!a.log_file.empty()), log_base(a.log_file),
//          record_dir_(a.record_dir), record_seconds_(a.record_seconds),
          record_dir_("/tmp/rec"), record_seconds_(a.record_seconds),
          record_enabled_(false),
          last_preview_update_(std::chrono::steady_clock::now()),
          calib_root_(std::filesystem::absolute(
              readMainConfig().value("calib_root", "."))) {
        if (log_enabled) rotateLogs();
        cam_mgr_.loadConfig(g_config_path.string());
        cam_mgr_.start(false);
        cam_id_ = camIdForDevice(cam_dev);

        calibration_results_manager_.setResultsDirectory(calib_root_ / "calibration" / "results");
        calibration_results_manager_.reload();
        }


    ~YOLOWebServer() {
        cleanup();
        cam_mgr_.stop();
    }

    bool initialize() {
        if (init_post_process(labels_path.c_str()) != 0) {
            fprintf(stderr, "init_post_process failed\n");
            return false;
        }
        if (init_yolov8_model(model_path.c_str(), &rknn_app_ctx) != 0) {
            fprintf(stderr, "init_yolov8_model failed: %s\n", model_path.c_str());
            deinit_post_process();
            return false;
        }
        // Устанавливаем маску NPU по флагу
        {
            int ret_mask = rknn_set_core_mask(rknn_app_ctx.rknn_ctx, npu_mask);
            if (ret_mask != RKNN_SUCC) fprintf(stderr, "warn: rknn_set_core_mask(%d) failed: %d\n", npu_mask, ret_mask);
            else                        fprintf(stderr, "rknn core mask set: %d\n", npu_mask);
        }
        model_initialized = true;

        loadStereoConfig();

        server.set_keep_alive_max_count(100);
        server.set_read_timeout(5, 0);
        server.set_write_timeout(5, 0);
        server.set_payload_max_length(1 * 1024 * 1024);

        setupRoutes();

        if (initCamera()) {
            cam_thread = std::thread(&YOLOWebServer::cameraLoop, this);
        } else {
            fprintf(stderr, "WARN: camera init failed (%s). Stream endpoints -> 503\n", cam_dev.c_str());
        }
        return true;
    }

    void run() {
        printf("Starting YOLOv8 Web Server on port %d\n", server_port);
        printf("UI: http://localhost:%d\n", server_port);
        server.listen("0.0.0.0", server_port);
    }

    void stop() { server.stop(); }

private:
    void cleanup() {
//        server.stop();
        cam_running = false;
        frame_cv.notify_all();

        stopVideoRecording();

        if (cam_thread.joinable()) cam_thread.join();
        deinitCamera();
        if (model_initialized) {
            deinit_post_process();
            release_yolov8_model(&rknn_app_ctx);
            model_initialized = false;
        }
    }

    void pauseCamera() { pauseForCalibration(); }

    bool pauseForCalibration() {
        bool was_running = cam_running.exchange(false);
        frame_cv.notify_all();
        if (cam_thread.joinable()) {
            cam_thread.join();
        }
        deinitCamera();
        return was_running;
    }

    void resumeAfterCalibration() {
        if (initCamera()) {
            cam_thread = std::thread(&YOLOWebServer::cameraLoop, this);
        } else {
            fprintf(stderr, "WARN: failed to resume primary camera after calibration\n");
        }
    }

    static json calibConfigToJson(const calibration::CalibConfig &cfg) {
        json j;
        j["pattern_cols"] = cfg.pattern_cols;
        j["pattern_rows"] = cfg.pattern_rows;
        j["square_size"] = cfg.square_size;
        j["min_frames"] = cfg.min_frames;
        j["max_frames"] = cfg.max_frames;
        j["max_center_diff_horizontal"] = cfg.max_center_diff_horizontal;
        j["max_center_diff_vertical"] = cfg.max_center_diff_vertical;
        j["max_tilt_diff"] = cfg.max_tilt_diff;
        j["min_coverage"] = cfg.min_coverage;
        j["max_coverage"] = cfg.max_coverage;
        j["min_distance_between_frames"] = cfg.min_distance_between_frames;
        return j;
    }

    static json monoSummaryToJson(const calibration::MonoCalibrationSummary &summary) {
        json j;
        j["camera_id"] = summary.camera_id;
        j["output_file"] = summary.output_file.string();
        j["image_width"] = summary.image_size.width;
        j["image_height"] = summary.image_size.height;
        j["frames_used"] = summary.frames_used;
        j["reprojection_error"] = summary.reprojection_error;
        j["per_view_errors"] = summary.per_view_errors;
        j["calibration_time"] = summary.calibration_time;
        return j;
    }

    static json stereoSummaryToJson(const calibration::StereoCalibrationSummary &summary) {
        json j;
        j["camera_a"] = summary.camera_a;
        j["camera_b"] = summary.camera_b;
        j["output_file"] = summary.output_file.string();
        j["frames_used"] = summary.frames_used;
        j["reprojection_error"] = summary.reprojection_error;
        j["calibration_time"] = summary.calibration_time;
        return j;
    }

    static calibration::CalibConfig calibConfigFromJson(const json &j,
                                                        calibration::CalibConfig base = {}) {
        calibration::CalibConfig cfg = base;
        if (!j.is_object()) {
            return cfg;
        }
        cfg.pattern_cols = j.value("pattern_cols", cfg.pattern_cols);
        cfg.pattern_rows = j.value("pattern_rows", cfg.pattern_rows);
        cfg.square_size = j.value("square_size", cfg.square_size);
        cfg.min_frames = j.value("min_frames", cfg.min_frames);
        cfg.max_frames = j.value("max_frames", cfg.max_frames);
        cfg.max_center_diff_horizontal = j.value("max_center_diff_horizontal", cfg.max_center_diff_horizontal);
        cfg.max_center_diff_vertical = j.value("max_center_diff_vertical", cfg.max_center_diff_vertical);
        cfg.max_tilt_diff = j.value("max_tilt_diff", cfg.max_tilt_diff);
        const float recommended_min_coverage = calibration::CalibConfig::recommendedMinCoverage(
            cfg.pattern_cols, cfg.pattern_rows);
        cfg.min_coverage = j.value("min_coverage", recommended_min_coverage);
        cfg.max_coverage = j.value("max_coverage", cfg.max_coverage);
        cfg.min_distance_between_frames = j.value("min_distance_between_frames", cfg.min_distance_between_frames);
        return cfg;
    }

    static const char* modeToString(LiveCalibrationMode mode) {
        switch (mode) {
            case LiveCalibrationMode::Mono: return "mono";
            case LiveCalibrationMode::Stereo: return "stereo";
            default: return "none";
        }
    }

    // ---------- HTTP ----------
    void setupRoutes() {
        server.set_pre_routing_handler([](const Request&, Response& res) {
            res.set_header("Access-Control-Allow-Origin", "*");
            res.set_header("Access-Control-Allow-Methods", "GET, POST, OPTIONS");
            res.set_header("Access-Control-Allow-Headers", "Content-Type, Authorization");
            return Server::HandlerResponse::Unhandled;
        });
        server.Options(".*", [](const Request&, Response&){});
        server.set_mount_point("/", "./web");

        server.Get("/api/health", [this](const Request&, Response& res) {
            json j;
            j["status"] = model_initialized ? "ready" : "not_ready";
            j["model_path"] = model_path;
            j["streaming"] = cam_running.load();
            j["cap_req_w"] = cam_w_req; j["cap_req_h"] = cam_h_req; j["cap_req_fps"] = cam_fps_req;
            j["cap_real_w"] = cam_w; j["cap_real_h"] = cam_h; j["cap_real_fps"] = cam_fps;
            j["jpeg_q"] = jpeg_q;
            j["npu_mask"] = npu_mask;
            res.set_content(j.dump(), "application/json");
        });


        server.Get("/api/status", [this](const Request&, Response& res) {
            json j;
            bool running = cam_running.load();
            j["mode"] = running ? "calibration" : "preview";
            j["detect"] = running && model_initialized;
            res.set_content(j.dump(), "application/json");
        });



        server.Get("/api/configured", [this](const Request&, Response& res) {
            auto cams = cam_mgr_.configuredCameras();
            json out = json::array();
            for (auto &c : cams) {
                out.push_back({
                    {"id", c.id},
                    {"present", c.present},
                    {"mode", c.mode == CameraManager::CamConfig::Mode::Detect
                                   ? "detect"
                                   : c.mode == CameraManager::CamConfig::Mode::Calibration
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
                    {"position",
                     {{"x", c.position.x},
                      {"y", c.position.y},
                      {"z", c.position.z}}},
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
                    {"log_file", c.log_file}
                });
            }
            res.set_content(out.dump(), "application/json");
        });



        server.Get("/api/model-info", [this](const Request&, Response& res) {
            if (!model_initialized) { res.status = 503; res.set_content("{\"error\":\"Model not initialized\"}", "application/json"); return; }
            json j;
            j["model_width"] = rknn_app_ctx.model_width;
            j["model_height"] = rknn_app_ctx.model_height;
            j["model_channel"] = rknn_app_ctx.model_channel;
            j["is_quantized"] = rknn_app_ctx.is_quant;
            j["input_count"] = rknn_app_ctx.io_num.n_input;
            j["output_count"] = rknn_app_ctx.io_num.n_output;
            res.set_content(j.dump(), "application/json");
        });

        server.Post("/api/detect", [](const Request&, Response& res) {
            res.status = 501;
            res.set_content("{\"error\":\"/api/detect disabled; use /api/stream.mjpg\"}", "application/json");
        });

        server.Get("/api/stream.mjpg", [this](const Request&, Response& res) {
            if (!cam_running.load()) { res.status = 503; res.set_content("camera not running", "text/plain"); return; }
            res.set_header("Cache-Control", "no-store, no-cache, must-revalidate");
            res.set_header("Pragma", "no-cache");
            res.set_header("Connection", "keep-alive");
            auto last_push = Clock::now();
            res.set_chunked_content_provider(
                "multipart/x-mixed-replace; boundary=frame",
                [this, last_push](size_t, DataSink& sink) mutable {
                    while (cam_running.load()) {
                        std::vector<uint8_t> jpg;
                        {
                            std::unique_lock<std::mutex> lk(frame_mtx);
                            frame_cv.wait_for(lk, std::chrono::milliseconds(1000),
                                              [this]{ return !last_jpeg.empty() || !cam_running.load(); });
                            if (!cam_running.load()) break;
                            jpg = last_jpeg;
                        }
                        // HTTP FPS limit
                        if (http_fps_limit > 0) {
                            auto now = Clock::now();
                            double elapsed = std::chrono::duration<double>(now - last_push).count();
                            double min_dt = 1.0 / http_fps_limit;
                            if (elapsed < min_dt) {
                                auto sleep_d = std::chrono::duration<double>(min_dt - elapsed);
                                std::this_thread::sleep_for(std::chrono::duration_cast<std::chrono::milliseconds>(sleep_d));
                            }
                            last_push = Clock::now();
                        }

                        std::string header = "--frame\r\n"
                                             "Content-Type: image/jpeg\r\n"
                                             "Content-Length: " + std::to_string(jpg.size()) + "\r\n\r\n";
                        if (!sink.write(header.data(), header.size())) break;
                        if (!sink.write(reinterpret_cast<const char*>(jpg.data()), jpg.size())) break;
                        if (!sink.write("\r\n", 2)) break;
                        std::this_thread::sleep_for(std::chrono::milliseconds(2));
                    }
                    return true;
                },
                [](bool){}
            );
        });

        server.Get("/api/last.json", [this](const Request&, Response& res) {
            json j;
            {
                std::lock_guard<std::mutex> lk(frame_mtx);
                j = last_meta.is_null() ? json::object() : last_meta;
            }
            res.set_content(j.dump(), "application/json");
        });

        server.Get("/api/frame.jpg", [this](const Request&, Response& res) {
            std::vector<uint8_t> jpg;
            {
                std::lock_guard<std::mutex> lk(frame_mtx);
                jpg = last_jpeg;
            }
            if (jpg.empty()) { res.status = 503; res.set_content("no frame", "text/plain"); return; }
            res.set_content(std::string(reinterpret_cast<const char*>(jpg.data()), jpg.size()), "image/jpeg");
        });

        server.Get("/api/stereo-config", [this](const Request&, Response& res) {
            res.set_content(stereo_cfg.dump(), "application/json");
        });

        server.Post("/api/stereo-config", [this](const Request& req, Response& res) {
            try {
                stereo_cfg = json::parse(req.body);
                parseStereoConfig();
                saveStereoConfig();
                res.set_content("{\"status\":\"ok\"}", "application/json");
            } catch (...) {
                res.status = 400;
                res.set_content("{\"error\":\"invalid json\"}", "application/json");
            }
        });

        // ------------------------------------------------------------------
        // Live calibration API (new prefix)
        server.Get("/api/calibration_new/cameras", [this](const Request&, Response& res) {
            json resp = json::object();
            json cams = json::array();
            auto infos = cam_mgr_.configuredCameras();
            for (const auto &ci : infos) {
                if (ci.mode != CameraManager::CamConfig::Mode::Calibration) {
                    continue;
                }
                json cam = json::object();
                cam["id"] = ci.id;
                cam["present"] = ci.present;
                cam["device"] = cam_mgr_.devicePath(ci.id);
                cam["preferred"] = {
                    {"width", ci.preferred.w},
                    {"height", ci.preferred.h},
                    {"fps", ci.preferred.fps}
                };
                cam["role"] = ci.role;
                cams.push_back(std::move(cam));
            }
            {
                std::lock_guard<std::mutex> lk(live_calib_mutex_);
                resp["active"] = live_calib_manager_.isActive();
                resp["mode"] = modeToString(live_calib_manager_.mode());
                json current = json::object();
                if (!live_calib_manager_.cameraPrimary().empty()) {
                    current["camera_a"] = live_calib_manager_.cameraPrimary();
                }
                if (!live_calib_manager_.cameraSecondary().empty()) {
                    current["camera_b"] = live_calib_manager_.cameraSecondary();
                }
                resp["current"] = std::move(current);
                resp["config"] = calibConfigToJson(live_calib_manager_.config());
            }
            resp["defaults"] = calibConfigToJson(calibration::CalibConfig{});
            resp["cameras"] = std::move(cams);
            res.set_content(resp.dump(), "application/json");
        });


        server.Post("/api/calibration_new/start", [this](const Request& req, Response& res) {
            json resp = json::object();
            json body = json::object();
            if (!req.body.empty()) {
                try {
                    body = json::parse(req.body);
                } catch (...) {
                    res.status = 400;
                    resp["status"] = "error";
                    resp["error"] = "invalid json";
                    res.set_content(resp.dump(), "application/json");
                    return;
                }
            }

            {
                std::lock_guard<std::mutex> lk(live_calib_mutex_);
                if (live_calib_manager_.isActive()) {
                    resp["status"] = "error";
                    resp["error"] = "calibration already active";
                    res.status = 409;
                    res.set_content(resp.dump(), "application/json");
                    return;
                }
            }

            std::string mode = body.value("mode", std::string("mono"));
            std::string camera_a = body.value("camera_a", body.value("camera", std::string{}));
            std::string camera_b = body.value("camera_b", std::string{});
            calibration::CalibConfig cfg = calibConfigFromJson(body.value("config", json::object()));
            bool client_error = false;

            try {
                if (mode != "mono" && mode != "stereo") {
                    client_error = true;
                    throw std::runtime_error("mode must be 'mono' or 'stereo'");
                }
                if (camera_a.empty()) {
                    client_error = true;
                    throw std::runtime_error("camera_a is required");
                }

                std::string dev_a = cam_mgr_.devicePath(camera_a);
                if (dev_a.empty()) {
                    client_error = true;
                    throw std::runtime_error("camera_a not available");
                }

                std::unique_ptr<CameraPauseGuard> guard = std::make_unique<CameraPauseGuard>(*this);
                bool ok = false;
                std::unique_ptr<calibration::MonoCalibrator> mono;
                std::unique_ptr<calibration::StereoCalibrator> stereo;

                if (mode == "stereo") {
                    if (camera_b.empty()) {
                        client_error = true;
                        throw std::runtime_error("camera_b is required for stereo");
                    }
                    std::string dev_b = cam_mgr_.devicePath(camera_b);
                    if (dev_b.empty()) {
                        client_error = true;
                        throw std::runtime_error("camera_b not available");
                    }
                    stereo = std::make_unique<calibration::StereoCalibrator>(camera_a, camera_b, dev_a, dev_b, cfg);
                    ok = stereo->startCameras();
                    if (!ok) {
                        throw std::runtime_error("failed to start stereo cameras");
                    }
                } else {
                    mode = "mono";
                    mono = std::make_unique<calibration::MonoCalibrator>(camera_a, dev_a, cfg);
                    ok = mono->startCamera();
                    if (!ok) {
                        throw std::runtime_error("failed to start camera");
                    }
                }

                {
                    std::lock_guard<std::mutex> lk(live_calib_mutex_);
                    if (mode == "mono") {
                        live_calib_manager_.setMono(std::move(mono), cfg, std::move(guard), camera_a);
                    } else {
                        live_calib_manager_.setStereo(std::move(stereo), cfg, std::move(guard), camera_a, camera_b);
                    }
                }

                resp["status"] = "ok";
                resp["mode"] = mode;
                resp["camera_a"] = camera_a;
                if (mode == "stereo") {
                    resp["camera_b"] = camera_b;
                }
                resp["config"] = calibConfigToJson(cfg);
            } catch (const std::exception &e) {
                resp["status"] = "error";
                resp["error"] = e.what();
                res.status = client_error ? 400 : 500;
            }

            res.set_content(resp.dump(), "application/json");
        });

        server.Post("/api/calibration_new/stop", [this](const Request&, Response& res) {
            json resp = json::object();
            std::unique_ptr<calibration::MonoCalibrator> mono;
            std::unique_ptr<calibration::StereoCalibrator> stereo;
            std::unique_ptr<CameraPauseGuard> guard;
            bool was_active = false;

            ActiveCalibratorManager::ReleasedCalibrators released;
            {
                std::lock_guard<std::mutex> lk(live_calib_mutex_);
                released = live_calib_manager_.releaseCalibrators();
            }

            was_active = released.mode != LiveCalibrationMode::None;
            mono = std::move(released.mono);
            stereo = std::move(released.stereo);
            guard = std::move(released.guard);

            if (stereo) {
                stereo->stopCameras();
            }
            if (mono) {
                mono->stopCamera();
            }
            guard.reset();

            resp["status"] = "ok";
            resp["was_active"] = was_active;
            res.set_content(resp.dump(), "application/json");
        });

        server.Post("/api/calibration_new/calibrate", [this](const Request&, Response& res) {
            json resp = json::object();
            bool ok = false;
            {
                std::lock_guard<std::mutex> lk(live_calib_mutex_);
                if (!live_calib_manager_.isActive()) {
                    resp["status"] = "error";
                    resp["error"] = "calibration not started";
                    res.status = 409;
                    res.set_content(resp.dump(), "application/json");
                    return;
                }

                auto mode = live_calib_manager_.mode();
                if (mode == LiveCalibrationMode::Mono) {
                    if (auto *mono = live_calib_manager_.mono()) {
                        mono->startCalibration();
                        ok = true;
                    }
                } else if (mode == LiveCalibrationMode::Stereo) {
                    if (auto *stereo = live_calib_manager_.stereo()) {
                        stereo->startCalibration();
                        ok = true;
                    }
                }
                live_calib_manager_.setProgress(0, live_calib_manager_.config().max_frames, false);
                live_calib_manager_.setResults({}, {});
                live_calib_manager_.setLastError({});
                live_calib_manager_.setHint({});
            }

            if (!ok) {
                resp["status"] = "error";
                resp["error"] = "calibrator unavailable";
                res.status = 500;
            } else {
                resp["status"] = "ok";
            }
            res.set_content(resp.dump(), "application/json");
        });


        server.Post("/api/calibration_new/compute", [this](const Request&, Response& res) {
            json resp = json::object();
            calibration::MonoCalibrator *mono_ptr = nullptr;
            calibration::StereoCalibrator *stereo_ptr = nullptr;
            LiveCalibrationMode mode = LiveCalibrationMode::None;
            std::string camera_a;
            std::string camera_b;
            {
                std::lock_guard<std::mutex> lk(live_calib_mutex_);
                if (!live_calib_manager_.isActive()) {
                    resp["status"] = "error";
                    resp["error"] = "calibration not started";
                    res.status = 409;
                    res.set_content(resp.dump(), "application/json");
                    return;
                }
                if (live_calib_manager_.computeInProgress()) {
                    resp["status"] = "error";
                    resp["error"] = "calibration compute already in progress";
                    res.status = 409;
                    res.set_content(resp.dump(), "application/json");
                    return;
                }
                live_calib_manager_.setComputeInProgress(true);
                mode = live_calib_manager_.mode();
                mono_ptr = live_calib_manager_.mono();
                stereo_ptr = live_calib_manager_.stereo();
                camera_a = live_calib_manager_.cameraPrimary();
                camera_b = live_calib_manager_.cameraSecondary();
            }

            bool ok = false;
            std::string err;
            std::vector<calibration::MonoCalibrationSummary> mono_results;
            std::vector<calibration::StereoCalibrationSummary> stereo_results;
            auto results_dir = calib_root_ / "calibration" / "results";
            std::error_code ec;
            std::filesystem::create_directories(results_dir, ec);
            (void)ec;

            if (mode == LiveCalibrationMode::Mono && mono_ptr) {
                calibration::MonoCalibrationSummary summary;
                ok = mono_ptr->calibrate(results_dir, summary, err);
                if (ok) {
                    mono_results.push_back(summary);
                }
            } else if (mode == LiveCalibrationMode::Stereo && stereo_ptr) {
                std::optional<calibration::MonoCalibrationSummary> mono_a;
                std::optional<calibration::MonoCalibrationSummary> mono_b;
                calibration::StereoCalibrationSummary stereo_summary;
                ok = stereo_ptr->calibrate(results_dir, mono_a, mono_b, stereo_summary, err, false);
                if (ok) {
                    if (mono_a) mono_results.push_back(*mono_a);
                    if (mono_b) mono_results.push_back(*mono_b);
                    stereo_results.push_back(stereo_summary);
                }
            } else {
                err = "calibrator unavailable";
            }
            if (ok) {
                calibration::updateCalibrationResults(mono_results, stereo_results, results_dir);
                calibration_results_manager_.reload();
            }

            {
                std::lock_guard<std::mutex> lk(live_calib_mutex_);
                live_calib_manager_.setComputeInProgress(false);
                if (ok) {
                    live_calib_manager_.setResults(mono_results, stereo_results);
                    live_calib_manager_.setLastError({});
                } else {
                    live_calib_manager_.setLastError(err);
                }
            }

            if (!ok) {
                resp["status"] = "error";
                resp["error"] = err.empty() ? "calibration failed" : err;
                res.status = 500;
            } else {
                resp["status"] = "ok";
                json mono_json = json::array();
                for (const auto &m : mono_results) mono_json.push_back(monoSummaryToJson(m));
                json stereo_json = json::array();
                for (const auto &s : stereo_results) stereo_json.push_back(stereoSummaryToJson(s));
                resp["mono_results"] = std::move(mono_json);
                resp["stereo_results"] = std::move(stereo_json);
            }

            res.set_content(resp.dump(), "application/json");
        });

        server.Get("/api/calibration_new/status", [this](const Request&, Response& res) {
            json resp = json::object();

            {
                std::lock_guard<std::mutex> lk(live_calib_mutex_);
                live_calib_manager_.refreshProgressFromCalibrator();
                resp["active"] = live_calib_manager_.isActive();
                resp["mode"] = modeToString(live_calib_manager_.mode());
                resp["camera_a"] = live_calib_manager_.cameraPrimary();
                if (!live_calib_manager_.cameraSecondary().empty()) {
                    resp["camera_b"] = live_calib_manager_.cameraSecondary();
                }
                resp["config"] = calibConfigToJson(live_calib_manager_.config());
                resp["hint"] = live_calib_manager_.lastHint();
                resp["pattern_visible"] = live_calib_manager_.patternVisible();
                resp["compute_in_progress"] = live_calib_manager_.computeInProgress();
                resp["progress"] = {
                    {"current", live_calib_manager_.progressCurrent()},
                    {"max", live_calib_manager_.progressMax()}
                };
                if (!live_calib_manager_.lastError().empty()) {
                    resp["error"] = live_calib_manager_.lastError();
                }
                json mono_json = json::array();
                for (const auto &m : live_calib_manager_.monoResults()) mono_json.push_back(monoSummaryToJson(m));
                json stereo_json = json::array();
                for (const auto &s : live_calib_manager_.stereoResults()) stereo_json.push_back(stereoSummaryToJson(s));
                resp["mono_results"] = std::move(mono_json);
                resp["stereo_results"] = std::move(stereo_json);

                if (live_calib_manager_.mode() == LiveCalibrationMode::Mono) {
                    if (auto *mono = live_calib_manager_.mono()) {
                        resp["running"] = mono->isRunning();
                        resp["calibrating"] = mono->isCalibrating();
                    } else {
                        resp["running"] = false;
                        resp["calibrating"] = false;
                    }
                } else if (live_calib_manager_.mode() == LiveCalibrationMode::Stereo) {
                    if (auto *stereo = live_calib_manager_.stereo()) {
                        resp["running"] = stereo->isRunning();
                        resp["calibrating"] = stereo->isCalibrating();
                    } else {
                        resp["running"] = false;
                        resp["calibrating"] = false;
                    }
                } else {
                    resp["running"] = false;
                    resp["calibrating"] = false;
                }
            }

            res.set_content(resp.dump(), "application/json");
        });


server.Get("/api/calibration_new/video", [this](const Request&, Response& res) {
    {
        std::lock_guard<std::mutex> lk(live_calib_mutex_);
        if (!live_calib_manager_.isActive()) {
            json err;
            err["status"] = "error";
            err["error"] = "calibration not started";
            res.status = 503;
            res.set_content(err.dump(), "application/json");
            return;
        }
        live_calib_manager_.setVideoActive(true);
    }

    res.set_header("Cache-Control", "no-store, no-cache, must-revalidate");
    res.set_header("Pragma", "no-cache");
    res.set_header("Connection", "keep-alive");

    res.set_chunked_content_provider(
        "multipart/x-mixed-replace; boundary=frame",
        [this](size_t, DataSink& sink) {
            auto last_push = Clock::now();
            
            while (true) {
                bool active = false;
                {
                    std::lock_guard<std::mutex> lk(live_calib_mutex_);
                    active = live_calib_manager_.isActive();
                    if (!active) break;
                }

                // Берем последний обработанный кадр из основного потока
                std::vector<uint8_t> jpg;
                {
                    std::unique_lock<std::mutex> lk(frame_mtx);
                    frame_cv.wait_for(lk, std::chrono::milliseconds(1000),
                                     [this]{ return !last_jpeg.empty() || !cam_running.load(); });
                    if (!cam_running.load()) break;
                    jpg = last_jpeg;
                }

                if (jpg.empty()) {
                    std::this_thread::sleep_for(std::chrono::milliseconds(33));
                    continue;
                }

                // Ограничение FPS
                auto now = Clock::now();
                double elapsed = std::chrono::duration<double>(now - last_push).count();
                double min_dt = 1.0 / 30.0; // 30 FPS max
                if (elapsed < min_dt) {
                    auto sleep_d = std::chrono::duration<double>(min_dt - elapsed);
                    std::this_thread::sleep_for(
                        std::chrono::duration_cast<std::chrono::milliseconds>(sleep_d));
                }
                last_push = Clock::now();

                std::string header = "--frame\r\n"
                                     "Content-Type: image/jpeg\r\n"
                                     "Content-Length: " + std::to_string(jpg.size()) + "\r\n\r\n";
                if (!sink.write(header.data(), header.size())) break;
                if (!sink.write(reinterpret_cast<const char*>(jpg.data()), jpg.size())) break;
                if (!sink.write("\r\n", 2)) break;
            }

            {
                std::lock_guard<std::mutex> lk(live_calib_mutex_);
                live_calib_manager_.setVideoActive(false);
            }
            return true;
        },
        [this](bool) {
            std::lock_guard<std::mutex> lk(live_calib_mutex_);
            live_calib_manager_.setVideoActive(false);
        }
    );
});


        // Video recording control
        server.Post("/api/record/start", [this](const Request& req, Response& res){
            json resp;
            try {
                auto j = json::parse(req.body);
                recording_duration_seconds_ = j.value("duration", 30);

                auto sanitize = [](const std::string& value) {
                    std::string cleaned;
                    cleaned.reserve(value.size());
                    for (unsigned char c : value) {
                        if (std::isalnum(c) || c == '_' || c == '-') {
                            cleaned.push_back(static_cast<char>(c));
                        } else {
                            cleaned.push_back('_');
                        }
                    }
                    return cleaned;
                };

                std::string requested_prefix;
                if (j.contains("filename_prefix") && j["filename_prefix"].is_string()) {
                    requested_prefix = j["filename_prefix"].get<std::string>();
                } else if (j.contains("prefix") && j["prefix"].is_string()) {
                    requested_prefix = j["prefix"].get<std::string>();
                }

                requested_prefix = sanitize(requested_prefix);

                if (!recording_active_) {
                    record_filename_prefix_ = requested_prefix;
                    should_start_recording_ = true;
                    record_enabled_ = true;
                    resp["status"] = "ok";
                    resp["message"] = "Recording will start";
                    resp["duration"] = recording_duration_seconds_;
                    if (!record_filename_prefix_.empty()) {
                        resp["filename_prefix"] = record_filename_prefix_;
                    }
                    printf("Video recording start requested, duration: %d seconds\n", recording_duration_seconds_);
                } else {
                    resp["status"] = "error";
                    resp["message"] = "Recording already active";
                }
            } catch(...) {
                resp["status"] = "error";
                resp["message"] = "Invalid JSON";
            }
            res.set_content(resp.dump(), "application/json");
        });

        server.Post("/api/record/stop", [this](const Request&, Response& res){
            json resp;
            should_stop_recording_ = true;
            record_enabled_ = false;
            resp["status"] = "ok";
            resp["message"] = "Recording will stop";
            printf("Video recording stop requested\n");
            res.set_content(resp.dump(), "application/json");
        });

        server.Get("/api/record/status", [this](const Request&, Response& res){
            json resp;
            resp["recording_active"] = recording_active_;
            resp["recording_enabled"] = record_enabled_;
            resp["duration_seconds"] = recording_duration_seconds_;
            if (recording_active_) {
                auto elapsed = std::chrono::steady_clock::now() - record_start_time_;
                int remaining = recording_duration_seconds_ - 
                    std::chrono::duration_cast<std::chrono::seconds>(elapsed).count();
                resp["time_remaining"] = std::max(0, remaining);
            } else {
                resp["time_remaining"] = 0;
            }
            res.set_content(resp.dump(), "application/json");
        });
    }

    // ---------- Camera (V4L2 MJPEG) ----------
    bool initCamera() {
        cam_fd = open(cam_dev.c_str(), O_RDWR | O_NONBLOCK, 0);
        if (cam_fd < 0) { perror("open camera"); return false; }

        v4l2_format fmt{};
        fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        fmt.fmt.pix.width = cam_w_req;
        fmt.fmt.pix.height = cam_h_req;
        fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_MJPEG;
        fmt.fmt.pix.field = V4L2_FIELD_ANY;
        if (ioctl(cam_fd, VIDIOC_S_FMT, &fmt) < 0) {
            perror("VIDIOC_S_FMT MJPEG");
            close(cam_fd); cam_fd = -1;
            return false;
        }
        cam_w = fmt.fmt.pix.width;
        cam_h = fmt.fmt.pix.height;

        v4l2_streamparm parm{};
        parm.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        parm.parm.capture.timeperframe.numerator = 1;
        parm.parm.capture.timeperframe.denominator = cam_fps_req <= 0 ? 30 : cam_fps_req;
        ioctl(cam_fd, VIDIOC_S_PARM, &parm);
        cam_fps = parm.parm.capture.timeperframe.denominator > 0
                  ? parm.parm.capture.timeperframe.denominator : cam_fps_req;

        v4l2_format gfmt{}; gfmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        ioctl(cam_fd, VIDIOC_G_FMT, &gfmt);
        char fcc[5]; fourcc_to_str(gfmt.fmt.pix.pixelformat, fcc);
        printf("CAP negotiated: %dx%d @ %d FPS, FOURCC=%s\n", cam_w, cam_h, cam_fps, fcc);

        v4l2_requestbuffers req{};
        req.count = std::max(1, cam_buffers);
        req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        req.memory = V4L2_MEMORY_MMAP;
        if (ioctl(cam_fd, VIDIOC_REQBUFS, &req) < 0 || req.count < 1) {
            perror("VIDIOC_REQBUFS");
            close(cam_fd); cam_fd = -1;
            return false;
        }

        cam_bufs.resize(req.count);
        for (unsigned int i = 0; i < req.count; ++i) {
            v4l2_buffer buf{};
            buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
            buf.memory = V4L2_MEMORY_MMAP;
            buf.index = i;
            if (ioctl(cam_fd, VIDIOC_QUERYBUF, &buf) < 0) { perror("VIDIOC_QUERYBUF"); return false; }
            cam_bufs[i].length = buf.length;
            cam_bufs[i].start = mmap(NULL, buf.length, PROT_READ | PROT_WRITE, MAP_SHARED, cam_fd, buf.m.offset);
            if (cam_bufs[i].start == MAP_FAILED) { perror("mmap"); return false; }
        }
        for (unsigned int i = 0; i < req.count; ++i) {
            v4l2_buffer buf{};
            buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
            buf.memory = V4L2_MEMORY_MMAP;
            buf.index = i;
            if (ioctl(cam_fd, VIDIOC_QBUF, &buf) < 0) { perror("VIDIOC_QBUF"); return false; }
        }
        v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        if (ioctl(cam_fd, VIDIOC_STREAMON, &type) < 0) { perror("VIDIOC_STREAMON"); return false; }

        cam_running = true;
        return true;
    }

    void deinitCamera() {
        if (cam_fd >= 0) {
            v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
            ioctl(cam_fd, VIDIOC_STREAMOFF, &type);
            for (auto& b : cam_bufs) {
                if (b.start && b.start != MAP_FAILED && b.length) munmap(b.start, b.length);
            }
            cam_bufs.clear();
            close(cam_fd);
            cam_fd = -1;
        }
    }

    bool grabMjpeg(std::vector<uint8_t>& out) {
        if (cam_fd < 0) return false;
        fd_set fds; FD_ZERO(&fds); FD_SET(cam_fd, &fds);
        timeval tv{0}; tv.tv_sec = 2;
        int r = select(cam_fd + 1, &fds, NULL, NULL, &tv);
        if (r <= 0) return false;

        v4l2_buffer buf{};
        buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory = V4L2_MEMORY_MMAP;
        if (ioctl(cam_fd, VIDIOC_DQBUF, &buf) < 0) return false;
        if (buf.index >= cam_bufs.size()) return false;

        auto& b = cam_bufs[buf.index];
        out.assign((uint8_t*)b.start, (uint8_t*)b.start + buf.bytesused);

        if (ioctl(cam_fd, VIDIOC_QBUF, &buf) < 0) return false;
        return true;
    }

    // ---------- JPEG <-> RGB (TurboJPEG) ----------
    static std::vector<uint8_t> decode_mjpeg_to_image(const uint8_t* jpeg, size_t jpeg_size, image_buffer_t* img) {
        std::vector<uint8_t> buf;
        tjhandle th = tjInitDecompress();
        if (!th) return buf;
        int w=0, h=0, subsamp=0, colorspace=0;
        if (tjDecompressHeader3(th, jpeg, (unsigned long)jpeg_size, &w, &h, &subsamp, &colorspace) != 0) {
            tjDestroy(th); return buf;
        }
        img->width = w; img->height = h;
        img->format = IMAGE_FORMAT_RGB888;
        buf.resize(w * h * 3);

        int rc = tjDecompress2(th, jpeg, (unsigned long)jpeg_size,
                               buf.data(), w, 0/*pitch*/, h,
                               TJPF_RGB, TJFLAG_FASTDCT | TJFLAG_FASTUPSAMPLE);
        tjDestroy(th);
        if (rc != 0) {
            buf.clear();
            return buf;
        }
        img->size = static_cast<int>(buf.size());
        img->virt_addr = buf.data();
        return buf;
    }

    std::vector<uint8_t> encode_rgb_to_jpeg(const unsigned char* rgb, int w, int h, int q) {
        tjhandle th = tjInitCompress();
        if (!th) return {};
        unsigned char* out = nullptr;
        unsigned long out_sz = 0;
        int rc = tjCompress2(th, rgb, w, 0/*pitch*/, h, TJPF_RGB,
                             &out, &out_sz, TJSAMP_420, q,
                             TJFLAG_FASTDCT);
        std::vector<uint8_t> buf;
        if (rc == 0 && out && out_sz > 0) buf.assign(out, out + out_sz);
        if (out) tjFree(out);
        tjDestroy(th);
        return buf;
    }

    static void freeImage(image_buffer_t& /*img*/) {
        // Memory handled by std::vector returned from decode_mjpeg_to_image.
    }

    // ---------- video recording helpers ----------
    bool initVideoRecording() {
        if (!record_enabled_) return true;

        std::filesystem::create_directories(record_dir_);

        auto now = std::time(nullptr);
        char timestamp[32];
        std::strftime(timestamp, sizeof(timestamp), "%Y%m%d_%H%M%S", std::localtime(&now));

        std::string base_name = cam_id_ + std::string("_") + timestamp;
        if (!record_filename_prefix_.empty()) {
            if (record_filename_prefix_.back() == '_') {
                base_name = record_filename_prefix_ + base_name;
            } else {
                base_name = record_filename_prefix_ + std::string("_") + base_name;
            }
        }

        std::string filename = record_dir_ + "/" + base_name + ".avi";
        printf("Starting video recording: %s\n", filename.c_str());

        video_writer_ = std::make_unique<cv::VideoWriter>(
            filename,
            cv::VideoWriter::fourcc('M','J','P','G'),
            cam_fps > 0 ? cam_fps : 30,
            cv::Size(cam_w, cam_h)
        );

        if (!video_writer_ || !video_writer_->isOpened()) {
            printf("ERROR: Failed to open video writer: %s\n", filename.c_str());
            video_writer_.reset();
            return false;
        }

        record_start_time_ = std::chrono::steady_clock::now();
        recording_active_ = true;
        return true;
    }

    void stopVideoRecording() {
        if (video_writer_ && recording_active_) {
            printf("Stopping video recording\n");
            video_writer_.reset();
            recording_active_ = false;
            record_filename_prefix_.clear();
        }
    }

    bool shouldStopRecording() {
        if (!recording_active_ || recording_duration_seconds_ <= 0) return false;
        auto elapsed = std::chrono::steady_clock::now() - record_start_time_;
        return std::chrono::duration_cast<std::chrono::seconds>(elapsed).count() >= recording_duration_seconds_;
    }

    // ---------- main loop ----------
// start main loop (DEBUG profiling) — ВНУТРИ КЛАССА!
    void cameraLoop() {
        static StageAcc acc; // DEBUG: аккумулируем времена по этапам кадра

        auto t_prev = Clock::now();
        double fps_smoothed = 0.0;

        bool video_init_success = true;
        if (record_enabled_ && !video_init_success) {
            printf("WARNING: Video recording initialization failed\n");
        }

        while (cam_running.load()) {
            TICK(loop);  // DEBUG: старт таймера полного цикла

            // ===== CAPTURE =====
            std::vector<uint8_t> mjpeg;
            TICK(cap);  // DEBUG
            bool okCap = grabMjpeg(mjpeg);
            if (okCap && !cam_id_.empty()) {
                uint64_t t_ns = cam_mgr_.nowMonoNs();
                cam_mgr_.pushFrame(cam_id_, cam_w, cam_h, mjpeg, t_ns);
            }
            TOCK(cap, acc.cap);  // DEBUG
            if (!okCap) {
                TOCK(loop, acc.loop);        // DEBUG
                acc.add_print_and_reset(30); // DEBUG: печать средних каждые 30 кадров
                std::this_thread::sleep_for(std::chrono::milliseconds(2));
                continue;
            }
            // ===== VIDEO RECORDING CONTROL =====
            if (should_start_recording_.load()) {
                should_start_recording_ = false;
                if (!recording_active_) {
                    video_init_success = initVideoRecording();
                    if (video_init_success) {
                        printf("Video recording started by user command\n");
                    } else {
                        printf("Failed to start video recording\n");
                        record_enabled_ = false;
                    }
                }
            }

            if (should_stop_recording_.load()) {
                should_stop_recording_ = false;
                if (recording_active_) {
                    stopVideoRecording();
                    record_enabled_ = false;
                    printf("Video recording stopped by user command\n");
                }
            }
            // ===== PREP (MJPEG -> RGB) =====
            TICK(prep);  // DEBUG
            image_buffer_t frame{}; // RGB888
            auto frame_buf = decode_mjpeg_to_image(mjpeg.data(), mjpeg.size(), &frame);
            bool okDec = !frame_buf.empty();
            TOCK(prep, acc.prep);  // DEBUG
            if (!okDec) {
                TOCK(loop, acc.loop);        // DEBUG
                acc.add_print_and_reset(30); // DEBUG
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
                continue;
            }

            // ===== VIDEO RECORDING =====
            if (recording_active_ && video_writer_ && record_enabled_) {
                cv::Mat bgr_frame(frame.height, frame.width, CV_8UC3);
                for (int y = 0; y < frame.height; ++y) {
                    for (int x = 0; x < frame.width; ++x) {
                        uint8_t* rgb_pixel = frame.virt_addr + (y * frame.width + x) * 3;
                        uint8_t* bgr_pixel = bgr_frame.ptr<uint8_t>(y) + x * 3;
                        bgr_pixel[0] = rgb_pixel[2];
                        bgr_pixel[1] = rgb_pixel[1];
                        bgr_pixel[2] = rgb_pixel[0];
                    }
                }
                video_writer_->write(bgr_frame);
                if (shouldStopRecording()) {
                    printf("Recording time limit reached, stopping...\n");
                    stopVideoRecording();
                    record_enabled_ = false;
                }
            }

            json meta = json::object();

// ===== CALIBRATION CHECK =====
{
    std::lock_guard<std::mutex> lk(live_calib_mutex_);
    if (live_calib_manager_.isActive() && 
        live_calib_manager_.mode() == LiveCalibrationMode::Mono) {
        
        // Конвертируем RGB в BGR для OpenCV
        cv::Mat bgr_frame(frame.height, frame.width, CV_8UC3);
        for (int y = 0; y < frame.height; ++y) {
            for (int x = 0; x < frame.width; ++x) {
                uint8_t* rgb_pixel = frame.virt_addr + (y * frame.width + x) * 3;
                uint8_t* bgr_pixel = bgr_frame.ptr<uint8_t>(y) + x * 3;
                bgr_pixel[0] = rgb_pixel[2]; // B
                bgr_pixel[1] = rgb_pixel[1]; // G
                bgr_pixel[2] = rgb_pixel[0]; // R
            }
        }
        
        // Передаем кадр в калибратор
        if (auto* mono = live_calib_manager_.mono()) {
            std::string hint;
            int prog_cur = 0, prog_max = 0;
            bool pattern_vis = false;
            
            // Обрабатываем кадр через calibrator
            mono->getFrame(bgr_frame, hint, prog_cur, prog_max, pattern_vis);
            
            // Обновляем статус
            live_calib_manager_.setProgress(prog_cur, prog_max, pattern_vis);
            live_calib_manager_.setHint(hint);
            
            // Рисуем подсказку на preview
            if (!hint.empty()) {
                std::string display_hint = hint + " (" + 
                    std::to_string(prog_cur) + "/" + 
                    std::to_string(prog_max) + ")";
                    
                cv::Scalar color = pattern_vis ? 
                    cv::Scalar(0, 255, 0) : cv::Scalar(0, 165, 255);
                
                // Конвертируем обратно в RGB для отрисовки
                cv::Mat rgb_annotated;
                cv::cvtColor(bgr_frame, rgb_annotated, cv::COLOR_BGR2RGB);
                
                cv::putText(rgb_annotated, display_hint, 
                           cv::Point(10, 40), cv::FONT_HERSHEY_SIMPLEX,
                           1.0, color, 2);
                
                // Обновляем frame buffer
                if (rgb_annotated.isContinuous()) {
                    memcpy(frame.virt_addr, rgb_annotated.data, 
                           frame.width * frame.height * 3);
                }
            }
        }
        
        // Пропускаем обычную детекцию если калибруемся
        goto skip_detection;
    }
}
            // ===== DETECTION =====
            {
            object_detect_result_list od{};
            int ret = 0;
            if (draw) {
                // ===== INFER (RKNN) =====
                TICK(infer);  // DEBUG
                {
                    std::lock_guard<std::mutex> lk(infer_mtx);
                    ret = inference_yolov8_model(&rknn_app_ctx, &frame, &od);
                }
                TOCK(infer, acc.infer);  // DEBUG
                if (ret == 0) {
                    // assign stable IDs to detections
                    tracker.update(&od, &frame);

                    if (log_enabled) {
                        auto now_sys = std::chrono::system_clock::now();
                        for (int i = 0; i < od.count; ++i) {
                            auto* d = &od.results[i];
                            if (d->track_id >= 0 && logged_ids.insert(d->track_id).second) {
                                event_counter++;
                                auto tt = std::time(nullptr);
                                char tbuf[32];
                                std::strftime(tbuf, sizeof(tbuf), "%d-%m-%Y %H:%M:%S", std::localtime(&tt));
                                std::string line = std::to_string(event_counter) + ", " +
                                                   coco_cls_to_name(d->cls_id) + ", ID " +
                                                   std::to_string(d->track_id) + ", " + tbuf;
                                appendLog(line);
                                minute_counts[d->cls_id]++;
                            }
                        }
                        if (now_sys - minute_start >= std::chrono::minutes(1)) {
                            logMinuteSummary(now_sys);
                        }
                    }

                    // ===== DRAW =====
                    TICK(draw);  // DEBUG
                    for (int i = 0; i < od.count; ++i) {
                        auto* d = &od.results[i];

                        int x = d->box.left, y = d->box.top;
                        int w = d->box.right - d->box.left;
                        int h = d->box.bottom - d->box.top;
                        draw_rectangle(&frame, x, y, w, h, COLOR_BLUE, 3);
                        if (show_fps) {
                            char text[96];
                            snprintf(text, sizeof(text), "#%d %s %.1f%%", d->track_id,
                                     coco_cls_to_name(d->cls_id), d->prop * 100.f);
                            draw_text(&frame, text, x, std::max(0, y - 18), COLOR_RED, 10);
                        }
                    }
                    TOCK(draw, acc.draw);  // DEBUG

                    meta = formatDetectionResults(&od, frame.width, frame.height, tracker);
                }
            }
            }
            skip_detection: 
            // ===== PREVIEW UPDATE =====
            auto now = std::chrono::steady_clock::now();
            if (now - last_preview_update_ >= preview_interval_) {
                TICK(enc);
                std::vector<uint8_t> jpg = encode_rgb_to_jpeg(
                    frame.virt_addr, frame.width, frame.height, jpeg_q);
                TOCK(enc, acc.enc);

                {
                    std::lock_guard<std::mutex> lk(frame_mtx);
                    last_jpeg.swap(jpg);
                    last_meta = std::move(meta);
                }
                frame_cv.notify_all();
                last_preview_update_ = now;
            }

            // ===== FPS DISPLAY =====
            if (show_fps) {
                auto t_now = Clock::now();
                double dt = std::chrono::duration<double>(t_now - t_prev).count();
                t_prev = t_now;
                double fps_inst = dt > 0 ? 1.0 / dt : 0.0;
                fps_smoothed = (fps_smoothed == 0.0) ? fps_inst
                                                     : (0.8 * fps_smoothed + 0.2 * fps_inst);
                static double acc_t = 0; acc_t += dt;
                if (acc_t >= 1.0) {
                    printf("Loop FPS: %.1f  (cap:%dx%d@%d, model:%dx%d) %s\n",
                           fps_smoothed, cam_w, cam_h, cam_fps,
                           rknn_app_ctx.model_width, rknn_app_ctx.model_height,
                           recording_active_ ? "[RECORDING]" : "");
                    acc_t = 0;

                }
            }


            TOCK(loop, acc.loop);         // DEBUG: конец таймера цикла
            acc.add_print_and_reset(30);  // DEBUG: печать каждые 30 кадров

            std::this_thread::sleep_for(std::chrono::milliseconds(2));
        }
        stopVideoRecording();
        if (log_enabled) logMinuteSummary(std::chrono::system_clock::now());
    }
// end main loop
    json formatDetectionResults(object_detect_result_list* results, int img_w, int img_h, const SimpleTracker& tracker) {
        json detections = json::array();
        for (int i = 0; i < results->count; i++) {
            auto* d = &(results->results[i]);
            json det;
            det["class_name"] = coco_cls_to_name(d->cls_id);
            det["class_id"] = d->cls_id;
            det["confidence"] = d->prop;
            det["track_id"] = d->track_id;
            float col[3];
            if (tracker.getColor(d->track_id, col)) {
                det["color"] = {col[0], col[1], col[2]};
            }
            json box;
            box["left"] = d->box.left;
            box["top"] = d->box.top;
            box["right"] = d->box.right;
            box["bottom"] = d->box.bottom;
            box["width"] = d->box.right - d->box.left;
            box["height"] = d->box.bottom - d->box.top;
            det["box"] = box;
            json nbox;
            nbox["left"] = (float)d->box.left / img_w;
            nbox["top"] = (float)d->box.top / img_h;
            nbox["right"] = (float)d->box.right / img_w;
            nbox["bottom"] = (float)d->box.bottom / img_h;
            nbox["width"] = (float)(d->box.right - d->box.left) / img_w;
            nbox["height"] = (float)(d->box.bottom - d->box.top) / img_h;
            det["normalized_box"] = nbox;

            // Add distance measurement if calibration is available
            if (calibration_results_manager_.hasMonoCalibration(cam_id_)) {
                float distance = calibration_results_manager_.measureMonoDistance(
                    cam_id_,
                    cv::Rect(d->box.left, d->box.top,
                             d->box.right - d->box.left,
                             d->box.bottom - d->box.top));
                if (distance > 0) {
                    det["distance_mm"] = distance;
                    det["distance_m"] = distance / 1000.0f;
                }
            }

            detections.push_back(det);
        }
        json resp;
        resp["detections"] = detections;
        resp["count"] = results->count;
        resp["image_width"] = img_w;
        resp["image_height"] = img_h;
        return resp;
    }
};

// ---- signals & main ----
static YOLOWebServer* g_server = nullptr;
static void signalHandler(int sig) {
    printf("\nSignal %d received, stopping...\n", sig);
    if (g_server) g_server->stop();
}

static void printUsage(const char* argv0){
    printf("Usage: %s <model.rknn> [--dev /dev/videoX] [--port N]\n", argv0);
    printf("  --size WxH           capture size\n");
    printf("  --cap-fps N          capture FPS request\n");
    printf("  --buffers N          V4L2 buffers (1..4)\n");
    printf("  --jpeg-quality N     30..95 (default 70)\n");
    printf("  --http-fps-limit N   limit MJPEG stream FPS (0=unlimited)\n");
    printf("  --no-draw            disable detection and overlay\n");
    printf("  --fps                print loop FPS to console and draw labels\n");
    printf("  --npu-core auto|0|1|2|01|012  choose NPU core mask\n");
    printf("  --log-file FILE NAME     write detection log to /tmp/npudet.DATE.FILE\n");
    printf("  --labels PATH         labels file path\n");
    printf("  --config PATH        configuration file path\n");
    printf("  --record-dir PATH    directory for video recording (default: ./rec)\n");
    printf("  --record-seconds N   recording duration in seconds (default: 180)\n");
    printf("  --no-record          disable video recording\n");
}

int main(int argc, char** argv) {
    if (argc < 2) { printUsage(argv[0]); return -1; }
    Args a = parseArgs(argc, argv);
    if (a.model.empty()) { fprintf(stderr, "Model path is required\n"); return -1; }

    g_exe_dir = std::filesystem::canonical(argv[0]).parent_path();
    g_config_path = a.config.empty() ? g_exe_dir / "config.json" : std::filesystem::path(a.config);

    mkdir("./web", 0755);
    std::signal(SIGINT,  signalHandler);
    std::signal(SIGTERM, signalHandler);

    YOLOWebServer app(a);
    g_server = &app;
    if (!app.initialize()) return -1;

    printf("Model: %s\n", a.model.c_str());
    printf("Open stream: http://localhost:%d/api/stream.mjpg\n", a.port);
    app.run();
    return 0;
}