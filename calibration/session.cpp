#include "calibration/session.h"
#include "camera_manager.h"
#include <opencv2/opencv.hpp>
#include <filesystem>
#include <thread>
#include <chrono>
#include <regex>
#include <cstdio>

using json = nlohmann::json;

PreviewGuard::PreviewGuard(bool &flag, bool new_state)
    : flag_(flag), prev_(flag) {
    flag_ = new_state;
}

PreviewGuard::~PreviewGuard() { flag_ = prev_; }

namespace {
struct ManagerGuard {
    CameraManager &mgr;
    explicit ManagerGuard(CameraManager &m) : mgr(m) { mgr.stop(); }
    ~ManagerGuard() { mgr.start(); }
};
} // namespace

CalibrationSession::CalibrationSession(CameraManager &mgr, bool &preview_flag,
                                       const std::filesystem::path &root)
    : mgr_(mgr), preview_(preview_flag),
      root_path_(std::filesystem::absolute(root)) {}

void CalibrationSession::start() {
    auto infos = mgr_.configuredCameras();
    prev_modes_.clear();
    for (auto &ci : infos) {
        if (ci.mode == CameraManager::CamConfig::Mode::Calibration) {
            // persist calibration mode to config if needed
            mgr_.setMode(ci.id, CameraManager::CamConfig::Mode::Calibration);
        }
    }
    mgr_.stop();
    preview_ = true; // keep preview accessible
}

void CalibrationSession::stop() {
    mgr_.start();
    prev_modes_.clear();
    preview_ = true;
}

SessionResult CalibrationSession::captureMono(const std::string &id) {
    SessionResult result{200, json{}};
    std::string dev = mgr_.devicePath(id);
    if (dev.empty()) {
        result.status = 400;
        result.body["error"] = "camera not found";
        return result;
    }

    ManagerGuard mg(mgr_);
    PreviewGuard pg(preview_, false);
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    auto dirPath = root_path_ / "calibration" / ("cam_" + id) / "images";
    std::error_code ec;
    std::filesystem::create_directories(dirPath, ec);
    if (ec) {
        result.status = 500;
        result.body["error"] = "create directory";
        return result;
    }

    auto absDir = std::filesystem::absolute(dirPath);
    printf("calibration dir %s\n", absDir.c_str());

    cv::VideoCapture cap(dev);
    if (!cap.isOpened()) {
        result.status = 500;
        result.body["error"] = "open camera";
        return result;
    }

    for (int t = 10; t > 0; --t) {
        printf("start in %d\n", t);
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }

    for (int i = 0; i < 50; i++) {
        cv::Mat frame;
        cap >> frame;
        if (frame.empty()) break;
        char buf[64];
        snprintf(buf, sizeof(buf), "img_%02d.jpg", i);
        auto filePath = dirPath / buf;
        cv::imwrite(filePath.string(), frame);
        std::this_thread::sleep_for(std::chrono::seconds(2));
    }
    cap.release();

    auto resultsDir = root_path_ / "calibration" / "results";
    std::filesystem::create_directories(resultsDir, ec);
    std::string outfile = (resultsDir / ("cam_" + id + ".yml")).string();
    std::string dirStr = dirPath.string();
    std::string cmd =
        "opencv_calib_mono -o " + outfile + " " + dirStr + "/img_*.jpg";
    int rc = system(cmd.c_str());
    result.body["status"] = rc == 0 ? "ok" : "error";
    result.body["out"] = outfile;
    result.body["cmd"] = cmd;
    return result;
}

SessionResult CalibrationSession::captureStereo(const std::vector<std::string> &ids,
                                                int frames, int interval) {
    SessionResult result{200, json{}};
    if (ids.size() < 2) {
        result.status = 400;
        result.body["error"] = "at least two cameras required";
        return result;
    }
    if (frames <= 0) {
        result.status = 400;
        result.body["error"] = "invalid frame count";
        return result;
    }

    ManagerGuard mg(mgr_);
    PreviewGuard pg(preview_, false);
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    std::vector<cv::VideoCapture> caps;
    caps.reserve(ids.size());
    for (auto &id : ids) {
        std::string dev = mgr_.devicePath(id);
        if (dev.empty()) {
            result.status = 400;
            result.body["error"] = "camera not found";
            return result;
        }
        cv::VideoCapture cap(dev);
        if (!cap.isOpened()) {
            result.status = 500;
            result.body["error"] = "open camera";
            return result;
        }
        caps.emplace_back(std::move(cap));
    }

    std::string dirName = "stereo";
    for (auto &id : ids) dirName += "_" + id;
    auto dirPath = root_path_ / "calibration" / dirName / "images";
    std::error_code ec;
    auto removed = std::filesystem::remove_all(dirPath, ec);
    if (ec) {
        printf("failed to clear %s: %s\n", dirPath.c_str(), ec.message().c_str());
        result.status = 500;
        result.body["error"] = "remove directory";
        return result;
    }
    printf("removed %llu entries from %s\n",
           static_cast<unsigned long long>(removed), dirPath.c_str());
    std::filesystem::create_directories(dirPath, ec);
    if (ec) {
        result.status = 500;
        result.body["error"] = "create directory";
        return result;
    }

    for (int i = 0; i < frames; ++i) {
        for (size_t c = 0; c < caps.size(); ++c) {
            cv::Mat frame;
            caps[c] >> frame;
            if (frame.empty()) {
                result.status = 500;
                result.body["error"] = "capture failed";
                return result;
            }
            char buf[64];
            snprintf(buf, sizeof(buf), "pair_%02d_cam%s.jpg", i,
                     ids[c].c_str());
            auto filePath = dirPath / buf;
            cv::imwrite(filePath.string(), frame);
        }
        if (interval > 0)
            std::this_thread::sleep_for(std::chrono::milliseconds(interval));
    }

    result.body["status"] = "ok";
    result.body["dir"] = dirPath.string();
    return result;
}

void CalibrationSession::beginCaptureAll() {
    if (capturing_.load()) return;

    // obtain active camera ids and ensure preview enabled
    auto infos = mgr_.configuredCameras();
    std::vector<std::string> ids;
    prev_modes_.clear();
    for (auto &ci : infos) {
        if (ci.present) {
            ids.push_back(ci.id);
            prev_modes_[ci.id] = ci.mode;
            if (ci.mode == CameraManager::CamConfig::Mode::Detect)
                mgr_.setMode(ci.id, CameraManager::CamConfig::Mode::Preview);
        }
    }
    printf("beginCaptureAll: %zu active cameras\n", ids.size());
    missed_frames_.clear();
    for (auto &id : ids) missed_frames_[id] = 0;

    capturing_ = true;
    capture_thread_ = std::thread([this, ids]() {
        namespace fs = std::filesystem;
        std::vector<fs::path> dirs;

        dirs.reserve(ids.size());
        for (auto &id : ids) {
            fs::path dir = root_path_ / "calibration" / ("cam_" + id) / "images";
            std::error_code ec; fs::create_directories(dir, ec);
            dirs.push_back(dir);

            auto log_dir = fs::current_path() / "calibration" / ("cam_" + id) / "images";
            printf("capture directory for cam %s: %s\n", id.c_str(), log_dir.c_str());
        }
        int idx = 0;
        while (capturing_.load()) {

            uint64_t t = mgr_.nowMonoNs();
            for (size_t i = 0; i < ids.size(); ++i) {
                CameraManager::Frame f;
                if (mgr_.getFrame(ids[i], t, f)) {
                    char buf[32];
                    snprintf(buf, sizeof(buf), "img_%02d.jpg", idx);
                    auto fp = dirs[i] / buf;
                    mgr_.saveFrameWithMeta(fp.string(), f, ids[i]);
                } else {
                    missed_frames_[ids[i]]++;
                    fprintf(stderr, "missed frame for camera %s\n", ids[i].c_str());
                }
            }
            ++idx;
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }
    });
}

std::map<std::string, std::size_t> CalibrationSession::endCaptureAll() {
    if (!capturing_.load()) return {};
    capturing_ = false;
    if (capture_thread_.joinable()) capture_thread_.join();

    // reset camera settings to defaults from config
    // restore original modes
    for (auto &kv : prev_modes_) {
        mgr_.setMode(kv.first, kv.second);
    }

    prev_modes_.clear();
    return missed_frames_;
}


int CalibrationSession::startMonoJob(const std::string &id, int bw, int bh) {
    int job_id;
    {
        std::lock_guard<std::mutex> lk(mtx_);
        job_id = next_job_id_++;
        mono_jobs_[job_id] = MonoJob{};
    }

    std::thread([this, job_id, id, bw, bh]() {
        auto update = [this, job_id](int p) {
            std::lock_guard<std::mutex> lk(mtx_);
            auto it = mono_jobs_.find(job_id);
            if (it != mono_jobs_.end()) it->second.progress = p;
        };

        std::string dev = mgr_.devicePath(id);
        if (dev.empty()) {
            std::lock_guard<std::mutex> lk(mtx_);
            mono_jobs_[job_id].done = true;
            mono_jobs_[job_id].ok = false;
            return;
        }

        ManagerGuard mg(mgr_);
        PreviewGuard pg(preview_, false);
        std::this_thread::sleep_for(std::chrono::milliseconds(500));

        auto dirPath = root_path_ / "calibration" / ("cam_" + id) / "images";
        std::error_code ec;
        std::filesystem::create_directories(dirPath, ec);
        if (ec) {
            std::lock_guard<std::mutex> lk(mtx_);
            auto &job = mono_jobs_[job_id];
            job.done = true;
            job.ok = false;
            return;
        }

        cv::VideoCapture cap(dev);
        if (!cap.isOpened()) {
            std::lock_guard<std::mutex> lk(mtx_);
            auto &job = mono_jobs_[job_id];
            job.done = true;
            job.ok = false;
            return;
        }

        for (int t = 10; t > 0; --t)
            std::this_thread::sleep_for(std::chrono::seconds(1));

        for (int i = 0; i < 50; i++) {
            cv::Mat frame; cap >> frame; if (frame.empty()) break;
            char buf[64]; snprintf(buf, sizeof(buf), "img_%02d.jpg", i);
            auto filePath = dirPath / buf;
            cv::imwrite(filePath.string(), frame);
            update((i + 1) * 50 / 50);
            std::this_thread::sleep_for(std::chrono::seconds(2));
        }
        cap.release();

        auto resultsDir = root_path_ / "calibration" / "results";
        std::filesystem::create_directories(resultsDir, ec);

        std::string outfile =
            (resultsDir / ("cam_" + id + ".yml")).string();
        std::string dirStr = dirPath.string();
        std::string cmd = "opencv_calib_mono -o " + outfile;
        if (bw > 0 && bh > 0)
            cmd += " --board " + std::to_string(bw) + "x" + std::to_string(bh);
        cmd += " " + dirStr + "/img_*.jpg";

        FILE *pipe = popen(cmd.c_str(), "r");
        if (pipe) {
            char line[256];
            std::regex rgx("(\\d+)%");
            while (fgets(line, sizeof(line), pipe)) {
                std::cmatch m;
                if (std::regex_search(line, m, rgx)) {
                    int p = std::stoi(m[1].str());
                    update(50 + p / 2); // map 0-100 -> 50-100
                }
            }
            int rc = pclose(pipe);
            std::lock_guard<std::mutex> lk(mtx_);
            auto &job = mono_jobs_[job_id];
            job.progress = 100;
            job.done = true;
            job.ok = (rc == 0);
        } else {
            std::lock_guard<std::mutex> lk(mtx_);
            auto &job = mono_jobs_[job_id];
            job.done = true;
            job.ok = false;
        }
    }).detach();

    return job_id;
}

SessionResult CalibrationSession::monoProgress(int job_id) {
    SessionResult result{200, json{}};
    std::lock_guard<std::mutex> lk(mtx_);
    auto it = mono_jobs_.find(job_id);
    if (it == mono_jobs_.end()) {
        result.status = 404;
        result.body["error"] = "job not found";
        return result;
    }
    result.body["progress"] = it->second.progress;
    result.body["status"] = it->second.done ? (it->second.ok ? "done" : "error") : "running";
    return result;
}

