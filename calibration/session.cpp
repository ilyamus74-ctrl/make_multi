#include "calibration/session.h"
#include "camera_manager.h"
#include "httplib.h"
#include <opencv2/opencv.hpp>
#include <opencv2/imgcodecs.hpp>
#include <filesystem>
#include <thread>
#include <chrono>
#include <algorithm>
#include <vector>
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

    calibration::MonoCalibrator calibrator(id, dev, calib_config_);
    if (!calibrator.startCamera()) {
        result.status = 500;
        result.body["error"] = "open camera";
        return result;
    }

    calibrator.startCalibration();

    for (int guard = 0; guard < calib_config_.max_frames * 200 && calibrator.isCalibrating(); ++guard) {
        cv::Mat frame;
        std::string hint;
        int progress = 0;
        int progress_max = 0;
        bool pattern_visible = false;
        if (!calibrator.getFrame(frame, hint, progress, progress_max, pattern_visible)) {
            break;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(33));
    }
    calibrator.stopCamera();

    auto resultsDir = root_path_ / "calibration" / "results";
    std::error_code ec;

    calibration::MonoCalibrationSummary summary;
    std::string err;
    bool ok = calibrator.calibrate(resultsDir, summary, err);
    if (!ok) {
        result.status = 500;
        result.body["error"] = err;
        return result;
    }

    calibration::updateCalibrationResults({summary}, {}, resultsDir);

    result.body["status"] = "ok";
    result.body["camera"] = summary.camera_id;
    result.body["frames"] = summary.frames_used;
    result.body["rms"] = summary.reprojection_error;
    result.body["file"] = summary.output_file.string();
    result.body["calibration_time"] = summary.calibration_time;
    result.body["per_view_errors"] = summary.per_view_errors;

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
    ManagerGuard mg(mgr_);
    PreviewGuard pg(preview_, false);

    std::string dev_a = mgr_.devicePath(ids[0]);
    std::string dev_b = mgr_.devicePath(ids[1]);
    if (dev_a.empty() || dev_b.empty()) {
        result.status = 400;
        result.body["error"] = "camera not found";
        return result;
    }

    calibration::CalibConfig config = calib_config_;
    if (frames > 0) {
        config.max_frames = frames;
        if (config.min_frames > config.max_frames) {
            config.min_frames = std::max(5, config.max_frames / 2);
        }
    }

    calibration::StereoCalibrator calibrator(ids[0], ids[1], dev_a, dev_b, config);
    if (!calibrator.startCameras()) {
        result.status = 500;
        result.body["error"] = "open cameras";
        return result;
    }

    calibrator.startCalibration();

    int sleep_interval = interval > 0 ? interval : 33;
    for (int guard = 0; guard < config.max_frames * 400 && calibrator.isCalibrating(); ++guard) {
        cv::Mat frame;
        std::string hint;
        int progress = 0;
        int progress_max = 0;
        if (!calibrator.getFrame(frame, hint, progress, progress_max)) {
            break;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(sleep_interval));
    }


    calibrator.stopCameras();

    auto resultsDir = root_path_ / "calibration" / "results";
    std::error_code ec;
    std::filesystem::create_directories(resultsDir, ec);

    std::optional<calibration::MonoCalibrationSummary> mono_a;
    std::optional<calibration::MonoCalibrationSummary> mono_b;
    calibration::StereoCalibrationSummary stereo_summary;
    std::string err;
    bool ok = calibrator.calibrate(resultsDir, mono_a, mono_b, stereo_summary, err, false);
    if (!ok) {
        result.status = 500;
        result.body["error"] = err;
        return result;
    }

    std::vector<calibration::MonoCalibrationSummary> mono_updates;
    if (mono_a) mono_updates.push_back(*mono_a);
    if (mono_b) mono_updates.push_back(*mono_b);
    calibration::updateCalibrationResults(mono_updates, {stereo_summary}, resultsDir);


    result.body["status"] = "ok";
    result.body["stereo_rms"] = stereo_summary.reprojection_error;
    result.body["frames"] = stereo_summary.frames_used;
    result.body["file"] = stereo_summary.output_file.string();
    result.body["calibration_time"] = stereo_summary.calibration_time;
    if (mono_a) {
        result.body["mono_a"] = {
            {"camera", mono_a->camera_id},
            {"frames", mono_a->frames_used},
            {"rms", mono_a->reprojection_error},
            {"file", mono_a->output_file.string()},
            {"per_view_errors", mono_a->per_view_errors}
        };
    }
    if (mono_b) {
        result.body["mono_b"] = {
            {"camera", mono_b->camera_id},
            {"frames", mono_b->frames_used},
            {"rms", mono_b->reprojection_error},
            {"file", mono_b->output_file.string()},
            {"per_view_errors", mono_b->per_view_errors}
        };
    }

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


int CalibrationSession::startMonoJob(const std::string &id, int bw, int bh,
                                     int max_frames, int frame_interval_ms) {
    int job_id;
    {
        std::lock_guard<std::mutex> lk(mtx_);
        job_id = next_job_id_++;
        MonoJob job{};
        job.camera_id = id;
        job.frames_needed = calib_config_.max_frames;
        mono_jobs_[job_id] = std::move(job);
    }

    std::thread([this, job_id, id, bw, bh, max_frames, frame_interval_ms]() {
        auto update_job = [this, job_id](int percent, int frames_collected,
                                         int frames_needed, const std::string &hint,
                                         bool board_visible, const std::string &preview,
                                         bool processing_flag = false) {
            std::lock_guard<std::mutex> lk(mtx_);
            auto it = mono_jobs_.find(job_id);
            if (it == mono_jobs_.end()) return;
            auto &job = it->second;
            job.progress = std::min(std::max(percent, 0), 100);
            job.frames_collected = std::max(frames_collected, 0);
            if (frames_needed > 0) {
                job.frames_needed = frames_needed;
            }
            job.hint = hint;
            job.board_visible = board_visible;
            job.processing = processing_flag;
            if (!preview.empty()) {
                job.preview_image = preview;
            }
        };

        auto mark_failure = [this, job_id](const std::string &err) {
            std::lock_guard<std::mutex> lk(mtx_);
            auto it = mono_jobs_.find(job_id);
            if (it == mono_jobs_.end()) return;
            auto &job = it->second;
            job.done = true;
            job.ok = false;
            job.error = err;
            job.hint = err;
            job.board_visible = false;
            job.processing = false;
        };

        std::string dev = mgr_.devicePath(id);
        if (dev.empty()) {
            mark_failure("camera not found");
            return;
        }

        ManagerGuard mg(mgr_);
        PreviewGuard pg(preview_, false);
        calibration::CalibConfig config = calib_config_;
        if (bw > 0 && bh > 0) {
            config.pattern_cols = bw;
            config.pattern_rows = bh;
        }
        if (max_frames > 0) {
            config.max_frames = max_frames;
            if (config.min_frames > config.max_frames) {
                config.min_frames = std::max(5, config.max_frames / 2);
            }
        }

        {
            std::lock_guard<std::mutex> lk(mtx_);
            auto it = mono_jobs_.find(job_id);
            if (it != mono_jobs_.end()) {
                it->second.frames_needed = config.max_frames;
            }
        }

        calibration::MonoCalibrator calibrator(id, dev, config);
        if (!calibrator.startCamera()) {
            mark_failure("failed to open camera");
            return;
        }

        calibrator.startCalibration();

        const int sleep_interval = frame_interval_ms > 0 ? frame_interval_ms : 33;
        bool first_preview = true;
        auto last_preview_sent = std::chrono::steady_clock::now();
        int last_reported_frames = -1;

        for (int guard = 0; guard < config.max_frames * 200 && calibrator.isCalibrating(); ++guard) {
            cv::Mat frame;
            std::string hint;
            int progress = 0;
            int progress_max = 0;
            bool pattern_visible = false;
            if (!calibrator.getFrame(frame, hint, progress, progress_max, pattern_visible)) {
                break;
            }

            int frames_needed = progress_max > 0 ? progress_max : config.max_frames;
            if (frames_needed <= 0) {
                frames_needed = config.max_frames;
            }
            int percent = frames_needed > 0 ? (progress * 100) / frames_needed : 0;
            percent = std::min(percent, 99);

            std::string preview_data;
            auto now = std::chrono::steady_clock::now();
            if (first_preview ||
                std::chrono::duration_cast<std::chrono::milliseconds>(now - last_preview_sent).count() >= 250 ||
                progress != last_reported_frames) {
                if (!frame.empty()) {
                    std::vector<uchar> buffer;
                    if (cv::imencode(".jpg", frame, buffer, {cv::IMWRITE_JPEG_QUALITY, 80})) {
                        std::string encoded(reinterpret_cast<const char *>(buffer.data()), buffer.size());
                        preview_data = "data:image/jpeg;base64," +
                                       httplib::detail::base64_encode(encoded);
                    }
                }
                last_preview_sent = now;
                first_preview = false;
            }

            update_job(percent, progress, frames_needed, hint, pattern_visible, preview_data);
            last_reported_frames = progress;
            std::this_thread::sleep_for(std::chrono::milliseconds(sleep_interval));
        }

        int frames_captured = last_reported_frames >= 0 ? last_reported_frames : 0;
        update_job(100, frames_captured, config.max_frames,
                   "Processing captured frames...", false, "", true);

        calibrator.stopCamera();

        auto resultsDir = root_path_ / "calibration" / "results";
        std::error_code ec;
        std::filesystem::create_directories(resultsDir, ec);

        calibration::MonoCalibrationSummary summary;
        std::string err;
        bool ok = calibrator.calibrate(resultsDir, summary, err);

        if (ok) {
            calibration::updateCalibrationResults({summary}, {}, resultsDir);
        }

        {

            std::lock_guard<std::mutex> lk(mtx_);
            auto it = mono_jobs_.find(job_id);
            if (it != mono_jobs_.end()) {
                auto &job = it->second;
                job.progress = 100;
                job.processing = false;
                if (ok) {
                    job.frames_collected = summary.frames_used;
                    job.frames_needed = summary.frames_used > 0 ? summary.frames_used : job.frames_needed;
                    job.done = true;
                    job.ok = true;
                    job.board_visible = false;
                    job.hint = "Calibration complete";
                    job.result = {
                        {"status", "ok"},
                        {"camera", summary.camera_id},
                        {"frames", summary.frames_used},
                        {"rms", summary.reprojection_error},
                        {"file", summary.output_file.string()},
                        {"calibration_time", summary.calibration_time},
                        {"per_view_errors", summary.per_view_errors}
                    };
                } else {
                    job.done = true;
                    job.ok = false;
                    job.board_visible = false;
                    job.error = err;
                    job.hint = err;
                }
            }
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
    result.body["camera"] = it->second.camera_id;
    result.body["progress"] = it->second.progress;
    result.body["frames_collected"] = it->second.frames_collected;
    result.body["frames_needed"] = it->second.frames_needed;
    result.body["hint"] = it->second.hint;
    result.body["board_visible"] = it->second.board_visible;
    if (!it->second.preview_image.empty()) {
        result.body["preview"] = it->second.preview_image;
    }
    std::string status;
    if (it->second.done) {
        status = it->second.ok ? "done" : "error";
    } else if (it->second.processing) {
        status = "processing";
    } else {
        status = "running";
    }
    result.body["status"] = status;
    if (it->second.done) {
        if (it->second.ok) {
            result.body["result"] = it->second.result;
        } else {
            result.body["error"] = it->second.error.empty() ? "mono calibration failed" : it->second.error;
        }
    }
    return result;
}

