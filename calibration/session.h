#pragma once

#include <string>
#include <vector>
#include <map>
#include <mutex>
#include <atomic>
#include <thread>
#include <filesystem>
#include "nlohmann/json.hpp"
#include "camera_manager.h"
#include "calibration/live_calibrator.h"

class CameraManager; // forward declaration

struct SessionResult {
    int status;           // HTTP status code
    nlohmann::json body;  // JSON response
};

// RAII helper to save/restore preview flag
class PreviewGuard {
public:
    PreviewGuard(bool &flag, bool new_state);
    ~PreviewGuard();
private:
    bool &flag_;
    bool prev_;
};

class CalibrationSession {
public:
    CalibrationSession(CameraManager &mgr, bool &preview_flag,
                       const std::filesystem::path &root);
    void start();
    void stop();

    SessionResult captureMono(const std::string &id);
    SessionResult captureStereo(const std::vector<std::string> &ids, int frames, int interval);

    // Start/stop capturing from all connected cameras.
    // Frames are stored to calibration/cam_<ID>/images/ synchronously.
    void beginCaptureAll();
    std::map<std::string, std::size_t> endCaptureAll();
    // Asynchronous mono calibration
    int startMonoJob(const std::string &id, int board_w, int board_h,
                     int max_frames = 0, int frame_interval_ms = 33);
    SessionResult monoProgress(int job_id);

private:
    CameraManager &mgr_;
    bool &preview_;
    std::filesystem::path root_path_;

    // background capture state
    std::atomic<bool> capturing_{false};
    std::thread capture_thread_{};

    struct MonoJob {
        int progress = 0;      // percentage [0-100]
        int frames_collected = 0;
        int frames_needed = 0;
        bool board_visible = false;
        bool processing = false;
        bool done = false;     // finished running
        bool ok = false;       // succeeded
        std::string hint;
        std::string preview_image;
        std::string camera_id;
        nlohmann::json result;
        std::string error;
    };
    std::mutex mtx_;
    int next_job_id_ = 1;
    std::map<int, MonoJob> mono_jobs_;

    // previous mode per camera
    std::map<std::string, CameraManager::CamConfig::Mode> prev_modes_;
    // missed frame count per camera during background capture
    std::map<std::string, std::size_t> missed_frames_;

    calibration::CalibConfig calib_config_{};
};

