#pragma once

#include <atomic>
#include <filesystem>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

#include <opencv2/opencv.hpp>

namespace calibration {

struct CalibConfig {
    int pattern_cols = 15;
    int pattern_rows = 7;
    float square_size = 22.0f;
    int min_frames = 20;
    int max_frames = 100;

    float max_center_diff_horizontal = 0.40f;
    float max_center_diff_vertical = 0.40f;
    float max_tilt_diff = 15.0f;
    float min_coverage = 0.25f;
    float max_coverage = 0.75f;
    float min_distance_between_frames = 0.03f;
};

enum class HintType {
    SEARCHING,
    TOO_CLOSE,
    TOO_FAR,
    TOO_FLAT,
    TOO_TILTED,
    MOVE_LEFT,
    MOVE_RIGHT,
    MOVE_UP,
    MOVE_DOWN,
    HOLD_STILL,
    CAPTURED,
    TOO_SIMILAR,
    COMPLETE,
    STEREO_NOT_BOTH,
    STEREO_TILT_DIFF,
    STEREO_CENTER_DIFF
};

std::string hintToString(HintType hint, float value = 0.0f);

struct MonoCalibrationSummary {
    std::string camera_id;
    std::filesystem::path output_file;
    cv::Size image_size{};
    int frames_used = 0;
    double reprojection_error = 0.0;
    std::vector<double> per_view_errors;
    std::string calibration_time;
};

struct StereoCalibrationSummary {
    std::string camera_a;
    std::string camera_b;
    std::filesystem::path output_file;
    double reprojection_error = 0.0;
    int frames_used = 0;
    std::string calibration_time;
    cv::Mat R;
    cv::Mat T;
    cv::Mat E;
    cv::Mat F;
    cv::Mat R1;
    cv::Mat R2;
    cv::Mat P1;
    cv::Mat P2;
    cv::Mat Q;
};

class MonoCalibrator {
public:
    MonoCalibrator(std::string camera_id,
                   std::string device,
                   CalibConfig config = {});

    bool startCamera();
    void stopCamera();

    void startCalibration();
    bool isRunning() const { return is_running_.load(); }
    bool isCalibrating() const { return is_calibrating_.load(); }

    bool getFrame(cv::Mat &frame, std::string &hint_text,
                  int &progress_current, int &progress_max,
                  bool &pattern_visible);

    bool calibrate(const std::filesystem::path &results_dir,
                   MonoCalibrationSummary &summary,
                   std::string &error_msg);

    cv::Size imageSize() const { return actual_img_size_; }
    int framesCollected() const;

private:
    std::string camera_id_;
    std::string device_;
    CalibConfig config_;

    cv::VideoCapture cap_;
    std::atomic<bool> is_running_{false};
    std::atomic<bool> is_calibrating_{false};

    std::vector<std::vector<cv::Point3f>> object_points_;
    std::vector<std::vector<cv::Point2f>> image_points_;
    std::vector<cv::Point3f> objp_;
    cv::Size actual_img_size_{};

    mutable std::mutex data_mutex_;
    time_t hold_start_time_ = 0;
    double hold_duration_ = 1.0;
};

class StereoCalibrator {
public:
    StereoCalibrator(std::string camera_a,
                     std::string camera_b,
                     std::string device_a,
                     std::string device_b,
                     CalibConfig config = {});

    bool startCameras();
    void stopCameras();

    void startCalibration();
    bool isRunning() const { return is_running_.load(); }
    bool isCalibrating() const { return is_calibrating_.load(); }

    bool getFrame(cv::Mat &combined, std::string &hint_text,
                  int &progress_current, int &progress_max);

    bool calibrate(const std::filesystem::path &results_dir,
                   std::optional<MonoCalibrationSummary> &mono_a,
                   std::optional<MonoCalibrationSummary> &mono_b,
                   StereoCalibrationSummary &stereo,
                   std::string &error_msg,
                   bool fix_intrinsics = false);

    cv::Size imageSize() const { return actual_img_size_; }
    int framesCollected() const;

private:
    HintType checkStereoQuality(const std::vector<cv::Point2f> &c1,
                                const std::vector<cv::Point2f> &c2,
                                cv::Size img_size,
                                float &hint_value);

    std::string camera_a_;
    std::string camera_b_;
    std::string device_a_;
    std::string device_b_;
    CalibConfig config_;

    cv::VideoCapture cap_a_;
    cv::VideoCapture cap_b_;

    std::atomic<bool> is_running_{false};
    std::atomic<bool> is_calibrating_{false};

    std::vector<std::vector<cv::Point3f>> object_points_;
    std::vector<std::vector<cv::Point2f>> image_points_a_;
    std::vector<std::vector<cv::Point2f>> image_points_b_;
    std::vector<cv::Point3f> objp_;
    cv::Size actual_img_size_{};

    mutable std::mutex data_mutex_;
    time_t hold_start_time_ = 0;
    double hold_duration_ = 1.0;
};

void updateCalibrationResults(const std::vector<MonoCalibrationSummary> &mono,
                              const std::vector<StereoCalibrationSummary> &stereo,
                              const std::filesystem::path &results_dir);

} // namespace calibration
