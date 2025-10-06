#include "calibration/live_calibrator.h"

#include <algorithm>
#include <cmath>
#include <ctime>
#include <fstream>
#include <numeric>
#include <thread>

#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>

#include "nlohmann/json.hpp"

namespace calibration {
namespace {

std::string currentTimeString() {
    std::time_t now = std::time(nullptr);
    std::tm tm{};
#ifdef _WIN32
    localtime_s(&tm, &now);
#else
    localtime_r(&now, &tm);
#endif
    char buf[32];
    std::strftime(buf, sizeof(buf), "%Y-%m-%d %H:%M:%S", &tm);
    return std::string(buf);
}

void saveMonoCalibrationYML(const MonoCalibrationSummary &summary,
                            const cv::Mat &camera_matrix,
                            const cv::Mat &dist_coeffs,
                            const std::vector<double> &per_view_errors,
                            const std::filesystem::path &results_dir) {
    namespace fs = std::filesystem;
    fs::create_directories(results_dir);
    cv::FileStorage fs_writer(summary.output_file.string(), cv::FileStorage::WRITE);
    fs_writer << "camera_matrix" << camera_matrix;
    fs_writer << "distortion_coefficients" << dist_coeffs;
    fs_writer << "image_width" << summary.image_size.width;
    fs_writer << "image_height" << summary.image_size.height;
    fs_writer << "reprojection_error" << summary.reprojection_error;
    fs_writer << "frames_used" << summary.frames_used;
    fs_writer << "calibration_time" << summary.calibration_time;
    fs_writer << "per_view_errors" << per_view_errors;
    fs_writer.release();
}

void saveStereoCalibrationYML(const StereoCalibrationSummary &summary,
                              const std::filesystem::path &results_dir) {
    namespace fs = std::filesystem;
    fs::create_directories(results_dir);
    cv::FileStorage fs_writer(summary.output_file.string(), cv::FileStorage::WRITE);
    fs_writer << "R" << summary.R;
    fs_writer << "T" << summary.T;
    fs_writer << "E" << summary.E;
    fs_writer << "F" << summary.F;
    fs_writer << "R1" << summary.R1;
    fs_writer << "R2" << summary.R2;
    fs_writer << "P1" << summary.P1;
    fs_writer << "P2" << summary.P2;
    fs_writer << "Q" << summary.Q;
    fs_writer << "stereo_rms_error" << summary.reprojection_error;
    fs_writer << "frames_used" << summary.frames_used;
    fs_writer << "calibration_time" << summary.calibration_time;
    fs_writer.release();
}

} // namespace

std::string hintToString(HintType hint, float value) {
    switch (hint) {
        case HintType::SEARCHING: return "Searching for chessboard...";
        case HintType::TOO_CLOSE: return "Move board AWAY";
        case HintType::TOO_FAR: return "Move board CLOSER";
        case HintType::TOO_FLAT: return "TILT board more";
        case HintType::TOO_TILTED: return "Board too tilted";
        case HintType::MOVE_LEFT: return "Move LEFT";
        case HintType::MOVE_RIGHT: return "Move RIGHT";
        case HintType::MOVE_UP: return "Move UP";
        case HintType::MOVE_DOWN: return "Move DOWN";
        case HintType::HOLD_STILL: {
            int remaining = static_cast<int>(std::round(value));
            return "HOLD STILL... " + std::to_string(std::max(0, remaining)) + "s";
        }
        case HintType::CAPTURED: return "CAPTURED!";
        case HintType::TOO_SIMILAR: return "Move to NEW position";
        case HintType::COMPLETE: return "Calibration COMPLETE!";
        case HintType::STEREO_NOT_BOTH: return "Board visible on ONE camera only";
        case HintType::STEREO_TILT_DIFF: return "Cameras see different tilt";
        case HintType::STEREO_CENTER_DIFF: return "Board in different positions";
        default: return {};
    }
}

MonoCalibrator::MonoCalibrator(std::string camera_id,
                               std::string device,
                               CalibConfig config)
    : camera_id_(std::move(camera_id)),
      device_(std::move(device)),
      config_(config) {
    for (int r = 0; r < config_.pattern_rows; ++r) {
        for (int c = 0; c < config_.pattern_cols; ++c) {
            objp_.emplace_back(static_cast<float>(c) * config_.square_size,
                               static_cast<float>(r) * config_.square_size,
                               0.0f);
        }
    }
}

bool MonoCalibrator::startCamera() {
    if (device_.empty()) {
        return false;
    }

    int cam_index = -1;
    const std::string pattern = "/dev/video";
    auto pos = device_.find(pattern);
    if (pos != std::string::npos) {
        cam_index = std::stoi(device_.substr(pos + pattern.size()));
    }

    if (cam_index < 0) {
        return false;
    }

    cap_.open(cam_index, cv::CAP_V4L2);
    if (!cap_.isOpened()) {
        return false;
    }

    cap_.set(cv::CAP_PROP_BUFFERSIZE, 3);
    cap_.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'));
    cap_.set(cv::CAP_PROP_FRAME_WIDTH, 800);
    cap_.set(cv::CAP_PROP_FRAME_HEIGHT, 600);
    cap_.set(cv::CAP_PROP_FPS, 30);

    if (static_cast<int>(cap_.get(cv::CAP_PROP_FRAME_WIDTH)) != 800) {
        cap_.set(cv::CAP_PROP_FRAME_WIDTH, 640);
        cap_.set(cv::CAP_PROP_FRAME_HEIGHT, 480);
    }

    actual_img_size_ = cv::Size(static_cast<int>(cap_.get(cv::CAP_PROP_FRAME_WIDTH)),
                                static_cast<int>(cap_.get(cv::CAP_PROP_FRAME_HEIGHT)));

    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    cv::Mat dummy;
    for (int i = 0; i < 10; ++i) {
        cap_.read(dummy);
    }

    is_running_ = true;
    return true;
}

void MonoCalibrator::stopCamera() {
    is_running_ = false;
    cap_.release();
}

void MonoCalibrator::startCalibration() {
    std::lock_guard<std::mutex> lk(data_mutex_);
    object_points_.clear();
    image_points_.clear();
    hold_start_time_ = 0;
    is_calibrating_ = true;
}

int MonoCalibrator::framesCollected() const {
    std::lock_guard<std::mutex> lk(data_mutex_);
    return static_cast<int>(image_points_.size());
}

bool MonoCalibrator::getFrame(cv::Mat &frame, std::string &hint_text,
                              int &progress_current, int &progress_max,
                              bool &pattern_visible) {
    pattern_visible = false;
    if (!is_running_) {
        return false;
    }

    if (!cap_.read(frame) || frame.empty()) {
        frame = cv::Mat::zeros(actual_img_size_.height > 0 ? actual_img_size_.height : 480,
                               actual_img_size_.width > 0 ? actual_img_size_.width : 640,
                               CV_8UC3);
        cv::putText(frame, "Waiting...", {20, frame.rows / 2},
                    cv::FONT_HERSHEY_SIMPLEX, 1.0, {0, 255, 255}, 2);
        hint_text = "Waiting for camera...";
        progress_current = 0;
        progress_max = config_.max_frames;
        return true;
    }

    HintType hint = HintType::SEARCHING;
    float hint_value = 0.0f;

    if (is_calibrating_) {
        std::vector<cv::Point2f> corners;
        bool found = cv::findChessboardCorners(frame,
                                               cv::Size(config_.pattern_cols, config_.pattern_rows),
                                               corners,
                                               cv::CALIB_CB_ADAPTIVE_THRESH |
                                                   cv::CALIB_CB_NORMALIZE_IMAGE |
                                                   cv::CALIB_CB_FAST_CHECK);
        if (found) {
            pattern_visible = true;
            cv::Mat gray;
            cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
            cv::cornerSubPix(gray, corners, {11, 11}, {-1, -1},
                             cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::MAX_ITER,
                                              30, 0.001));
            cv::drawChessboardCorners(frame,
                                      cv::Size(config_.pattern_cols, config_.pattern_rows),
                                      corners, true);

            bool pose_ok = true;

            cv::Point2f sum(0.0f, 0.0f);
            for (const auto &pt : corners) {
                sum += pt;
            }
            cv::Point2f center = sum * (1.0f / static_cast<float>(corners.size()));
            cv::Point2f norm_center(center.x / static_cast<float>(frame.cols),
                                    center.y / static_cast<float>(frame.rows));
            float diff_x = norm_center.x - 0.5f;
            float diff_y = norm_center.y - 0.5f;

            if (diff_x < -config_.max_center_diff_horizontal) {
                hint = HintType::MOVE_RIGHT;
                pose_ok = false;
            } else if (diff_x > config_.max_center_diff_horizontal) {
                hint = HintType::MOVE_LEFT;
                pose_ok = false;
            } else if (diff_y < -config_.max_center_diff_vertical) {
                hint = HintType::MOVE_DOWN;
                pose_ok = false;
            } else if (diff_y > config_.max_center_diff_vertical) {
                hint = HintType::MOVE_UP;
                pose_ok = false;
            }

            if (pose_ok) {
                std::vector<cv::Point2f> hull;
                cv::convexHull(corners, hull);
                double area = std::abs(cv::contourArea(hull));
                double frame_area = static_cast<double>(frame.cols) * frame.rows;
                float coverage = frame_area > 0.0 ? static_cast<float>(area / frame_area) : 0.0f;

                if (coverage < config_.min_coverage) {
                    hint = HintType::TOO_FAR;
                    pose_ok = false;
                } else if (coverage > config_.max_coverage) {
                    hint = HintType::TOO_CLOSE;
                    pose_ok = false;
                }
            }

            if (pose_ok) {
                const cv::Point2f &start = corners.front();
                const cv::Point2f &end = corners[config_.pattern_cols - 1];
                float tilt = std::atan2(end.y - start.y, end.x - start.x) * 180.0f / CV_PI;
                float abs_tilt = std::abs(tilt);
                float min_tilt = config_.max_tilt_diff * 0.2f;

                if (abs_tilt < min_tilt) {
                    hint = HintType::TOO_FLAT;
                    pose_ok = false;
                } else if (abs_tilt > config_.max_tilt_diff) {
                    hint = HintType::TOO_TILTED;
                    pose_ok = false;
                }
            }


            if (pose_ok) {
                if (hold_start_time_ == 0) {
                    hold_start_time_ = std::time(nullptr);
                }

                auto elapsed = std::time(nullptr) - hold_start_time_;
                if (elapsed < hold_duration_) {
                    hint = HintType::HOLD_STILL;
                    hint_value = static_cast<float>(hold_duration_ - elapsed);
                } else {
                    {
                        std::lock_guard<std::mutex> lk(data_mutex_);
                        object_points_.push_back(objp_);
                        image_points_.push_back(corners);
                    }
                    hold_start_time_ = 0;
                    hint = HintType::CAPTURED;
                    if (framesCollected() >= config_.max_frames) {
                        is_calibrating_ = false;
                        hint = HintType::COMPLETE;
                    }
                }
            } else {
                hold_start_time_ = 0;
            }
        } else {
            hold_start_time_ = 0;
        }
    }

    {
        std::lock_guard<std::mutex> lk(data_mutex_);
        progress_current = static_cast<int>(image_points_.size());
        progress_max = config_.max_frames;
    }

    hint_text = hintToString(hint, hint_value);

    cv::Scalar hint_color =
        (hint == HintType::CAPTURED || hint == HintType::COMPLETE)
            ? cv::Scalar(0, 255, 0)
            : cv::Scalar(0, 165, 255);
    cv::putText(frame, hint_text, {20, 40}, cv::FONT_HERSHEY_SIMPLEX, 0.8,
                hint_color, 2);

    if (is_calibrating_) {
        std::string progress_str = std::to_string(progress_current) + "/" +
                                   std::to_string(progress_max);
        cv::putText(frame, progress_str, {20, 80}, cv::FONT_HERSHEY_SIMPLEX,
                    0.7, cv::Scalar(255, 255, 0), 2);
    }
    return true;
}

bool MonoCalibrator::calibrate(const std::filesystem::path &results_dir,
                               MonoCalibrationSummary &summary,
                               std::string &error_msg) {
    std::vector<std::vector<cv::Point3f>> object_points;
    std::vector<std::vector<cv::Point2f>> image_points;
    {
        std::lock_guard<std::mutex> lk(data_mutex_);
        object_points = object_points_;
        image_points = image_points_;
    }

    if (static_cast<int>(image_points.size()) < config_.min_frames) {
        error_msg = "Not enough frames: " + std::to_string(image_points.size());
        return false;
    }

    cv::Mat camera_matrix, dist_coeffs;
    std::vector<cv::Mat> rvecs, tvecs;

    double rms = cv::calibrateCamera(object_points, image_points, actual_img_size_,
                                     camera_matrix, dist_coeffs, rvecs, tvecs, 0);

    std::vector<double> per_view_errors;
    per_view_errors.reserve(object_points.size());
    for (size_t i = 0; i < object_points.size(); ++i) {
        std::vector<cv::Point2f> proj;
        cv::projectPoints(object_points[i], rvecs[i], tvecs[i],
                          camera_matrix, dist_coeffs, proj);
        double err = 0.0;
        for (size_t j = 0; j < proj.size(); ++j) {
            err += cv::norm(image_points[i][j] - proj[j]);
        }
        per_view_errors.push_back(err / static_cast<double>(proj.size()));
    }

    summary.camera_id = camera_id_;
    summary.frames_used = static_cast<int>(image_points.size());
    summary.image_size = actual_img_size_;
    summary.reprojection_error = rms;
    summary.per_view_errors = per_view_errors;
    summary.calibration_time = currentTimeString();
    summary.output_file = results_dir / ("cam_" + camera_id_ + ".yml");

    saveMonoCalibrationYML(summary, camera_matrix, dist_coeffs,
                           per_view_errors, results_dir);

    error_msg = "RMS: " + std::to_string(rms);
    return true;
}

StereoCalibrator::StereoCalibrator(std::string camera_a,
                                   std::string camera_b,
                                   std::string device_a,
                                   std::string device_b,
                                   CalibConfig config)
    : camera_a_(std::move(camera_a)),
      camera_b_(std::move(camera_b)),
      device_a_(std::move(device_a)),
      device_b_(std::move(device_b)),
      config_(config) {
    for (int r = 0; r < config_.pattern_rows; ++r) {
        for (int c = 0; c < config_.pattern_cols; ++c) {
            objp_.emplace_back(static_cast<float>(c) * config_.square_size,
                               static_cast<float>(r) * config_.square_size,
                               0.0f);
        }
    }
}

bool StereoCalibrator::startCameras() {
    auto open = [](const std::string &device) -> cv::VideoCapture {
        int cam_index = -1;
        const std::string pattern = "/dev/video";
        auto pos = device.find(pattern);
        if (pos != std::string::npos) {
            cam_index = std::stoi(device.substr(pos + pattern.size()));
        }
        cv::VideoCapture cap;
        if (cam_index >= 0) {
            cap.open(cam_index, cv::CAP_V4L2);
        }
        return cap;
    };

    cap_a_ = open(device_a_);
    cap_b_ = open(device_b_);

    if (!cap_a_.isOpened() || !cap_b_.isOpened()) {
        return false;
    }

    auto configure = [](cv::VideoCapture &cap) {
        cap.set(cv::CAP_PROP_BUFFERSIZE, 3);
        cap.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'));
        cap.set(cv::CAP_PROP_FRAME_WIDTH, 800);
        cap.set(cv::CAP_PROP_FRAME_HEIGHT, 600);
        cap.set(cv::CAP_PROP_FPS, 30);
        if (static_cast<int>(cap.get(cv::CAP_PROP_FRAME_WIDTH)) != 800) {
            cap.set(cv::CAP_PROP_FRAME_WIDTH, 640);
            cap.set(cv::CAP_PROP_FRAME_HEIGHT, 480);
        }
    };

    configure(cap_a_);
    configure(cap_b_);

    actual_img_size_ = cv::Size(static_cast<int>(cap_a_.get(cv::CAP_PROP_FRAME_WIDTH)),
                                static_cast<int>(cap_a_.get(cv::CAP_PROP_FRAME_HEIGHT)));

    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    for (int i = 0; i < 10; ++i) {
        cap_a_.grab();
        cap_b_.grab();
    }

    is_running_ = true;
    return true;
}

void StereoCalibrator::stopCameras() {
    is_running_ = false;
    cap_a_.release();
    cap_b_.release();
}

void StereoCalibrator::startCalibration() {
    std::lock_guard<std::mutex> lk(data_mutex_);
    object_points_.clear();
    image_points_a_.clear();
    image_points_b_.clear();
    hold_start_time_ = 0;
    is_calibrating_ = true;
}

int StereoCalibrator::framesCollected() const {
    std::lock_guard<std::mutex> lk(data_mutex_);
    return static_cast<int>(image_points_a_.size());
}

bool StereoCalibrator::getFrame(cv::Mat &combined, std::string &hint_text,
                                int &progress_current, int &progress_max) {
    if (!is_running_) {
        return false;
    }

    cv::Mat frame_a, frame_b;
    bool ok_a = cap_a_.read(frame_a);
    bool ok_b = cap_b_.read(frame_b);

    if (!ok_a || !ok_b || frame_a.empty() || frame_b.empty()) {
        if (combined.empty()) {
            combined = cv::Mat::zeros(actual_img_size_.height > 0 ? actual_img_size_.height : 480,
                                      (actual_img_size_.width > 0 ? actual_img_size_.width : 640) * 2,
                                      CV_8UC3);
        }
        hint_text = "Waiting for cameras...";
        return true;
    }

    if (frame_a.size() != frame_b.size()) {
        cv::resize(frame_b, frame_b, frame_a.size());
    }
    if (frame_a.type() != frame_b.type()) {
        if (frame_a.channels() == 1) cv::cvtColor(frame_a, frame_a, cv::COLOR_GRAY2BGR);
        if (frame_b.channels() == 1) cv::cvtColor(frame_b, frame_b, cv::COLOR_GRAY2BGR);
    }

    HintType hint = HintType::SEARCHING;
    float hint_value = 0.0f;

    if (is_calibrating_) {
        std::vector<cv::Point2f> corners_a, corners_b;
        bool found_a = cv::findChessboardCorners(frame_a,
                                                 cv::Size(config_.pattern_cols, config_.pattern_rows),
                                                 corners_a,
                                                 cv::CALIB_CB_ADAPTIVE_THRESH |
                                                     cv::CALIB_CB_NORMALIZE_IMAGE |
                                                     cv::CALIB_CB_FAST_CHECK);
        bool found_b = cv::findChessboardCorners(frame_b,
                                                 cv::Size(config_.pattern_cols, config_.pattern_rows),
                                                 corners_b,
                                                 cv::CALIB_CB_ADAPTIVE_THRESH |
                                                     cv::CALIB_CB_NORMALIZE_IMAGE |
                                                     cv::CALIB_CB_FAST_CHECK);
        if (found_a && found_b) {
            cv::Mat gray_a, gray_b;
            cv::cvtColor(frame_a, gray_a, cv::COLOR_BGR2GRAY);
            cv::cvtColor(frame_b, gray_b, cv::COLOR_BGR2GRAY);
            cv::cornerSubPix(gray_a, corners_a, {11, 11}, {-1, -1},
                             cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::MAX_ITER,
                                              30, 0.001));
            cv::cornerSubPix(gray_b, corners_b, {11, 11}, {-1, -1},
                             cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::MAX_ITER,
                                              30, 0.001));
            cv::drawChessboardCorners(frame_a,
                                      cv::Size(config_.pattern_cols, config_.pattern_rows),
                                      corners_a, true);
            cv::drawChessboardCorners(frame_b,
                                      cv::Size(config_.pattern_cols, config_.pattern_rows),
                                      corners_b, true);

            hint = checkStereoQuality(corners_a, corners_b, frame_a.size(), hint_value);
            if (hint == HintType::CAPTURED) {
                std::lock_guard<std::mutex> lk(data_mutex_);
                object_points_.push_back(objp_);
                image_points_a_.push_back(corners_a);
                image_points_b_.push_back(corners_b);
                hold_start_time_ = 0;
                if (image_points_a_.size() >= static_cast<size_t>(config_.max_frames)) {
                    is_calibrating_ = false;
                    hint = HintType::COMPLETE;
                }
            }
        } else if (found_a || found_b) {
            hint = HintType::STEREO_NOT_BOTH;
            hold_start_time_ = 0;
        } else {
            hold_start_time_ = 0;
        }
    }

    cv::hconcat(frame_a, frame_b, combined);
    cv::line(combined, {frame_a.cols, 0},
             {frame_a.cols, combined.rows}, cv::Scalar(255, 255, 255), 2);

    {
        std::lock_guard<std::mutex> lk(data_mutex_);
        progress_current = static_cast<int>(image_points_a_.size());
        progress_max = config_.max_frames;
    }

    hint_text = hintToString(hint, hint_value);
    cv::Scalar hint_color =
        (hint == HintType::CAPTURED || hint == HintType::COMPLETE)
            ? cv::Scalar(0, 255, 0)
            : cv::Scalar(0, 165, 255);
    cv::putText(combined, hint_text, {20, 40}, cv::FONT_HERSHEY_SIMPLEX, 0.8,
                hint_color, 2);

    if (is_calibrating_) {
        std::string progress_str = std::to_string(progress_current) + "/" +
                                   std::to_string(progress_max);
        cv::putText(combined, progress_str, {20, 80},
                    cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(255, 255, 0),
                    2);
    }

    return true;
}

HintType StereoCalibrator::checkStereoQuality(const std::vector<cv::Point2f> &c1,
                                              const std::vector<cv::Point2f> &c2,
                                              cv::Size img_size,
                                              float &hint_value) {
    cv::Point2f center1 = std::accumulate(c1.begin(), c1.end(), cv::Point2f()) /
                          static_cast<float>(c1.size());
    cv::Point2f center2 = std::accumulate(c2.begin(), c2.end(), cv::Point2f()) /
                          static_cast<float>(c2.size());

    cv::Point2f norm1(center1.x / img_size.width, center1.y / img_size.height);
    cv::Point2f norm2(center2.x / img_size.width, center2.y / img_size.height);

    float diff_x = std::abs(norm1.x - norm2.x);
    float diff_y = std::abs(norm1.y - norm2.y);

    if (diff_x > config_.max_center_diff_horizontal ||
        diff_y > config_.max_center_diff_vertical) {
        hold_start_time_ = 0;
        return HintType::STEREO_CENTER_DIFF;
    }

    float tilt1 = std::atan2(c1[config_.pattern_cols - 1].y - c1[0].y,
                             c1[config_.pattern_cols - 1].x - c1[0].x) * 180.0f / CV_PI;
    float tilt2 = std::atan2(c2[config_.pattern_cols - 1].y - c2[0].y,
                             c2[config_.pattern_cols - 1].x - c2[0].x) * 180.0f / CV_PI;

    if (std::abs(tilt1 - tilt2) > config_.max_tilt_diff) {
        hold_start_time_ = 0;
        return HintType::STEREO_TILT_DIFF;
    }

    if (!image_points_a_.empty()) {
        float diff = 0.0f;
        for (size_t i = 0; i < c1.size(); ++i) {
            diff += cv::norm(c1[i] - image_points_a_.back()[i]);
        }
        diff /= static_cast<float>(c1.size());
        float norm_diff = diff / std::sqrt(static_cast<float>(img_size.width * img_size.width +
                                                              img_size.height * img_size.height));
        if (norm_diff < config_.min_distance_between_frames) {
            hold_start_time_ = 0;
            return HintType::TOO_SIMILAR;
        }
    }

    if (hold_start_time_ == 0) {
        hold_start_time_ = std::time(nullptr);
    }

    auto elapsed = std::time(nullptr) - hold_start_time_;
    if (elapsed < hold_duration_) {
        hint_value = static_cast<float>(hold_duration_ - elapsed);
        return HintType::HOLD_STILL;
    }

    return HintType::CAPTURED;
}

bool StereoCalibrator::calibrate(const std::filesystem::path &results_dir,
                                 std::optional<MonoCalibrationSummary> &mono_a,
                                 std::optional<MonoCalibrationSummary> &mono_b,
                                 StereoCalibrationSummary &stereo,
                                 std::string &error_msg,
                                 bool fix_intrinsics) {
    std::vector<std::vector<cv::Point3f>> object_points;
    std::vector<std::vector<cv::Point2f>> points_a;
    std::vector<std::vector<cv::Point2f>> points_b;
    {
        std::lock_guard<std::mutex> lk(data_mutex_);
        object_points = object_points_;
        points_a = image_points_a_;
        points_b = image_points_b_;
    }

    if (points_a.size() < static_cast<size_t>(config_.min_frames)) {
        error_msg = "Not enough frames: " + std::to_string(points_a.size());
        return false;
    }

    cv::Mat K1, D1, K2, D2, R, T, E, F;
    int flags = 0;

    if (fix_intrinsics) {
        cv::FileStorage fs1((results_dir / ("cam_" + camera_a_ + ".yml")).string(), cv::FileStorage::READ);
        cv::FileStorage fs2((results_dir / ("cam_" + camera_b_ + ".yml")).string(), cv::FileStorage::READ);
        if (fs1.isOpened() && fs2.isOpened()) {
            fs1["camera_matrix"] >> K1;
            fs1["distortion_coefficients"] >> D1;
            fs2["camera_matrix"] >> K2;
            fs2["distortion_coefficients"] >> D2;
            if (!K1.empty() && !D1.empty() && !K2.empty() && !D2.empty()) {
                flags = cv::CALIB_FIX_INTRINSIC;
            }
        }
    }

    double rms = cv::stereoCalibrate(object_points, points_a, points_b,
                                     K1, D1, K2, D2, actual_img_size_, R, T, E, F,
                                     flags,
                                     cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::MAX_ITER,
                                                      100, 1e-5));
    cv::Mat R1, R2, P1, P2, Q;
    cv::stereoRectify(K1, D1, K2, D2, actual_img_size_, R, T, R1, R2, P1, P2, Q);

    std::vector<double> errors_a(points_a.size());
    std::vector<double> errors_b(points_b.size());
    for (size_t i = 0; i < object_points.size(); ++i) {
        cv::Mat rvec1, tvec1, rvec2, tvec2;
        cv::solvePnP(object_points[i], points_a[i], K1, D1, rvec1, tvec1);
        cv::solvePnP(object_points[i], points_b[i], K2, D2, rvec2, tvec2);

        std::vector<cv::Point2f> proj1, proj2;
        cv::projectPoints(object_points[i], rvec1, tvec1, K1, D1, proj1);
        cv::projectPoints(object_points[i], rvec2, tvec2, K2, D2, proj2);

        double err1 = 0.0, err2 = 0.0;
        for (size_t j = 0; j < proj1.size(); ++j) {
            err1 += cv::norm(points_a[i][j] - proj1[j]);
            err2 += cv::norm(points_b[i][j] - proj2[j]);
        }
        errors_a[i] = err1 / static_cast<double>(proj1.size());
        errors_b[i] = err2 / static_cast<double>(proj2.size());
    }

    MonoCalibrationSummary mono_summary_a;
    mono_summary_a.camera_id = camera_a_;
    mono_summary_a.frames_used = static_cast<int>(points_a.size());
    mono_summary_a.image_size = actual_img_size_;
    mono_summary_a.reprojection_error = std::accumulate(errors_a.begin(), errors_a.end(), 0.0) /
                                        static_cast<double>(errors_a.size());
    mono_summary_a.per_view_errors = errors_a;
    mono_summary_a.calibration_time = currentTimeString();
    mono_summary_a.output_file = results_dir / ("cam_" + camera_a_ + ".yml");

    MonoCalibrationSummary mono_summary_b;
    mono_summary_b.camera_id = camera_b_;
    mono_summary_b.frames_used = static_cast<int>(points_b.size());
    mono_summary_b.image_size = actual_img_size_;
    mono_summary_b.reprojection_error = std::accumulate(errors_b.begin(), errors_b.end(), 0.0) /
                                        static_cast<double>(errors_b.size());
    mono_summary_b.per_view_errors = errors_b;
    mono_summary_b.calibration_time = mono_summary_a.calibration_time;
    mono_summary_b.output_file = results_dir / ("cam_" + camera_b_ + ".yml");

    saveMonoCalibrationYML(mono_summary_a, K1, D1, errors_a, results_dir);
    saveMonoCalibrationYML(mono_summary_b, K2, D2, errors_b, results_dir);

    stereo.camera_a = camera_a_;
    stereo.camera_b = camera_b_;
    stereo.frames_used = static_cast<int>(points_a.size());
    stereo.reprojection_error = rms;
    stereo.calibration_time = mono_summary_a.calibration_time;
    stereo.output_file = results_dir / ("stereo_" + camera_a_ + "_" + camera_b_ + ".yml");

    stereo.R = R.clone();
    stereo.T = T.clone();
    stereo.E = E.clone();
    stereo.F = F.clone();
    stereo.R1 = R1.clone();
    stereo.R2 = R2.clone();
    stereo.P1 = P1.clone();
    stereo.P2 = P2.clone();
    stereo.Q = Q.clone();

    saveStereoCalibrationYML(stereo, results_dir);

    mono_a = mono_summary_a;
    mono_b = mono_summary_b;
    error_msg = "Stereo RMS: " + std::to_string(rms);
    return true;
}

void updateCalibrationResults(const std::vector<MonoCalibrationSummary> &mono,
                              const std::vector<StereoCalibrationSummary> &stereo,
                              const std::filesystem::path &results_dir) {
    namespace fs = std::filesystem;
    fs::create_directories(results_dir);
    fs::path json_path = results_dir / "calibration_results.json";

    nlohmann::json data;
    if (fs::exists(json_path)) {
        std::ifstream in(json_path);
        if (in.good()) {
            try {
                in >> data;
            } catch (...) {
                data = nlohmann::json::object();
            }
        }
    }

    if (!data.is_object()) {
        data = nlohmann::json::object();
    }

    if (!data.contains("mono_calibrations") || !data["mono_calibrations"].is_array()) {
        data["mono_calibrations"] = nlohmann::json::array();
    }
    if (!data.contains("stereo_calibrations") || !data["stereo_calibrations"].is_array()) {
        data["stereo_calibrations"] = nlohmann::json::array();
    }

    auto &mono_array = data["mono_calibrations"];
    auto &stereo_array = data["stereo_calibrations"];

    auto upsert_mono = [&mono_array](const MonoCalibrationSummary &summary) {
        for (auto &entry : mono_array) {
            if (entry.contains("camera_id") && entry["camera_id"].get<std::string>() == summary.camera_id) {
                entry["calibration_file"] = summary.output_file.filename().string();
                entry["calibration_time"] = summary.calibration_time;
                entry["frames_used"] = summary.frames_used;
                entry["image_width"] = summary.image_size.width;
                entry["image_height"] = summary.image_size.height;
                entry["reprojection_error"] = summary.reprojection_error;
                entry["success"] = true;
                if (!entry.contains("mode")) entry["mode"] = "";
                return;
            }
        }
        nlohmann::json entry = nlohmann::json::object();
        entry["camera_id"] = summary.camera_id;
        entry["calibration_file"] = summary.output_file.filename().string();
        entry["calibration_time"] = summary.calibration_time;
        entry["frames_used"] = summary.frames_used;
        entry["image_width"] = summary.image_size.width;
        entry["image_height"] = summary.image_size.height;
        entry["mode"] = "";
        entry["reprojection_error"] = summary.reprojection_error;
        entry["success"] = true;
        mono_array.push_back(std::move(entry));
    };

    auto upsert_stereo = [&stereo_array](const StereoCalibrationSummary &summary) {
        std::string key = summary.camera_a + "_" + summary.camera_b;
        for (auto &entry : stereo_array) {
            if (entry.contains("camera_pair") && entry["camera_pair"].get<std::string>() == key) {
                entry["calibration_file"] = summary.output_file.filename().string();
                entry["calibration_time"] = summary.calibration_time;
                entry["reprojection_error"] = summary.reprojection_error;
                entry["frames_used"] = summary.frames_used;
                entry["success"] = true;
                return;
            }
        }
        nlohmann::json entry = nlohmann::json::object();
        entry["camera_pair"] = key;
        entry["calibration_file"] = summary.output_file.filename().string();
        entry["calibration_time"] = summary.calibration_time;
        entry["reprojection_error"] = summary.reprojection_error;
        entry["frames_used"] = summary.frames_used;
        entry["success"] = true;
        stereo_array.push_back(std::move(entry));
    };

    for (const auto &m : mono) {
        upsert_mono(m);
    }
    for (const auto &s : stereo) {
        upsert_stereo(s);
    }

    std::ofstream out(json_path);
    out << data.dump(2);
}

} // namespace calibration

