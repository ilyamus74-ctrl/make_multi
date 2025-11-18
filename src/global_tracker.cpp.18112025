#include "global_tracker.h"
#include "calibration_watcher.h"
#include "nlohmann/json.hpp"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <iostream>
#include <limits>
#include <iomanip>
#include <queue>
#include <set>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <sstream>
#include <system_error>

namespace {

bool isPrimaryWideRole(CameraRole role) {
    return role == CameraRole::PRIMARY_WIDE || role == CameraRole::WIDE_ANGLE_PRIMARY;
}

bool isSecondaryWideRole(CameraRole role) {
    return role == CameraRole::SECONDARY_WIDE || role == CameraRole::WIDE_ANGLE_SECONDARY;
}

bool isZoomRole(CameraRole role) {
    return role == CameraRole::ZOOM || role == CameraRole::ZOOM_CAMERA;
}
std::set<std::string> default_height_warned_cameras;

constexpr double kMaxPredictionHorizonSeconds = 0.75;
constexpr double kVelocityResetGapSeconds = 1.5;
constexpr double kConfidenceDecayOnClamp = 0.05;
constexpr int kCameraMissCountThreshold = 10; //было 5
constexpr size_t kResidualSummaryInterval = 50;

std::filesystem::path detectDefaultCalibrationDirectory() {
    std::error_code ec;
    std::filesystem::path exe_path = std::filesystem::read_symlink("/proc/self/exe", ec);
    std::filesystem::path exe_dir;
    if (!ec) {
        exe_dir = exe_path.parent_path();
    }

    const std::vector<std::filesystem::path> candidates = {
        exe_dir / "calibration" / "results",
        exe_dir / "build" / "calibration" / "results",
        exe_dir / "build" / "result",
        std::filesystem::path("calibration") / "results",
        std::filesystem::path("build") / "calibration" / "results",
        std::filesystem::path("build") / "result"
    };

    for (const auto& candidate : candidates) {
        ec.clear();
        auto json_path =  "calibration_results.json";
//        auto json_path = exe_dir / "calibration/results/calibration_results.json";
        if (std::filesystem::exists(json_path, ec)) {
            auto absolute = std::filesystem::absolute(candidate, ec);
            return ec ? candidate : absolute;
        }
        ec.clear();
        if (std::filesystem::exists(candidate, ec)) {
            auto absolute = std::filesystem::absolute(candidate, ec);
            return ec ? candidate : absolute;
        }
    }

    ec.clear();
    auto fallback = std::filesystem::absolute(std::filesystem::path("build") / "result", ec);
    return ec ? std::filesystem::path("build") / "result" : fallback;
}

class DirectoryCalibrationSource {
public:
    explicit DirectoryCalibrationSource(const std::filesystem::path& directory)
        : results_dir_(std::filesystem::absolute(directory)) {}

    bool load() {
        camera_matrices_.clear();
        dist_coeffs_.clear();
        stereo_results_.clear();
        stereo_params_.clear();

        std::filesystem::path json_path = results_dir_ / "calibration_results.json";
//        std::filesystem::path json_path = exe_dir / "calibration/results/calibration_results.json";
        std::ifstream json_file(json_path);
        if (!json_file.is_open()) {
            return false;
        }

        nlohmann::json results;
        try {
            json_file >> results;
        } catch (...) {
            return false;
        }

        if (!results.is_object()) {
            return false;
        }

        bool loaded = false;

        if (results.contains("mono_calibrations") && results["mono_calibrations"].is_array()) {
            for (const auto& mono_json : results["mono_calibrations"]) {
                if (!mono_json.value("success", false)) {
                    continue;
                }

                std::string camera_id = mono_json.value("camera_id", std::string());
                if (camera_id.empty()) {
                    continue;
                }

                std::string mode = CalibrationWatcher::normalizeModeName(
                    mono_json.value("mode", std::string()));

                std::string calibration_file = mono_json.value("calibration_file", std::string());
                if (calibration_file.empty()) {
                    continue;
                }

                std::filesystem::path yaml_path = results_dir_ / calibration_file;
                cv::FileStorage fs(yaml_path.string(), cv::FileStorage::READ);
                if (!fs.isOpened()) {
                    continue;
                }

                cv::Mat camera_matrix;
                cv::Mat dist_coeffs;
                fs["camera_matrix"] >> camera_matrix;
                fs["distortion_coefficients"] >> dist_coeffs;
                fs.release();

                if (camera_matrix.empty() || dist_coeffs.empty()) {
                    continue;
                }

                camera_matrix.convertTo(camera_matrix, CV_64F);
                dist_coeffs.convertTo(dist_coeffs, CV_64F);
                camera_matrices_[camera_id][mode] = camera_matrix.clone();
                dist_coeffs_[camera_id][mode] = dist_coeffs.clone();
                loaded = true;
            }
        }

        if (results.contains("stereo_calibrations") && results["stereo_calibrations"].is_array()) {
            for (const auto& stereo_json : results["stereo_calibrations"]) {
                if (!stereo_json.value("success", false)) {
                    continue;
                }

                std::string camera_pair = stereo_json.value("camera_pair", std::string());
                std::string calibration_file = stereo_json.value("calibration_file", std::string());
                if (camera_pair.empty() || calibration_file.empty()) {
                    continue;
                }

                std::filesystem::path yaml_path = results_dir_ / calibration_file;
                cv::FileStorage fs(yaml_path.string(), cv::FileStorage::READ);
                if (!fs.isOpened()) {
                    continue;
                }

                StereoCalibrationResult result;
                result.camera_pair = camera_pair;
                result.reprojection_error = stereo_json.value("reprojection_error", 0.0);
                result.calibration_time = stereo_json.value("calibration_time", std::string());
                result.success = true;

                fs["R"] >> result.R;
                fs["T"] >> result.T;
                fs["E"] >> result.E;
                fs["F"] >> result.F;
                fs["R1"] >> result.R1;
                fs["R2"] >> result.R2;
                fs["P1"] >> result.P1;
                fs["P2"] >> result.P2;
                fs["Q"] >> result.Q;
                fs.release();

                auto convert_to_double = [](cv::Mat& mat) {
                    if (!mat.empty() && mat.type() != CV_64F) {
                        mat.convertTo(mat, CV_64F);
                    }
                };

                convert_to_double(result.R);
                convert_to_double(result.T);
                convert_to_double(result.E);
                convert_to_double(result.F);
                convert_to_double(result.R1);
                convert_to_double(result.R2);
                convert_to_double(result.P1);
                convert_to_double(result.P2);
                convert_to_double(result.Q);

                stereo_results_.push_back(result);
                stereo_params_[camera_pair] = result;
                loaded = true;
            }
        }

        return loaded;
    }

    bool getCameraMatrix(const std::string& camera_id, cv::Mat& camera_matrix, cv::Mat& dist_coeffs) const {
        auto it = camera_matrices_.find(camera_id);
        auto dist_it = dist_coeffs_.find(camera_id);
        if (it == camera_matrices_.end() || dist_it == dist_coeffs_.end() || it->second.empty()) {
            return false;
        }

        const auto& mode_map = it->second;
        auto mode_it = mode_map.find(std::string());
        if (mode_it == mode_map.end()) {
            mode_it = mode_map.begin();
        }

        const auto& dist_mode_map = dist_it->second;
        auto dist_mode_it = dist_mode_map.find(mode_it->first);
        if (dist_mode_it == dist_mode_map.end()) {
            return false;
        }

        camera_matrix = mode_it->second.clone();
        dist_coeffs = dist_mode_it->second.clone();
        return true;
    }

    bool getStereoParams(const std::string& camera_pair, cv::Mat& R, cv::Mat& T, cv::Mat& Q) const {
        auto it = stereo_params_.find(camera_pair);
        if (it == stereo_params_.end()) {
            return false;
        }

        R = it->second.R.clone();
        T = it->second.T.clone();
        Q = it->second.Q.clone();
        return true;
    }

    const std::vector<StereoCalibrationResult>& stereo_results() const { return stereo_results_; }

private:
    std::filesystem::path results_dir_;
    std::map<std::string, std::map<std::string, cv::Mat>> camera_matrices_;
    std::map<std::string, std::map<std::string, cv::Mat>> dist_coeffs_;
    std::vector<StereoCalibrationResult> stereo_results_;
    std::map<std::string, StereoCalibrationResult> stereo_params_;
};


} // namespace


void TrackerStatistics::update(double active_objects,
                               double miss_ratio,
                               std::optional<double> match_distance) {
    constexpr double kAlpha = 0.2;
    active_objects = std::max(0.0, active_objects);
    miss_ratio = std::clamp(miss_ratio, 0.0, 1.0);

    if (!initialized_) {
        active_average_ = active_objects;
        miss_average_ = miss_ratio;
        if (match_distance) {
            match_distance_average_ = std::max(0.0, *match_distance);
            distance_valid_ = true;
        } else {
            match_distance_average_ = 0.0;
            distance_valid_ = false;
        }
        initialized_ = true;
        return;
    }

    active_average_ = (1.0 - kAlpha) * active_average_ + kAlpha * active_objects;
    miss_average_ = (1.0 - kAlpha) * miss_average_ + kAlpha * miss_ratio;

    if (match_distance) {
        double distance = std::max(0.0, *match_distance);
        if (!distance_valid_) {
            match_distance_average_ = distance;
            distance_valid_ = true;
        } else {
            match_distance_average_ = (1.0 - kAlpha) * match_distance_average_ + kAlpha * distance;
        }
    }
}


GlobalTracker::GlobalTracker(CameraSchemeManager* scheme_manager,
                             CameraManager* camera_manager)
    : scheme_manager_(scheme_manager), camera_manager_(camera_manager),
      next_global_id_(1), debug_log_path_("/tmp/global_tracker_debug.log") {

    // Parameter validation
    if (!scheme_manager_) {
        throw std::invalid_argument("scheme_manager cannot be null");
    }

    external_calibration_dir_ = detectDefaultCalibrationDirectory();

    auto config = scheme_manager_->getTrackingConfig();

    // Validate configuration parameters
    distance_threshold_ = std::max(0.1, config.max_distance_threshold);
    base_distance_threshold_ = distance_threshold_;
    min_distance_threshold_ = std::max(0.05, base_distance_threshold_ * 0.5);
    max_distance_threshold_ = std::max(base_distance_threshold_, base_distance_threshold_ * 1.5);
    speed_threshold_ = std::max(0.1, config.max_object_speed_mps);
    base_speed_threshold_ = speed_threshold_;
    min_speed_threshold_ = std::max(0.05, base_speed_threshold_ * 0.5);
    max_speed_threshold_ = std::max(base_speed_threshold_, base_speed_threshold_ * 2.0);

    retention_time_ms_ = static_cast<uint64_t>(std::max(2.0, config.id_retention_seconds) * 1000);   // было max(1.0
    area_change_threshold_ = std::max(0.1, config.max_area_ratio_change);
    base_area_change_threshold_ = area_change_threshold_;
    min_area_change_threshold_ = std::max(0.05, base_area_change_threshold_ * 0.5);
    max_area_change_threshold_ = std::max(base_area_change_threshold_, base_area_change_threshold_ * 2.0);

    pending_reid_retention_ms_ = static_cast<uint64_t>(std::max(0.0, config.dormant_id_retention_seconds) * 1000);
    pending_reid_descriptor_threshold_ = std::max(0.0, config.pending_reid_descriptor_threshold);


    residual_position_threshold_ = std::max(0.05, distance_threshold_ * 0.5);
    residual_speed_threshold_ = std::max(0.05, speed_threshold_ * 0.5);
    residual_summary_log_interval_ = kResidualSummaryInterval;
    residual_last_logged_sample_ = 0;

    setDebugLogPath(debug_log_path_);
}

GlobalTracker::~GlobalTracker() {
    // Proper cleanup with RAII pattern
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    tracked_objects_.clear();

    // Clear camera calibrations
    for (auto& [id, calib] : camera_calibrations_) {
        // OpenCV Mat objects have automatic memory management
        // but we explicitly release to be safe
        calib.camera_matrix.release();
        calib.dist_coeffs.release();
        calib.rotation_vector.release();
        calib.translation_vector.release();
        calib.homography_matrix.release();
    }
    camera_calibrations_.clear();


    for (auto& [first, partners] : stereo_calibrations_) {
        for (auto& [second, params] : partners) {
            params.R.release();
            params.T.release();
            params.Q.release();
        }
    }
    stereo_calibrations_.clear();


    if (debug_log_stream_.is_open()) {
        debug_log_stream_.flush();
        debug_log_stream_.close();
    }

    pending_reid_.clear();
}

void GlobalTracker::assignStereoMap(
    std::map<std::string, std::map<std::string, StereoPairCalibration>> stereo_pairs) {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    for (auto& [first, partners] : stereo_calibrations_) {
        for (auto& [second, params] : partners) {
            params.R.release();
            params.T.release();
            params.Q.release();
        }
    }
    stereo_calibrations_.clear();
    stereo_calibrations_ = std::move(stereo_pairs);
}

void GlobalTracker::setDebugLogPath(const std::string& path) {
    std::lock_guard<std::recursive_mutex> lock(mutex_);

    if (debug_log_stream_.is_open()) {
        debug_log_stream_.flush();
        debug_log_stream_.close();
    }

    debug_log_path_ = path;
    if (debug_log_path_.empty()) {
        return;
    }

    std::filesystem::path fs_path(debug_log_path_);
    if (fs_path.has_parent_path()) {
        std::error_code ec;
        std::filesystem::create_directories(fs_path.parent_path(), ec);
        if (ec) {
            std::cerr << "Failed to create directories for debug log at "
                      << debug_log_path_ << ": " << ec.message() << std::endl;
        }
    }

    debug_log_stream_.open(debug_log_path_, std::ios::out | std::ios::app);
    if (!debug_log_stream_.is_open()) {
        std::cerr << "Failed to open debug log at " << debug_log_path_ << std::endl;
    }
}


void GlobalTracker::setExternalCalibrationDirectory(const std::filesystem::path& path) {
    std::error_code ec;
    auto absolute_path = std::filesystem::absolute(path, ec);
    external_calibration_dir_ = ec ? path : absolute_path;
}

void GlobalTracker::logDebugLine(const std::string& line) {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    if (!debug_log_stream_.is_open()) {
        return;
    }
    debug_log_stream_ << line << '\n';
}

void GlobalTracker::flushDebugLog() {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    if (debug_log_stream_.is_open()) {
        debug_log_stream_.flush();
    }
}

void GlobalTracker::logCameraDrop(const std::string& camera_id,
                                  int global_id,
                                  const LocalDetection& detection,
                                  double confidence,
                                  uint64_t timestamp,
                                  const std::string& reason) {
    std::ostringstream oss;
    oss << "timestamp=" << timestamp
        << " camera=" << camera_id
        << " event=camera_entry_dropped"
        << " global_id=" << global_id
        << " reason=" << reason
        << " track_id=" << detection.track_id
        << " box=[" << detection.box.x << ',' << detection.box.y << ','
        << detection.box.width << ',' << detection.box.height << ']'
        << " confidence=" << confidence;
    logDebugLine(oss.str());
}


std::optional<double> GlobalTracker::computeDescriptorDistance(const LocalDetection& a,
                                                               const LocalDetection& b) const {
    std::optional<double> best;

    if (a.descriptor.has_color && b.descriptor.has_color) {
        double dr = (static_cast<double>(a.descriptor.color[0]) - b.descriptor.color[0]) / 255.0;
        double dg = (static_cast<double>(a.descriptor.color[1]) - b.descriptor.color[1]) / 255.0;
        double db = (static_cast<double>(a.descriptor.color[2]) - b.descriptor.color[2]) / 255.0;
        double diff = std::sqrt(dr * dr + dg * dg + db * db) / std::sqrt(3.0);
        best = diff;
    }

    if (a.descriptor.has_grayscale && b.descriptor.has_grayscale) {
        double intensity_diff = std::abs(static_cast<double>(a.descriptor.grayscale_intensity) -
                                         b.descriptor.grayscale_intensity) / 255.0;
        double texture_diff = 0.0;
        texture_diff += std::abs(static_cast<double>(a.descriptor.texture[0]) - b.descriptor.texture[0]) / 100.0;
        texture_diff += std::abs(static_cast<double>(a.descriptor.texture[1]) - b.descriptor.texture[1]) / 10000.0;
        texture_diff += std::abs(static_cast<double>(a.descriptor.texture[2]) - b.descriptor.texture[2]) / 255.0;
        texture_diff += std::abs(static_cast<double>(a.descriptor.texture[3]) - b.descriptor.texture[3]) / 65535.0;
        double combined = intensity_diff * 0.4 + texture_diff * 0.6;
        if (!best || combined < *best) {
            best = combined;
        }
    }

    return best;
}

std::optional<double> GlobalTracker::computeDescriptorCost(const GlobalObject& obj,
                                                           const LocalDetection& detection,
                                                           const std::string& camera_id) const {
    std::optional<double> best;
    for (const auto& [other_camera, other_detection] : obj.camera_detections) {
        if (other_camera == camera_id) {
            continue;
        }
        auto dist = computeDescriptorDistance(detection, other_detection);
        if (dist) {
            if (!best || *dist < *best) {
                best = dist;
            }
        }
    }
    return best;
}

void GlobalTracker::updateReidScore(GlobalObject& obj,
                                    const std::string& camera_id,
                                    const LocalDetection& detection) {
    auto descriptor_cost = computeDescriptorCost(obj, detection, camera_id);
    if (descriptor_cost) {
        obj.camera_reid_scores[camera_id] = *descriptor_cost;
    } else if (detection.descriptor.has_color || detection.descriptor.has_grayscale) {
        obj.camera_reid_scores[camera_id] = 0.0;
    } else {
        obj.camera_reid_scores.erase(camera_id);
    }
}


void GlobalTracker::prunePendingReid(uint64_t current_timestamp) {
    if (pending_reid_.empty()) {
        return;
    }

    auto it = pending_reid_.begin();
    while (it != pending_reid_.end()) {
        uint64_t age = (current_timestamp > it->second.stored_timestamp)
                           ? (current_timestamp - it->second.stored_timestamp)
                           : 0;
        if (pending_reid_retention_ms_ == 0 ||
            (pending_reid_retention_ms_ > 0 && age > pending_reid_retention_ms_)) {
            it = pending_reid_.erase(it);
        } else {
            ++it;
        }
    }
}

int GlobalTracker::tryRevivePending(const std::string& camera_id,
                                    const LocalDetection& detection,
                                    const cv::Point3f& detection_world,
                                    uint64_t timestamp,
                                    double* descriptor_score,
                                    uint64_t* dormant_ms_out) {
    prunePendingReid(timestamp);

    if (pending_reid_.empty()) {
        return -1;
    }

    double best_descriptor_score = pending_reid_descriptor_threshold_;
    double best_spatial_distance = std::numeric_limits<double>::infinity();
    int best_id = -1;

    auto has_descriptor = detection.descriptor.has_color || detection.descriptor.has_grayscale;

    for (const auto& [global_id, entry] : pending_reid_) {
        const GlobalObject& dormant = entry.object;
        if (!dormant.has_last_descriptor || !has_descriptor) {
            continue;
        }

        double spatial_distance = cv::norm(dormant.world_position - detection_world);
        if (!std::isfinite(spatial_distance) || spatial_distance > distance_threshold_) {
            continue;
        }

        LocalDetection stored_detection;
        stored_detection.descriptor = dormant.last_descriptor;
        auto descriptor_distance = computeDescriptorDistance(detection, stored_detection);
        if (!descriptor_distance) {
            continue;
        }

        if (*descriptor_distance > pending_reid_descriptor_threshold_) {
            continue;
        }

        if (best_id < 0 || *descriptor_distance < best_descriptor_score ||
            (std::abs(*descriptor_distance - best_descriptor_score) < 1e-6 &&
             spatial_distance < best_spatial_distance)) {
            best_descriptor_score = *descriptor_distance;
            best_spatial_distance = spatial_distance;
            best_id = global_id;
        }
    }

    if (best_id < 0) {
        return -1;
    }

    auto entry_it = pending_reid_.find(best_id);
    if (entry_it == pending_reid_.end()) {
        return -1;
    }

    GlobalObject revived = std::move(entry_it->second.object);
    uint64_t dormant_ms = (timestamp > revived.last_seen_timestamp)
                              ? (timestamp - revived.last_seen_timestamp)
                              : 0;
    if (descriptor_score) {
        *descriptor_score = best_descriptor_score;
    }
    if (dormant_ms_out) {
        *dormant_ms_out = dormant_ms;
    }
    pending_reid_.erase(entry_it);

    revived.camera_detections.clear();
    revived.camera_last_seen.clear();
    revived.camera_missing_since.clear();
    revived.camera_miss_counts.clear();
    revived.camera_reid_scores.clear();

    revived.camera_detections[camera_id] = detection;
    if (has_descriptor) {
        revived.camera_reid_scores[camera_id] = best_descriptor_score;
        revived.last_descriptor = detection.descriptor;
        revived.has_last_descriptor = true;
    } else {
        revived.has_last_descriptor = false;
    }
    revived.camera_last_seen[camera_id] = timestamp;
    revived.camera_miss_counts[camera_id] = 0;
    revived.primary_camera_id = camera_id;
    revived.last_seen_timestamp = timestamp;
    revived.world_position = detection_world;
    revived.world_position_from_stereo = false;
    revived.velocity = cv::Point3f(0.0f, 0.0f, 0.0f);
    revived.confidence = std::max(revived.confidence, 0.5);

    revived.position_history.push_back(detection_world);
    if (revived.position_history.size() > 50) {
        revived.position_history.erase(revived.position_history.begin(),
                                       revived.position_history.begin() +
                                           (revived.position_history.size() - 50));
    }

    tracked_objects_[revived.global_id] = std::move(revived);

    auto obj_it = tracked_objects_.find(best_id);
    if (obj_it != tracked_objects_.end()) {
        updateReidScore(obj_it->second, camera_id, detection);
    }

    {
        std::ostringstream oss;
        oss << "timestamp=" << timestamp
            << " camera=" << camera_id
            << " event=object_revived"
            << " global_id=" << best_id
            << " dormant_ms=" << dormant_ms
            << " descriptor_score=" << best_descriptor_score
            << " spatial_distance=" << best_spatial_distance;
        logDebugLine(oss.str());
    }

    std::cout << "Возрожден объект ID=" << best_id
              << " после простоя " << dormant_ms << " мс" << std::endl;

    return best_id;
}


std::optional<cv::Point3f> GlobalTracker::triangulateStereoPoint(const std::string& reference_camera,
                                                                 const cv::Point2f& reference_point,
                                                                 const std::string& partner_camera,
                                                                 const cv::Point2f& partner_point) const {
    std::lock_guard<std::recursive_mutex> lock(mutex_);

    auto ref_calib_it = camera_calibrations_.find(reference_camera);
    auto partner_calib_it = camera_calibrations_.find(partner_camera);
    if (ref_calib_it == camera_calibrations_.end() ||
        partner_calib_it == camera_calibrations_.end()) {
        return std::nullopt;
    }

    if (!ref_calib_it->second.is_calibrated || !partner_calib_it->second.is_calibrated) {
        return std::nullopt;
    }

    auto stereo_outer = stereo_calibrations_.find(reference_camera);
    if (stereo_outer == stereo_calibrations_.end()) {
        return std::nullopt;
    }

    auto stereo_inner = stereo_outer->second.find(partner_camera);
    if (stereo_inner == stereo_outer->second.end()) {
        return std::nullopt;
    }

    const StereoPairCalibration& stereo = stereo_inner->second;
    if (stereo.R.empty() || stereo.T.empty()) {
        return std::nullopt;
    }

    const CameraCalibration& ref_calib = ref_calib_it->second;
    const CameraCalibration& partner_calib = partner_calib_it->second;

    if (ref_calib.camera_matrix.empty() || partner_calib.camera_matrix.empty()) {
        return std::nullopt;
    }

    if (ref_calib.rotation_vector.empty() || ref_calib.translation_vector.empty()) {
        return std::nullopt;
    }

    if (partner_calib.rotation_vector.empty() || partner_calib.translation_vector.empty()) {
        return std::nullopt;
    }

    std::vector<cv::Point2f> ref_points = {reference_point};
    std::vector<cv::Point2f> partner_points = {partner_point};
    std::vector<cv::Point2f> ref_undistorted;
    std::vector<cv::Point2f> partner_undistorted;

    cv::undistortPoints(ref_points, ref_undistorted,
                        ref_calib.camera_matrix, ref_calib.dist_coeffs);
    cv::undistortPoints(partner_points, partner_undistorted,
                        partner_calib.camera_matrix, partner_calib.dist_coeffs);

    if (ref_undistorted.empty() || partner_undistorted.empty()) {
        return std::nullopt;
    }

    cv::Mat proj1 = cv::Mat::zeros(3, 4, CV_64F);
    proj1.at<double>(0, 0) = 1.0;
    proj1.at<double>(1, 1) = 1.0;
    proj1.at<double>(2, 2) = 1.0;

    cv::Mat proj2 = cv::Mat::zeros(3, 4, CV_64F);
    cv::Mat R_ref_to_partner;
    stereo.R.convertTo(R_ref_to_partner, CV_64F);
    cv::Mat T_ref_to_partner;
    stereo.T.convertTo(T_ref_to_partner, CV_64F);
    R_ref_to_partner.copyTo(proj2(cv::Rect(0, 0, 3, 3)));
    T_ref_to_partner.copyTo(proj2(cv::Rect(3, 0, 1, 3)));

    cv::Mat ref_mat(2, 1, CV_64F);
    ref_mat.at<double>(0, 0) = ref_undistorted[0].x;
    ref_mat.at<double>(1, 0) = ref_undistorted[0].y;

    cv::Mat partner_mat(2, 1, CV_64F);
    partner_mat.at<double>(0, 0) = partner_undistorted[0].x;
    partner_mat.at<double>(1, 0) = partner_undistorted[0].y;

    cv::Mat point4d;
    cv::triangulatePoints(proj1, proj2, ref_mat, partner_mat, point4d);

    if (point4d.cols < 1 || point4d.rows < 4) {
        return std::nullopt;
    }

    double w = point4d.at<double>(3, 0);
    if (std::abs(w) < 1e-9) {
        return std::nullopt;
    }

    cv::Vec3d point_cam_ref(
        point4d.at<double>(0, 0) / w,
        point4d.at<double>(1, 0) / w,
        point4d.at<double>(2, 0) / w);

    if (point_cam_ref[2] <= 0.0) {
        return std::nullopt;
    }

    cv::Mat ref_rotation;
    cv::Rodrigues(ref_calib.rotation_vector, ref_rotation);
    ref_rotation.convertTo(ref_rotation, CV_64F);

    cv::Mat ref_translation;
    ref_calib.translation_vector.convertTo(ref_translation, CV_64F);

    cv::Mat cam_point = (cv::Mat_<double>(3, 1)
                         << point_cam_ref[0], point_cam_ref[1], point_cam_ref[2]);

    cv::Mat world_point = ref_rotation.t() * (cam_point - ref_translation);

    return cv::Point3f(static_cast<float>(world_point.at<double>(0)),
                       static_cast<float>(world_point.at<double>(1)),
                       static_cast<float>(world_point.at<double>(2)));
}

std::optional<cv::Point3f> GlobalTracker::tryStereoTriangulation(const GlobalObject& obj,
                                                                 const std::string& primary_camera,
                                                                 const cv::Point2f& preferred_point,
                                                                 const cv::Point2f& fallback_point,
                                                                 uint64_t timestamp,
                                                                 std::string* partner_used) const {
    constexpr uint64_t kMaxStereoAgeMs = 200;

    std::lock_guard<std::recursive_mutex> lock(mutex_);

    auto stereo_it = stereo_calibrations_.find(primary_camera);
    if (stereo_it == stereo_calibrations_.end()) {
        return std::nullopt;
    }

    std::optional<cv::Point3f> best_point;
    uint64_t best_delta = std::numeric_limits<uint64_t>::max();
    std::string best_partner;

    for (const auto& [partner_camera, params] : stereo_it->second) {
        (void)params;
        auto other_det_it = obj.camera_detections.find(partner_camera);
        if (other_det_it == obj.camera_detections.end()) {
            continue;
        }

        if (obj.camera_missing_since.find(partner_camera) != obj.camera_missing_since.end()) {
            continue;
        }

        auto last_seen_it = obj.camera_last_seen.find(partner_camera);
        if (last_seen_it == obj.camera_last_seen.end()) {
            continue;
        }

        uint64_t last_seen = last_seen_it->second;
        uint64_t delta = (timestamp > last_seen) ? (timestamp - last_seen)
                                                : (last_seen - timestamp);
        if (delta > kMaxStereoAgeMs) {
            continue;
        }

        const cv::Rect& partner_box = other_det_it->second.box;
        if (partner_box.width <= 0 || partner_box.height <= 0) {
            continue;
        }

        cv::Point2f partner_feet(
            partner_box.x + partner_box.width * 0.5f,
            partner_box.y + partner_box.height);
        cv::Point2f partner_center(
            partner_box.x + partner_box.width * 0.5f,
            partner_box.y + partner_box.height * 0.5f);

        std::optional<cv::Point3f> candidate =
            triangulateStereoPoint(primary_camera, preferred_point, partner_camera, partner_feet);

        if (!candidate) {
            candidate = triangulateStereoPoint(primary_camera, preferred_point, partner_camera, partner_center);
        }

        if (!candidate && (fallback_point != preferred_point)) {
            candidate = triangulateStereoPoint(primary_camera, fallback_point, partner_camera, partner_feet);
            if (!candidate) {
                candidate = triangulateStereoPoint(primary_camera, fallback_point, partner_camera, partner_center);
            }
        }

        if (candidate && delta < best_delta) {
            best_point = candidate;
            best_delta = delta;
            best_partner = partner_camera;
        }
    }

    if (best_point && partner_used) {
        *partner_used = best_partner;
    }

    return best_point;
}



void GlobalTracker::adjustAdaptiveThresholds(size_t active_objects,
                                             size_t detection_count,
                                             uint64_t timestamp,
                                             const std::string& camera_id) {
    double previous_distance = distance_threshold_;
    double previous_speed = speed_threshold_;
    double previous_area = area_change_threshold_;

    if (!tracker_stats_.initialized()) {
        distance_threshold_ = base_distance_threshold_;
        speed_threshold_ = base_speed_threshold_;
        area_change_threshold_ = base_area_change_threshold_;
        return;
    }

    double active_average = std::max(1e-3, tracker_stats_.activeAverage());
    double miss_average = std::clamp(tracker_stats_.missAverage(), 0.0, 1.0);
    auto match_distance = tracker_stats_.distanceAverage();

    double density_ratio = static_cast<double>(active_objects) / active_average;
    density_ratio = std::clamp(density_ratio, 0.7, 1.6);
    double density_factor = 1.0 / density_ratio;

    double miss_factor = 1.0 + std::clamp(miss_average - 0.15, -0.35, 0.75);

    double distance_factor = 1.0;
    if (match_distance) {
        double normalized = *match_distance / std::max(1e-3, base_distance_threshold_);
        distance_factor = std::clamp(normalized, 0.7, 1.4);
    }

    double detection_ratio = active_objects > 0
                                 ? static_cast<double>(detection_count) /
                                       static_cast<double>(std::max<size_t>(1, active_objects))
                                 : 1.0;
    detection_ratio = std::clamp(detection_ratio, 0.6, 2.5);
    double area_density_factor = 1.0 / detection_ratio;

    double new_distance = base_distance_threshold_ * distance_factor * miss_factor * density_factor;
    new_distance = std::clamp(new_distance, min_distance_threshold_, max_distance_threshold_);

    double adjusted_density = std::clamp(density_factor, 0.5, 1.5);
    double new_speed = base_speed_threshold_ * miss_factor / adjusted_density;
    new_speed = std::clamp(new_speed, min_speed_threshold_, max_speed_threshold_);

    double new_area = base_area_change_threshold_ * miss_factor * area_density_factor * density_factor;
    new_area = std::clamp(new_area, min_area_change_threshold_, max_area_change_threshold_);

    distance_threshold_ = new_distance;
    speed_threshold_ = new_speed;
    area_change_threshold_ = new_area;

    bool distance_clamped = (std::abs(new_distance - min_distance_threshold_) < 1e-6) ||
                            (std::abs(new_distance - max_distance_threshold_) < 1e-6);
    bool speed_clamped = (std::abs(new_speed - min_speed_threshold_) < 1e-6) ||
                         (std::abs(new_speed - max_speed_threshold_) < 1e-6);
    bool area_clamped = (std::abs(new_area - min_area_change_threshold_) < 1e-6) ||
                        (std::abs(new_area - max_area_change_threshold_) < 1e-6);

    auto shouldLog = [&](double previous, double current, double& last_logged) {
        if (!std::isfinite(last_logged) || std::abs(current - last_logged) > 1e-3 ||
            std::abs(current - previous) > 1e-3) {
            last_logged = current;
            return true;
        }
        return false;
    };

    bool log_change = shouldLog(previous_distance, new_distance, last_logged_distance_threshold_) ||
                      shouldLog(previous_speed, new_speed, last_logged_speed_threshold_) ||
                      shouldLog(previous_area, new_area, last_logged_area_change_threshold_) ||
                      distance_clamped || speed_clamped || area_clamped;

    if (log_change) {
        std::ostringstream oss;
        oss << "timestamp=" << timestamp
            << " camera=" << camera_id
            << " event=threshold_update"
            << " active_objects=" << active_objects
            << " detections=" << detection_count
            << " active_avg=" << tracker_stats_.activeAverage()
            << " miss_avg=" << miss_average;
        if (match_distance) {
            oss << " distance_avg=" << *match_distance;
        } else {
            oss << " distance_avg=nan";
        }
        oss << " density_factor=" << density_factor
            << " miss_factor=" << miss_factor
            << " area_density_factor=" << area_density_factor
            << " distance_threshold=" << new_distance
            << " speed_threshold=" << new_speed
            << " area_change_threshold=" << new_area;
        if (distance_clamped) {
            oss << " distance_clamped=1";
        }
        if (speed_clamped) {
            oss << " speed_clamped=1";
        }
        if (area_clamped) {
            oss << " area_clamped=1";
        }
        logDebugLine(oss.str());
    }
}


bool GlobalTracker::initialize() {
    std::cout << "Инициализация глобального трекера..." << std::endl;

    {
        std::lock_guard<std::recursive_mutex> lock(mutex_);
        camera_calibrations_.clear();
    }

    if (calibration_watcher_) {
        auto results_file = calibration_watcher_->getResultsPath() / "calibration_results.json";
//        auto results_file = exe_dir / "calibration/results/calibration_results.json";
        if (calibration_watcher_->loadResults()) {
            std::cout << "Загружены данные калибровки при старте из "
                      << results_file << std::endl;
            updateCalibrationTimestamp(*calibration_watcher_);
        } else {
            std::cerr << "Не удалось загрузить калибровку из " << results_file
                      << " при инициализации" << std::endl;
        }
    }


    if (!performAutoCalibration()) {
        std::cerr << "Ошибка автокалибровки" << std::endl;
        return false;

    }

    size_t calibrated_count = 0;
    {
        std::lock_guard<std::recursive_mutex> lock(mutex_);
        calibrated_count = camera_calibrations_.size();
    }
    std::cout << "Глобальный трекер инициализирован для "
              << calibrated_count << " камер" << std::endl;
    return true;
}

bool GlobalTracker::performAutoCalibration() {
    std::cout << "Выполнение автоматической калибровки..." << std::endl;
    
    SchemeType scheme = scheme_manager_->getCurrentScheme();

    switch (scheme) {
        case SchemeType::SPHERE:
        case SchemeType::SPHERE_ZOOM:
            return calibrateSphereSetup(scheme);
        case SchemeType::HEMISPHERE_SINGLE:
        case SchemeType::HEMISPHERE_MULTI:
        case SchemeType::HEMISPHERE_ZOOM:
            return calibrateHemisphereSetup(scheme);
        default:
            std::cerr << "Неподдерживаемая схема для автокалибровки" << std::endl;
            return false;
    }
}

bool GlobalTracker::calibrateSphereSetup(SchemeType scheme) {
    std::cout << "Калибровка сферической установки..." << std::endl;

    if (calibration_watcher_) {
        if (loadCalibrationFromWatcher(*calibration_watcher_, scheme)) {
            return validateCalibration();
        }
        std::cout << "Данные калибровки из CalibrationWatcher недоступны, используем резервные значения." << std::endl;
    }

    if (loadExternalCalibration(scheme)) {
        return validateCalibration();
    }
    std::cout << "Данные калибровки из внешнего каталога недоступны, используем резервные значения." << std::endl;

    return calibrateSphereFallback();
}

bool GlobalTracker::calibrateSphereFallback() {

    auto cameras = scheme_manager_->getCameras();
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    for (const auto& cam : cameras) {
        if (cam.status != CameraStatus::ACTIVE) continue;

        CameraCalibration calib;
        calib.is_calibrated = false;

        setupFallbackCameraIntrinsics(cam, calib);

        bool extrinsics_initialized = false;
        if (isPrimaryWideRole(cam.role)) {
            setupFallbackSphereExtrinsics(cam, calib, 0.0, 180.0);
            extrinsics_initialized = true;
        } else if (isSecondaryWideRole(cam.role)) {
            setupFallbackSphereExtrinsics(cam, calib, 180.0, 180.0);
            extrinsics_initialized = true;
        } else if (isZoomRole(cam.role)) {
            setupFallbackZoomExtrinsics(cam, calib);
            extrinsics_initialized = true;
        }

        if (!extrinsics_initialized) {
            std::cerr << "Неизвестная роль камеры " << cam.id
                      << ": " << scheme_manager_->roleToString(cam.role)
                      << ". Пропускаем вычисление гомографии." << std::endl;
            continue;
        }

        if (calib.rotation_vector.empty() || calib.translation_vector.empty()) {
            std::cerr << "Не заданы внешние параметры для камеры " << cam.id
                      << " (" << scheme_manager_->roleToString(cam.role)
                      << ")." << std::endl;
            continue;
        }

        calib.is_calibrated = true;
        computeHomography(calib);
        camera_calibrations_[cam.id] = calib;

        std::cout << "Калибровка камеры " << cam.id << " завершена" << std::endl;
    }

    return validateCalibration();
}

bool GlobalTracker::calibrateHemisphereSetup(SchemeType scheme) {
    std::cout << "Калибровка полусферической установки..." << std::endl;

  if (calibration_watcher_) {
        if (loadCalibrationFromWatcher(*calibration_watcher_, scheme)) {
            return validateCalibration();
        }
        std::cout << "Данные калибровки из CalibrationWatcher недоступны, используем резервные значения." << std::endl;
    }

    if (loadExternalCalibration(scheme)) {
        return validateCalibration();
    }
    std::cout << "Данные калибровки из внешнего каталога недоступны, используем резервные значения." << std::endl;

    return calibrateHemisphereFallback();
}

bool GlobalTracker::calibrateHemisphereFallback() {

    auto cameras = scheme_manager_->getCameras();
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    for (const auto& cam : cameras) {
        if (cam.status != CameraStatus::ACTIVE) continue;

        CameraCalibration calib;
        setupFallbackCameraIntrinsics(cam, calib);

        bool extrinsics_initialized = false;
        if (isPrimaryWideRole(cam.role)) {
            setupFallbackHemisphereExtrinsics(cam, calib, 0.0, 180.0);
            extrinsics_initialized = true;
        } else if (isSecondaryWideRole(cam.role)) {
            setupFallbackHemisphereExtrinsics(cam, calib, 30.0, 120.0);
            extrinsics_initialized = true;
        } else if (isZoomRole(cam.role)) {
            setupFallbackZoomExtrinsics(cam, calib);
            extrinsics_initialized = true;
        }

        if (!extrinsics_initialized) {
            std::cerr << "Неизвестная роль камеры " << cam.id
                      << ": " << scheme_manager_->roleToString(cam.role)
                      << ". Пропускаем вычисление гомографии." << std::endl;
            continue;
        }

        if (calib.rotation_vector.empty() || calib.translation_vector.empty()) {
            std::cerr << "Не заданы внешние параметры для камеры " << cam.id
                      << " (" << scheme_manager_->roleToString(cam.role)
                      << ")." << std::endl;
            continue;
        }

        calib.is_calibrated = true;
        computeHomography(calib);
        camera_calibrations_[cam.id] = calib;
    }

    return validateCalibration();
}

void GlobalTracker::setupFallbackCameraIntrinsics(const CameraConfig& cam, CameraCalibration& calib) {
    // Настройка внутренних параметров камеры
    // В реальной системе эти значения должны быть получены из калибровки

    double fx, fy, cx, cy;

    if (isZoomRole(cam.role)) {
        // Зум камера - более узкий угол обзора
        fx = fy = cam.width * 2.0;  // Большее фокусное расстояние
        cx = cam.width / 2.0;
        cy = cam.height / 2.0;
    } else {
        // Широкоугольные камеры
        fx = fy = cam.width * 0.8;  // Меньшее фокусное расстояние
        cx = cam.width / 2.0;
        cy = cam.height / 2.0;
    }

    calib.camera_matrix = (cv::Mat_<double>(3, 3) << 
        fx, 0,  cx,
        0,  fy, cy,
        0,  0,  1);

    // Коэффициенты дисторсии (для начала нулевые)
    calib.dist_coeffs = cv::Mat::zeros(5, 1, CV_64F);

    std::cout << "Настроены внутренние параметры для " << cam.id 
              << " (fx=" << fx << ", fy=" << fy << ")" << std::endl;
}

void GlobalTracker::setupFallbackSphereExtrinsics(const CameraConfig& cam, CameraCalibration& calib,
                                                  double start_angle, double coverage_angle) {
    // Позиционирование камеры для сферической схемы
    double angle_rad = start_angle * CV_PI / 180.0;
    
    // Позиция камеры на сфере
    double radius = 0.5; // 50 см от центра
    double height = cam.position.z;
    if (height <= 0.0) {
        height = 0.45;
        if (default_height_warned_cameras.insert(cam.id).second) {
            std::cerr << "Предупреждение: для камеры " << cam.id
                      << " используется высота по умолчанию " << height
                      << " м. Проверьте конфигурацию." << std::endl;
        }
    }
    cv::Point3d cam_position(
        radius * cos(angle_rad),
        radius * sin(angle_rad),
        height
    );
    
    // Вектор поворота (камера смотрит к центру)
    cv::Vec3d rvec(0, 0, angle_rad + CV_PI); // Поворот + 180°
    calib.rotation_vector = (cv::Mat_<double>(3, 1) << rvec[0], rvec[1], rvec[2]);

    cv::Mat R;
    cv::Rodrigues(calib.rotation_vector, R);

    cv::Mat camera_position = (cv::Mat_<double>(3, 1) <<
        cam_position.x,
        cam_position.y,
        cam_position.z);

    calib.translation_vector = -R * camera_position;

}

void GlobalTracker::setupFallbackHemisphereExtrinsics(const CameraConfig& cam, CameraCalibration& calib,
                                                      double center_angle, double coverage_angle) {
    // Позиционирование для полусферической схемы
    double angle_rad = center_angle * CV_PI / 180.0;
    
    double radius = 0.4; // 40 см от центра
    double height = cam.position.z;
    if (height <= 0.0) {
        height = 0.45;
        if (default_height_warned_cameras.insert(cam.id).second) {
            std::cerr << "Предупреждение: для камеры " << cam.id
                      << " используется высота по умолчанию " << height
                      << " м. Проверьте конфигурацию." << std::endl;
        }
    }
    cv::Point3d cam_position(
        radius * cos(angle_rad),
        radius * sin(angle_rad),
        height
    );
    
    cv::Vec3d rvec(0, 0, angle_rad + CV_PI);
    calib.rotation_vector = (cv::Mat_<double>(3, 1) << rvec[0], rvec[1], rvec[2]);

    cv::Mat R;
    cv::Rodrigues(calib.rotation_vector, R);


    cv::Mat camera_position = (cv::Mat_<double>(3, 1) <<
        cam_position.x,
        cam_position.y,
        cam_position.z);

    calib.translation_vector = -R * camera_position;

    std::cout << "Внешние параметры полусферы для " << cam.id 
              << " (центр=" << center_angle << "°, покрытие=" << coverage_angle << "°)" << std::endl;
}

void GlobalTracker::setupFallbackZoomExtrinsics(const CameraConfig& cam, CameraCalibration& calib) {
    // Зум камера - центральное положение с возможностью поворота
    cv::Vec3d rvec(0, 0, 0); // Начальное положение - прямо
    calib.rotation_vector = (cv::Mat_<double>(3, 1) << rvec[0], rvec[1], rvec[2]);

    cv::Mat R;
    cv::Rodrigues(calib.rotation_vector, R);

    double height = cam.position.z;
    if (height <= 0.0) {
        height = 0.45;
        if (default_height_warned_cameras.insert(cam.id).second) {
            std::cerr << "Предупреждение: для камеры " << cam.id
                      << " используется высота по умолчанию " << height
                      << " м. Проверьте конфигурацию." << std::endl;
        }
    }


    cv::Mat camera_position = (cv::Mat_<double>(3, 1) <<
        0.0,
        0.0,
        height);

    calib.translation_vector = -R * camera_position;

    std::cout << "Внешние параметры зума для " << cam.id << std::endl;
}

bool GlobalTracker::applyFallbackExtrinsics(const CameraConfig& cam, SchemeType scheme, CameraCalibration& calib) {
    if (isZoomRole(cam.role)) {
        setupFallbackZoomExtrinsics(cam, calib);
        if (!calib.rotation_vector.empty() && !calib.translation_vector.empty()) {
            updateCalibrationWorldPosition(calib);
        }
        return true;
    }

    bool extrinsics_initialized = false;
    switch (scheme) {
        case SchemeType::SPHERE_ZOOM:
            if (isSecondaryWideRole(cam.role)) {
                setupFallbackSphereExtrinsics(cam, calib, 180.0, 180.0);
                extrinsics_initialized = true;
            } else if (isPrimaryWideRole(cam.role)) {
                setupFallbackSphereExtrinsics(cam, calib, 0.0, 180.0);
                extrinsics_initialized = true;
            }
            break;
        default:
            if (isSecondaryWideRole(cam.role)) {
                setupFallbackHemisphereExtrinsics(cam, calib, 30.0, 120.0);
                extrinsics_initialized = true;
            } else if (isPrimaryWideRole(cam.role)) {
                setupFallbackHemisphereExtrinsics(cam, calib, 0.0, 180.0);
                extrinsics_initialized = true;
            }
            break;
    }
    if (!extrinsics_initialized) {
        std::cerr << "Не удалось применить резервные внешние параметры для камеры "
                  << cam.id << " (" << scheme_manager_->roleToString(cam.role)
                  << ")" << std::endl;
    }
    if (extrinsics_initialized &&
        !calib.rotation_vector.empty() && !calib.translation_vector.empty()) {
        updateCalibrationWorldPosition(calib);
    }
    return extrinsics_initialized;
}

bool GlobalTracker::applyWatcherStereoExtrinsics(const CalibrationWatcher& watcher,
                                                const std::vector<CameraConfig>& cameras,
                                                std::map<std::string, CameraCalibration>& calibrations) {
    auto get_params = [&watcher](const std::string& camera_pair, cv::Mat& R, cv::Mat& T, cv::Mat& Q) {
        return watcher.getStereoParams(camera_pair, R, T, Q);
    };
    return applyStereoExtrinsicsFromResults(watcher.getStereoResults(), get_params, cameras, calibrations);
}

bool GlobalTracker::loadCalibrationFromWatcher(const CalibrationWatcher& watcher, SchemeType scheme) {
    auto get_camera_matrix = [&watcher](const std::string& camera_id, cv::Mat& camera_matrix, cv::Mat& dist_coeffs) {
        return watcher.getCameraMatrix(camera_id, camera_matrix, dist_coeffs);
    };
    auto get_stereo_params = [&watcher](const std::string& camera_pair, cv::Mat& R, cv::Mat& T, cv::Mat& Q) {
        return watcher.getStereoParams(camera_pair, R, T, Q);
    };
    auto stereo_results = watcher.getStereoResults();
    return loadCalibrationFromProvider(get_camera_matrix, get_stereo_params, stereo_results, scheme,
                                       "CalibrationWatcher");
}

bool GlobalTracker::reloadCalibration(const CalibrationWatcher& watcher) {
    if (!loadCalibrationFromWatcher(watcher, scheme_manager_->getCurrentScheme())) {
        return false;
    }
    return validateCalibration();
}

bool GlobalTracker::updateCalibrationTimestamp(const CalibrationWatcher& watcher) {
    std::error_code ec;

    auto results_file = watcher.getResultsPath() / "calibration_results.json";
//    auto results_file = exe_dir / "calibration/results/calibration_results.json";
    auto ts = std::filesystem::last_write_time(results_file, ec);
    if (ec) {
        return false;
    }
    {
        std::lock_guard<std::recursive_mutex> lock(mutex_);
        last_calibration_write_time_ = ts;
        has_last_calibration_write_time_ = true;
    }
    return true;
}

bool GlobalTracker::checkAndUpdateCalibration() {
    if (!calibration_watcher_) {
        return false;
    }
    std::error_code ec;
    auto results_file = calibration_watcher_->getResultsPath() / "calibration_results.json";
//    auto results_file = exe_dir / "calibration/results/calibration_results.json";
    auto current_time = std::filesystem::last_write_time(results_file, ec);
    if (ec) {
        return false;
    }
    {
        std::lock_guard<std::recursive_mutex> lock(mutex_);
        if (has_last_calibration_write_time_ &&
            current_time <= last_calibration_write_time_) {
            return false;
        }
     }

    if (!calibration_watcher_->loadResults()) {
        return false;
    }

    std::unique_lock<std::recursive_mutex> lock(mutex_);
    if (has_last_calibration_write_time_ &&
        current_time <= last_calibration_write_time_) {
        return false;
    }

    if (reloadCalibration(*calibration_watcher_)) {
        last_calibration_write_time_ = current_time;
        has_last_calibration_write_time_ = true;
        std::cout << "Обновлены данные калибровки из CalibrationWatcher" << std::endl;
        return true;
    }
    return false;
}

bool GlobalTracker::validateCalibration() {
    std::cout << "Проверка качества калибровки..." << std::endl;

    // Проверяем наличие калибровочных данных для всех активных камер
    auto cameras = scheme_manager_->getCameras();
    int calibrated_count = 0;

    std::lock_guard<std::recursive_mutex> lock(mutex_);
    for (const auto& cam : cameras) {
        if (cam.status == CameraStatus::ACTIVE) {
            auto it = camera_calibrations_.find(cam.id);
            if (it != camera_calibrations_.end() && it->second.is_calibrated) {
                calibrated_count++;
            } else {
                std::cerr << "Камера " << cam.id << " не откалибрована" << std::endl;
                return false;
            }
        }
    }
    std::cout << "Калибровка завершена для " << calibrated_count << " камер" << std::endl;
    return calibrated_count >= 2; // Минимум 2 камеры
}

void GlobalTracker::updateDetections(const std::string& camera_id,
                                   const std::vector<LocalDetection>& detections,
                                   uint64_t timestamp) {
    checkAndUpdateCalibration();
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    {
        std::ostringstream oss;
        oss << "timestamp=" << timestamp
            << " camera=" << camera_id
            << " event=batch_start"
            << " detection_count=" << detections.size();
        logDebugLine(oss.str());
    }

    // Ассоциируем новые детекции с существующими объектами
    associateDetections(camera_id, detections, timestamp);

    // Обновляем позиции объектов в мировых координатах
    updateWorldPositions(camera_id, timestamp);

    // Очищаем старые объекты после обновления данных текущей итерации
    cleanupOldObjects(timestamp);
    rebuildAssignmentsLocked();
    flushDebugLog();
}

void GlobalTracker::associateDetections(const std::string& camera_id,
                                       const std::vector<LocalDetection>& detections,
                                       uint64_t timestamp) {
    std::lock_guard<std::recursive_mutex> lock(mutex_);

    // Parameter validation
    if (camera_id.empty() || timestamp == 0) {
        return;
    }

    std::vector<std::string> detection_path(detections.size(), "unassigned");
    std::vector<std::string> detection_decision(detections.size());
    std::vector<std::string> detection_notes(detections.size());
    std::vector<int> detection_global_id(detections.size(), -1);
    std::vector<double> detection_confidence(detections.size(), 0.0);
    std::vector<bool> detection_confidence_valid(detections.size(), false);
    std::vector<double> detection_cost(detections.size(), 0.0);
    std::vector<bool> detection_cost_valid(detections.size(), false);
    std::vector<double> detection_penalty(detections.size(), 1.0);
    std::vector<double> detection_area_ratio(detections.size(), -1.0);
    std::vector<double> detection_reid_score(detections.size(), -1.0);
    std::vector<bool> detection_reid_valid(detections.size(), false);
    size_t active_objects_total = tracked_objects_.size();
    double total_match_distance = 0.0;
    size_t match_distance_count = 0;
    bool stats_updated = false;

    if (detections.empty()) {
        size_t camera_objects = 0;

        // уменьшение уверенности только для объектов, у которых есть детекции с этой камеры
        for (auto& [global_id, obj] : tracked_objects_) {
            auto prev_it = obj.camera_detections.find(camera_id);
            if (prev_it == obj.camera_detections.end()) {
                continue;
            }

            ++camera_objects;

            obj.confidence = std::max(0.0, obj.confidence - 0.02); //было 0.05

            auto missing_it = obj.camera_missing_since.find(camera_id);
            if (missing_it == obj.camera_missing_since.end()) {
                obj.camera_missing_since.emplace(camera_id, timestamp);
                std::ostringstream oss;
                oss << "timestamp=" << timestamp
                    << " camera=" << camera_id
                    << " event=camera_missing"
                    << " global_id=" << global_id
                    << " confidence=" << obj.confidence;
                logDebugLine(oss.str());
            }
            int& miss_count = obj.camera_miss_counts[camera_id];
            if (miss_count < std::numeric_limits<int>::max()) {
                ++miss_count;
            }
        }
        double miss_ratio = camera_objects > 0 ? 1.0 : 0.0;
        tracker_stats_.update(static_cast<double>(active_objects_total), miss_ratio, std::nullopt);
        std::ostringstream oss;
        oss << "timestamp=" << timestamp
            << " camera=" << camera_id
            << " event=tracker_state"
            << " active_objects=" << active_objects_total
            << " miss_ratio=" << miss_ratio
            << " match_distance=nan"
            << " active_avg=" << tracker_stats_.activeAverage()
            << " miss_avg=" << tracker_stats_.missAverage();
        auto avg_distance = tracker_stats_.distanceAverage();
        if (avg_distance) {
            oss << " distance_avg=" << *avg_distance;
        }
        logDebugLine(oss.str());
        stats_updated = true;
        adjustAdaptiveThresholds(tracked_objects_.size(), 0, timestamp, camera_id);
        return;
    }

    // Подготовка данных: мировые координаты центров детекций
    std::vector<cv::Point3f> detection_world;
    detection_world.reserve(detections.size());
    std::vector<int> detection_world_index(detections.size(), -1);
    std::vector<bool> detection_used(detections.size(), false);
    std::unordered_map<int, std::vector<int>> detections_by_track_id;

    for (size_t idx = 0; idx < detections.size(); ++idx) {
        const auto& det = detections[idx];
        const cv::Rect& box = det.box;
        if (box.width <= 0 || box.height <= 0) {
            detection_path[idx] = "invalid_box";
            detection_decision[idx] = "skipped";
        } else {
            cv::Point2f center(box.x + box.width * 0.5f,
                               box.y + box.height * 0.5f);
            auto projection = imageToWorld(camera_id, center);
            if (!projection.valid) {
                detection_path[idx] = "projection_invalid";
                detection_decision[idx] = "skipped";
                detection_used[idx] = true;
                detection_notes[idx] = "world_projection_failed";
                std::ostringstream oss;
                oss << "timestamp=" << timestamp
                    << " camera=" << camera_id
                    << " event=detection_projection_failed"
                    << " detection_idx=" << idx
                    << " track_id=" << det.track_id;
                logDebugLine(oss.str());
            } else {
                detection_world_index[idx] = static_cast<int>(detection_world.size());
                detection_world.push_back(projection.world_point);
            }
        }

        if (det.track_id >= 0) {
            detections_by_track_id[det.track_id].push_back(static_cast<int>(idx));
        }
    }

    // Список существующих объектов
    std::vector<int> object_ids;
    std::vector<cv::Point3f> predicted_positions;

    object_ids.reserve(tracked_objects_.size());
    predicted_positions.reserve(tracked_objects_.size());

    for (auto& pair : tracked_objects_) {
        object_ids.push_back(pair.first);
        predicted_positions.push_back(predictPosition(pair.second, timestamp));
    }


    bool has_objects = !object_ids.empty();

    adjustAdaptiveThresholds(object_ids.size(), detections.size(), timestamp, camera_id);

    std::vector<bool> object_matched(object_ids.size(), false);

    if (!has_objects) {
        for (size_t idx = 0; idx < detections.size(); ++idx) {
            const auto& det = detections[idx];
            if (det.box.width > 0 && det.box.height > 0) {

                int new_id = createNewObject(camera_id, det, timestamp);
                if (new_id > 0) {
                    detection_path[idx] = "new_object";
                    detection_decision[idx] = "created";
                    detection_global_id[idx] = new_id;
                    auto obj_it = tracked_objects_.find(new_id);
                    if (obj_it != tracked_objects_.end()) {
                        detection_confidence[idx] = obj_it->second.confidence;
                        detection_confidence_valid[idx] = true;
                    }
                }
            }
        }
    } else {
        std::vector<bool> object_track_matched(object_ids.size(), false);
        for (size_t i = 0; i < object_ids.size(); ++i) {
            int obj_id = object_ids[i];
            GlobalObject& obj = tracked_objects_[obj_id];

            auto prev_it = obj.camera_detections.find(camera_id);
            if (prev_it == obj.camera_detections.end()) {
                continue;
            }
            int prev_track_id = prev_it->second.track_id;
            if (prev_track_id < 0) {
                continue;
            }
            auto det_it = detections_by_track_id.find(prev_track_id);
            if (det_it == detections_by_track_id.end()) {
                continue;
            }
            int matched_detection_index = -1;
            for (int candidate_index : det_it->second) {
                if (candidate_index >= 0 &&
                    candidate_index < static_cast<int>(detection_used.size()) &&
                    !detection_used[candidate_index]) {
                    matched_detection_index = candidate_index;
                    break;
                }
            }

            if (matched_detection_index < 0) {
                continue;

            }

            double reuse_penalty = 1.0;
            double reuse_ratio = -1.0;
            if (prev_it != obj.camera_detections.end()) {
                double prev_area = static_cast<double>(prev_it->second.box.width) * prev_it->second.box.height;
                double new_area = static_cast<double>(detections[matched_detection_index].box.width) *
                                  detections[matched_detection_index].box.height;
                if (prev_area > 0.0) {
                    reuse_ratio = std::abs(new_area - prev_area) / prev_area;
                    if (reuse_ratio > area_change_threshold_) {
                        reuse_penalty = 1.0 + std::min(reuse_ratio - area_change_threshold_, 1.0);
                    }
                }
            }

            obj.camera_detections[camera_id] = detections[matched_detection_index];
            obj.camera_last_seen[camera_id] = timestamp;
            obj.camera_missing_since.erase(camera_id);
            obj.camera_miss_counts[camera_id] = 0;
            if (detections[matched_detection_index].descriptor.has_color ||
                detections[matched_detection_index].descriptor.has_grayscale) {
                obj.last_descriptor = detections[matched_detection_index].descriptor;
                obj.has_last_descriptor = true;
            }
            if (reuse_penalty > 1.0) {
                double reduction = 0.02 * reuse_penalty;
                obj.confidence = std::max(0.0, obj.confidence - reduction);
            } else {
                obj.confidence = std::min(1.0, obj.confidence + 0.1);
            }
            detection_used[matched_detection_index] = true;
            object_track_matched[i] = true;

            detection_penalty[matched_detection_index] = reuse_penalty;
            detection_area_ratio[matched_detection_index] = reuse_ratio;

            detection_path[matched_detection_index] = "track_id";
            detection_decision[matched_detection_index] = "reuse";
            detection_global_id[matched_detection_index] = obj.global_id;
            detection_confidence[matched_detection_index] = obj.confidence;
            detection_confidence_valid[matched_detection_index] = true;
            auto descriptor_cost = computeDescriptorCost(obj, detections[matched_detection_index], camera_id);
            if (descriptor_cost) {
                detection_reid_score[matched_detection_index] = *descriptor_cost;
                detection_reid_valid[matched_detection_index] = true;
            }
            updateReidScore(obj, camera_id, detections[matched_detection_index]);

            int world_index = -1;
            if (matched_detection_index >= 0 &&
                matched_detection_index < static_cast<int>(detection_world_index.size())) {
                world_index = detection_world_index[matched_detection_index];
            }
            if (world_index >= 0 &&
                world_index < static_cast<int>(detection_world.size())) {
                double distance = cv::norm(predicted_positions[i] -
                                            detection_world[world_index]);
                if (std::isfinite(distance)) {
                    total_match_distance += distance;
                    ++match_distance_count;
                }
            }
            if (i < object_matched.size()) {
                object_matched[i] = true;
            }

            double current_reid = std::numeric_limits<double>::infinity();
            auto current_reid_it = obj.camera_reid_scores.find(camera_id);
            if (current_reid_it != obj.camera_reid_scores.end()) {
                current_reid = current_reid_it->second;
            }
            double best_reid = std::numeric_limits<double>::infinity();
            if (!obj.primary_camera_id.empty()) {
                auto best_reid_it = obj.camera_reid_scores.find(obj.primary_camera_id);
                if (best_reid_it != obj.camera_reid_scores.end()) {
                    best_reid = best_reid_it->second;
                }
            }
            bool prefer_by_reid = std::isfinite(current_reid) &&
                                  (!std::isfinite(best_reid) || current_reid + 0.05 < best_reid);


            if (obj.primary_camera_id.empty() ||
                prefer_by_reid ||
                getPrimaryCameraPriority(camera_id) > getPrimaryCameraPriority(obj.primary_camera_id)) {
                obj.primary_camera_id = camera_id;
            }
        }

        std::vector<int> hungarian_object_indices;
        hungarian_object_indices.reserve(object_ids.size());
        for (size_t i = 0; i < object_ids.size(); ++i) {
            if (!object_track_matched[i]) {
                hungarian_object_indices.push_back(static_cast<int>(i));
            }
        }

        std::vector<int> hungarian_detection_indices;
        hungarian_detection_indices.reserve(detections.size());
        for (size_t j = 0; j < detections.size(); ++j) {
            if (!detection_used[j]) {
                hungarian_detection_indices.push_back(static_cast<int>(j));
            }
        }

        struct CandidateEntry {
            int detection_index = -1;
            double distance = 0.0;
        };

        std::vector<std::vector<CandidateEntry>> candidate_entries(hungarian_object_indices.size());
        std::vector<char> detection_candidate_mask(detections.size(), 0);

        for (size_t obj_idx = 0; obj_idx < hungarian_object_indices.size(); ++obj_idx) {
            int original_obj_index = hungarian_object_indices[obj_idx];
            const cv::Point3f& predicted = predicted_positions[original_obj_index];
            GlobalObject& obj = tracked_objects_[object_ids[original_obj_index]];

            for (int detection_index : hungarian_detection_indices) {
                if (detection_index < 0 ||
                    detection_index >= static_cast<int>(detection_world_index.size())) {
                    continue;
                }
                int world_index = detection_world_index[detection_index];
                if (world_index < 0 ||
                    world_index >= static_cast<int>(detection_world.size())) {
                    continue;
                }
                auto gate_result = passesSpatialGate(obj,
                                                     camera_id,
                                                     predicted,
                                                     detection_world[world_index],
                                                     timestamp);
                if (!gate_result) {
                    continue;
                }
                CandidateEntry entry;
                entry.detection_index = detection_index;
                entry.distance = *gate_result;
                candidate_entries[obj_idx].push_back(entry);
                detection_candidate_mask[detection_index] = 1;
            }
        }

        std::vector<int> active_object_rows;
        std::vector<int> object_to_row(hungarian_object_indices.size(), -1);
        active_object_rows.reserve(hungarian_object_indices.size());
        for (size_t obj_idx = 0; obj_idx < candidate_entries.size(); ++obj_idx) {
            if (!candidate_entries[obj_idx].empty()) {
                object_to_row[obj_idx] = static_cast<int>(active_object_rows.size());
                active_object_rows.push_back(static_cast<int>(obj_idx));
            }
        }

        std::vector<int> sparse_detection_indices;
        sparse_detection_indices.reserve(hungarian_detection_indices.size());
        std::unordered_map<int, int> detection_to_column;
        for (int detection_index : hungarian_detection_indices) {
            if (detection_index >= 0 &&
                detection_index < static_cast<int>(detection_candidate_mask.size()) &&
                detection_candidate_mask[detection_index]) {
                detection_to_column[detection_index] = static_cast<int>(sparse_detection_indices.size());
                sparse_detection_indices.push_back(detection_index);
            }
        }


        std::vector<std::vector<double>> cost;
        std::vector<std::vector<double>> penalties;
        std::vector<std::vector<double>> area_ratio_matrix;
        std::vector<std::vector<double>> descriptor_matrix;
        std::vector<std::vector<double>> distance_matrix;

        if (!active_object_rows.empty() && !sparse_detection_indices.empty()) {
            const double kInfCost = distance_threshold_ * 1000.0;
            cost.assign(active_object_rows.size(),
                        std::vector<double>(sparse_detection_indices.size(), kInfCost));
            penalties.assign(active_object_rows.size(),
                              std::vector<double>(sparse_detection_indices.size(), 1.0));
            area_ratio_matrix.assign(active_object_rows.size(),
                                     std::vector<double>(sparse_detection_indices.size(), -1.0));
            descriptor_matrix.assign(active_object_rows.size(),
                                     std::vector<double>(sparse_detection_indices.size(), -1.0));
            distance_matrix.assign(active_object_rows.size(),
                                   std::vector<double>(sparse_detection_indices.size(), kInfCost));

            for (size_t row_idx = 0; row_idx < active_object_rows.size(); ++row_idx) {
                int obj_idx = active_object_rows[row_idx];

                int original_obj_index = hungarian_object_indices[obj_idx];
                const cv::Point3f& predicted = predicted_positions[original_obj_index];
                GlobalObject& obj = tracked_objects_[object_ids[original_obj_index]];

                for (const auto& candidate : candidate_entries[obj_idx]) {
                    auto col_it = detection_to_column.find(candidate.detection_index);
                    if (col_it == detection_to_column.end()) {
                        continue;
                    }
                    size_t col_idx = static_cast<size_t>(col_it->second);
                    double base_distance = candidate.distance;
                    double penalty = 1.0;
                    double ratio = -1.0;

                    auto prev_it = obj.camera_detections.find(camera_id);
                    if (prev_it != obj.camera_detections.end()) {
                        double prev_area = static_cast<double>(prev_it->second.box.width) * prev_it->second.box.height;
                        double new_area = static_cast<double>(detections[candidate.detection_index].box.width) *
                                          detections[candidate.detection_index].box.height;
                        if (prev_area > 0.0) {
                            ratio = std::abs(new_area - prev_area) / prev_area;
                            if (ratio > area_change_threshold_) {
                                penalty = 1.0 + std::min(ratio - area_change_threshold_, 1.0);
                            }
                        }
                    }
                    auto descriptor_cost =
                        computeDescriptorCost(obj, detections[candidate.detection_index], camera_id);
                    if (descriptor_cost) {
                        descriptor_matrix[row_idx][col_idx] = *descriptor_cost;
                    }
                    double geometry_component = base_distance * penalty;
                    double geometry_weight = descriptor_cost ? 0.7 : 1.0;
                    double reid_weight = descriptor_cost ? 0.3 : 0.0;
                    double reid_component = 0.0;
                    if (descriptor_cost) {
                        double normalized = std::clamp(*descriptor_cost, 0.0, 1.5);
                        reid_component = normalized * distance_threshold_ * penalty;
                    }
                    cost[row_idx][col_idx] = geometry_weight * geometry_component + reid_weight * reid_component;
                    penalties[row_idx][col_idx] = penalty;
                    area_ratio_matrix[row_idx][col_idx] = ratio;
                    distance_matrix[row_idx][col_idx] = base_distance;

                }
            }
        }

        std::vector<int> assignment;
        if (!cost.empty()) {
            assignment = hungarianMatch(cost);
        }

        std::vector<int> detection_assignment(detections.size(), -1);
        std::vector<int> object_assignment(hungarian_object_indices.size(), -1);

        if (!assignment.empty()) {
            for (size_t row_idx = 0; row_idx < active_object_rows.size(); ++row_idx) {
                int assigned_col = assignment[row_idx];
                if (assigned_col >= 0 &&
                    assigned_col < static_cast<int>(sparse_detection_indices.size())) {
                    int obj_idx = active_object_rows[row_idx];
                    int detection_index = sparse_detection_indices[assigned_col];
                    object_assignment[obj_idx] = detection_index;
                    detection_assignment[detection_index] = obj_idx;
                }
            }
        }

        for (size_t idx = 0; idx < hungarian_object_indices.size(); ++idx) {
            int original_obj_index = hungarian_object_indices[idx];
            int obj_id = object_ids[original_obj_index];
            GlobalObject& obj = tracked_objects_[obj_id];

            bool matched = false;
            int detection_index = object_assignment[idx];
            if (detection_index >= 0) {
                int row_idx = object_to_row[idx];
                auto col_it = detection_to_column.find(detection_index);
                if (row_idx >= 0 && col_it != detection_to_column.end()) {
                    int col_idx = col_it->second;
                    double match_cost = cost[row_idx][col_idx];
                    detection_assignment[detection_index] = static_cast<int>(idx);
                    detection_cost[detection_index] = match_cost;
                    detection_cost_valid[detection_index] = true;
                    double penalty = penalties.empty() ? 1.0 : penalties[row_idx][col_idx];
                    double area_ratio = area_ratio_matrix.empty() ? -1.0 : area_ratio_matrix[row_idx][col_idx];
                    detection_penalty[detection_index] = penalty;
                    detection_area_ratio[detection_index] = area_ratio;
                    double threshold_with_penalty = distance_threshold_ * penalty;

                    if (match_cost < threshold_with_penalty) {
                        double base_distance = distance_matrix.empty() ? match_cost
                                                                       : distance_matrix[row_idx][col_idx];
                        if (std::isfinite(base_distance)) {
                            total_match_distance += base_distance;
                            ++match_distance_count;
                        }
                        if (original_obj_index < static_cast<int>(object_matched.size())) {
                            object_matched[original_obj_index] = true;
                        }
                        obj.camera_detections[camera_id] = detections[detection_index];
                        obj.camera_last_seen[camera_id] = timestamp;
                        obj.camera_missing_since.erase(camera_id);
                        obj.camera_miss_counts[camera_id] = 0;
                        if (detections[detection_index].descriptor.has_color ||
                            detections[detection_index].descriptor.has_grayscale) {
                            obj.last_descriptor = detections[detection_index].descriptor;
                            obj.has_last_descriptor = true;
                        }
                        if (penalty > 1.0) {
                            double reduction = 0.02 * penalty;
                            obj.confidence = std::max(0.0, obj.confidence - reduction);
                        } else {
                            obj.confidence = std::min(1.0, obj.confidence + 0.1);
                        }
                        detection_used[detection_index] = true;
                        matched = true;

                        detection_path[detection_index] = "hungarian";
                        detection_decision[detection_index] = "accept";
                        detection_global_id[detection_index] = obj.global_id;
                        detection_confidence[detection_index] = obj.confidence;
                        detection_confidence_valid[detection_index] = true;
                        double descriptor_value = descriptor_matrix.empty() ? -1.0
                                                                            : descriptor_matrix[row_idx][col_idx];
                        if (descriptor_value >= 0.0) {
                            detection_reid_score[detection_index] = descriptor_value;
                            detection_reid_valid[detection_index] = true;
                        }
                        updateReidScore(obj, camera_id, detections[detection_index]);

                        if (obj.primary_camera_id.empty() ||
                            [&]() {
                                double current_reid = std::numeric_limits<double>::infinity();
                                auto current_it = obj.camera_reid_scores.find(camera_id);
                                if (current_it != obj.camera_reid_scores.end()) {
                                    current_reid = current_it->second;
                                }
                                double best_reid = std::numeric_limits<double>::infinity();
                                if (!obj.primary_camera_id.empty()) {
                                    auto best_it = obj.camera_reid_scores.find(obj.primary_camera_id);
                                    if (best_it != obj.camera_reid_scores.end()) {
                                        best_reid = best_it->second;
                                    }
                                }
                                bool prefer_by_reid = std::isfinite(current_reid) &&
                                                      (!std::isfinite(best_reid) || current_reid + 0.05 < best_reid);
                                return prefer_by_reid;
                            }() ||
                            getPrimaryCameraPriority(camera_id) > getPrimaryCameraPriority(obj.primary_camera_id)) {
                            obj.primary_camera_id = camera_id;
                        }
                    } else {
                        detection_path[detection_index] = "hungarian";
                        detection_decision[detection_index] = "reject";
                    }
                }
            }

            if (!matched) {
                obj.confidence = std::max(0.0, obj.confidence - 0.05);
                auto prev_it = obj.camera_detections.find(camera_id);
                if (prev_it != obj.camera_detections.end()) {

                    LocalDetection prev_detection = prev_it->second;
                    if (prev_detection.descriptor.has_color || prev_detection.descriptor.has_grayscale) {
                        obj.last_descriptor = prev_detection.descriptor;
                        obj.has_last_descriptor = true;
                    }
                    obj.camera_detections.erase(prev_it);
                    obj.camera_last_seen.erase(camera_id);
                    obj.camera_missing_since.erase(camera_id);
                    obj.camera_miss_counts.erase(camera_id);
                    obj.camera_reid_scores.erase(camera_id);
                    logCameraDrop(camera_id, obj.global_id, prev_detection,
                                  obj.confidence, timestamp, "matching_failed");
                }
            }
        }

        for (int detection_index : hungarian_detection_indices) {
            if (detection_assignment[detection_index] < 0 &&
                detection_path[detection_index] == "unassigned") {
                detection_path[detection_index] = "hungarian";
                detection_decision[detection_index] = "no_assignment";
            }
        }

        for (size_t i = 0; i < detections.size(); ++i) {
            if (!detection_used[i]) {
                if (detections[i].box.width > 0 && detections[i].box.height > 0) {
                    int world_index = (i < detection_world_index.size()) ?
                                         detection_world_index[i] : -1;
                    double revived_descriptor = -1.0;
                    uint64_t dormant_ms = 0;
                    int revived_id = -1;
                    if (world_index >= 0 &&
                        world_index < static_cast<int>(detection_world.size())) {
                        revived_id = tryRevivePending(camera_id,
                                                       detections[i],
                                                       detection_world[world_index],
                                                       timestamp,
                                                       &revived_descriptor,
                                                       &dormant_ms);
                    }

                    if (revived_id > 0) {
                        detection_path[i] = "pending_reid";
                        detection_decision[i] = "revived";
                        detection_global_id[i] = revived_id;
                        auto obj_it = tracked_objects_.find(revived_id);
                        if (obj_it != tracked_objects_.end()) {
                            detection_confidence[i] = obj_it->second.confidence;
                            detection_confidence_valid[i] = true;
                            if (revived_descriptor >= 0.0) {
                                detection_reid_score[i] = revived_descriptor;
                                detection_reid_valid[i] = true;
                            }
                        }
                        if (dormant_ms > 0) {
                            detection_notes[i] = "dormant_ms=" + std::to_string(dormant_ms);
                        }
                        continue;
                    }

                    int new_id = createNewObject(camera_id, detections[i], timestamp);
                    if (new_id > 0) {
                        std::string previous_path = detection_path[i];
                        std::string previous_decision = detection_decision[i];

                        detection_path[i] = "new_object";
                        detection_decision[i] = "created";
                        detection_global_id[i] = new_id;
                        auto obj_it = tracked_objects_.find(new_id);
                        if (obj_it != tracked_objects_.end()) {
                            detection_confidence[i] = obj_it->second.confidence;
                            detection_confidence_valid[i] = true;
                            if (detections[i].descriptor.has_color || detections[i].descriptor.has_grayscale) {
                                detection_reid_score[i] = 0.0;
                                detection_reid_valid[i] = true;
                            }
                        }

                        if (!previous_path.empty() && previous_path != "unassigned" &&
                            previous_path != "invalid_box") {
                            if (!detection_notes[i].empty()) {
                                detection_notes[i] += ' ';
                            }
                            detection_notes[i] += "prev_path=" + previous_path;
                            if (!previous_decision.empty()) {
                                detection_notes[i] += " prev_decision=" + previous_decision;
                            }
                        }
                    }
                } else {
                    detection_path[i] = "invalid_box";
                    detection_decision[i] = "skipped";
                }
            }
        }
    }


    if (!stats_updated) {
        size_t matched_objects_count = 0;
        for (bool matched : object_matched) {
            if (matched) {
                ++matched_objects_count;
            }
        }
        double miss_ratio = object_matched.empty()
                                ? 0.0
                                : static_cast<double>(object_matched.size() - matched_objects_count) /
                                      static_cast<double>(object_matched.size());
        std::optional<double> average_distance;
        if (match_distance_count > 0) {
            average_distance = total_match_distance / static_cast<double>(match_distance_count);
        }
        tracker_stats_.update(static_cast<double>(active_objects_total), miss_ratio, average_distance);
        std::ostringstream oss;
        oss << "timestamp=" << timestamp
            << " camera=" << camera_id
            << " event=tracker_state"
            << " active_objects=" << active_objects_total
            << " miss_ratio=" << miss_ratio;
        if (average_distance) {
            oss << " match_distance=" << *average_distance;
        } else {
            oss << " match_distance=nan";
        }
        oss << " active_avg=" << tracker_stats_.activeAverage()
            << " miss_avg=" << tracker_stats_.missAverage();
        auto avg_distance = tracker_stats_.distanceAverage();
        if (avg_distance) {
            oss << " distance_avg=" << *avg_distance;
        }
        logDebugLine(oss.str());
    }

    for (size_t idx = 0; idx < detections.size(); ++idx) {
        const auto& det = detections[idx];
        std::ostringstream oss;
        oss << "timestamp=" << timestamp
            << " camera=" << camera_id
            << " event=detection_update"
            << " detection_idx=" << idx
            << " track_id=" << det.track_id
            << " box=[" << det.box.x << ',' << det.box.y << ','
            << det.box.width << ',' << det.box.height << ']';

        if (detection_global_id[idx] >= 0) {
            oss << " global_id=" << detection_global_id[idx];
        } else {
            oss << " global_id=-";
        }

        oss << " path=" << (detection_path[idx].empty() ? "unassigned" : detection_path[idx]);

        if (!detection_decision[idx].empty()) {
            oss << " decision=" << detection_decision[idx];
        }

        if (detection_cost_valid[idx]) {
            oss << " cost=" << detection_cost[idx]
                << " threshold=" << distance_threshold_;
        }

        if (detection_area_ratio[idx] >= 0.0) {
            oss << " area_ratio=" << detection_area_ratio[idx];
        } else {
            oss << " area_ratio=-";
        }

        oss << " penalty=" << detection_penalty[idx];

        if (detection_confidence_valid[idx]) {
            oss << " confidence=" << detection_confidence[idx];
        } else {
            oss << " confidence=-";
        }

        if (detection_reid_valid[idx]) {
            oss << " reid=" << detection_reid_score[idx];
        } else {
            oss << " reid=-";
        }

        if (!detection_notes[idx].empty()) {
            oss << " notes=" << detection_notes[idx];
        }

        logDebugLine(oss.str());
    }
}


std::optional<double> GlobalTracker::passesSpatialGate(const GlobalObject& obj,
                                                       const std::string& camera_id,
                                                       const cv::Point3f& predicted,
                                                       const cv::Point3f& detection,
                                                       uint64_t timestamp) const {
    if (!std::isfinite(predicted.x) || !std::isfinite(predicted.y) || !std::isfinite(predicted.z) ||
        !std::isfinite(detection.x) || !std::isfinite(detection.y) || !std::isfinite(detection.z)) {
        return std::nullopt;
    }

    double distance = cv::norm(predicted - detection);
    if (!std::isfinite(distance)) {
        return std::nullopt;
    }

    double dt_seconds = 0.0;
    if (timestamp > obj.last_seen_timestamp) {
        dt_seconds = static_cast<double>(timestamp - obj.last_seen_timestamp) / 1000.0;
    }

    double velocity_norm = cv::norm(obj.velocity);
    if (!std::isfinite(velocity_norm)) {
        velocity_norm = 0.0;
    }

    double dynamic_radius = distance_threshold_ + velocity_norm * dt_seconds;

    auto miss_it = obj.camera_miss_counts.find(camera_id);
    if (miss_it != obj.camera_miss_counts.end()) {
        int miss_count = std::clamp(miss_it->second, 0, kCameraMissCountThreshold);
        dynamic_radius *= 1.0 + static_cast<double>(miss_count) * 0.25;
    }

    dynamic_radius = std::clamp(dynamic_radius,
                                 min_distance_threshold_,
                                 max_distance_threshold_ * 2.0);

    if (distance > dynamic_radius) {
        return std::nullopt;
    }

    return distance;
}


cv::Point3f GlobalTracker::predictPosition(GlobalObject& obj, uint64_t timestamp) {
    if (timestamp <= obj.last_seen_timestamp) {
        return obj.world_position;
    }

    const double raw_dt = (timestamp - obj.last_seen_timestamp) / 1000.0; // seconds
    double dt = raw_dt;
    bool dt_clamped = false;
    bool velocity_reset = false;

    if (dt > kMaxPredictionHorizonSeconds) {
        dt = kMaxPredictionHorizonSeconds;
        dt_clamped = true;
    }

    cv::Point3f velocity = obj.velocity;
    if (raw_dt > kVelocityResetGapSeconds) {
        velocity = cv::Point3f(0.0f, 0.0f, 0.0f);
        obj.velocity = velocity;
        velocity_reset = true;
    }

    cv::Point3f predicted = obj.world_position + velocity * static_cast<float>(dt);

    if (dt_clamped || velocity_reset) {
        double previous_confidence = obj.confidence;
        obj.confidence = std::max(0.0, obj.confidence - kConfidenceDecayOnClamp);

        std::ostringstream oss;
        oss << "timestamp=" << timestamp
            << " event=predict_clamp"
            << " global_id=" << obj.global_id
            << " raw_dt=" << raw_dt
            << " used_dt=" << dt
            << " dt_clamped=" << (dt_clamped ? 1 : 0)
            << " velocity_reset=" << (velocity_reset ? 1 : 0)
            << " confidence_before=" << previous_confidence
            << " confidence_after=" << obj.confidence;
        logDebugLine(oss.str());
    }

    return predicted;
}

std::vector<int> GlobalTracker::hungarianMatch(const std::vector<std::vector<double>>& cost) {
    int n = static_cast<int>(cost.size());
    int m = n ? static_cast<int>(cost[0].size()) : 0;
    int dim = std::max(n, m);
    const double INF = 1e9;

    std::vector<std::vector<double>> a(dim, std::vector<double>(dim, INF));
    for (int i = 0; i < n; ++i)
        for (int j = 0; j < m; ++j)
            a[i][j] = cost[i][j];

    std::vector<double> u(dim + 1), v(dim + 1);
    std::vector<int> p(dim + 1), way(dim + 1);

    for (int i = 1; i <= dim; ++i) {
        p[0] = i;
        int j0 = 0;
        std::vector<double> minv(dim + 1, INF);
        std::vector<char> used(dim + 1, false);
        do {
            used[j0] = true;
            int i0 = p[j0], j1 = 0;
            double delta = INF;
            for (int j = 1; j <= dim; ++j) {
                if (!used[j]) {
                    double cur = a[i0 - 1][j - 1] - u[i0] - v[j];
                    if (cur < minv[j]) {
                        minv[j] = cur;
                        way[j] = j0;
                    }
                    if (minv[j] < delta) {
                        delta = minv[j];
                        j1 = j;
                    }
                }
            }
            for (int j = 0; j <= dim; ++j) {
                if (used[j]) {
                    u[p[j]] += delta;
                    v[j] -= delta;
                } else {
                    minv[j] -= delta;
                }
            }
            j0 = j1;
        } while (p[j0] != 0);
        do {
            int j1 = way[j0];
            p[j0] = p[j1];
            j0 = j1;
        } while (j0);
    }

    std::vector<int> assignment(n, -1);
    for (int j = 1; j <= dim; ++j) {
        if (p[j] <= n && j <= m) {
            assignment[p[j] - 1] = j - 1;
        }
    }
    return assignment;
}


int GlobalTracker::createNewObject(const std::string& camera_id,
                                   const LocalDetection& detection,
                                   uint64_t timestamp) {
    std::lock_guard<std::recursive_mutex> lock(mutex_);


    // Validate detection bounds
    if (detection.box.width <= 0 || detection.box.height <= 0) {
        std::cerr << "Invalid detection bounds for camera " << camera_id << std::endl;
        return -1;
    }


    cv::Point2f detection_center(
        detection.box.x + detection.box.width * 0.5f,
        detection.box.y + detection.box.height * 0.5f
    );
    cv::Point2f detection_feet(
        detection.box.x + detection.box.width * 0.5f,
        detection.box.y + detection.box.height
    );

    ImageToWorldResult projection = imageToWorld(camera_id, detection_feet);
    if (!projection.valid) {
        ImageToWorldResult fallback_projection = imageToWorld(camera_id, detection_center);
        if (fallback_projection.valid) {
            projection = fallback_projection;
        }
    }

    if (!projection.valid) {
        std::ostringstream oss;
        oss << "timestamp=" << timestamp
            << " camera=" << camera_id
            << " event=object_creation_skipped"
            << " reason=projection_failed"
            << " track_id=" << detection.track_id
            << " box=[" << detection.box.x << ',' << detection.box.y << ','
            << detection.box.width << ',' << detection.box.height << ']';
        logDebugLine(oss.str());
        std::cerr << "Failed to project detection for camera " << camera_id
                  << ", skipping object creation" << std::endl;
        return -1;
    }


    GlobalObject new_obj;

    // Prevent integer overflow in global ID
    if (next_global_id_ >= std::numeric_limits<int>::max() - 1000) {
        next_global_id_ = 1;
    }
    new_obj.global_id = next_global_id_++;

    new_obj.camera_detections[camera_id] = detection;
    if (detection.descriptor.has_color || detection.descriptor.has_grayscale) {
        new_obj.camera_reid_scores[camera_id] = 0.0;
        new_obj.last_descriptor = detection.descriptor;
        new_obj.has_last_descriptor = true;
    } else {
        new_obj.has_last_descriptor = false;
    }
    new_obj.camera_last_seen[camera_id] = timestamp;
    new_obj.camera_miss_counts[camera_id] = 0;
    new_obj.primary_camera_id = camera_id;
    new_obj.last_seen_timestamp = timestamp;
    new_obj.confidence = 0.5; // Начальная уверенность
    new_obj.velocity = cv::Point3f(0, 0, 0); // Начальная скорость
    new_obj.residual = cv::Point3f(0, 0, 0);
    new_obj.residual_magnitude = 0.0f;
    new_obj.residual_speed = 0.0f;
    new_obj.residual_position_threshold_exceeded = false;
    new_obj.residual_speed_threshold_exceeded = false;


    auto stereo_world = tryStereoTriangulation(new_obj, camera_id,
                                               detection_feet,
                                               detection_center,
                                               timestamp);
    if (stereo_world) {
        new_obj.world_position = *stereo_world;
        new_obj.world_position_from_stereo = true;
    } else {
        new_obj.world_position = projection.world_point;
        new_obj.world_position_from_stereo = false;
    }
    new_obj.position_history.reserve(20); // Pre-allocate for efficiency
    new_obj.position_history.push_back(new_obj.world_position);

    int created_id = new_obj.global_id;
    double created_confidence = new_obj.confidence;
    tracked_objects_[created_id] = std::move(new_obj); // Use move for efficiency

    std::ostringstream oss;
    oss << "timestamp=" << timestamp
        << " camera=" << camera_id
        << " event=object_created"
        << " global_id=" << created_id
        << " track_id=" << detection.track_id
        << " box=[" << detection.box.x << ',' << detection.box.y << ','
        << detection.box.width << ',' << detection.box.height << ']'
        << " confidence=" << created_confidence;
    logDebugLine(oss.str());

    std::cout << "Создан новый объект ID=" << created_id
              << " на камере " << camera_id << std::endl;

    return created_id;
}

void GlobalTracker::updateWorldPositions(const std::string& camera_id, uint64_t timestamp) {
    std::function<void(const ResidualStatisticsSnapshot&)> metrics_callback;
    ResidualStatisticsSnapshot metrics_snapshot;

    {
        std::lock_guard<std::recursive_mutex> lock(mutex_);
        // Parameter validation
        if (camera_id.empty() || timestamp == 0) {
            return;
        }

        for (auto& pair : tracked_objects_) {
            GlobalObject& obj = pair.second;

            // Обновляем позицию только если объект виден на этой камере
            auto det_it = obj.camera_detections.find(camera_id);
            if (det_it != obj.camera_detections.end()) {
                auto missing_it = obj.camera_missing_since.find(camera_id);
                if (missing_it != obj.camera_missing_since.end()) {
                    continue;
                }
                const cv::Rect& detection = det_it->second.box;
                // Validate detection
                if (detection.width <= 0 || detection.height <= 0) {
                    continue;
                }

                cv::Point2f detection_center(
                    detection.x + detection.width * 0.5f,
                    detection.y + detection.height * 0.5f
                );

                cv::Point2f detection_feet(
                    detection.x + detection.width * 0.5f,
                    detection.y + detection.height
                );

                cv::Point3f previous_world = obj.world_position;
                cv::Point3f predicted = predictPosition(obj, timestamp);

                cv::Point3f new_world_pos;
                ImageToWorldResult projection;

                auto stereo_world = tryStereoTriangulation(obj, camera_id,
                                                          detection_feet,
                                                          detection_center,
                                                          timestamp);

                if (stereo_world) {
                    new_world_pos = *stereo_world;
                    obj.world_position_from_stereo = true;
                } else {
                    projection = imageToWorld(camera_id, detection_feet);
                    if (!projection.valid) {
                        ImageToWorldResult fallback_projection =
                            imageToWorld(camera_id, detection_center);
                        if (fallback_projection.valid) {
                            projection = fallback_projection;
                        } else {
                            bool ray_parallel_failure =
                                projection.ray_parallel || fallback_projection.ray_parallel;
                            if (ray_parallel_failure) {
                                residual_stats_.ground_projection_ray_parallel_skipped += 1;
                                std::ostringstream oss;
                                oss << "timestamp=" << timestamp
                                    << " camera=" << camera_id
                                    << " event=ground_projection_skipped"
                                    << " global_id=" << obj.global_id
                                    << " reason=ray_parallel"
                                    << " box=[" << detection.x << ',' << detection.y << ','
                                    << detection.width << ',' << detection.height << ']';
                                logDebugLine(oss.str());
                            } else {
                                std::ostringstream oss;
                                oss << "timestamp=" << timestamp
                                    << " camera=" << camera_id
                                    << " event=ground_projection_skipped"
                                    << " global_id=" << obj.global_id
                                    << " reason=projection_failed"
                                    << " box=[" << detection.x << ',' << detection.y << ','
                                    << detection.width << ',' << detection.height << ']';
                                logDebugLine(oss.str());
                            }
                            continue;
                        }
                    }
                    if (projection.reused_previous_world) {
                        residual_stats_.ground_projection_ray_parallel_reused += 1;
                    }
                    if (projection.nudged_ray) {
                        residual_stats_.ground_projection_ray_parallel_nudged += 1;
                    }

                    new_world_pos = projection.world_point;
                    obj.world_position_from_stereo = false;
                }

                double dt_seconds = 0.0;
                bool dt_available = false;
                if (obj.last_seen_timestamp > 0 && timestamp > obj.last_seen_timestamp) {
                    dt_seconds = (timestamp - obj.last_seen_timestamp) / 1000.0;
                    dt_available = true;
                }

                bool dt_valid = dt_available && dt_seconds > 0.0 && dt_seconds < 10.0;

                cv::Point3f residual = new_world_pos - predicted;
                obj.residual = residual;
                obj.residual_magnitude = cv::norm(residual);
                obj.residual_speed = (dt_valid && dt_seconds > 0.0)
                                         ? static_cast<float>(obj.residual_magnitude / dt_seconds)
                                         : 0.0f;
                obj.residual_position_threshold_exceeded =
                    obj.residual_magnitude > residual_position_threshold_;
                obj.residual_speed_threshold_exceeded =
                    obj.residual_speed > residual_speed_threshold_;

                residual_stats_.total_residual_magnitude += obj.residual_magnitude;
                residual_stats_.total_residual_speed += obj.residual_speed;
                residual_stats_.sample_count += 1;
                residual_stats_.max_residual_magnitude = std::max(
                    residual_stats_.max_residual_magnitude,
                    static_cast<double>(obj.residual_magnitude));
                residual_stats_.max_residual_speed = std::max(
                    residual_stats_.max_residual_speed,
                    static_cast<double>(obj.residual_speed));

                if (obj.residual_position_threshold_exceeded) {
                    residual_stats_.residual_threshold_exceeded += 1;
                    residual_stats_.calibration_suspect = true;
                }
                if (obj.residual_speed_threshold_exceeded) {
                    residual_stats_.speed_threshold_exceeded += 1;
                    residual_stats_.timestamp_delay_suspect = true;
                }

                if (obj.residual_position_threshold_exceeded ||
                    obj.residual_speed_threshold_exceeded) {
                    std::ostringstream oss;
                    oss << "timestamp=" << timestamp
                        << " camera=" << camera_id
                        << " event=residual_alert"
                        << " global_id=" << obj.global_id
                        << " residual=" << obj.residual_magnitude
                        << " residual_speed=" << obj.residual_speed
                        << " residual_threshold=" << residual_position_threshold_
                        << " residual_speed_threshold=" << residual_speed_threshold_;
                    if (obj.residual_position_threshold_exceeded) {
                        oss << " calibration_suspect=1";
                    }
                    if (obj.residual_speed_threshold_exceeded) {
                        oss << " timestamp_delay_suspect=1";
                    }
                    logDebugLine(oss.str());
                }

                if (residual_summary_log_interval_ > 0 &&
                    residual_stats_.sample_count - residual_last_logged_sample_ >=
                        residual_summary_log_interval_) {
                    double avg_residual = residual_stats_.sample_count > 0
                                              ? residual_stats_.total_residual_magnitude /
                                                    static_cast<double>(residual_stats_.sample_count)
                                              : 0.0;
                    double avg_residual_speed = residual_stats_.sample_count > 0
                                                    ? residual_stats_.total_residual_speed /
                                                          static_cast<double>(residual_stats_.sample_count)
                                                    : 0.0;

                    std::ostringstream summary;
                    summary << "timestamp=" << timestamp
                            << " event=residual_summary"
                            << " samples=" << residual_stats_.sample_count
                            << " avg_residual=" << avg_residual
                            << " avg_residual_speed=" << avg_residual_speed
                            << " max_residual=" << residual_stats_.max_residual_magnitude
                            << " max_residual_speed=" << residual_stats_.max_residual_speed
                            << " residual_threshold_hits=" << residual_stats_.residual_threshold_exceeded
                            << " speed_threshold_hits=" << residual_stats_.speed_threshold_exceeded
                            << " ground_projection_ray_parallel_skipped="
                            << residual_stats_.ground_projection_ray_parallel_skipped
                            << " ground_projection_ray_parallel_reused="
                            << residual_stats_.ground_projection_ray_parallel_reused
                            << " ground_projection_ray_parallel_nudged="
                            << residual_stats_.ground_projection_ray_parallel_nudged
                            << " calibration_suspect=" << (residual_stats_.calibration_suspect ? 1 : 0)
                            << " timestamp_delay_suspect="
                            << (residual_stats_.timestamp_delay_suspect ? 1 : 0);
                    logDebugLine(summary.str());
                    residual_last_logged_sample_ = residual_stats_.sample_count;
                }

                if (dt_valid) {
                    cv::Point3f displacement = new_world_pos - previous_world;
                    obj.velocity = displacement / static_cast<float>(dt_seconds);

                    // Limit velocity to reasonable values
                    float speed = cv::norm(obj.velocity);
                    if (speed > speed_threshold_) {
                        obj.velocity = obj.velocity * (speed_threshold_ / speed);
                    }
                } else if (dt_available) {
                    obj.velocity = cv::Point3f(0, 0, 0); // Reset on time jump
                }

                obj.world_position = new_world_pos;
                obj.position_history.push_back(new_world_pos);

                // Limit history size for memory efficiency
                const size_t max_history = 20;
                if (obj.position_history.size() > max_history) {
                    obj.position_history.erase(obj.position_history.begin(),
                                              obj.position_history.end() - max_history);
                }

                obj.last_seen_timestamp = timestamp;
            }
        }
        if (residual_metrics_callback_) {
            metrics_snapshot = residual_stats_;
            metrics_callback = residual_metrics_callback_;
        }
    }
    if (metrics_callback) {
        metrics_callback(metrics_snapshot);
    }
}

int GlobalTracker::getPrimaryCameraPriority(const std::string& camera_id) {
    CameraConfig* cam = scheme_manager_->getCamera(camera_id);
    if (cam) {
        return cam->priority;
    }
    return 0;
}
ImageToWorldResult GlobalTracker::imageToWorld(const std::string& camera_id,
                                               const cv::Point2f& image_point,
                                               std::optional<cv::Point3f> previous_world) {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    ImageToWorldResult result;
    result.sampled_image_point = image_point;

    // Validate input parameters
    if (camera_id.empty()) {
        std::cerr << "Invalid camera ID" << std::endl;
        return result;
    }

    auto calib_it = camera_calibrations_.find(camera_id);
    if (calib_it == camera_calibrations_.end() || !calib_it->second.is_calibrated) {
        std::cerr << "Камера " << camera_id << " не откалибрована" << std::endl;
        return result;
    }

    const CameraCalibration& calib = calib_it->second;

    // Validate calibration matrices
    if (calib.camera_matrix.empty() || calib.rotation_vector.empty() || 
        calib.translation_vector.empty()) {
        std::cerr << "Некорректные калибровочные данные для камеры " << camera_id << std::endl;
        return result;
    }

    try {
        constexpr double kRayParallelThreshold = 1e-6;
        constexpr float kGroundNudgePixels = 2.0f;
        constexpr int kMaxNudges = 5;

        bool used_nudge = false;
        cv::Point2f adjusted_point = image_point;

        cv::Mat R;
        cv::Rodrigues(calib.rotation_vector, R);
        R.convertTo(R, CV_64F);
        cv::Mat R_inv = R.t();

        cv::Mat cam_position = cv::Mat::zeros(3, 1, CV_64F);
        if (calib.has_world_position) {
            cam_position.at<double>(0, 0) = static_cast<double>(calib.world_position.x);
            cam_position.at<double>(1, 0) = static_cast<double>(calib.world_position.y);
            cam_position.at<double>(2, 0) = static_cast<double>(calib.world_position.z);
        } else {
            cv::Mat translation_vector;
            calib.translation_vector.convertTo(translation_vector, CV_64F);
            cam_position = -R_inv * translation_vector;
        }

        for (int attempt = 0; attempt <= kMaxNudges; ++attempt) {
            std::vector<cv::Point2f> image_points = {adjusted_point};
            std::vector<cv::Point2f> undistorted_points;

            cv::undistortPoints(image_points, undistorted_points,
                                calib.camera_matrix, calib.dist_coeffs);

            if (undistorted_points.empty()) {
                return result;
            }

            cv::Mat ray = (cv::Mat_<double>(3, 1)
                               << static_cast<double>(undistorted_points[0].x),
                               static_cast<double>(undistorted_points[0].y),
                               1.0);
            cv::Mat ray_world = R_inv * ray;
            double ray_z = ray_world.at<double>(2);

            if (std::abs(ray_z) < kRayParallelThreshold) {
                if (previous_world) {
                    result.world_point = *previous_world;
                    result.valid = true;
                    result.reused_previous_world = true;
                    result.nudged_ray = used_nudge;
                    result.sampled_image_point = adjusted_point;
                    return result;
                }

                if (attempt < kMaxNudges) {
                    adjusted_point.y += kGroundNudgePixels;
                    used_nudge = true;
                    continue;
                }

                result.ray_parallel = true;
                return result;
            }

            double t = -cam_position.at<double>(2) / ray_z;

            result.world_point = cv::Point3f(
                static_cast<float>(cam_position.at<double>(0) + t * ray_world.at<double>(0)),
                static_cast<float>(cam_position.at<double>(1) + t * ray_world.at<double>(1)),
                0.0f);
            result.valid = true;
            result.nudged_ray = used_nudge;
            result.sampled_image_point = adjusted_point;
            return result;
        }
        return result;
    } catch (const cv::Exception& e) {
        std::cerr << "OpenCV error in imageToWorld: " << e.what() << std::endl;
        return result;
    }
}

cv::Point2f GlobalTracker::worldToImage(const std::string& camera_id, const cv::Point3f& world_point) {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    auto calib_it = camera_calibrations_.find(camera_id);
    if (calib_it == camera_calibrations_.end() || !calib_it->second.is_calibrated) {
        return cv::Point2f(-1, -1);
    }

    const CameraCalibration& calib = calib_it->second;

    // Преобразуем мировую точку в координаты камеры
    std::vector<cv::Point3f> world_points = {world_point};
    std::vector<cv::Point2f> image_points;
    
    cv::projectPoints(world_points, calib.rotation_vector, calib.translation_vector,
                     calib.camera_matrix, calib.dist_coeffs, image_points);

    return image_points[0];
}



std::optional<cv::Point3f> GlobalTracker::getCameraWorldPosition(const std::string& camera_id) const {
    std::lock_guard<std::recursive_mutex> lock(mutex_);

    auto calib_it = camera_calibrations_.find(camera_id);
    if (calib_it == camera_calibrations_.end() || !calib_it->second.is_calibrated) {
        return std::nullopt;
    }

    const CameraCalibration& calib = calib_it->second;
    if (calib.has_world_position) {
        return calib.world_position;
    }
    if (calib.rotation_vector.empty() || calib.translation_vector.empty()) {
        return std::nullopt;
    }

    cv::Mat rotation_vector;
    cv::Mat translation_vector;
    calib.rotation_vector.convertTo(rotation_vector, CV_64F);
    calib.translation_vector.convertTo(translation_vector, CV_64F);

    if (rotation_vector.empty() || translation_vector.empty()) {
        return std::nullopt;
    }

    cv::Mat R;
    cv::Rodrigues(rotation_vector, R);
    cv::Mat R_inv = R.t();
    cv::Mat cam_position = -R_inv * translation_vector;

    if (cam_position.rows != 3 || cam_position.cols != 1) {
        return std::nullopt;
    }

    cv::Point3f position(
        static_cast<float>(cam_position.at<double>(0)),
        static_cast<float>(cam_position.at<double>(1)),
        static_cast<float>(cam_position.at<double>(2))
    );

    return position;
}

void GlobalTracker::setResidualMetricsCallback(
    std::function<void(const ResidualStatisticsSnapshot&)> callback) {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    residual_metrics_callback_ = std::move(callback);
}

ResidualStatisticsSnapshot GlobalTracker::getResidualStatistics() const {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    return residual_stats_;
}


bool GlobalTracker::calibrateCamera(const std::string& camera_id,
                                   const std::vector<cv::Point3f>& world_points,
                                   const std::vector<cv::Point2f>& image_points) {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    if (world_points.size() != image_points.size() || world_points.size() < 4) {
        std::cerr << "Недостаточно точек для калибровки камеры " << camera_id << std::endl;
        return false;
    }

    CameraCalibration& calib = camera_calibrations_[camera_id];

    // Получаем параметры камеры
    CameraConfig* cam = scheme_manager_->getCamera(camera_id);
    if (!cam) {
        std::cerr << "Камера " << camera_id << " не найдена" << std::endl;
        return false;
    }

    // Инициализируем внутренние параметры если не заданы
    if (calib.camera_matrix.empty()) {
        setupFallbackCameraIntrinsics(*cam, calib);
    }

    // Вычисляем внешние параметры
    bool success = cv::solvePnP(world_points, image_points, 
                               calib.camera_matrix, calib.dist_coeffs,
                               calib.rotation_vector, calib.translation_vector);

    if (success) {
        calib.is_calibrated = true;
        std::cout << "Калибровка камеры " << camera_id << " успешна" << std::endl;

        // Вычисляем матрицу гомографии для плоскости z=0
        computeHomography(calib);
        updateCalibrationWorldPosition(calib);
        return true;
    }
    
    std::cerr << "Ошибка калибровки камеры " << camera_id << std::endl;
    return false;
}

void GlobalTracker::computeHomography(CameraCalibration& calib) {
    // Создаем точки на плоскости z=0 для вычисления гомографии
    std::vector<cv::Point3f> plane_points = {
        cv::Point3f(-5, -5, 0), cv::Point3f(5, -5, 0),
        cv::Point3f(5, 5, 0), cv::Point3f(-5, 5, 0)
    };
    
    std::vector<cv::Point2f> image_points;
    cv::projectPoints(plane_points, calib.rotation_vector, calib.translation_vector,
                     calib.camera_matrix, calib.dist_coeffs, image_points);
    
    std::vector<cv::Point2f> plane_points_2d;
    for (const auto& pt : plane_points) {
        plane_points_2d.push_back(cv::Point2f(pt.x, pt.y));
    }
    
    calib.homography_matrix = cv::findHomography(image_points, plane_points_2d);
}

void GlobalTracker::updateCalibrationWorldPosition(CameraCalibration& calib) {
    calib.has_world_position = false;
    if (calib.rotation_vector.empty() || calib.translation_vector.empty()) {
        return;
    }

    cv::Mat rvec64;
    calib.rotation_vector.convertTo(rvec64, CV_64F);
    cv::Mat tvec64;
    calib.translation_vector.convertTo(tvec64, CV_64F);

    if (rvec64.rows != 3 || rvec64.cols != 1 ||
        tvec64.rows != 3 || tvec64.cols != 1) {
        return;
    }

    cv::Mat R;
    cv::Rodrigues(rvec64, R);
    if (R.rows != 3 || R.cols != 3) {
        return;
    }

    cv::Mat cam_center = -R.t() * tvec64;
    if (cam_center.rows != 3 || cam_center.cols != 1) {
        return;
    }

    calib.world_position = cv::Point3f(
        static_cast<float>(cam_center.at<double>(0, 0)),
        static_cast<float>(cam_center.at<double>(1, 0)),
        static_cast<float>(cam_center.at<double>(2, 0))
    );
    calib.has_world_position = true;
}


void GlobalTracker::handleSchemeChange(SchemeType new_scheme) {

    std::cout << "Переключение схемы на: "
              << scheme_manager_->schemeTypeToString(new_scheme) << std::endl;

    scheme_manager_->setScheme(new_scheme);

    if (camera_manager_) {
        std::map<std::string, std::string> roles;
        for (const auto& cam : scheme_manager_->getCameras()) {
            roles[cam.id] = scheme_manager_->roleToString(cam.role);
        }
        camera_manager_->saveConfig(
            scheme_manager_->schemeTypeToString(new_scheme), roles);
    }


    // Очищаем текущие объекты при смене схемы
    {
        std::lock_guard<std::recursive_mutex> lock(mutex_);
        tracked_objects_.clear();
        next_global_id_ = 1;
        // Переинициализируем калибровку для новой схемы
        camera_calibrations_.clear();
    }

    if (calibration_watcher_) {
        calibration_watcher_->loadResults();
        updateCalibrationTimestamp(*calibration_watcher_);
    }

    if (!performAutoCalibration()) {
        std::cerr << "Не удалось обновить калибровку для схемы "
                  << scheme_manager_->schemeTypeToString(new_scheme) << std::endl;
    }
}

double GlobalTracker::calculateDistance(const cv::Point3f& p1, const cv::Point3f& p2) {
    return cv::norm(p1 - p2);
}

bool GlobalTracker::initializeCameraCalibration(const std::string& camera_id) {
    {
        std::lock_guard<std::recursive_mutex> lock(mutex_);
        if (camera_calibrations_.find(camera_id) != camera_calibrations_.end()) {
            return true;
        }
    }

    CameraConfig* cam = scheme_manager_->getCamera(camera_id);
    if (!cam) {
        std::cerr << "Камера " << camera_id << " не найдена" << std::endl;
        return false;
    }

    CameraCalibration calib;
    SchemeType scheme = scheme_manager_->getCurrentScheme();

    cv::Mat K, D;
    if (calibration_watcher_ && calibration_watcher_->getCameraMatrix(camera_id, K, D)) {
        calib.camera_matrix = K.clone();
        calib.dist_coeffs = D.clone();
    } else {
        setupFallbackCameraIntrinsics(*cam, calib);
    }

    if (calib.rotation_vector.empty()) {
        calib.rotation_vector = cv::Mat::zeros(3, 1, CV_64F);
    }
    if (calib.translation_vector.empty()) {
        calib.translation_vector = cv::Mat::zeros(3, 1, CV_64F);
    }
    calib.homography_matrix = cv::Mat::eye(3, 3, CV_64F);
    bool extrinsics_ok = applyFallbackExtrinsics(*cam, scheme, calib);
    if (!extrinsics_ok || calib.rotation_vector.empty() || calib.translation_vector.empty()) {
        std::cerr << "Не удалось применить внешние параметры для камеры " << camera_id
                  << " (" << scheme_manager_->roleToString(cam->role) << ")." << std::endl;
        return false;
    }
    calib.is_calibrated = true;
    computeHomography(calib);
    updateCalibrationWorldPosition(calib);

    {
        std::lock_guard<std::recursive_mutex> lock(mutex_);
        camera_calibrations_[camera_id] = std::move(calib);
    }
    return true;
}

void GlobalTracker::cleanupOldObjects(uint64_t current_timestamp) {
    std::lock_guard<std::recursive_mutex> lock(mutex_);

    if (current_timestamp == 0) {
        return;
    }


    prunePendingReid(current_timestamp);

    // Use iterator-based removal for better performance
    auto it = tracked_objects_.begin();
    size_t removed_count = 0;
    while (it != tracked_objects_.end()) {
        if (it->second.last_seen_timestamp == 0) {
            // Handle invalid timestamp
            {
                std::ostringstream oss;
                oss << "timestamp=" << current_timestamp
                    << " event=object_removed"
                    << " global_id=" << it->second.global_id
                    << " reason=invalid_timestamp"
                    << " confidence=" << it->second.confidence;
                logDebugLine(oss.str());
            }
            it = tracked_objects_.erase(it);
            removed_count++;
            continue;
        }

        uint64_t age = (current_timestamp > it->second.last_seen_timestamp) ?
                       (current_timestamp - it->second.last_seen_timestamp) : 0;

        bool has_camera_mappings = false;
        auto camera_it = it->second.camera_detections.begin();
        while (camera_it != it->second.camera_detections.end()) {
            const std::string& mapped_camera_id = camera_it->first;
            uint64_t last_seen = 0;
            auto last_seen_it = it->second.camera_last_seen.find(mapped_camera_id);
            if (last_seen_it != it->second.camera_last_seen.end()) {
                last_seen = last_seen_it->second;
            } else {
                last_seen = it->second.last_seen_timestamp;
            }

            uint64_t gap = (last_seen > 0 && current_timestamp > last_seen)
                               ? (current_timestamp - last_seen)
                               : 0;
            bool camera_expired = gap > retention_time_ms_;
            int miss_count = 0;
            auto miss_count_it = it->second.camera_miss_counts.find(mapped_camera_id);
            if (miss_count_it != it->second.camera_miss_counts.end()) {
                miss_count = miss_count_it->second;
            }
            bool camera_miss_threshold_exceeded = miss_count >= kCameraMissCountThreshold;


            if (camera_expired || camera_miss_threshold_exceeded) {
                std::string reason;
                if (camera_expired) {
                    reason = "retention_expired";
                } else {
                    reason = "miss_count_exceeded";
                }
                LocalDetection prev_detection = camera_it->second;
                if (prev_detection.descriptor.has_color || prev_detection.descriptor.has_grayscale) {
                    it->second.last_descriptor = prev_detection.descriptor;
                    it->second.has_last_descriptor = true;
                }
                logCameraDrop(mapped_camera_id, it->second.global_id, prev_detection,
                              it->second.confidence, current_timestamp, reason);
                it->second.camera_last_seen.erase(mapped_camera_id);
                it->second.camera_missing_since.erase(mapped_camera_id);
                it->second.camera_miss_counts.erase(mapped_camera_id);
                camera_it = it->second.camera_detections.erase(camera_it);
            } else {
                has_camera_mappings = true;
                ++camera_it;
            }
        }
        bool is_expired = age > retention_time_ms_;
        bool is_low_confidence = it->second.confidence < 0.1;
        bool cache_dormant = pending_reid_retention_ms_ > 0;

        if (!has_camera_mappings && (is_expired || is_low_confidence)) {
            std::string reason = is_expired ? "expired" : "low_confidence";
            {
                std::ostringstream oss;
                oss << "timestamp=" << current_timestamp
                    << " event=object_removed"
                    << " global_id=" << it->second.global_id
                    << " reason=" << reason
                    << " age_ms=" << age
                    << " confidence=" << it->second.confidence
                    << " dormant_cached=" << (cache_dormant ? 1 : 0);
                logDebugLine(oss.str());
            }

            if (cache_dormant) {
                int dormant_id = it->second.global_id;
                PendingReidEntry entry;
                entry.object = std::move(it->second);
                entry.stored_timestamp = current_timestamp;
                entry.object.camera_detections.clear();
                entry.object.camera_last_seen.clear();
                entry.object.camera_missing_since.clear();
                entry.object.camera_miss_counts.clear();
                entry.object.camera_reid_scores.clear();

                auto [inserted_it, _] = pending_reid_.insert_or_assign(dormant_id, std::move(entry));

                {
                    std::ostringstream oss;
                    oss << "timestamp=" << current_timestamp
                        << " event=object_dormant"
                        << " global_id=" << dormant_id
                        << " age_ms=" << age
                        << " descriptor_available="
                        << (inserted_it->second.object.has_last_descriptor ? 1 : 0);
                    logDebugLine(oss.str());
                }

                std::cout << "Объект ID=" << dormant_id
                          << " сохранен для повторной идентификации (возраст="
                          << age << "мс)" << std::endl;
            } else {
                std::cout << "Удален объект ID=" << it->second.global_id
                          << " (возраст=" << age << "мс, уверенность="
                          << it->second.confidence << ")" << std::endl;
            }

            it = tracked_objects_.erase(it);
            removed_count++;
        } else if (has_camera_mappings && is_expired) {
            it->second.confidence = std::max(0.0, it->second.confidence * 0.95);
            {
                std::ostringstream oss;
                oss << "timestamp=" << current_timestamp
                    << " event=deferred_cleanup"
                    << " global_id=" << it->second.global_id
                    << " reason=active_camera_mappings"
                    << " age_ms=" << age
                    << " confidence=" << it->second.confidence;
                logDebugLine(oss.str());
            }
            ++it;
        } else {
            ++it;
        }
    }

    if (removed_count > 0) {
        std::cout << "Очистка завершена: удалено " << removed_count << " объектов" << std::endl;
    }
}

void GlobalTracker::rebuildAssignmentsLocked() {
    last_assignments_.clear();
    size_t estimate = 0;
    for (const auto& pair : tracked_objects_) {
        estimate += pair.second.camera_detections.size();
    }
    last_assignments_.reserve(estimate);
    for (const auto& [global_id, obj] : tracked_objects_) {
        for (const auto& [camera_id, detection] : obj.camera_detections) {
            if (detection.track_id < 0) {
                continue;
            }
            GlobalAssignment assignment;
            assignment.cam_id = camera_id;
            assignment.local_id = detection.track_id;
            assignment.global_id = global_id;
            last_assignments_.push_back(std::move(assignment));
        }
    }
}


std::vector<GlobalObject> GlobalTracker::getActiveObjects() {
    std::vector<GlobalObject> active_objects;

    std::lock_guard<std::recursive_mutex> lock(mutex_);
    for (const auto& pair : tracked_objects_) {
        if (pair.second.confidence > 0.3) { // Порог уверенности
            active_objects.push_back(pair.second);
        }
    }


    return active_objects;
}

std::vector<GlobalTracker::TrackGlobalMapping> GlobalTracker::getTrackToGlobalMapForCamera(
    const std::string& camera_id) {
    std::vector<TrackGlobalMapping> mapping;
    if (camera_id.empty()) {
        return mapping;
    }


    std::unordered_set<std::string> active_camera_ids;
    if (scheme_manager_) {
        auto active_ptrs = scheme_manager_->getActiveCameras();
        for (const auto* cam : active_ptrs) {
            if (cam) {
                active_camera_ids.insert(cam->id);
            }
        }
        if (active_camera_ids.empty()) {
            for (const auto& cam : scheme_manager_->getCameras()) {
                if (cam.status == CameraStatus::ACTIVE) {
                    active_camera_ids.insert(cam.id);
                }
            }
        }
    }
    const int total_active_cameras = static_cast<int>(active_camera_ids.size());

    auto camera_position = getCameraWorldPosition(camera_id);

    std::lock_guard<std::recursive_mutex> lock(mutex_);
    for (const auto& [global_id, obj] : tracked_objects_) {
        auto det_it = obj.camera_detections.find(camera_id);
        if (det_it != obj.camera_detections.end() && det_it->second.track_id >= 0) {
            TrackGlobalMapping info;
            info.track_id = det_it->second.track_id;
            info.global_id = global_id;
            int visible_cameras = 0;
            for (const auto& [mapped_camera_id, detection] : obj.camera_detections) {
                if (!active_camera_ids.empty() &&
                    active_camera_ids.find(mapped_camera_id) == active_camera_ids.end()) {
                    continue;
                }
                if (detection.track_id < 0) {
                    continue;
                }
                auto missing_it = obj.camera_missing_since.find(mapped_camera_id);
                if (missing_it != obj.camera_missing_since.end()) {
                    continue;
                }
                ++visible_cameras;
            }
            info.visible_camera_count = visible_cameras;
            info.total_active_cameras = total_active_cameras;
            if (camera_position) {
                if (obj.world_position_from_stereo) {
                    info.distance_m = calculateDistance(obj.world_position, *camera_position);
                } else {
                    info.distance_m = calculateDistance(obj.world_position, *camera_position);
                }
    // ДОБАВИТЬ для отладки:
    //cout << "[DEBUG] Cam=" << camera_id 
    //     << " GID=" << global_id
    //     << " obj_pos=[" << obj.world_position.x << "," << obj.world_position.y << "," << obj.world_position.z << "]"
    //     << " cam_pos=[" << camera_position->x << "," << camera_position->y << "," << camera_position->z << "]"
    //     << " dist=" << dist << "m" << endl;

            }
            mapping.push_back(info);
        }
    }

    return mapping;
}

const std::vector<GlobalTracker::GlobalAssignment>& GlobalTracker::get_last_assignments() const {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    last_assignments_snapshot_ = last_assignments_;
    return last_assignments_snapshot_;
}


bool GlobalTracker::applyStereoExtrinsicsFromResults(
    const std::vector<StereoCalibrationResult>& stereo_results,
    const std::function<bool(const std::string&, cv::Mat&, cv::Mat&, cv::Mat&)>& get_stereo_params,
    const std::vector<CameraConfig>& cameras,
    std::map<std::string, CameraCalibration>& calibrations) {
    if (calibrations.empty() || !get_stereo_params) {
        assignStereoMap({});
        return false;
    }

    if (stereo_results.empty()) {
        assignStereoMap({});
        return false;

    }
    struct Edge {
        std::string neighbor;
        cv::Mat R;
        cv::Mat T;
    };

    std::map<std::string, std::vector<Edge>> adjacency;
    std::map<std::string, std::map<std::string, StereoPairCalibration>> stereo_pairs;
    for (const auto& result : stereo_results) {
        if (!result.success) {
            continue;
        }
        cv::Mat R, T, Q;
        if (!get_stereo_params(result.camera_pair, R, T, Q)) {
            continue;
        }
        auto delim = result.camera_pair.find('_');
        if (delim == std::string::npos) {
            continue;
        }

        std::string cam1 = result.camera_pair.substr(0, delim);
        std::string cam2 = result.camera_pair.substr(delim + 1);

        if (calibrations.find(cam1) == calibrations.end() ||
            calibrations.find(cam2) == calibrations.end()) {
            continue;
        }

        cv::Mat R64;
        R.convertTo(R64, CV_64F);
        cv::Mat T64;
        T.convertTo(T64, CV_64F);

        constexpr double kMillimetersToMeters = 0.001;
        T64 *= kMillimetersToMeters;

        StereoPairCalibration forward_pair;
        forward_pair.R = R64.clone();
        forward_pair.T = T64.clone();
        if (!Q.empty()) {
            forward_pair.Q = Q.clone();
            forward_pair.has_Q = true;

        }
        stereo_pairs[cam1][cam2] = forward_pair;

        Edge forward_edge{cam2, R64.clone(), T64.clone()};
        adjacency[cam1].push_back(forward_edge);

        cv::Mat R_inv = R64.t();
        cv::Mat T_inv = -R_inv * T64;
        Edge backward{cam1, R_inv.clone(), T_inv.clone()};
        adjacency[cam2].push_back(backward);

        StereoPairCalibration backward_params;
        backward_params.R = R_inv.clone();
        backward_params.T = T_inv.clone();
        backward_params.has_Q = false;
        stereo_pairs[cam2][cam1] = backward_params;
    }

    if (adjacency.empty()) {
        assignStereoMap(std::move(stereo_pairs));
        return false;
    }

    std::string base_camera;
    if (!calibrations.empty()) {
        base_camera = calibrations.begin()->first;
    }
    if (base_camera.empty()) {
        return false;
    }
    cv::Mat base_rotation = cv::Mat::eye(3, 3, CV_64F);
    cv::Mat base_translation = cv::Mat::zeros(3, 1, CV_64F);

    std::queue<std::string> pending;
    std::set<std::string> visited;
    std::map<std::string, cv::Mat> rotations;
    std::map<std::string, cv::Mat> translations;


    rotations[base_camera] = base_rotation.clone();
    translations[base_camera] = base_translation.clone();

    pending.push(base_camera);
    visited.insert(base_camera);

    while (!pending.empty()) {
        std::string current = pending.front();
        pending.pop();

        auto adj_it = adjacency.find(current);
        if (adj_it == adjacency.end()) {
            continue;
        }

        const cv::Mat& R_current = rotations[current];
        const cv::Mat& t_current = translations[current];

        for (const auto& edge : adj_it->second) {
            if (visited.count(edge.neighbor)) {
                continue;
            }

            cv::Mat R_next = edge.R * R_current;
            cv::Mat t_next = edge.R * t_current + edge.T;

            rotations[edge.neighbor] = R_next.clone();
            translations[edge.neighbor] = t_next.clone();
            visited.insert(edge.neighbor);
            pending.push(edge.neighbor);
        }
    }
    bool updated = false;
    std::vector<std::string> calibration_logs;
    for (auto& [id, calib] : calibrations) {
        auto rot_it = rotations.find(id);
        auto trans_it = translations.find(id);
        if (rot_it == rotations.end() || trans_it == translations.end()) {
            continue;
        }

        cv::Mat rvec;
        cv::Rodrigues(rot_it->second, rvec);
        rvec.convertTo(rvec, CV_64F);

        cv::Mat tvec;
        trans_it->second.convertTo(tvec, CV_64F);

        calib.rotation_vector = rvec.clone();
        calib.translation_vector = tvec.clone();
        calib.is_calibrated = true;
        updateCalibrationWorldPosition(calib);

        computeHomography(calib);
        if (calib.has_world_position) {
            std::ostringstream oss;
            oss << "calibration_camera_center camera=" << id
                << " world_x=" << std::fixed << std::setprecision(4)
                << calib.world_position.x
                << " world_y=" << calib.world_position.y
                << " world_z=" << calib.world_position.z;
            calibration_logs.push_back(oss.str());
        }
        updated = true;
    }
    for (const auto& line : calibration_logs) {
        logDebugLine(line);
        std::cout << line << std::endl;
    }
    assignStereoMap(std::move(stereo_pairs));
    return updated;
}

bool GlobalTracker::loadCalibrationFromProvider(
    const std::function<bool(const std::string&, cv::Mat&, cv::Mat&)>& get_camera_matrix,
    const std::function<bool(const std::string&, cv::Mat&, cv::Mat&, cv::Mat&)>& get_stereo_params,
    const std::vector<StereoCalibrationResult>& stereo_results,
    SchemeType scheme,
    const std::string& source_name) {
    auto cameras = scheme_manager_->getCameras();
    std::map<std::string, CameraCalibration> loaded;
    bool intrinsics_loaded = false;

    if (get_camera_matrix) {
        for (const auto& cam : cameras) {
            if (cam.status != CameraStatus::ACTIVE) {
                continue;
            }

            cv::Mat K, D;
            if (get_camera_matrix(cam.id, K, D)) {
                CameraCalibration calib;
                calib.camera_matrix = K.clone();
                calib.dist_coeffs = D.clone();
                calib.rotation_vector = cv::Mat::zeros(3, 1, CV_64F);
                calib.translation_vector = cv::Mat::zeros(3, 1, CV_64F);
                calib.homography_matrix = cv::Mat::eye(3, 3, CV_64F);
                calib.is_calibrated = false;
                loaded.emplace(cam.id, std::move(calib));
                intrinsics_loaded = true;
            }
        }
    }
    bool extrinsics_loaded = false;
    if (get_stereo_params) {
        extrinsics_loaded = applyStereoExtrinsicsFromResults(stereo_results, get_stereo_params, cameras, loaded);
    }

    for (const auto& cam : cameras) {
        if (cam.status != CameraStatus::ACTIVE) {
            continue;
        }

        auto it = loaded.find(cam.id);
        if (it == loaded.end()) {
            CameraCalibration calib;
            setupFallbackCameraIntrinsics(cam, calib);
            bool extrinsics_ok = applyFallbackExtrinsics(cam, scheme, calib);
            if (!extrinsics_ok || calib.rotation_vector.empty() || calib.translation_vector.empty()) {
                std::cerr << "Не удалось вычислить внешние параметры для камеры "
                          << cam.id << " (" << scheme_manager_->roleToString(cam.role)
                          << ")." << std::endl;
                continue;
            }
            calib.is_calibrated = true;
            computeHomography(calib);
            loaded[cam.id] = std::move(calib);
        } else if (!it->second.is_calibrated) {
            bool extrinsics_ok = applyFallbackExtrinsics(cam, scheme, it->second);
            if (!extrinsics_ok || it->second.rotation_vector.empty() || it->second.translation_vector.empty()) {
                std::cerr << "Не удалось обновить внешние параметры для камеры "
                          << cam.id << " (" << scheme_manager_->roleToString(cam.role)
                          << ")." << std::endl;
                continue;
            }
            it->second.is_calibrated = true;
            computeHomography(it->second);
        }
    }
    bool has_calibration = false;
    std::vector<std::string> calibration_logs;
    {
        std::lock_guard<std::recursive_mutex> lock(mutex_);
        camera_calibrations_ = std::move(loaded);
        has_calibration = !camera_calibrations_.empty();
        if (has_calibration && (intrinsics_loaded || extrinsics_loaded) && !source_name.empty()) {
            std::cout << "Загружены калибровочные данные из " << source_name << std::endl;
        }
        if (has_calibration) {
            for (const auto& [cam_id, calib] : camera_calibrations_) {
                if (calib.has_world_position) {
                    std::ostringstream oss;
                    oss << "calibration_world_position camera=" << cam_id
                        << " world_x=" << std::fixed << std::setprecision(4)
                        << calib.world_position.x
                        << " world_y=" << calib.world_position.y
                        << " world_z=" << calib.world_position.z;
                    calibration_logs.push_back(oss.str());
                } else {
                    std::ostringstream oss;
                    oss << "calibration_world_position camera=" << cam_id
                        << " world_position_unavailable=1";
                    calibration_logs.push_back(oss.str());
                }

            }
        }
    }

    for (const auto& line : calibration_logs) {
        logDebugLine(line);
        std::cout << line << std::endl;
    }
    return has_calibration;
}

bool GlobalTracker::loadCalibrationFromDirectory(const std::filesystem::path& directory, SchemeType scheme) {
    DirectoryCalibrationSource source(directory);
    if (!source.load()) {
        return false;
    }

    auto get_camera_matrix = [&source](const std::string& camera_id, cv::Mat& camera_matrix, cv::Mat& dist_coeffs) {
        return source.getCameraMatrix(camera_id, camera_matrix, dist_coeffs);
    };
    auto get_stereo_params = [&source](const std::string& camera_pair, cv::Mat& R, cv::Mat& T, cv::Mat& Q) {
        return source.getStereoParams(camera_pair, R, T, Q);
    };
    std::string source_label = "директории " + directory.string();
    return loadCalibrationFromProvider(get_camera_matrix, get_stereo_params, source.stereo_results(), scheme,
                                       source_label);
}

bool GlobalTracker::loadExternalCalibration(SchemeType scheme) {
    if (external_calibration_dir_.empty()) {
        return false;
    }

    std::error_code ec;
    auto json_path = external_calibration_dir_ / "calibration_results.json";
//    auto json_path = exe_dir / "calibration/results/calibration_results.json";
    if (!std::filesystem::exists(json_path, ec) || ec) {
        std::cerr << "Файл " << json_path
                  << " не найден или недоступен, используем резервную калибровку" << std::endl;
        return false;
    }

    return loadCalibrationFromDirectory(external_calibration_dir_, scheme);
}
