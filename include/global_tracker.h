#ifndef GLOBAL_TRACKER_H
#define GLOBAL_TRACKER_H

#include "camera_scheme.h"
#include "camera_manager.h"
#include <opencv2/opencv.hpp>
#include <map>
#include <vector>
#include <array>
#include <optional>
#include <unordered_map>
#include <filesystem>
#include <functional>
#include <mutex>
#include <limits>
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <utility>


class CalibrationWatcher; // forward declaration
class GlobalTrackerTestHelper;
struct StereoCalibrationResult;

struct DetectionDescriptor {
    bool has_color = false;
    std::array<float, 3> color = {0.0f, 0.0f, 0.0f};
    bool has_grayscale = false;
    float grayscale_intensity = 0.0f;
    std::array<float, 4> texture = {0.0f, 0.0f, 0.0f, 0.0f};
};

struct LocalDetection {
    cv::Rect box;
    int track_id = -1;
    DetectionDescriptor descriptor;
};


// Структура для объекта в глобальной системе координат
struct GlobalObject {
    int global_id;
    cv::Point3f world_position;
    bool world_position_from_stereo = false;
    cv::Point3f velocity;
    cv::Point3f residual;
    float residual_magnitude = 0.0f;
    float residual_speed = 0.0f;
    bool residual_position_threshold_exceeded = false;
    bool residual_speed_threshold_exceeded = false;
    double confidence;
    uint64_t last_seen_timestamp;
    std::map<std::string, LocalDetection> camera_detections;
    std::map<std::string, double> camera_reid_scores;
    std::map<std::string, uint64_t> camera_last_seen;
    std::map<std::string, uint64_t> camera_missing_since;
    std::map<std::string, int> camera_miss_counts;
    std::string primary_camera_id;
    std::vector<cv::Point3f> position_history;
    DetectionDescriptor last_descriptor;
    bool has_last_descriptor = false;
};

// Структура для калибровочных данных камеры
struct CameraCalibration {
    cv::Mat camera_matrix;
    cv::Mat dist_coeffs;
    cv::Mat rotation_vector;
    cv::Mat translation_vector;
    cv::Mat homography_matrix;
    bool is_calibrated;
    cv::Point3f world_position = cv::Point3f(0.0f, 0.0f, 0.0f);
    bool has_world_position = false;
};


struct StereoPairCalibration {
    cv::Mat R;
    cv::Mat T;
    cv::Mat Q;
    bool has_Q = false;
};

struct TrackerStatistics {
public:
    void update(double active_objects,
                double miss_ratio,
                std::optional<double> match_distance);

    double activeAverage() const { return active_average_; }
    double missAverage() const { return miss_average_; }
    std::optional<double> distanceAverage() const {
        if (!distance_valid_) {
            return std::nullopt;
        }
        return match_distance_average_;
    }
    bool initialized() const { return initialized_; }

private:
    double active_average_ = 0.0;
    double miss_average_ = 0.0;
    double match_distance_average_ = 0.0;
    bool distance_valid_ = false;
    bool initialized_ = false;
};

struct ResidualStatisticsSnapshot {
    double total_residual_magnitude = 0.0;
    double total_residual_speed = 0.0;
    double max_residual_magnitude = 0.0;
    double max_residual_speed = 0.0;
    size_t sample_count = 0;
    size_t residual_threshold_exceeded = 0;
    size_t speed_threshold_exceeded = 0;
    size_t ground_projection_ray_parallel_skipped = 0;
    size_t ground_projection_ray_parallel_reused = 0;
    size_t ground_projection_ray_parallel_nudged = 0;
    bool calibration_suspect = false;
    bool timestamp_delay_suspect = false;
};

struct ImageToWorldResult {
    cv::Point3f world_point = cv::Point3f(0.0f, 0.0f, 0.0f);
    cv::Point2f sampled_image_point = cv::Point2f(0.0f, 0.0f);
    bool valid = false;
    bool reused_previous_world = false;
    bool nudged_ray = false;
    bool ray_parallel = false;
};

class GlobalTracker {
public:
    struct GlobalAssignment {
        std::string cam_id;
        int local_id = -1;
        int global_id = -1;
    };
private:
    friend class GlobalTrackerTestHelper;
    struct PendingReidEntry {
        GlobalObject object;
        uint64_t stored_timestamp = 0;
    };
    std::map<int, GlobalObject> tracked_objects_;
    std::map<std::string, CameraCalibration> camera_calibrations_;
    std::map<std::string, std::map<std::string, StereoPairCalibration>> stereo_calibrations_;
    CameraSchemeManager* scheme_manager_;
    CameraManager* camera_manager_;
    int next_global_id_;
    double distance_threshold_;
    double speed_threshold_;
    uint64_t retention_time_ms_;
    double area_change_threshold_;
    double base_distance_threshold_;
    double min_distance_threshold_;
    double max_distance_threshold_;
    double base_speed_threshold_;
    double min_speed_threshold_;
    double max_speed_threshold_;
    double base_area_change_threshold_;
    double min_area_change_threshold_;
    double max_area_change_threshold_;
    double last_logged_distance_threshold_ = std::numeric_limits<double>::quiet_NaN();
    double last_logged_speed_threshold_ = std::numeric_limits<double>::quiet_NaN();
    double last_logged_area_change_threshold_ = std::numeric_limits<double>::quiet_NaN();
    TrackerStatistics tracker_stats_;
    ResidualStatisticsSnapshot residual_stats_{};
    CalibrationWatcher* calibration_watcher_ = nullptr;
    std::filesystem::path external_calibration_dir_{};
    std::filesystem::file_time_type last_calibration_write_time_{};
    bool has_last_calibration_write_time_ = false;
    mutable std::recursive_mutex mutex_;
    double residual_position_threshold_ = 0.0;
    double residual_speed_threshold_ = 0.0;
    size_t residual_summary_log_interval_ = 0;
    size_t residual_last_logged_sample_ = 0;
    std::function<void(const ResidualStatisticsSnapshot&)> residual_metrics_callback_;
    std::unordered_map<int, PendingReidEntry> pending_reid_;
    uint64_t pending_reid_retention_ms_ = 0;
    double pending_reid_descriptor_threshold_ = 0.0;
    std::vector<GlobalAssignment> last_assignments_;
    mutable std::vector<GlobalAssignment> last_assignments_snapshot_;

public:
    GlobalTracker(CameraSchemeManager* scheme_manager,
                  CameraManager* camera_manager = nullptr);
    ~GlobalTracker();

    void setDebugLogPath(const std::string& path);

     // Установка CalibrationWatcher для автоматической загрузки калибровки
    void setCalibrationWatcher(CalibrationWatcher* watcher) { calibration_watcher_ = watcher; }
    void setExternalCalibrationDirectory(const std::filesystem::path& path);

    // Проверка и обновление калибровки
    bool checkAndUpdateCalibration();

    // Инициализация и калибровка
    bool initialize();
    bool calibrateCamera(const std::string& camera_id,
                        const std::vector<cv::Point3f>& world_points,
                        const std::vector<cv::Point2f>& image_points);


    // Reload camera calibration matrices from external watcher
    bool reloadCalibration(const CalibrationWatcher& watcher);

    // Трекинг объектов
    void updateDetections(const std::string& camera_id,
                         const std::vector<LocalDetection>& detections,
                         uint64_t timestamp);

    // Координатные преобразования
    ImageToWorldResult imageToWorld(const std::string& camera_id,
                                    const cv::Point2f& image_point,
                                    std::optional<cv::Point3f> previous_world = std::nullopt);
    cv::Point2f worldToImage(const std::string& camera_id, const cv::Point3f& world_point);

   std::optional<cv::Point3f> getCameraWorldPosition(const std::string& camera_id) const;

    void setResidualMetricsCallback(std::function<void(const ResidualStatisticsSnapshot&)> callback);
    ResidualStatisticsSnapshot getResidualStatistics() const;

    // Управление схемами
    void handleSchemeChange(SchemeType new_scheme);
    struct TrackGlobalMapping {
        int track_id = -1;
        int global_id = -1;
        std::optional<double> distance_m;
        int visible_camera_count = 0;
        int total_active_cameras = 0;
    };

    std::vector<GlobalObject> getActiveObjects();
    std::vector<TrackGlobalMapping> getTrackToGlobalMapForCamera(
        const std::string& camera_id,
        const std::unordered_set<std::string>& active_camera_ids);

    const std::vector<GlobalAssignment>& get_last_assignments() const;

private:
    // Вспомогательные методы
    void associateDetections(const std::string& camera_id,
                            const std::vector<LocalDetection>& detections,
                            uint64_t timestamp);

    cv::Point3f predictPosition(GlobalObject& obj, uint64_t timestamp);
protected:
    std::optional<double> passesSpatialGate(const GlobalObject& obj,
                                            const std::string& camera_id,
                                            const cv::Point3f& predicted,
                                            const cv::Point3f& detection,
                                            uint64_t timestamp) const;
private:
    int createNewObject(const std::string& camera_id,
                        const LocalDetection& detection,
                        uint64_t timestamp);
    void updateWorldPositions(const std::string& camera_id, uint64_t timestamp);
    std::vector<int> hungarianMatch(const std::vector<std::vector<double>>& cost);
    int getPrimaryCameraPriority(const std::string& camera_id);
    double calculateDistance(const cv::Point3f& p1, const cv::Point3f& p2);
    void cleanupOldObjects(uint64_t current_timestamp);
    void rebuildAssignmentsLocked();
    void logDebugLine(const std::string& line);
    void flushDebugLog();
    void logCameraDrop(const std::string& camera_id,
                       int global_id,
                       const LocalDetection& detection,
                       double confidence,
                       uint64_t timestamp,
                       const std::string& reason);
    std::optional<double> computeDescriptorDistance(const LocalDetection& a,
                                                    const LocalDetection& b) const;
    std::optional<double> computeDescriptorCost(const GlobalObject& obj,
                                                const LocalDetection& detection,
                                                const std::string& camera_id) const;
    void updateReidScore(GlobalObject& obj,
                         const std::string& camera_id,
                         const LocalDetection& detection);
    int tryRevivePending(const std::string& camera_id,
                         const LocalDetection& detection,
                         const cv::Point3f& detection_world,
                         uint64_t timestamp,
                         double* descriptor_score = nullptr,
                         uint64_t* dormant_ms = nullptr);
    void prunePendingReid(uint64_t current_timestamp);
    void updateCalibrationWorldPosition(CameraCalibration& calib);
    std::optional<cv::Point3f> triangulateStereoPoint(const std::string& reference_camera,
                                                      const cv::Point2f& reference_point,
                                                      const std::string& partner_camera,
                                                      const cv::Point2f& partner_point) const;
    std::optional<cv::Point3f> tryStereoTriangulation(const GlobalObject& obj,
                                                      const std::string& primary_camera,
                                                      const cv::Point2f& preferred_point,
                                                      const cv::Point2f& fallback_point,
                                                      uint64_t timestamp,
                                                      std::string* partner_used = nullptr) const;
    void adjustAdaptiveThresholds(size_t active_objects,
                                  size_t detection_count,
                                  uint64_t timestamp,
                                  const std::string& camera_id);
    // Калибровка
    bool performAutoCalibration();
//    bool calibrateSphereSetup();
//    bool calibrateHemisphereSetup();
    bool calibrateSphereSetup(SchemeType scheme);
    bool calibrateHemisphereSetup(SchemeType scheme);
    bool loadCalibrationFromWatcher(const CalibrationWatcher& watcher, SchemeType scheme);
    bool applyWatcherStereoExtrinsics(const CalibrationWatcher& watcher,
                                      const std::vector<CameraConfig>& cameras,
                                      std::map<std::string, CameraCalibration>& calibrations);

    bool loadExternalCalibration(SchemeType scheme);
    bool loadCalibrationFromDirectory(const std::filesystem::path& directory, SchemeType scheme);
    bool loadCalibrationFromProvider(
        const std::function<bool(const std::string&, cv::Mat&, cv::Mat&)>& get_camera_matrix,
        const std::function<bool(const std::string&, cv::Mat&, cv::Mat&, cv::Mat&)>& get_stereo_params,
        const std::vector<StereoCalibrationResult>& stereo_results,
        SchemeType scheme,
        const std::string& source_name);
    bool applyStereoExtrinsicsFromResults(
        const std::vector<StereoCalibrationResult>& stereo_results,
        const std::function<bool(const std::string&, cv::Mat&, cv::Mat&, cv::Mat&)>& get_stereo_params,
        const std::vector<CameraConfig>& cameras,
        std::map<std::string, CameraCalibration>& calibrations);
    void assignStereoMap(
        std::map<std::string, std::map<std::string, StereoPairCalibration>> stereo_pairs);
    bool applyFallbackExtrinsics(const CameraConfig& cam, SchemeType scheme, CameraCalibration& calib);
    bool calibrateSphereFallback();
    bool calibrateHemisphereFallback();
    void setupFallbackCameraIntrinsics(const CameraConfig&, CameraCalibration&);
    void setupFallbackSphereExtrinsics(const CameraConfig&, CameraCalibration&, double, double);
    void setupFallbackHemisphereExtrinsics(const CameraConfig&, CameraCalibration&, double, double);
    void setupFallbackZoomExtrinsics(const CameraConfig&, CameraCalibration&);
    bool validateCalibration();
    void computeHomography(CameraCalibration&);
    bool initializeCameraCalibration(const std::string&);
    bool updateCalibrationTimestamp(const CalibrationWatcher& watcher);


    std::string debug_log_path_;
    std::ofstream debug_log_stream_;
};

#endif
