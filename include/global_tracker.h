#ifndef GLOBAL_TRACKER_H
#define GLOBAL_TRACKER_H

#include "camera_scheme.h"
#include "camera_manager.h"
#include <opencv2/opencv.hpp>
#include <map>
#include <vector>
#include <filesystem>
#include <mutex>
#include <limits>
#include <iostream>

class CalibrationWatcher; // forward declaration

// Структура для объекта в глобальной системе координат
struct GlobalObject {
    int global_id;
    cv::Point3f world_position;
    cv::Point3f velocity;
    double confidence;
    uint64_t last_seen_timestamp;
    std::map<std::string, cv::Rect> camera_detections;
    std::string primary_camera_id;
    std::vector<cv::Point3f> position_history;
};

// Структура для калибровочных данных камеры
struct CameraCalibration {
    cv::Mat camera_matrix;
    cv::Mat dist_coeffs;
    cv::Mat rotation_vector;
    cv::Mat translation_vector;
    cv::Mat homography_matrix;
    bool is_calibrated;
};

class GlobalTracker {
private:
    std::map<int, GlobalObject> tracked_objects_;
    std::map<std::string, CameraCalibration> camera_calibrations_;
    CameraSchemeManager* scheme_manager_;
    CameraManager* camera_manager_;
    int next_global_id_;
    double distance_threshold_;
    double speed_threshold_;
    uint64_t retention_time_ms_;
    double area_change_threshold_;
    CalibrationWatcher* calibration_watcher_ = nullptr;
    std::filesystem::file_time_type last_calibration_write_time_{};
    bool has_last_calibration_write_time_ = false;
    mutable std::recursive_mutex mutex_;

public:
    GlobalTracker(CameraSchemeManager* scheme_manager,
                  CameraManager* camera_manager = nullptr);
    ~GlobalTracker();
    
     // Установка CalibrationWatcher для автоматической загрузки калибровки
    void setCalibrationWatcher(CalibrationWatcher* watcher) { calibration_watcher_ = watcher; }
    
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
                         const std::vector<cv::Rect>& detections,
                         uint64_t timestamp);
    
    // Координатные преобразования
    cv::Point3f imageToWorld(const std::string& camera_id, const cv::Point2f& image_point);
    cv::Point2f worldToImage(const std::string& camera_id, const cv::Point3f& world_point);
    
    // Управление схемами
    void handleSchemeChange(SchemeType new_scheme);
    std::vector<GlobalObject> getActiveObjects();
    
private:
    // Вспомогательные методы
    void associateDetections(const std::string& camera_id,
                           const std::vector<cv::Rect>& detections,
                           uint64_t timestamp);
    cv::Point3f predictPosition(const GlobalObject& obj, uint64_t timestamp);
    void createNewObject(const std::string& camera_id,
                         const cv::Rect& detection,
                         uint64_t timestamp);
    void updateWorldPositions(const std::string& camera_id, uint64_t timestamp);
    std::vector<int> hungarianMatch(const std::vector<std::vector<double>>& cost);
    int getPrimaryCameraPriority(const std::string& camera_id);
    double calculateDistance(const cv::Point3f& p1, const cv::Point3f& p2);
    void cleanupOldObjects(uint64_t current_timestamp);

    // Калибровка
    bool performAutoCalibration();
    bool calibrateSphereSetup();
    bool calibrateHemisphereSetup();
    bool loadCalibrationFromWatcher(const CalibrationWatcher& watcher, SchemeType scheme);
    bool applyWatcherStereoExtrinsics(const CalibrationWatcher& watcher,
                                      const std::vector<CameraConfig>& cameras,
                                      std::map<std::string, CameraCalibration>& calibrations);
    void applyFallbackExtrinsics(const CameraConfig& cam, SchemeType scheme, CameraCalibration& calib);
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
};

#endif
