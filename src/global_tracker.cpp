#include "global_tracker.h"
#include "calibration_watcher.h"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <iostream>
#include <limits>
#include <queue>
#include <set>

GlobalTracker::GlobalTracker(CameraSchemeManager* scheme_manager,
                             CameraManager* camera_manager)
    : scheme_manager_(scheme_manager), camera_manager_(camera_manager),
      next_global_id_(1) {

    // Parameter validation
    if (!scheme_manager_) {
        throw std::invalid_argument("scheme_manager cannot be null");
    }

    auto config = scheme_manager_->getTrackingConfig();
    
    // Validate configuration parameters
    distance_threshold_ = std::max(0.1, config.max_distance_threshold);
    speed_threshold_ = std::max(0.1, config.max_object_speed_mps);
    retention_time_ms_ = static_cast<uint64_t>(std::max(1.0, config.id_retention_seconds) * 1000);
    area_change_threshold_ = std::max(0.1, config.max_area_ratio_change);
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
}

bool GlobalTracker::initialize() {
    std::cout << "Инициализация глобального трекера..." << std::endl;

    {
        std::lock_guard<std::recursive_mutex> lock(mutex_);
        camera_calibrations_.clear();
    }

    if (calibration_watcher_) {
        calibration_watcher_->loadResults();
        updateCalibrationTimestamp(*calibration_watcher_);

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
        case SchemeType::SPHERE_ZOOM:
            return calibrateSphereSetup();
        case SchemeType::HEMISPHERE_ZOOM:
            return calibrateHemisphereSetup();
        default:
            std::cerr << "Неподдерживаемая схема для автокалибровки" << std::endl;
            return false;
    }
}

bool GlobalTracker::calibrateSphereSetup() {
    std::cout << "Калибровка сферической установки..." << std::endl;

    if (calibration_watcher_) {
        if (loadCalibrationFromWatcher(*calibration_watcher_, SchemeType::SPHERE_ZOOM)) {
            return validateCalibration();
        }
        std::cout << "Данные калибровки из CalibrationWatcher недоступны, используем резервные значения." << std::endl;
    }

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

        if (cam.role == CameraRole::PRIMARY_WIDE) {
            setupFallbackSphereExtrinsics(cam, calib, 0.0, 180.0);
        } else if (cam.role == CameraRole::SECONDARY_WIDE) {
            setupFallbackSphereExtrinsics(cam, calib, 180.0, 180.0);
        } else if (cam.role == CameraRole::ZOOM) {
            setupFallbackZoomExtrinsics(cam, calib);
        }

        calib.is_calibrated = true;
        computeHomography(calib);
        camera_calibrations_[cam.id] = calib;

        std::cout << "Калибровка камеры " << cam.id << " завершена" << std::endl;
    }

    return validateCalibration();
}

bool GlobalTracker::calibrateHemisphereSetup() {
    std::cout << "Калибровка полусферической установки..." << std::endl;

  if (calibration_watcher_) {
        if (loadCalibrationFromWatcher(*calibration_watcher_, SchemeType::HEMISPHERE_ZOOM)) {
            return validateCalibration();
        }
        std::cout << "Данные калибровки из CalibrationWatcher недоступны, используем резервные значения." << std::endl;
    }

    return calibrateHemisphereFallback();
}

bool GlobalTracker::calibrateHemisphereFallback() {

    auto cameras = scheme_manager_->getCameras();
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    for (const auto& cam : cameras) {
        if (cam.status != CameraStatus::ACTIVE) continue;

        CameraCalibration calib;
        setupFallbackCameraIntrinsics(cam, calib);

        if (cam.role == CameraRole::PRIMARY_WIDE) {
            setupFallbackHemisphereExtrinsics(cam, calib, 0.0, 180.0);
        } else if (cam.role == CameraRole::SECONDARY_WIDE) {
            setupFallbackHemisphereExtrinsics(cam, calib, 30.0, 120.0);
        } else if (cam.role == CameraRole::ZOOM) {
            setupFallbackZoomExtrinsics(cam, calib);
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

    if (cam.role == CameraRole::ZOOM) {
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
    cv::Point3f cam_position(
        radius * cos(angle_rad),
        radius * sin(angle_rad), 
        cam.position.z
    );
    
    // Вектор поворота (камера смотрит к центру)
    cv::Vec3d rvec(0, 0, angle_rad + CV_PI); // Поворот + 180°
    calib.rotation_vector = cv::Mat(rvec);
    
    // Вектор перемещения
    calib.translation_vector = (cv::Mat_<double>(3, 1) << 
        cam_position.x, cam_position.y, cam_position.z);
    
    std::cout << "Внешние параметры сферы для " << cam.id 
              << " (угол=" << start_angle << "°)" << std::endl;
}

void GlobalTracker::setupFallbackHemisphereExtrinsics(const CameraConfig& cam, CameraCalibration& calib,
                                                      double center_angle, double coverage_angle) {
    // Позиционирование для полусферической схемы
    double angle_rad = center_angle * CV_PI / 180.0;
    
    double radius = 0.4; // 40 см от центра
    cv::Point3f cam_position(
        radius * cos(angle_rad),
        radius * sin(angle_rad),
        cam.position.z
    );
    
    cv::Vec3d rvec(0, 0, angle_rad + CV_PI);
    calib.rotation_vector = cv::Mat(rvec);
    
    calib.translation_vector = (cv::Mat_<double>(3, 1) << 
        cam_position.x, cam_position.y, cam_position.z);
    
    std::cout << "Внешние параметры полусферы для " << cam.id 
              << " (центр=" << center_angle << "°, покрытие=" << coverage_angle << "°)" << std::endl;
}

void GlobalTracker::setupFallbackZoomExtrinsics(const CameraConfig& cam, CameraCalibration& calib) {
    // Зум камера - центральное положение с возможностью поворота
    cv::Vec3d rvec(0, 0, 0); // Начальное положение - прямо
    calib.rotation_vector = cv::Mat(rvec);

    calib.translation_vector = (cv::Mat_<double>(3, 1) <<
        0.0, 0.0, cam.position.z);

    std::cout << "Внешние параметры зума для " << cam.id << std::endl;
}

void GlobalTracker::applyFallbackExtrinsics(const CameraConfig& cam, SchemeType scheme, CameraCalibration& calib) {
    if (cam.role == CameraRole::ZOOM) {
        setupFallbackZoomExtrinsics(cam, calib);
        return;
    }

    switch (scheme) {
        case SchemeType::SPHERE_ZOOM:
            if (cam.role == CameraRole::SECONDARY_WIDE) {
                setupFallbackSphereExtrinsics(cam, calib, 180.0, 180.0);
            } else {
                setupFallbackSphereExtrinsics(cam, calib, 0.0, 180.0);
            }
            break;
        default:
            if (cam.role == CameraRole::SECONDARY_WIDE) {
                setupFallbackHemisphereExtrinsics(cam, calib, 30.0, 120.0);
            } else {
                setupFallbackHemisphereExtrinsics(cam, calib, 0.0, 180.0);
            }
            break;
    }
}

bool GlobalTracker::applyWatcherStereoExtrinsics(const CalibrationWatcher& watcher,
                                                const std::vector<CameraConfig>& cameras,
                                                std::map<std::string, CameraCalibration>& calibrations) {
    if (calibrations.empty()) {
        return false;
    }

    auto stereo_results = watcher.getStereoResults();
    if (stereo_results.empty()) {
        return false;
    }

    std::map<std::string, CameraConfig> config_map;
    for (const auto& cam : cameras) {
        config_map[cam.id] = cam;
    }

    struct Edge {
        std::string neighbor;
        cv::Mat R;
        cv::Mat T;
    };

    std::map<std::string, std::vector<Edge>> adjacency;

    for (const auto& result : stereo_results) {
        if (!result.success) {
            continue;
        }

        cv::Mat R, T, Q;
        if (!watcher.getStereoParams(result.camera_pair, R, T, Q)) {
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

        Edge forward{cam2, R.clone(), T.clone()};
        adjacency[cam1].push_back(forward);

        cv::Mat R_inv = R.t();
        cv::Mat T_inv = -R_inv * T;
        Edge backward{cam1, R_inv.clone(), T_inv.clone()};
        adjacency[cam2].push_back(backward);
    }

    if (adjacency.empty()) {
        return false;
    }

    std::string base_camera;
    if (!calibrations.empty()) {
        base_camera = calibrations.begin()->first;
    }

    if (base_camera.empty()) {
        return false;
    }

    cv::Mat base_translation = cv::Mat::zeros(3, 1, CV_64F);
    auto cfg_it = config_map.find(base_camera);
    if (cfg_it != config_map.end()) {
        base_translation = (cv::Mat_<double>(3, 1) <<
            cfg_it->second.position.x,
            cfg_it->second.position.y,
            cfg_it->second.position.z);
    }

    std::queue<std::string> pending;
    std::set<std::string> visited;
    std::map<std::string, cv::Mat> rotations;
    std::map<std::string, cv::Mat> translations;

    rotations[base_camera] = cv::Mat::eye(3, 3, CV_64F);
    translations[base_camera] = base_translation;
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
    for (auto& [id, calib] : calibrations) {
        auto rot_it = rotations.find(id);
        auto trans_it = translations.find(id);
        if (rot_it == rotations.end() || trans_it == translations.end()) {
            continue;
        }

        cv::Mat rvec;
        cv::Rodrigues(rot_it->second, rvec);
        calib.rotation_vector = rvec.clone();
        calib.translation_vector = trans_it->second.clone();
        calib.is_calibrated = true;
        computeHomography(calib);
        updated = true;
    }

    return updated;
}

bool GlobalTracker::loadCalibrationFromWatcher(const CalibrationWatcher& watcher, SchemeType scheme) {
    auto cameras = scheme_manager_->getCameras();
    std::map<std::string, CameraCalibration> loaded;
    bool watcher_intrinsics = false;

    for (const auto& cam : cameras) {
        if (cam.status != CameraStatus::ACTIVE) {
            continue;
        }

        cv::Mat K, D;
        if (watcher.getCameraMatrix(cam.id, K, D)) {
            CameraCalibration calib;
            calib.camera_matrix = K.clone();
            calib.dist_coeffs = D.clone();
            calib.rotation_vector = cv::Mat::zeros(3, 1, CV_64F);
            calib.translation_vector = cv::Mat::zeros(3, 1, CV_64F);
            calib.homography_matrix = cv::Mat::eye(3, 3, CV_64F);
            calib.is_calibrated = false;
            loaded.emplace(cam.id, std::move(calib));
            watcher_intrinsics = true;
        }
    }

    bool watcher_extrinsics = applyWatcherStereoExtrinsics(watcher, cameras, loaded);

    for (const auto& cam : cameras) {
        if (cam.status != CameraStatus::ACTIVE) {
            continue;
        }

        auto it = loaded.find(cam.id);
        if (it == loaded.end()) {
            CameraCalibration calib;
            setupFallbackCameraIntrinsics(cam, calib);
            applyFallbackExtrinsics(cam, scheme, calib);
            calib.is_calibrated = true;
            computeHomography(calib);
            loaded[cam.id] = std::move(calib);
        } else if (!it->second.is_calibrated) {
            applyFallbackExtrinsics(cam, scheme, it->second);
            it->second.is_calibrated = true;
            computeHomography(it->second);
        }
    }

    bool has_calibration = false;
    {
        std::lock_guard<std::recursive_mutex> lock(mutex_);
        camera_calibrations_ = std::move(loaded);
        has_calibration = !camera_calibrations_.empty();
        if (has_calibration && (watcher_intrinsics || watcher_extrinsics)) {
            std::cout << "Загружены калибровочные данные из CalibrationWatcher" << std::endl;
        }
    }
    return has_calibration;
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
                                   const std::vector<cv::Rect>& detections,
                                   uint64_t timestamp) {
    checkAndUpdateCalibration();
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    // Очищаем старые объекты
    cleanupOldObjects(timestamp);

    // Ассоциируем новые детекции с существующими объектами
    associateDetections(camera_id, detections, timestamp);

    // Обновляем позиции объектов в мировых координатах
    updateWorldPositions(camera_id, timestamp);
}

void GlobalTracker::associateDetections(const std::string& camera_id,
                                       const std::vector<cv::Rect>& detections,
                                       uint64_t timestamp) {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    if (detections.empty()) {
        // уменьшение уверенности для всех объектов, если на камере нет детекций
        for (auto& pair : tracked_objects_) {
            pair.second.confidence = std::max(0.0, pair.second.confidence - 0.05);
            pair.second.camera_detections.erase(camera_id);
        }
        return;
    }

    // Подготовка данных: мировые координаты центров детекций
    std::vector<cv::Point3f> detection_world;
    for (const auto& det : detections) {
        cv::Point2f center(det.x + det.width / 2.0f,
                           det.y + det.height / 2.0f);
        detection_world.push_back(imageToWorld(camera_id, center));
    }

    // Список существующих объектов
    std::vector<int> object_ids;
    std::vector<cv::Point3f> predicted_positions;

    for (auto& pair : tracked_objects_) {

        object_ids.push_back(pair.first);
        predicted_positions.push_back(predictPosition(pair.second, timestamp));
    }

    // Если нет активных объектов - создаем новые для всех детекций
    if (object_ids.empty()) {
        for (const auto& det : detections) {
            createNewObject(camera_id, det, timestamp);
        }
        return;
    }

    // Формируем матрицу стоимостей на основе расстояний в мировых координатах
    std::vector<std::vector<double>> cost(predicted_positions.size(),
                                          std::vector<double>(detection_world.size(), 0.0));
    for (size_t i = 0; i < predicted_positions.size(); ++i) {
        for (size_t j = 0; j < detection_world.size(); ++j) {
            cost[i][j] = cv::norm(predicted_positions[i] - detection_world[j]);


            auto prev_it = tracked_objects_[object_ids[i]].camera_detections.find(camera_id);
            if (prev_it != tracked_objects_[object_ids[i]].camera_detections.end()) {
                double prev_area = static_cast<double>(prev_it->second.width) * prev_it->second.height;
                double new_area = static_cast<double>(detections[j].width) * detections[j].height;
                if (prev_area > 0.0) {
                    double area_ratio = std::abs(new_area - prev_area) / prev_area;
                    if (area_ratio > area_change_threshold_) {
                        cost[i][j] = distance_threshold_ * 10.0;
                    }
                }
            }
        }
    }

    // Получаем соответствия с помощью алгоритма Венгера
    std::vector<int> assignment = hungarianMatch(cost);

    std::vector<bool> detection_used(detections.size(), false);
    // Применяем сопоставление
    for (size_t i = 0; i < object_ids.size(); ++i) {
        int obj_id = object_ids[i];
        GlobalObject& obj = tracked_objects_[obj_id];
        int j = assignment[i];
        if (j >= 0 && j < static_cast<int>(detections.size()) &&
            cost[i][j] < distance_threshold_) {
            // успешное сопоставление
            obj.camera_detections[camera_id] = detections[j];
            obj.confidence = std::min(1.0, obj.confidence + 0.1);
            detection_used[j] = true;

            if (obj.primary_camera_id.empty() ||
                getPrimaryCameraPriority(camera_id) > getPrimaryCameraPriority(obj.primary_camera_id)) {
                obj.primary_camera_id = camera_id;
            }
        } else {
            obj.confidence = std::max(0.0, obj.confidence - 0.05);
            obj.camera_detections.erase(camera_id);
        }
    }
    // Создаем новые объекты для неиспользованных детекций
    for (size_t i = 0; i < detections.size(); ++i) {
        if (!detection_used[i]) {
            createNewObject(camera_id, detections[i], timestamp);
        }
    }
}

cv::Point3f GlobalTracker::predictPosition(const GlobalObject& obj, uint64_t timestamp) {
    // Простое линейное предсказание на основе скорости
    double dt = (timestamp - obj.last_seen_timestamp) / 1000.0; // в секундах
    
    cv::Point3f predicted = obj.world_position + obj.velocity * dt;
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


void GlobalTracker::createNewObject(const std::string& camera_id,
                                   const cv::Rect& detection,
                                   uint64_t timestamp) {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    GlobalObject new_obj;
    
    // Prevent integer overflow in global ID
    if (next_global_id_ >= std::numeric_limits<int>::max() - 1000) {
        next_global_id_ = 1;
    }
    new_obj.global_id = next_global_id_++;
    
    new_obj.camera_detections[camera_id] = detection;
    new_obj.primary_camera_id = camera_id;
    new_obj.last_seen_timestamp = timestamp;
    new_obj.confidence = 0.5; // Начальная уверенность
    new_obj.velocity = cv::Point3f(0, 0, 0); // Начальная скорость
    
    // Validate detection bounds
    if (detection.width <= 0 || detection.height <= 0) {
        std::cerr << "Invalid detection bounds for camera " << camera_id << std::endl;
        return;
    }
    
    // Вычисляем начальную мировую позицию
    cv::Point2f detection_center(
        detection.x + detection.width * 0.5f,
        detection.y + detection.height * 0.5f
    );
    new_obj.world_position = imageToWorld(camera_id, detection_center);
    new_obj.position_history.reserve(20); // Pre-allocate for efficiency
    new_obj.position_history.push_back(new_obj.world_position);
    
    tracked_objects_[new_obj.global_id] = std::move(new_obj); // Use move for efficiency
    std::cout << "Создан новый объект ID=" << new_obj.global_id
              << " на камере " << camera_id << std::endl;
}

void GlobalTracker::updateWorldPositions(const std::string& camera_id, uint64_t timestamp) {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    for (auto& pair : tracked_objects_) {
        GlobalObject& obj = pair.second;

        // Обновляем позицию только если объект виден на этой камере
        auto det_it = obj.camera_detections.find(camera_id);
        if (det_it != obj.camera_detections.end()) {
            cv::Point2f detection_center(
                det_it->second.x + det_it->second.width / 2.0,
                det_it->second.y + det_it->second.height / 2.0
            );
            
            cv::Point3f new_world_pos = imageToWorld(camera_id, detection_center);
            
            // Обновляем скорость
            if (obj.last_seen_timestamp > 0) {
                double dt = (timestamp - obj.last_seen_timestamp) / 1000.0;
                if (dt > 0) {
                    obj.velocity = (new_world_pos - obj.world_position) / dt;
                }
            }
            obj.world_position = new_world_pos;
            obj.position_history.push_back(new_world_pos);
            if (obj.position_history.size() > 20) {
                obj.position_history.erase(obj.position_history.begin());
            }
            obj.last_seen_timestamp = timestamp;
        }
    }
}

int GlobalTracker::getPrimaryCameraPriority(const std::string& camera_id) {
    CameraConfig* cam = scheme_manager_->getCamera(camera_id);
    if (cam) {
        return cam->priority;
    }
    return 0;
}

cv::Point3f GlobalTracker::imageToWorld(const std::string& camera_id, const cv::Point2f& image_point) {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    
    // Validate input parameters
    if (camera_id.empty()) {
        std::cerr << "Invalid camera ID" << std::endl;
        return cv::Point3f(0, 0, 0);
    }
    
    auto calib_it = camera_calibrations_.find(camera_id);
    if (calib_it == camera_calibrations_.end() || !calib_it->second.is_calibrated) {
        std::cerr << "Камера " << camera_id << " не откалибрована" << std::endl;
        return cv::Point3f(0, 0, 0);
    }
    
    const CameraCalibration& calib = calib_it->second;
    
    // Validate calibration matrices
    if (calib.camera_matrix.empty() || calib.rotation_vector.empty() || 
        calib.translation_vector.empty()) {
        std::cerr << "Некорректные калибровочные данные для камеры " << camera_id << std::endl;
        return cv::Point3f(0, 0, 0);
    }
    
    try {
        // Предполагаем, что объект находится на уровне земли (z = 0)
        std::vector<cv::Point2f> image_points = {image_point};
        std::vector<cv::Point2f> undistorted_points;
        
        // Создаем луч от камеры через точку изображения
        cv::undistortPoints(image_points, undistorted_points, 
                           calib.camera_matrix, calib.dist_coeffs);
        
        if (undistorted_points.empty()) {
            return cv::Point3f(0, 0, 0);
        }
        
        // Преобразуем в однородные координаты
        cv::Point3f ray(undistorted_points[0].x, undistorted_points[0].y, 1.0);
        
        // Применяем обратное преобразование поворота
        cv::Mat R;
        cv::Rodrigues(calib.rotation_vector, R);
        cv::Mat R_inv = R.t();
        
        // Позиция камеры в мировых координатах
        cv::Mat cam_position = -R_inv * calib.translation_vector;
        
        // Направление луча в мировых координатах
        cv::Mat ray_world = R_inv * cv::Mat(ray);
        
        // Пересечение луча с плоскостью z = 0
        double ray_z = ray_world.at<double>(2);
        if (std::abs(ray_z) < 1e-6) {
            // Ray is parallel to ground plane
            return cv::Point3f(0, 0, 0);
        }
        
        double t = -cam_position.at<double>(2) / ray_z;
        
        cv::Point3f world_point(
            static_cast<float>(cam_position.at<double>(0) + t * ray_world.at<double>(0)),
            static_cast<float>(cam_position.at<double>(1) + t * ray_world.at<double>(1)),
            0.0f
        );
        
        return world_point;
    } catch (const cv::Exception& e) {
        std::cerr << "OpenCV error in imageToWorld: " << e.what() << std::endl;
        return cv::Point3f(0, 0, 0);
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
    applyFallbackExtrinsics(*cam, scheme, calib);
    calib.is_calibrated = true;
    computeHomography(calib);

    {
        std::lock_guard<std::recursive_mutex> lock(mutex_);
        camera_calibrations_[camera_id] = std::move(calib);
    }
    return true;
}

void GlobalTracker::cleanupOldObjects(uint64_t current_timestamp) {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    auto it = tracked_objects_.begin();
    while (it != tracked_objects_.end()) {
        uint64_t age = current_timestamp - it->second.last_seen_timestamp;

        if (age > retention_time_ms_ || it->second.confidence < 0.1) {
            std::cout << "Удален объект ID=" << it->second.global_id 
                      << " (возраст=" << age << "мс, уверенность=" 
                      << it->second.confidence << ")" << std::endl;
            it = tracked_objects_.erase(it);
        } else {
            ++it;
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