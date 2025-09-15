#include "global_tracker.h"
#include "calibration_watcher.h"
#include <algorithm>
#include <chrono>
#include <iostream>
#include <limits>
#include <cmath>


GlobalTracker::GlobalTracker(CameraSchemeManager* scheme_manager,
                             CameraManager* camera_manager)
    : scheme_manager_(scheme_manager), camera_manager_(camera_manager),
      next_global_id_(1) {

    auto config = scheme_manager_->getTrackingConfig();
    distance_threshold_ = config.max_distance_threshold;
    speed_threshold_ = config.max_object_speed_mps;
    retention_time_ms_ = static_cast<uint64_t>(config.id_retention_seconds * 1000);
    area_change_threshold_ = config.max_area_ratio_change;
}

GlobalTracker::~GlobalTracker() {
}

bool GlobalTracker::initialize() {
    std::cout << "Инициализация глобального трекера..." << std::endl;
    
    // Сначала пытаемся загрузить существующую калибровку
    bool has_existing_calibration = false;
    
    // Инициализируем структуры калибровки для всех активных камер
    auto cameras = scheme_manager_->getCameras();
    for (const auto& cam : cameras) {
        if (cam.status == CameraStatus::ACTIVE) {
            if (!initializeCameraCalibration(cam.id)) {
                std::cerr << "Не удалось инициализировать структуру калибровки камеры " << cam.id << std::endl;
                return false;
            }
        }
    }
    
    // Пытаемся загрузить калибровку из CalibrationWatcher
    if (scheme_manager_->isCalibrated()) {
        // Если есть CalibrationWatcher, используем его
        std::cout << "Найдена существующая калибровка системы" << std::endl;
        has_existing_calibration = true;
    } else {
        std::cout << "Калибровка не найдена. Запуск автокалибровки..." << std::endl;
        if (!performAutoCalibration()) {
            std::cerr << "Ошибка автокалибровки" << std::endl;
            return false;
        }
    }
    
    // Проверяем результат
    int calibrated_cameras = 0;
    for (const auto& cam : cameras) {
        if (cam.status == CameraStatus::ACTIVE) {
            auto it = camera_calibrations_.find(cam.id);
            if (it != camera_calibrations_.end() && it->second.is_calibrated) {
                calibrated_cameras++;
                std::cout << "Камера " << cam.id << " откалибрована" << std::endl;
            }
        }
    }
    
    if (calibrated_cameras < 2) {
        std::cerr << "Недостаточно откалиброванных камер: " << calibrated_cameras << std::endl;
        return false;
    }
    
    std::cout << "Глобальный трекер инициализирован для " << calibrated_cameras << " камер" << std::endl;
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
    
    // Для сферической установки используем заранее известные точки
    // В реальной системе эти точки должны быть физически размещены
    std::vector<cv::Point3f> reference_points = {
        cv::Point3f(5.0, 0.0, 0.0),    // Точка справа
        cv::Point3f(-5.0, 0.0, 0.0),   // Точка слева  
        cv::Point3f(0.0, 5.0, 0.0),    // Точка спереди
        cv::Point3f(0.0, -5.0, 0.0),   // Точка сзади
        cv::Point3f(0.0, 0.0, 2.0),    // Точка сверху
        cv::Point3f(3.5, 3.5, 0.0),    // Диагональная точка
        cv::Point3f(-3.5, -3.5, 0.0)   // Противоположная диагональ
    };
    
    // Для каждой камеры вычисляем соответствующие проекции
    auto cameras = scheme_manager_->getCameras();
    for (const auto& cam : cameras) {
        if (cam.status != CameraStatus::ACTIVE) continue;
        
        CameraCalibration calib;
        calib.is_calibrated = false;
        
        // Настройка базовых параметров камеры
        setupCameraIntrinsics(cam, calib);
        
        // Вычисление внешних параметров для сферы
        if (cam.role == CameraRole::PRIMARY_WIDE) {
            // Передняя камера (0° - 180°)
            setupSphereExtrinsics(cam, calib, 0.0, 180.0);
        } else if (cam.role == CameraRole::SECONDARY_WIDE) {
            // Задняя камера (180° - 360°) 
            setupSphereExtrinsics(cam, calib, 180.0, 180.0);
        } else if (cam.role == CameraRole::ZOOM) {
            // Зум камера (центральное положение)
            setupZoomExtrinsics(cam, calib);
        }
        
        calib.is_calibrated = true;
        camera_calibrations_[cam.id] = calib;
        
        std::cout << "Калибровка камеры " << cam.id << " завершена" << std::endl;
    }
    
    return validateCalibration();
}

bool GlobalTracker::calibrateHemisphereSetup() {
    std::cout << "Калибровка полусферической установки..." << std::endl;
    
    // Опорные точки для полусферы (только передняя часть)
    std::vector<cv::Point3f> reference_points = {
        cv::Point3f(4.0, 3.0, 0.0),    // Правая передняя
        cv::Point3f(-4.0, 3.0, 0.0),   // Левая передняя
        cv::Point3f(0.0, 5.0, 0.0),    // Центр спереди
        cv::Point3f(6.0, 0.0, 0.0),    // Правая сторона
        cv::Point3f(-6.0, 0.0, 0.0),   // Левая сторона
        cv::Point3f(0.0, 0.0, 2.5)     // Центр сверху
    };
    
    auto cameras = scheme_manager_->getCameras();
    for (const auto& cam : cameras) {
        if (cam.status != CameraStatus::ACTIVE) continue;
        
        CameraCalibration calib;
        setupCameraIntrinsics(cam, calib);
        
        if (cam.role == CameraRole::PRIMARY_WIDE) {
            // Основная широкоугольная (180°)
            setupHemisphereExtrinsics(cam, calib, 0.0, 180.0);
        } else if (cam.role == CameraRole::SECONDARY_WIDE) {
            // Боковая камера (120°)
            setupHemisphereExtrinsics(cam, calib, 30.0, 120.0);
        } else if (cam.role == CameraRole::ZOOM) {
            setupZoomExtrinsics(cam, calib);
        }
        
        calib.is_calibrated = true;
        camera_calibrations_[cam.id] = calib;
    }
    
    return validateCalibration();
}

void GlobalTracker::setupCameraIntrinsics(const CameraConfig& cam, CameraCalibration& calib) {
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

void GlobalTracker::setupSphereExtrinsics(const CameraConfig& cam, CameraCalibration& calib, 
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

void GlobalTracker::setupHemisphereExtrinsics(const CameraConfig& cam, CameraCalibration& calib,
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

void GlobalTracker::setupZoomExtrinsics(const CameraConfig& cam, CameraCalibration& calib) {
    // Зум камера - центральное положение с возможностью поворота
    cv::Vec3d rvec(0, 0, 0); // Начальное положение - прямо
    calib.rotation_vector = cv::Mat(rvec);
    
    calib.translation_vector = (cv::Mat_<double>(3, 1) << 
        0.0, 0.0, cam.position.z);
    
    std::cout << "Внешние параметры зума для " << cam.id << std::endl;
}

bool GlobalTracker::validateCalibration() {
    std::cout << "Проверка качества калибровки..." << std::endl;
    
    // Проверяем наличие калибровочных данных для всех активных камер
    auto cameras = scheme_manager_->getCameras();
    int calibrated_count = 0;
    
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
    // Проверяем калибровку камеры
    auto calib_it = camera_calibrations_.find(camera_id);
    if (calib_it == camera_calibrations_.end() || !calib_it->second.is_calibrated) {
        std::cerr << "Камера " << camera_id << " не откалибрована, пропускаем детекции" << std::endl;
        return;
    }
    
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

bool GlobalTracker::checkAndUpdateCalibration() {
    if (!calibration_watcher_) {
        return false;
    }

    // Проверяем, есть ли новые результаты калибровки
    auto mono_results = calibration_watcher_->getMonoResults();
    bool updated = false;

    for (const auto& result : mono_results) {
        if (result.success) {
            auto& calib = camera_calibrations_[result.camera_id];
            calib.camera_matrix = result.camera_matrix.clone();
            calib.dist_coeffs = result.dist_coeffs.clone();
            calib.is_calibrated = true;
            
            // Настройка внешних параметров согласно схеме
            CameraConfig* cam = scheme_manager_->getCamera(result.camera_id);
            if (cam) {
                SchemeType scheme = scheme_manager_->getCurrentScheme();
                switch (scheme) {
                    case SchemeType::SPHERE_ZOOM:
                        if (cam->role == CameraRole::PRIMARY_WIDE) {
                            setupSphereExtrinsics(*cam, calib, 0.0, 180.0);
                        } else if (cam->role == CameraRole::SECONDARY_WIDE) {
                            setupSphereExtrinsics(*cam, calib, 180.0, 180.0);
                        } else if (cam->role == CameraRole::ZOOM) {
                            setupZoomExtrinsics(*cam, calib);
                        }
                        break;
                    case SchemeType::HEMISPHERE_ZOOM:
                        if (cam->role == CameraRole::PRIMARY_WIDE) {
                            setupHemisphereExtrinsics(*cam, calib, 0.0, 180.0);
                        } else if (cam->role == CameraRole::SECONDARY_WIDE) {
                            setupHemisphereExtrinsics(*cam, calib, 30.0, 120.0);
                        } else if (cam->role == CameraRole::ZOOM) {
                            setupZoomExtrinsics(*cam, calib);
                        }
                        break;
                }
                computeHomography(calib);
                updated = true;
            }
        }
    }
    
    return updated;
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
    GlobalObject new_obj;
    new_obj.global_id = next_global_id_++;
    new_obj.camera_detections[camera_id] = detection;
    new_obj.primary_camera_id = camera_id;
    new_obj.last_seen_timestamp = timestamp;
    new_obj.confidence = 0.5; // Начальная уверенность
    new_obj.velocity = cv::Point3f(0, 0, 0); // Начальная скорость
    
    // Вычисляем начальную мировую позицию
    cv::Point2f detection_center(
        detection.x + detection.width / 2.0,
        detection.y + detection.height / 2.0
    );
    new_obj.world_position = imageToWorld(camera_id, detection_center);
    new_obj.position_history.push_back(new_obj.world_position);
    tracked_objects_[new_obj.global_id] = new_obj;
    std::cout << "Создан новый объект ID=" << new_obj.global_id
              << " на камере " << camera_id << std::endl;
}

void GlobalTracker::updateWorldPositions(const std::string& camera_id, uint64_t timestamp) {
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
    auto calib_it = camera_calibrations_.find(camera_id);
    if (calib_it == camera_calibrations_.end() || !calib_it->second.is_calibrated) {
        std::cerr << "Камера " << camera_id << " не откалибрована" << std::endl;
        return cv::Point3f(0, 0, 0);
    }
    
    const CameraCalibration& calib = calib_it->second;
    
    try {
        // Используем гомографию если доступна (более точно для плоскости z=0)
        if (!calib.homography_matrix.empty()) {
            std::vector<cv::Point2f> image_points = {image_point};
            std::vector<cv::Point2f> world_points_2d;
            
            cv::perspectiveTransform(image_points, world_points_2d, calib.homography_matrix);
            return cv::Point3f(world_points_2d[0].x, world_points_2d[0].y, 0.0);
        }
        
        // Fallback к методу с лучом (исправленная версия)
        std::vector<cv::Point2f> image_points = {image_point};
        std::vector<cv::Point2f> undistorted_points;
        
        cv::undistortPoints(image_points, undistorted_points, calib.camera_matrix, calib.dist_coeffs);
        
        // Создаем луч в системе координат камеры
        cv::Point3f ray_camera(undistorted_points[0].x, undistorted_points[0].y, 1.0);
        
        // Преобразуем в мировые координаты
        cv::Mat R;
        cv::Rodrigues(calib.rotation_vector, R);
        cv::Mat R_inv = R.t();
        
        // Позиция камеры в мировых координатах
        cv::Mat cam_position = -R_inv * calib.translation_vector;
        
        // Направление луча в мировых координатах
        cv::Mat ray_world_mat = R_inv * cv::Mat(ray_camera);
        cv::Point3f ray_world(ray_world_mat.at<double>(0), ray_world_mat.at<double>(1), ray_world_mat.at<double>(2));
        
        // Пересечение с плоскостью z = 0
        if (std::abs(ray_world.z) < 1e-6) {
            return cv::Point3f(0, 0, 0); // Луч параллелен плоскости
        }
        
        double t = -cam_position.at<double>(2) / ray_world.z;
        
        cv::Point3f world_point(
            cam_position.at<double>(0) + t * ray_world.x,
            cam_position.at<double>(1) + t * ray_world.y,
            0.0
        );
        
        return world_point;
        
    } catch (const cv::Exception& e) {
        std::cerr << "Error in imageToWorld for camera " << camera_id << ": " << e.what() << std::endl;
        return cv::Point3f(0, 0, 0);
    }
}

cv::Point2f GlobalTracker::worldToImage(const std::string& camera_id, const cv::Point3f& world_point) {
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
        setupCameraIntrinsics(*cam, calib);
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
    tracked_objects_.clear();
    next_global_id_ = 1;
    
    // Переинициализируем калибровку для новой схемы
    camera_calibrations_.clear();
    
    // Запускаем калибровку для новой схемы
    switch (new_scheme) {
        case SchemeType::SPHERE_ZOOM:
            calibrateSphereSetup();
            break;
        case SchemeType::HEMISPHERE_ZOOM:
            calibrateHemisphereSetup();
            break;
        default:
            std::cerr << "Неизвестная схема" << std::endl;
    }
}

double GlobalTracker::calculateDistance(const cv::Point3f& p1, const cv::Point3f& p2) {
    return cv::norm(p1 - p2);
}

bool GlobalTracker::initializeCameraCalibration(const std::string& camera_id) {
    CameraConfig* cam = scheme_manager_->getCamera(camera_id);
    if (!cam) {
        std::cerr << "Камера " << camera_id << " не найдена" << std::endl;
        return false;
    }

    CameraCalibration calib;
    calib.camera_matrix = cv::Mat::eye(3, 3, CV_64F);
    calib.dist_coeffs = cv::Mat::zeros(5, 1, CV_64F);
    calib.rotation_vector = cv::Mat::zeros(3, 1, CV_64F);
    calib.translation_vector = cv::Mat::zeros(3, 1, CV_64F);
    calib.homography_matrix = cv::Mat::eye(3, 3, CV_64F);
    calib.is_calibrated = false;

    camera_calibrations_[camera_id] = calib;
    return true;
}

bool GlobalTracker::reloadCalibration(const CalibrationWatcher& watcher) {
    bool updated = false;
    auto cameras = scheme_manager_->getCameras();
    
    for (const auto& cam : cameras) {
        if (cam.status != CameraStatus::ACTIVE) continue;
        
        cv::Mat K, D;
        if (watcher.getCameraMatrix(cam.id, K, D)) {
            auto& calib = camera_calibrations_[cam.id];
            calib.camera_matrix = K.clone();
            calib.dist_coeffs = D.clone();
            calib.is_calibrated = true;
            
            // Важно: пересчитываем внешние параметры после загрузки калибровки
            SchemeType scheme = scheme_manager_->getCurrentScheme();
            switch (scheme) {
                case SchemeType::SPHERE_ZOOM:
                    if (cam.role == CameraRole::PRIMARY_WIDE) {
                        setupSphereExtrinsics(cam, calib, 0.0, 180.0);
                    } else if (cam.role == CameraRole::SECONDARY_WIDE) {
                        setupSphereExtrinsics(cam, calib, 180.0, 180.0);
                    } else if (cam.role == CameraRole::ZOOM) {
                        setupZoomExtrinsics(cam, calib);
                    }
                    break;
                case SchemeType::HEMISPHERE_ZOOM:
                    if (cam.role == CameraRole::PRIMARY_WIDE) {
                        setupHemisphereExtrinsics(cam, calib, 0.0, 180.0);
                    } else if (cam.role == CameraRole::SECONDARY_WIDE) {
                        setupHemisphereExtrinsics(cam, calib, 30.0, 120.0);
                    } else if (cam.role == CameraRole::ZOOM) {
                        setupZoomExtrinsics(cam, calib);
                    }
                    break;
            }
            
            // Вычисляем гомографию для плоскости z=0
            computeHomography(calib);
            updated = true;
            
            std::cout << "Loaded calibration for camera " << cam.id 
                      << " (fx=" << K.at<double>(0,0) << ", fy=" << K.at<double>(1,1) << ")" << std::endl;
        }
    }
    
    if (updated) {
        std::cout << "Calibration data reloaded successfully" << std::endl;
    }
    
    return updated;
}

void GlobalTracker::cleanupOldObjects(uint64_t current_timestamp) {
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
    
    for (const auto& pair : tracked_objects_) {
        if (pair.second.confidence > 0.3) { // Порог уверенности
            active_objects.push_back(pair.second);
        }
    }
    
    return active_objects;
}
