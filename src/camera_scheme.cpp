#include "camera_scheme.h"
#include "global_tracker.h"
#include <fstream>
#include <iostream>
#include <algorithm>
#include <nlohmann/json.hpp>
#include <cmath>
#include <limits>

using json = nlohmann::json;

CameraSchemeManager::CameraSchemeManager()
    : current_scheme_(SchemeType::UNKNOWN),
      is_calibrated_(false),
      sphere_frame_counter_(0),
      next_track_id_(1),
      total_successful_handoffs_(0),
      total_failed_handoffs_(0) {
    tracking_config_.max_distance_threshold = 2.0;
    tracking_config_.max_object_speed_mps = 5.0;
    tracking_config_.id_retention_seconds = 3.0;
    tracking_config_.max_area_ratio_change = 0.5;
    tracking_config_.dormant_id_retention_seconds = 5.0;
    tracking_config_.pending_reid_descriptor_threshold = 0.2;
}

CameraSchemeManager::~CameraSchemeManager() {
}

bool CameraSchemeManager::initialize(const std::string& config_path) {
    std::cout << "🚀 Инициализация менеджера схем камер..." << std::endl;
    
    if (!loadConfigFromFile(config_path)) {
        std::cerr << "❌ Ошибка загрузки конфигурации: " << config_path << std::endl;
        return false;
    }
    
    std::cout << "📷 Схема камер: " << schemeTypeToString(current_scheme_) << std::endl;
    std::cout << "🔢 Количество камер: " << cameras_.size() << std::endl;
    
    // Валидация сферической конфигурации если необходимо
    if (current_scheme_ == SchemeType::SPHERE && !validateSphereConfiguration()) {
        std::cerr << "❌ Ошибка валидации сферической конфигурации" << std::endl;
        return false;
    }
    
    return true;
}

bool CameraSchemeManager::loadConfigFromFile(const std::string& config_path) {
    std::ifstream config_file(config_path);
    if (!config_file.is_open()) {
        std::cerr << "❌ Не удается открыть файл конфигурации: " << config_path << std::endl;
        return false;
    }
    
    json config;
    try {
        config_file >> config;
    } catch (const std::exception& e) {
        std::cerr << "❌ Ошибка парсинга JSON: " << e.what() << std::endl;
        return false;
    }
    
    // Загружаем тип схемы
    if (config.contains("scheme_type")) {
        current_scheme_ = stringToSchemeType(config["scheme_type"]);
    }
    
    // Загружаем камеры
    if (config.contains("cameras")) {
        cameras_.clear();
        for (const auto& cam_json : config["cameras"]) {
            CameraConfig cam;
            cam.id = cam_json.value("id", "");
            cam.device = cam_json.value("device", "");
            cam.role = stringToRole(cam_json.value("role", ""));
            cam.priority = cam_json.value("priority", 99);
            cam.mode = cam_json.value("mode", "detect");
            cam.width = cam_json.value("width", 640);
            cam.height = cam_json.value("height", 480);
            cam.fps = cam_json.value("fps", 30);

            // Если в конфигурации задан предпочтительный режим захвата, используем его
            if (cam_json.contains("preferred") && cam_json["preferred"].is_object()) {
                const auto& preferred = cam_json["preferred"];
                cam.width = preferred.value("w", cam.width);
                cam.height = preferred.value("h", cam.height);
                cam.fps = preferred.value("fps", cam.fps);
            }
            cam.status = CameraStatus::ACTIVE;
            
            // Загружаем покрытие
            if (cam_json.contains("coverage")) {
                auto cov = cam_json["coverage"];
                cam.coverage.angle = cov.value("angle", 60);
                cam.coverage.direction = cov.value("direction", "front");
                cam.coverage.zone = cov.value("zone", "main_area");
                cam.coverage.azimuth_start = cov.value("azimuth_start", 0.0);
                cam.coverage.azimuth_end = cov.value("azimuth_end", 360.0);
                cam.coverage.elevation_start = cov.value("elevation_start", -45.0);
                cam.coverage.elevation_end = cov.value("elevation_end", 45.0);
            }
            
            // Загружаем позицию
            if (cam_json.contains("position")) {
                auto pos = cam_json["position"];
                cam.position.x = pos.value("x", 0.0);
                cam.position.y = pos.value("y", 0.0);
                cam.position.z = pos.value("z", 0.0);
                cam.position.description = pos.value("description", "");
                cam.position.azimuth = pos.value("azimuth", 0.0);
                cam.position.elevation = pos.value("elevation", 0.0);
                cam.position.tilt = pos.value("tilt", 0.0);
            }
            
            // Дополнительные параметры
            cam.is_ptz = cam_json.value("is_ptz", false);
            cam.focal_length = cam_json.value("focal_length", 3.6);
            cam.sensor_size = cam_json.value("sensor_size", 6.17);
            
            cameras_.push_back(cam);
        }
    }
    
    // Загружаем сферическую конфигурацию
    if (config.contains("sphere_configuration")) {
        auto sphere_cfg = config["sphere_configuration"];
        sphere_config_.primary_wide_id = sphere_cfg.value("primary_wide_id", "");
        sphere_config_.secondary_wide_id = sphere_cfg.value("secondary_wide_id", "");
        sphere_config_.zoom_camera_id = sphere_cfg.value("zoom_camera_id", "");
        sphere_config_.sphere_radius = sphere_cfg.value("sphere_radius", 10.0);
        
        if (sphere_cfg.contains("sphere_center")) {
            auto center = sphere_cfg["sphere_center"];
            sphere_config_.sphere_center.x = center.value("x", 0.0);
            sphere_config_.sphere_center.y = center.value("y", 0.0);
            sphere_config_.sphere_center.z = center.value("z", 0.0);
        }
        
        sphere_config_.overlap_angle = sphere_cfg.value("overlap_angle", 20.0);
        sphere_config_.enable_cross_camera_tracking = sphere_cfg.value("enable_cross_camera_tracking", true);
        sphere_config_.handoff_overlap_threshold = sphere_cfg.value("handoff_overlap_threshold", 0.3);
        sphere_config_.zoom_trigger_distance = sphere_cfg.value("zoom_trigger_distance", 5.0);
        
        // Загружаем зоны зума
        if (sphere_cfg.contains("zoom_zones")) {
            sphere_config_.zoom_zones.clear();
            for (const auto& zone_json : sphere_cfg["zoom_zones"]) {
                SphereConfiguration::ZoomZone zone;
                zone.zone_name = zone_json.value("zone_name", "");
                zone.min_azimuth = zone_json.value("min_azimuth", 0.0);
                zone.max_azimuth = zone_json.value("max_azimuth", 360.0);
                zone.min_elevation = zone_json.value("min_elevation", -90.0);
                zone.max_elevation = zone_json.value("max_elevation", 90.0);
                zone.zoom_level = zone_json.value("zoom_level", 1.0);
                zone.priority = zone_json.value("priority", 1);
                sphere_config_.zoom_zones.push_back(zone);
            }
        }
    }

    // Загружаем конфигурацию трекинга
    if (config.contains("tracking_config")) {
        auto tracking = config["tracking_config"];
        tracking_config_.max_distance_threshold = tracking.value("max_distance_threshold", 2.0);
        tracking_config_.max_object_speed_mps = tracking.value("max_object_speed_mps", 5.0);
        tracking_config_.id_retention_seconds = tracking.value("id_retention_seconds", 3.0);
        tracking_config_.max_area_ratio_change = tracking.value("max_area_ratio_change", 0.5);
        tracking_config_.dormant_id_retention_seconds = tracking.value("dormant_id_retention_seconds", 5.0);
        tracking_config_.pending_reid_descriptor_threshold =  tracking.value("pending_reid_descriptor_threshold", 0.2);
    }

    return true;
}

// Установка новой схемы камер
bool CameraSchemeManager::setScheme(SchemeType scheme) {
    // Если схема не изменяется, просто возвращаем успех
    if (current_scheme_ == scheme) {
        return true;
    }

    // Обновляем текущую схему
    current_scheme_ = scheme;

    // При смене схемы очищаем активные треки, так как конфигурация меняется
    active_tracks_.clear();
    sphere_frame_counter_ = 0;
    next_track_id_ = 1;
    total_successful_handoffs_ = 0;
    total_failed_handoffs_ = 0;

    // Для сферической схемы необходимо убедиться, что камеры имеют
    // соответствующие роли согласно сохранённой конфигурации
    if (scheme == SchemeType::SPHERE) {
        if (!sphere_config_.primary_wide_id.empty()) {
            if (auto cam = getCameraById(sphere_config_.primary_wide_id)) {
                cam->role = CameraRole::WIDE_ANGLE_PRIMARY;
            }
        }

        if (!sphere_config_.secondary_wide_id.empty()) {
            if (auto cam = getCameraById(sphere_config_.secondary_wide_id)) {
                cam->role = CameraRole::WIDE_ANGLE_SECONDARY;
            }
        }

        if (!sphere_config_.zoom_camera_id.empty()) {
            if (auto cam = getCameraById(sphere_config_.zoom_camera_id)) {
                cam->role = CameraRole::ZOOM_CAMERA;
            }
        }

        // Валидация сферической конфигурации
        return validateSphereConfiguration();
    }

    return true;
}

// Поиск камеры по ID
CameraConfig* CameraSchemeManager::getCamera(const std::string& id) {
    return getCameraById(id);
}

std::vector<CameraConfig*> CameraSchemeManager::getActiveCameras() {
    std::vector<CameraConfig*> active;
    for (auto& camera : cameras_) {
        if (camera.status == CameraStatus::ACTIVE) {
            active.push_back(&camera);
        }
    }
    return active;
}


// === РЕАЛИЗАЦИЯ МЕТОДОВ ДЛЯ СФЕРИЧЕСКОЙ КОНФИГУРАЦИИ ===

bool CameraSchemeManager::setupSphereConfiguration(const std::string& primary_wide_id,
                                                   const std::string& secondary_wide_id,
                                                   const std::string& zoom_camera_id) {
    std::cout << "🌐 Настройка сферической конфигурации..." << std::endl;
    
    // Проверяем существование камер
    auto primary = getCameraById(primary_wide_id);
    auto secondary = getCameraById(secondary_wide_id);
    auto zoom = getCameraById(zoom_camera_id);
    
    if (!primary || !secondary || !zoom) {
        std::cerr << "❌ Не найдены требуемые камеры для сферической конфигурации" << std::endl;
        return false;
    }
    
    // Обновляем роли камер
    primary->role = CameraRole::WIDE_ANGLE_PRIMARY;
    secondary->role = CameraRole::WIDE_ANGLE_SECONDARY;
    zoom->role = CameraRole::ZOOM_CAMERA;
    
    // Сохраняем конфигурацию
    sphere_config_.primary_wide_id = primary_wide_id;
    sphere_config_.secondary_wide_id = secondary_wide_id;
    sphere_config_.zoom_camera_id = zoom_camera_id;
    
    // Устанавливаем схему
    current_scheme_ = SchemeType::SPHERE;
    
    std::cout << "✅ Сферическая конфигурация настроена:" << std::endl;
    std::cout << "  📷 Основная широкофокусная: " << primary_wide_id << std::endl;
    std::cout << "  📷 Дополнительная широкофокусная: " << secondary_wide_id << std::endl;
    std::cout << "  🔍 Зум-камера: " << zoom_camera_id << std::endl;
    
    return validateSphereConfiguration();
}

bool CameraSchemeManager::validateSphereConfiguration() const {
    if (current_scheme_ != SchemeType::SPHERE) {
        return true; // Валидация только для сферы
    }
    
    std::cout << "🔍 Валидация сферической конфигурации..." << std::endl;
    
    // Проверяем наличие всех необходимых камер
    const CameraConfig* primary = getCameraById(sphere_config_.primary_wide_id);
    const CameraConfig* secondary = getCameraById(sphere_config_.secondary_wide_id);
    const CameraConfig* zoom = getCameraById(sphere_config_.zoom_camera_id);

    if (!primary || !secondary || !zoom) {
        std::cerr << "❌ Отсутствуют необходимые камеры" << std::endl;
        return false;
    }
    
    // Проверяем статус камер
    if (primary->status != CameraStatus::ACTIVE || 
        secondary->status != CameraStatus::ACTIVE || 
        zoom->status != CameraStatus::ACTIVE) {
        std::cerr << "❌ Не все камеры активны" << std::endl;
        return false;
    }
    
    // Проверяем покрытие
    double total_coverage = calculateTotalCoverage();
    if (total_coverage < 300.0) { // Минимум 300 градусов покрытия
        std::cerr << "❌ Недостаточное покрытие: " << total_coverage << "°" << std::endl;
        return false;
    }
    
    // Проверяем перекрытие между широкофокусными камерами
    double overlap = calculateCameraOverlap(*primary, *secondary);
    if (overlap < 10.0) { // Минимум 10 градусов перекрытия
        std::cerr << "❌ Недостаточное перекрытие между камерами: " << overlap << "°" << std::endl;
        return false;
    }
    
    std::cout << "✅ Сферическая конфигурация валидна" << std::endl;
    std::cout << "  🌐 Общее покрытие: " << total_coverage << "°" << std::endl;
    std::cout << "  🔗 Перекрытие камер: " << overlap << "°" << std::endl;
    
    return true;
}

double CameraSchemeManager::calculateTotalCoverage() const {
    double total = 0.0;
    
    for (const auto& camera : cameras_) {
        if (camera.status == CameraStatus::ACTIVE && 
            (camera.role == CameraRole::WIDE_ANGLE_PRIMARY || 
             camera.role == CameraRole::WIDE_ANGLE_SECONDARY)) {
            
            double coverage = camera.coverage.azimuth_end - camera.coverage.azimuth_start;
            if (coverage < 0) coverage += 360.0; // Обработка перехода через 0°
            total += coverage;
        }
    }
    
    return total;
}

double CameraSchemeManager::calculateCameraOverlap(const CameraConfig& cam1, const CameraConfig& cam2) const {
    // Упрощенный расчет перекрытия по азимуту
    double start1 = cam1.coverage.azimuth_start;
    double end1 = cam1.coverage.azimuth_end;
    double start2 = cam2.coverage.azimuth_start;
    double end2 = cam2.coverage.azimuth_end;
    
    // Нормализация углов
    if (end1 < start1) end1 += 360.0;
    if (end2 < start2) end2 += 360.0;
    
    // Находим пересечение
    double overlap_start = std::max(start1, start2);
    double overlap_end = std::min(end1, end2);
    
    if (overlap_end > overlap_start) {
        return overlap_end - overlap_start;
    }
    return 0.0;
}

double CameraSchemeManager::calculateNormalizedOverlapRatio(const CameraConfig& cam1, const CameraConfig& cam2) const {
    double overlap = calculateCameraOverlap(cam1, cam2);
    if (overlap <= 0.0) {
        return 0.0;
    }

    auto coverage_span = [](const CameraCoverage& coverage) {
        double start = coverage.azimuth_start;
        double end = coverage.azimuth_end;
        if (end < start) {
            end += 360.0;
        }
        return std::max(0.0, end - start);
    };

    double cam1_span = coverage_span(cam1.coverage);
    double cam2_span = coverage_span(cam2.coverage);
    double normalization = std::max(1e-6, std::min(cam1_span, cam2_span));

    return std::min(1.0, overlap / normalization);
}

std::vector<std::pair<double, double>> CameraSchemeManager::getUncoveredAreas() const {
    std::vector<std::pair<double, double>> uncovered;
    std::vector<std::pair<double, double>> covered_ranges;
    
    // Собираем все покрытые диапазоны
    for (const auto& camera : cameras_) {
        if (camera.status == CameraStatus::ACTIVE && 
            (camera.role == CameraRole::WIDE_ANGLE_PRIMARY || 
             camera.role == CameraRole::WIDE_ANGLE_SECONDARY)) {
            
            double start = camera.coverage.azimuth_start;
            double end = camera.coverage.azimuth_end;
            
            if (end < start) {
                // Диапазон переходит через 0°
                covered_ranges.push_back({start, 360.0});
                covered_ranges.push_back({0.0, end});
            } else {
                covered_ranges.push_back({start, end});
            }
        }
    }
    
    // Сортируем диапазоны
    std::sort(covered_ranges.begin(), covered_ranges.end());
    
    // Находим непокрытые области
    double current_pos = 0.0;
    for (const auto& range : covered_ranges) {
        if (range.first > current_pos) {
            uncovered.push_back({current_pos, range.first});
        }
        current_pos = std::max(current_pos, range.second);
    }
    
    if (current_pos < 360.0) {
        uncovered.push_back({current_pos, 360.0});
    }
    
    return uncovered;
}

bool CameraSchemeManager::updateSphereTracking(const std::map<std::string, std::vector<Point2D>>& detections) {
    if (current_scheme_ != SchemeType::SPHERE) {
        return false;
    }

    sphere_frame_counter_++;

    auto computeWorldPosition = [](const Point2D& image_point, const CameraConfig& camera) {
        Point3D position;
        position.x = image_point.x - camera.width / 2.0;
        position.y = image_point.y - camera.height / 2.0;
        position.z = 0.0;
        return position;
    };

    auto distance3d = [](const Point3D& a, const Point3D& b) {
        double dx = a.x - b.x;
        double dy = a.y - b.y;
        double dz = a.z - b.z;
        return std::sqrt(dx * dx + dy * dy + dz * dz);
    };

    std::vector<std::string> tracks_to_remove;


    for (const auto& [camera_id, points] : detections) {
        const CameraConfig* camera = getCameraById(camera_id);
        if (!camera || camera->status != CameraStatus::ACTIVE) {
            continue;
        }

        for (const auto& detection : points) {
            Point3D world_position = computeWorldPosition(detection, *camera);

            auto matched_it = active_tracks_.end();
            double best_distance = std::numeric_limits<double>::max();
            double distance_threshold = tracking_config_.max_distance_threshold;
            double fallback_threshold = std::min(camera->width, camera->height) * 0.1;
            if (distance_threshold < fallback_threshold) {
                distance_threshold = fallback_threshold;
            }

            for (auto it = active_tracks_.begin(); it != active_tracks_.end(); ++it) {
                SphereTrackingResult& track = it->second;

                if (track.last_update_frame == sphere_frame_counter_) {
                    continue;
                }

                double distance = distance3d(track.world_position, world_position);
                if (distance > distance_threshold) {
                    continue;
                }

                bool can_use_camera = (track.primary_camera == camera_id);
                if (!can_use_camera && sphere_config_.enable_cross_camera_tracking) {
                    const CameraConfig* from_camera = getCameraById(track.primary_camera);
                    if (from_camera) {
                        double normalized_overlap = calculateNormalizedOverlapRatio(*from_camera, *camera);
                        if (normalized_overlap >= sphere_config_.handoff_overlap_threshold) {
                            can_use_camera = true;
                        }
                    }
                }

                if (!can_use_camera) {
                    continue;
                }

                if (distance < best_distance) {
                    best_distance = distance;
                    matched_it = it;
                }
            }

            if (matched_it != active_tracks_.end()) {
                SphereTrackingResult& track = matched_it->second;

                if (track.primary_camera != camera_id && sphere_config_.enable_cross_camera_tracking) {
                    handoffObjectBetweenCameras(track.object_id, track.primary_camera, camera_id);
                }

                track.world_position = world_position;
                track.last_detected_camera = camera_id;
                track.last_update_frame = sphere_frame_counter_;
                track.last_seen_frame = sphere_frame_counter_;
                track.camera_last_seen_frame[camera_id] = sphere_frame_counter_;

                if (std::find(track.visible_cameras.begin(), track.visible_cameras.end(), camera_id) == track.visible_cameras.end()) {
                    track.visible_cameras.push_back(camera_id);
                }

                track.confidence = std::min(1.0, track.confidence + 0.05);
            } else {
                std::string new_id = "track_" + std::to_string(next_track_id_++);
                SphereTrackingResult& new_track = active_tracks_[new_id];
                new_track.object_id = new_id;
                new_track.world_position = world_position;
                new_track.visible_cameras.clear();
                new_track.visible_cameras.push_back(camera_id);
                new_track.primary_camera = camera_id;
                new_track.zoom_recommended = false;
                new_track.confidence = 0.6;
                new_track.last_detected_camera = camera_id;
                new_track.last_update_frame = sphere_frame_counter_;
                new_track.last_seen_frame = sphere_frame_counter_;
                new_track.camera_last_seen_frame.clear();
                new_track.camera_last_seen_frame[camera_id] = sphere_frame_counter_;
            }
        }
    }


    for (auto& [id, track] : active_tracks_) {
        if (track.last_update_frame != sphere_frame_counter_) {
            track.confidence = std::max(0.0, track.confidence - 0.1);
        }

        track.visible_cameras.erase(
            std::remove_if(track.visible_cameras.begin(), track.visible_cameras.end(),
                           [&](const std::string& cam) {
                               auto seen_it = track.camera_last_seen_frame.find(cam);
                               if (seen_it == track.camera_last_seen_frame.end()) {
                                   return true;
                               }
                               return (sphere_frame_counter_ - seen_it->second) > 1;
                           }),
            track.visible_cameras.end());

        if ((sphere_frame_counter_ - track.last_seen_frame) > 5 && track.confidence < 0.2) {
            tracks_to_remove.push_back(id);
        }
    }

    for (const auto& id : tracks_to_remove) {
        active_tracks_.erase(id);
    }

    return true;
}

bool CameraSchemeManager::handoffObjectBetweenCameras(const std::string& object_id,
                                                      const std::string& from_camera,
                                                      const std::string& to_camera) {
    auto track_it = active_tracks_.find(object_id);
    if (track_it == active_tracks_.end()) {
        std::cerr << "❌ Не найден трек для передачи объекта: " << object_id << std::endl;
        return false;
    }

    if (!sphere_config_.enable_cross_camera_tracking) {
        std::cerr << "⚠️ Кросс-камерный трекинг отключен, передача невозможна" << std::endl;
        SphereTrackingResult::HandoffEvent event{from_camera, to_camera, false,
                                                "Кросс-камерный трекинг отключен", 0.0};
        track_it->second.handoff_log.push_back(event);
        track_it->second.handoff_attempts++;
        track_it->second.failed_handoffs++;
        total_failed_handoffs_++;
        return false;
    }

    SphereTrackingResult& track = track_it->second;
    track.handoff_attempts++;

    if (!from_camera.empty() && track.primary_camera != from_camera) {
        std::cerr << "⚠️ Попытка передачи от камеры " << from_camera
                  << ", но текущая основная камера трека: " << track.primary_camera << std::endl;
    }

    const CameraConfig* from_config = getCameraById(from_camera);
    if (!from_config) {
        std::cerr << "❌ Камера-источник недоступна: " << from_camera << std::endl;
        SphereTrackingResult::HandoffEvent event{from_camera, to_camera, false,
                                                "Камера-источник недоступна", 0.0};
        track.handoff_log.push_back(event);
        track.failed_handoffs++;
        total_failed_handoffs_++;
        return false;
    }

    std::vector<std::string> candidate_cameras;
    if (!to_camera.empty() && to_camera != from_camera) {
        candidate_cameras.push_back(to_camera);
    }

    const std::vector<std::string> sphere_ids = {
        sphere_config_.primary_wide_id,
        sphere_config_.secondary_wide_id,
        sphere_config_.zoom_camera_id
    };

    for (const auto& id : sphere_ids) {
        if (id.empty() || id == from_camera) {
            continue;
        }
        if (std::find(candidate_cameras.begin(), candidate_cameras.end(), id) == candidate_cameras.end()) {
            candidate_cameras.push_back(id);
        }
    }

    if (candidate_cameras.empty()) {
        for (const auto& camera_cfg : cameras_) {
            if (camera_cfg.id.empty() || camera_cfg.id == from_camera) {
                continue;
            }
            if (camera_cfg.status != CameraStatus::ACTIVE) {
                continue;
            }
            if (std::find(candidate_cameras.begin(), candidate_cameras.end(), camera_cfg.id) == candidate_cameras.end()) {
                candidate_cameras.push_back(camera_cfg.id);
            }
        }
    }

    double best_overlap = 0.0;
    std::string best_camera;
    std::string failure_reason;

    for (const auto& candidate_id : candidate_cameras) {
        const CameraConfig* candidate_config = getCameraById(candidate_id);
        if (!candidate_config || candidate_config->status != CameraStatus::ACTIVE) {
            failure_reason = "Целевая камера недоступна";
            continue;
        }

        double normalized_overlap = calculateNormalizedOverlapRatio(*from_config, *candidate_config);
        if (normalized_overlap < sphere_config_.handoff_overlap_threshold) {
            failure_reason = "Перекрытие ниже порога";
            continue;
        }

        if (normalized_overlap > best_overlap) {
            best_overlap = normalized_overlap;
            best_camera = candidate_id;
        }
    }

    if (best_camera.empty()) {
        track.failed_handoffs++;
        total_failed_handoffs_++;
        SphereTrackingResult::HandoffEvent event{from_camera, to_camera, false,
                                                failure_reason.empty() ? "Подходящая камера не найдена" : failure_reason,
                                                best_overlap};
        track.handoff_log.push_back(event);
        std::cerr << "❌ Не удалось выполнить передачу объекта " << object_id
                  << " от камеры " << from_camera << ": " << event.reason << std::endl;
        return false;
    }

    track.primary_camera = best_camera;
    track.last_detected_camera = best_camera;
    if (std::find(track.visible_cameras.begin(), track.visible_cameras.end(), best_camera) == track.visible_cameras.end()) {
        track.visible_cameras.push_back(best_camera);
    }

    track.successful_handoffs++;
    total_successful_handoffs_++;
    SphereTrackingResult::HandoffEvent event{from_camera, best_camera, true,
                                            "Успешная передача", best_overlap};
    track.handoff_log.push_back(event);

    std::cout << "🔄 Передача объекта " << object_id << " с камеры " << from_camera
              << " на камеру " << best_camera << " (нормализованное перекрытие: "
              << best_overlap << ")" << std::endl;

    return true;
}

std::vector<SphereTrackingResult> CameraSchemeManager::getActiveSphereTracks() const {
    std::vector<SphereTrackingResult> tracks;
    for (const auto& [id, track] : active_tracks_) {
        tracks.push_back(track);
    }
    return tracks;
}

bool CameraSchemeManager::setZoomTarget(const Point3D& target_position, double zoom_level) {
    auto zoom_camera = getCameraById(sphere_config_.zoom_camera_id);
    if (!zoom_camera || !zoom_camera->is_ptz) {
        std::cerr << "❌ Зум-камера недоступна или не поддерживает PTZ" << std::endl;
        return false;
    }
    
    // Вычисляем углы для наведения на цель
    double azimuth = std::atan2(target_position.y, target_position.x) * 180.0 / M_PI;
    double distance = std::sqrt(target_position.x * target_position.x + target_position.y * target_position.y);
    double elevation = std::atan2(target_position.z, distance) * 180.0 / M_PI;
    
    std::cout << "🎯 Наведение зум-камеры:" << std::endl;
    std::cout << "  📍 Позиция: (" << target_position.x << ", " << target_position.y << ", " << target_position.z << ")" << std::endl;
    std::cout << "  🔄 Азимут: " << azimuth << "°" << std::endl;
    std::cout << "  📐 Элевация: " << elevation << "°" << std::endl;
    std::cout << "  🔍 Зум: " << zoom_level << "x" << std::endl;
    
    // TODO: Реализовать реальное PTZ управление
    // Здесь должен быть код для отправки команд на PTZ камеру
    
    return true;
}

bool CameraSchemeManager::setZoomToTrackObject(const std::string& object_id) {
    auto it = active_tracks_.find(object_id);
    if (it == active_tracks_.end()) {
        std::cerr << "❌ Объект не найден: " << object_id << std::endl;
        return false;
    }
    
    const auto& track = it->second;
    return setZoomTarget(track.world_position, 2.0); // Зум 2x по умолчанию
}

bool CameraSchemeManager::addZoomZone(const std::string& zone_name, 
                                     double min_azimuth, double max_azimuth,
                                     double min_elevation, double max_elevation,
                                     double zoom_level, int priority) {
    
    SphereConfiguration::ZoomZone zone;
    zone.zone_name = zone_name;
    zone.min_azimuth = min_azimuth;
    zone.max_azimuth = max_azimuth;
    zone.min_elevation = min_elevation;
    zone.max_elevation = max_elevation;
    zone.zoom_level = zoom_level;
    zone.priority = priority;
    
    sphere_config_.zoom_zones.push_back(zone);
    
    std::cout << "📊 Добавлена зона зума: " << zone_name << std::endl;
    std::cout << "  🌍 Азимут: " << min_azimuth << "° - " << max_azimuth << "°" << std::endl;
    std::cout << "  📏 Элевация: " << min_elevation << "° - " << max_elevation << "°" << std::endl;
    std::cout << "  🔍 Зум: " << zoom_level << "x" << std::endl;
    
    return true;
}

bool CameraSchemeManager::removeZoomZone(const std::string& zone_name) {
    auto it = std::remove_if(sphere_config_.zoom_zones.begin(), sphere_config_.zoom_zones.end(),
                            [&zone_name](const SphereConfiguration::ZoomZone& zone) {
                                return zone.zone_name == zone_name;
                            });
    
    if (it != sphere_config_.zoom_zones.end()) {
        sphere_config_.zoom_zones.erase(it, sphere_config_.zoom_zones.end());
        std::cout << "🗑️ Удалена зона зума: " << zone_name << std::endl;
        return true;
    }
    
    return false;
}

CameraSchemeManager::SphereStats CameraSchemeManager::getSphereStats() const {
    SphereStats stats{};
    stats.total_objects_tracked = static_cast<int>(active_tracks_.size());
    stats.successful_handoffs = total_successful_handoffs_;
    stats.failed_handoffs = total_failed_handoffs_;

    double confidence_sum = 0.0;
    for (const auto& [object_id, track] : active_tracks_) {
        confidence_sum += track.confidence;
        for (const auto& camera_id : track.visible_cameras) {
            stats.camera_usage_stats[camera_id]++;
        }
    }

    if (!active_tracks_.empty()) {
        stats.average_tracking_accuracy = confidence_sum / static_cast<double>(active_tracks_.size());
    } else {
        stats.average_tracking_accuracy = 0.0;
    }

    return stats;
}

// Существующие методы...
CameraConfig* CameraSchemeManager::getCameraById(const std::string& id) {
    for (auto& camera : cameras_) {
        if (camera.id == id) {
            return &camera;
        }
    }
    return nullptr;
}

const CameraConfig* CameraSchemeManager::getCameraById(const std::string& id) const {
    for (const auto& camera : cameras_) {
        if (camera.id == id) {
            return &camera;
        }
    }
    return nullptr;
}

std::string CameraSchemeManager::schemeTypeToString(SchemeType type) {
    switch (type) {
        case SchemeType::SPHERE: return "sphere";
        case SchemeType::HEMISPHERE_SINGLE: return "hemisphere_single";
        case SchemeType::HEMISPHERE_MULTI: return "hemisphere_multi";
        case SchemeType::HEMISPHERE_ZOOM: return "hemisphere_zoom";
        case SchemeType::SPHERE_ZOOM: return "sphere_zoom";
        default: return "unknown";
    }
}

SchemeType CameraSchemeManager::stringToSchemeType(const std::string& str) {
    if (str == "sphere") return SchemeType::SPHERE;
    if (str == "hemisphere_single") return SchemeType::HEMISPHERE_SINGLE;
    if (str == "hemisphere_multi") return SchemeType::HEMISPHERE_MULTI;
    if (str == "hemisphere_zoom") return SchemeType::HEMISPHERE_ZOOM;
    if (str == "sphere_zoom") return SchemeType::SPHERE_ZOOM;
    return SchemeType::UNKNOWN;
}

std::string CameraSchemeManager::roleToString(CameraRole role) {
    switch (role) {
        case CameraRole::WIDE_ANGLE_PRIMARY: return "wide_angle_primary";
        case CameraRole::WIDE_ANGLE_SECONDARY: return "wide_angle_secondary";
        case CameraRole::ZOOM_CAMERA: return "zoom_camera";
        case CameraRole::PRIMARY_WIDE: return "primary_wide";
        case CameraRole::SECONDARY_WIDE: return "secondary_wide";
        case CameraRole::ZOOM: return "zoom";
        case CameraRole::DISABLED: return "disabled";
        default: return "unknown";
    }
}

CameraRole CameraSchemeManager::stringToRole(const std::string& str) {
    if (str == "wide_angle_primary") return CameraRole::WIDE_ANGLE_PRIMARY;
    if (str == "wide_angle_secondary") return CameraRole::WIDE_ANGLE_SECONDARY;
    if (str == "wide_angle") {
        const bool has_primary = std::any_of(
            cameras_.begin(), cameras_.end(),
            [](const CameraConfig& camera) {
                return camera.role == CameraRole::WIDE_ANGLE_PRIMARY ||
                       camera.role == CameraRole::PRIMARY_WIDE;
            });

        const bool has_secondary = std::any_of(
            cameras_.begin(), cameras_.end(),
            [](const CameraConfig& camera) {
                return camera.role == CameraRole::WIDE_ANGLE_SECONDARY ||
                       camera.role == CameraRole::SECONDARY_WIDE;
            });

        const bool prefer_sphere_roles =
            current_scheme_ == SchemeType::SPHERE ||
            current_scheme_ == SchemeType::SPHERE_ZOOM;

        if (prefer_sphere_roles) {
            if (!has_primary) {
                return CameraRole::WIDE_ANGLE_PRIMARY;
            }
            if (!has_secondary) {
                return CameraRole::WIDE_ANGLE_SECONDARY;
            }
            return CameraRole::WIDE_ANGLE_SECONDARY;
        }

        if (!has_primary) {
            return CameraRole::PRIMARY_WIDE;
        }
        if (!has_secondary) {
            return CameraRole::SECONDARY_WIDE;
        }
        return CameraRole::SECONDARY_WIDE;
    }
    if (str == "zoom_camera") return CameraRole::ZOOM_CAMERA;
    if (str == "primary_wide") return CameraRole::PRIMARY_WIDE;
    if (str == "secondary_wide") return CameraRole::SECONDARY_WIDE;
    if (str == "zoom") return CameraRole::ZOOM;
    if (str == "disabled") return CameraRole::DISABLED;
    return CameraRole::DISABLED;
}

void CameraSchemeManager::setCalibrated(bool calibrated, GlobalTracker* tracker) {
    is_calibrated_ = calibrated;
    if (calibrated && tracker) {
        tracker->initialize();
    }
}