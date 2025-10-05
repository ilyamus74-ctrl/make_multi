
#ifndef CAMERA_SCHEME_H
#define CAMERA_SCHEME_H

#include <string>
#include <vector>
#include <map>
#include <memory>
#include <cstdint>

class GlobalTracker;

// Простые структуры точек без OpenCV
struct Point3D {
    double x, y, z;
    Point3D() : x(0), y(0), z(0) {}
    Point3D(double x_, double y_, double z_) : x(x_), y(y_), z(z_) {}
};

struct Point2D {
    double x, y;
    Point2D() : x(0), y(0) {}
    Point2D(double x_, double y_) : x(x_), y(y_) {}
};

// Расширенные роли камер для сферической системы
enum class CameraRole {
    // Широкофокусные камеры для сферы
    WIDE_ANGLE_PRIMARY,    // Первая широкофокусная камера (основная)
    WIDE_ANGLE_SECONDARY,  // Вторая широкофокусная камера (дополнительная)
    
    // Зум камера
    ZOOM_CAMERA,           // PTZ или фиксированная зум-камера
    
    // Существующие роли (для совместимости)
    PRIMARY_WIDE,          // Главная широкоугольная
    SECONDARY_WIDE,        // Резервная широкоугольная
    ZOOM,                  // Зум камера для детализации
    DISABLED               // Отключена
};

// Типы схем расположения камер
enum class SchemeType {
    SPHERE,              // Сфера: 2 широкофокусные + 1 зум
    HEMISPHERE_SINGLE,   // Полусфера: 1 широкофокусная + 1 зум
    HEMISPHERE_MULTI,    // Полусфера мульти: 1 широкофокусная + N зум
    HEMISPHERE_ZOOM,     // Полусфера + зум (legacy)
    SPHERE_ZOOM,         // Полная сфера + зум (legacy)
    UNKNOWN
};

// Состояние камеры
enum class CameraStatus {
    ACTIVE,
    FAILED,
    STANDBY,
    CALIBRATING
};

// Информация о покрытии камеры
struct CameraCoverage {
    int angle;                    // Угол обзора в градусах
    std::string direction;        // Направление: front, back, side, center
    std::string zone;            // Зона покрытия: main_area, backup_area, detail_area
    double azimuth_start;        // Начальный азимут (градусы)
    double azimuth_end;          // Конечный азимут (градусы)
    double elevation_start;      // Начальная элевация (градусы)
    double elevation_end;        // Конечная элевация (градусы)
};

// Позиция камеры в пространстве
struct CameraPosition {
    double x, y, z;              // Координаты в см
    std::string description;     // Описание позиции
    double azimuth;              // Азимут камеры (градусы)
    double elevation;            // Элевация камеры (градусы)
    double tilt;                 // Наклон камеры (градусы)
};

// Конфигурация отдельной камеры
struct CameraConfig {
    std::string id;              // Уникальный ID
    std::string device;          // Путь к устройству
    CameraRole role;             // Роль в системе
    int priority;                // Приоритет (1 = высший)
    std::string mode;            // Режим: detect, preview, calibration
    
    // Параметры захвата
    int width, height, fps;
    
    // Покрытие и позиция
    CameraCoverage coverage;
    CameraPosition position;
    
    // Статус
    CameraStatus status;
    
    // Дополнительные параметры для сферы
    bool is_ptz;                 // PTZ камера или нет
    double focal_length;         // Фокусное расстояние (мм)
    double sensor_size;          // Размер сенсора (мм)
};

// Специальная конфигурация для сферической системы
struct SphereConfiguration {
    std::string primary_wide_id;     // ID первой широкофокусной камеры
    std::string secondary_wide_id;   // ID второй широкофокусной камеры
    std::string zoom_camera_id;      // ID зум-камеры
    
    // Параметры сферы
    double sphere_radius;            // Радиус сферы наблюдения (м)
    Point3D sphere_center;           // Центр сферы
    double overlap_angle;            // Угол перекрытия между широкофокусными камерами
    
    // Зоны автоматического зума
    struct ZoomZone {
        std::string zone_name;
        double min_azimuth, max_azimuth;
        double min_elevation, max_elevation;
        double zoom_level;           // Уровень зума (1.0 = без зума)
        int priority;                // Приоритет зоны
    };
    std::vector<ZoomZone> zoom_zones;
    
    // Настройки трекинга
    bool enable_cross_camera_tracking;  // Трекинг между камерами
    double handoff_overlap_threshold;   // Порог передачи объекта между камерами
    double zoom_trigger_distance;       // Расстояние для активации зума (м)
};

// Настройки схемы камер
struct SchemeConfig {
    std::string name;
    std::string description;
    int coverage_degrees;
    std::vector<std::string> required_roles;
    std::map<std::string, std::string> failover;
    
    // Специальная конфигурация для сферы
    SphereConfiguration sphere_config;
};

// Глобальные настройки трекинга
struct GlobalTrackingConfig {
    double max_distance_threshold;    // Максимальное расстояние для сопоставления (м)
    double max_object_speed_mps;     // Максимальная скорость объекта (м/с)
    double id_retention_seconds;     // Время хранения ID после потери (сек)
    double max_area_ratio_change;    // Максимальное изменение площади bbox (доля)
    double dormant_id_retention_seconds; // Время хранения уснувших ID для повторной идентификации (сек)
    double pending_reid_descriptor_threshold; // Порог схожести дескрипторов для повторного использования ID
    std::string coordinate_system;    // Система координат
    
    // Параметры для сферы/полусферы
    double sphere_radius;            // Радиус сферы наблюдения (м)
    Point3D sphere_center;           // Центр сферы в мировых координатах
    double overlap_threshold;        // Порог перекрытия камер (0.0-1.0)
    bool auto_calibration_enabled;   // Автоматическая калибровка
    
    // Параметры зон покрытия
    struct ZoneConfig {
        std::string zone_name;
        double priority_weight;
        std::vector<Point3D> boundary_points;
    };
    std::vector<ZoneConfig> coverage_zones;
};

// Структура для калибровочной точки
struct CalibrationPoint {
    Point3D world_coord;
    std::map<std::string, Point2D> camera_coords;
    bool is_verified;
};

// Результат трекинга объекта в сферической системе
struct SphereTrackingResult {
    struct HandoffEvent {
        std::string from_camera;
        std::string to_camera;
        bool success;
        std::string reason;
        double normalized_overlap;
    };

    std::string object_id;
    Point3D world_position;                          // Позиция в мировых координатах
    std::vector<std::string> visible_cameras;        // Камеры, которые видят объект
    std::string primary_camera;                      // Основная камера для этого объекта
    bool zoom_recommended = false;                   // Рекомендуется ли зум
    double confidence = 0.0;                         // Уверенность в трекинге (0.0-1.0)
    std::string last_detected_camera;                // Последняя камера, где обнаружен объект
    uint64_t last_update_frame = 0;                  // Последний кадр обновления
    uint64_t last_seen_frame = 0;                    // Последний кадр наблюдения
    std::map<std::string, uint64_t> camera_last_seen_frame;  // Последние кадры по камерам
    int handoff_attempts = 0;                        // Количество попыток передачи
    int successful_handoffs = 0;                     // Успешные передачи
    int failed_handoffs = 0;                         // Неудачные передачи
    std::vector<HandoffEvent> handoff_log;           // Журнал событий передачи
};

// Основной класс управления схемой камер
class CameraSchemeManager {
private:
    SchemeType current_scheme_;
    std::vector<CameraConfig> cameras_;
    std::map<std::string, SchemeConfig> available_schemes_;
    GlobalTrackingConfig tracking_config_;
    
    // Калибровочные данные
    std::vector<CalibrationPoint> calibration_points_;
    bool is_calibrated_;
    
    // Специальные данные для сферической конфигурации
    SphereConfiguration sphere_config_;
    std::map<std::string, SphereTrackingResult> active_tracks_;
    uint64_t sphere_frame_counter_;
    int next_track_id_;
    int total_successful_handoffs_;
    int total_failed_handoffs_;

    // Загрузка конфигурации
    bool loadConfigFromFile(const std::string& config_path);
    
    // Внутренние методы для сферы
    bool validateSphereConfiguration() const;
    double calculateCameraOverlap(const CameraConfig& cam1, const CameraConfig& cam2) const;
    double calculateNormalizedOverlapRatio(const CameraConfig& cam1, const CameraConfig& cam2) const;
    Point3D triangulatePosition(const Point2D& pos1, const Point2D& pos2,
                               const CameraConfig& cam1, const CameraConfig& cam2) const;
    
public:
    CameraSchemeManager();
    ~CameraSchemeManager();
    
    // Инициализация
    bool initialize(const std::string& config_path);
    
    // Управление схемами
    bool setScheme(SchemeType scheme);
    SchemeType getCurrentScheme() const { return current_scheme_; }
    
    // Управление камерами
    std::vector<CameraConfig> getCameras() const { return cameras_; }
    CameraConfig* getCamera(const std::string& id);
    bool updateCameraStatus(const std::string& id, CameraStatus status);
    
    // Методы для работы с камерами
    CameraConfig* getCameraById(const std::string& id);
    const CameraConfig* getCameraById(const std::string& id) const;
    CameraConfig* getPrimaryCameraByRole(CameraRole role);
    std::vector<CameraConfig*> getCamerasByRole(CameraRole role);
    std::vector<CameraConfig*> getActiveCameras();
    
    // Обработка отказов
    bool handleCameraFailure(const std::string& failed_camera_id);
    CameraConfig* getFailoverCamera(const std::string& failed_camera_id);
    
    // Управление схемами
    std::vector<std::string> getAvailableSchemes();
    bool saveConfig(const std::string& config_path);
    bool setCameraStatus(const std::string& camera_id, CameraStatus status);
    SchemeConfig getSchemeConfig(const std::string& scheme_name);
    bool validateScheme(const SchemeConfig& scheme);
    
    // === СПЕЦИАЛЬНЫЕ МЕТОДЫ ДЛЯ СФЕРИЧЕСКОЙ КОНФИГУРАЦИИ ===
    
    // Настройка сферической системы
    bool setupSphereConfiguration(const std::string& primary_wide_id,
                                 const std::string& secondary_wide_id,
                                 const std::string& zoom_camera_id);
    
    // Получение конфигурации сферы
    SphereConfiguration getSphereConfiguration() const { return sphere_config_; }
    
    // Проверка покрытия сферы
    bool validateSphereCoverage() const;
    double calculateTotalCoverage() const;
    std::vector<std::pair<double, double>> getUncoveredAreas() const;
    
    // Трекинг в сферической системе
    bool updateSphereTracking(const std::map<std::string, std::vector<Point2D>>& detections);
    std::vector<SphereTrackingResult> getActiveSphereTracks() const;
    bool handoffObjectBetweenCameras(const std::string& object_id, 
                                   const std::string& from_camera,
                                   const std::string& to_camera);
    
    // Управление зум-камерой
    bool setZoomTarget(const Point3D& target_position, double zoom_level = 1.0);
    bool setZoomToTrackObject(const std::string& object_id);
    bool resetZoomCamera();
    
    // Автоматическое переключение зон
    bool enableAutoZoomZones(bool enable = true);
    bool addZoomZone(const std::string& zone_name, 
                    double min_azimuth, double max_azimuth,
                    double min_elevation, double max_elevation,
                    double zoom_level, int priority = 1);
    bool removeZoomZone(const std::string& zone_name);
    
    // Калибровка сферической системы
    bool startSphereCalibration();
    bool addSphereCalibrationPoint(const Point3D& world_point);
    bool finalizeSphereCalibration();
    
    // Статистика и мониторинг
    struct SphereStats {
        int total_objects_tracked;
        int successful_handoffs;
        int failed_handoffs;
        double average_tracking_accuracy;
        std::map<std::string, int> camera_usage_stats;
    };
    SphereStats getSphereStats() const;
    
    // Калибровка системы (существующие методы)
    bool startCalibration();
    bool addCalibrationPoint(const Point3D& world_point, 
                           const std::map<std::string, Point2D>& camera_points);
    bool finalizeCalibration();
    bool isCalibrated() const { return is_calibrated_; }

    // Установка статуса калибровки и обновление глобального трекера
    void setCalibrated(bool calibrated, GlobalTracker* tracker = nullptr);

    // Управление зонами покрытия для сферы/полусферы
    bool setupSphereZones();
    bool setupHemisphereZones();
    bool validateCoverage();
    
    // Утилиты
    std::string schemeTypeToString(SchemeType type);
    SchemeType stringToSchemeType(const std::string& str);
    std::string roleToString(CameraRole role);
    CameraRole stringToRole(const std::string& str);
    
    // Конфигурация трекинга
    GlobalTrackingConfig getTrackingConfig() const { return tracking_config_; }
    void setTrackingConfig(const GlobalTrackingConfig& config) { tracking_config_ = config; }
};
#endif