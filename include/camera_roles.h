
#pragma once

#include <string>
#include <vector>
#include <map>
#include <memory>
#include "nlohmann/json.hpp"

namespace MultiCamera {

enum class CameraRole {
    WIDE_ANGLE,      // Широкофокусная камера
    ZOOM,            // Камера с зумом
    ZOOM_VARIABLE    // Камера с переменным зумом
};

enum class SystemConfiguration {
    SPHERE,          // 2 широкофокусные + 1 зум
    HEMISPHERE,      // 1 широкофокусная + 1 зум
    HEMISPHERE_MULTI // 1 широкофокусная + N зум камер
};

struct CameraRoleConfig {
    std::string camera_id;
    CameraRole role;
    int priority;                    // Приоритет камеры в системе
    bool supports_color;             // Поддержка цветного режима
    bool supports_bw;                // Поддержка ч/б режима
    bool is_active;                  // Активна ли камера
    
    // Параметры для зум-камер
    double min_zoom = 1.0;
    double max_zoom = 1.0;
    double current_zoom = 1.0;
    
    // Позиция и ориентация
    struct Position {
        double x, y, z;
        double roll, pitch, yaw;
    } position;
    
    // Область покрытия (field of view)
    struct FOV {
        double horizontal_deg;
        double vertical_deg;
    } fov;
};

class CameraRoleManager {
public:
    CameraRoleManager();
    ~CameraRoleManager();
    
    // Загрузка конфигурации системы
    bool loadSystemConfiguration(SystemConfiguration config_type, 
                                const std::string& config_file = "");
    
    // Управление ролями камер
    bool assignRole(const std::string& camera_id, CameraRole role);
    bool assignRole(const std::string& camera_id, const std::string& role);
    nlohmann::json getRoles() const;
    bool removeRole(const std::string& camera_id);
    CameraRole getCameraRole(const std::string& camera_id) const;
    
    // Получение списка камер по роли
    std::vector<std::string> getCamerasByRole(CameraRole role) const;
    std::vector<CameraRoleConfig> getAllCameraConfigs() const;
    
    // Проверка совместимости конфигурации
    bool validateConfiguration() const;
    bool isConfigurationComplete() const;
    
    // Управление зумом
    bool setZoom(const std::string& camera_id, double zoom_level);
    double getZoom(const std::string& camera_id) const;
    
    // Переключение цветовых режимов
    bool setColorMode(const std::string& camera_id, bool color_enabled);
    bool getColorMode(const std::string& camera_id) const;
    
    // Получение текущей конфигурации системы
    SystemConfiguration getCurrentConfiguration() const;
    
    // Получение основной (primary) камеры для трекинга
    std::string getPrimaryCameraId() const;
    
    // Получение камер для стерео-пар
    std::vector<std::pair<std::string, std::string>> getStereoPairs() const;

private:
    SystemConfiguration current_config_;
    std::map<std::string, CameraRoleConfig> camera_roles_;
    std::string primary_camera_id_;
    
    bool validateSphereConfiguration() const;
    bool validateHemisphereConfiguration() const;
    bool validateHemisphereMultiConfiguration() const;
    
    void setupDefaultConfiguration();
};

} // namespace MultiCamera
