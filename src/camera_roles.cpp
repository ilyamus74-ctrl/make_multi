#include "camera_roles.h"
#include "camera_manager.h"
#include <algorithm>
#include <cctype>
#include <iostream>
#include <fstream>
#include <string_view>
#include "nlohmann/json.hpp"

extern CameraManager g_camera_manager;

using json = nlohmann::json;

namespace MultiCamera {

namespace {

bool hasNumericSuffix(std::string_view id, std::string_view prefix) {
    if (id.rfind(prefix, 0) != 0 || id.size() <= prefix.size()) {
        return false;
    }
    return std::all_of(id.begin() + prefix.size(), id.end(), [](unsigned char ch) {
        return std::isdigit(ch) != 0;
    });
}

bool isPlaceholderId(const std::string& id) {
    static constexpr std::string_view kWidePrefix{"cam_wide_"};
    static constexpr std::string_view kZoomPrefix{"cam_zoom_"};
    return hasNumericSuffix(id, kWidePrefix) || hasNumericSuffix(id, kZoomPrefix);
}

std::vector<std::string> activeRealCamerasByRole(
        const std::map<std::string, CameraRoleConfig>& camera_roles,
        CameraRole role) {
    std::vector<std::string> result;
    for (const auto& [id, cfg] : camera_roles) {
        if (!cfg.is_active || cfg.role != role || isPlaceholderId(id)) {
            continue;
        }
        result.push_back(id);
    }
    return result;
}

bool hasMinimumRealCamerasForValidation(
        const std::map<std::string, CameraRoleConfig>& camera_roles,
        SystemConfiguration config) {
    auto wide = activeRealCamerasByRole(camera_roles, CameraRole::WIDE_ANGLE);
    auto zoom = activeRealCamerasByRole(camera_roles, CameraRole::ZOOM);
    auto zoom_variable =
            activeRealCamerasByRole(camera_roles, CameraRole::ZOOM_VARIABLE);

    switch (config) {
        case SystemConfiguration::SPHERE:
            return wide.size() >= 2 && zoom.size() >= 1;
        case SystemConfiguration::HEMISPHERE:
            return wide.size() >= 1 && zoom.size() >= 1;
        case SystemConfiguration::HEMISPHERE_MULTI:
            return wide.size() >= 1 && (zoom.size() + zoom_variable.size()) >= 1;
        default:
            return false;
    }
}

void deactivatePlaceholdersForRole(
        std::map<std::string, CameraRoleConfig>& camera_roles,
        CameraRole role,
        const std::string& active_camera_id) {
    for (auto& [id, cfg] : camera_roles) {
        if (id == active_camera_id) {
            continue;
        }
        if (cfg.role == role && isPlaceholderId(id)) {
            cfg.is_active = false;
        }
    }
}

} // namespace

CameraRoleManager::CameraRoleManager()
    : current_config_(SystemConfiguration::HEMISPHERE) {
    setupDefaultConfiguration();
}

CameraRoleManager::~CameraRoleManager() = default;

bool CameraRoleManager::loadSystemConfiguration(SystemConfiguration config_type, 
                                               const std::string& config_file) {
    current_config_ = config_type;
    
    if (!config_file.empty()) {
        try {
            std::ifstream file(config_file);
            if (!file.is_open()) {
                std::cerr << "Cannot open config file: " << config_file << std::endl;
                return false;
            }
            
            json config;
            file >> config;
            
            // Загрузка конфигурации из JSON
            if (config.contains("cameras")) {
                camera_roles_.clear();
                
                for (const auto& cam_config : config["cameras"]) {
                    CameraRoleConfig role_config;
                    role_config.camera_id = cam_config["id"];
                    
                    std::string role_str = cam_config["role"];
                    if (role_str == "wide_angle") {
                        role_config.role = CameraRole::WIDE_ANGLE;
                    } else if (role_str == "zoom") {
                        role_config.role = CameraRole::ZOOM;
                    } else if (role_str == "zoom_variable") {
                        role_config.role = CameraRole::ZOOM_VARIABLE;
                    }
                    
                    role_config.priority = cam_config.value("priority", 0);
                    role_config.supports_color = cam_config.value("supports_color", true);
                    role_config.supports_bw = cam_config.value("supports_bw", true);
//                    role_config.is_active = cam_config.value("is_active", false);
                    role_config.is_active = cam_config.value("is_active", true);
                    
                    if (cam_config.contains("zoom")) {
                        role_config.min_zoom = cam_config["zoom"].value("min", 1.0);
                        role_config.max_zoom = cam_config["zoom"].value("max", 1.0);
                        role_config.current_zoom = cam_config["zoom"].value("current", 1.0);
                    }
                    
                    if (cam_config.contains("position")) {
                        auto pos = cam_config["position"];
                        role_config.position.x = pos.value("x", 0.0);
                        role_config.position.y = pos.value("y", 0.0);
                        role_config.position.z = pos.value("z", 0.0);
                        role_config.position.roll = pos.value("roll", 0.0);
                        role_config.position.pitch = pos.value("pitch", 0.0);
                        role_config.position.yaw = pos.value("yaw", 0.0);
                    }
                    
                    if (cam_config.contains("fov")) {
                        auto fov = cam_config["fov"];
                        role_config.fov.horizontal_deg = fov.value("horizontal", 60.0);
                        role_config.fov.vertical_deg = fov.value("vertical", 45.0);
                    }
                    
                    camera_roles_[role_config.camera_id] = role_config;
                }
            }
            
            if (config.contains("primary_camera")) {
                primary_camera_id_ = config["primary_camera"];
            }
            
        } catch (const std::exception& e) {
            std::cerr << "Error loading configuration: " << e.what() << std::endl;
            return false;
        }
    } else {
        setupDefaultConfiguration();
    }
    
    return validateConfiguration();
}

bool CameraRoleManager::assignRole(const std::string& camera_id, CameraRole role) {
    if (camera_roles_.find(camera_id) != camera_roles_.end()) {
//        camera_roles_[camera_id].role = role;
        auto& existing_config = camera_roles_[camera_id];
        existing_config.role = role;
        existing_config.is_active = true;
    } else {
        CameraRoleConfig config;
        config.camera_id = camera_id;
        config.role = role;
        config.priority = 0;
        config.supports_color = true;
        config.supports_bw = true;
        //config.is_active = false;
        config.is_active = true;
        
        // Установка базовых параметров в зависимости от роли
        switch (role) {
            case CameraRole::WIDE_ANGLE:
                config.fov.horizontal_deg = 120.0;
                config.fov.vertical_deg = 90.0;
                config.min_zoom = 1.0;
                config.max_zoom = 1.0;
                break;
            case CameraRole::ZOOM:
                config.fov.horizontal_deg = 30.0;
                config.fov.vertical_deg = 22.5;
                config.min_zoom = 1.0;
                config.max_zoom = 10.0;
                break;
            case CameraRole::ZOOM_VARIABLE:
                config.fov.horizontal_deg = 60.0;
                config.fov.vertical_deg = 45.0;
                config.min_zoom = 1.0;
                config.max_zoom = 20.0;
                break;
        }

        camera_roles_[camera_id] = config;
    }

    if (!isPlaceholderId(camera_id)) {
        deactivatePlaceholdersForRole(camera_roles_, role, camera_id);
    }

    if (!hasMinimumRealCamerasForValidation(camera_roles_, current_config_)) {
        return true;
    }

    return validateConfiguration();
}

bool CameraRoleManager::assignRole(const std::string& camera_id,
                                   const std::string& role) {
    CameraRole r = CameraRole::WIDE_ANGLE;
    if (role == "zoom") {
        r = CameraRole::ZOOM;
    } else if (role == "zoom_variable") {
        r = CameraRole::ZOOM_VARIABLE;
    }
    return assignRole(camera_id, r);
}

nlohmann::json CameraRoleManager::getRoles() const {
    nlohmann::json out = nlohmann::json::array();
    for (const auto& [id, cfg] : camera_roles_) {
        if (!cfg.is_active) {
            continue;
        }
        std::string role = "wide_angle";
        if (cfg.role == CameraRole::ZOOM)
            role = "zoom";
        else if (cfg.role == CameraRole::ZOOM_VARIABLE)
            role = "zoom_variable";
        out.push_back({{"id", id}, {"role", role}});
    }
    return out;
}


bool CameraRoleManager::removeRole(const std::string& camera_id) {
    auto it = camera_roles_.find(camera_id);
    if (it != camera_roles_.end()) {
        camera_roles_.erase(it);
        if (primary_camera_id_ == camera_id) {
            primary_camera_id_.clear();
        }
        return true;
    }
    return false;
}

CameraRole CameraRoleManager::getCameraRole(const std::string& camera_id) const {
    auto it = camera_roles_.find(camera_id);
    if (it != camera_roles_.end()) {
        return it->second.role;
    }
    return CameraRole::WIDE_ANGLE; // По умолчанию
}

std::vector<std::string> CameraRoleManager::getCamerasByRole(CameraRole role) const {
    std::vector<std::string> result;
    for (const auto& pair : camera_roles_) {
        if (pair.second.role == role && pair.second.is_active) {
            result.push_back(pair.first);
        }
    }
    return result;
}

std::vector<CameraRoleConfig> CameraRoleManager::getAllCameraConfigs() const {
    std::vector<CameraRoleConfig> result;
    for (const auto& pair : camera_roles_) {
        result.push_back(pair.second);
    }
    return result;
}

bool CameraRoleManager::validateConfiguration() const {
    switch (current_config_) {
        case SystemConfiguration::SPHERE:
            return validateSphereConfiguration();
        case SystemConfiguration::HEMISPHERE:
            return validateHemisphereConfiguration();
        case SystemConfiguration::HEMISPHERE_MULTI:
            return validateHemisphereMultiConfiguration();
        default:
            return false;
    }
}

bool CameraRoleManager::validateSphereConfiguration() const {
    auto wide_cameras = activeRealCamerasByRole(camera_roles_, CameraRole::WIDE_ANGLE);
    auto zoom_cameras = activeRealCamerasByRole(camera_roles_, CameraRole::ZOOM);

    return (wide_cameras.size() == 2 && zoom_cameras.size() == 1);
}

bool CameraRoleManager::validateHemisphereConfiguration() const {
    auto wide_cameras = getCamerasByRole(CameraRole::WIDE_ANGLE);
    auto zoom_cameras = getCamerasByRole(CameraRole::ZOOM);
    
    return (wide_cameras.size() == 1 && zoom_cameras.size() == 1);
}

bool CameraRoleManager::validateHemisphereMultiConfiguration() const {
    auto wide_cameras = activeRealCamerasByRole(camera_roles_, CameraRole::WIDE_ANGLE);
    auto zoom_cameras = activeRealCamerasByRole(camera_roles_, CameraRole::ZOOM);
    auto zoom_variable_cameras =
            activeRealCamerasByRole(camera_roles_, CameraRole::ZOOM_VARIABLE);

    return (wide_cameras.size() == 1 &&
            (zoom_cameras.size() + zoom_variable_cameras.size()) >= 1);
}

bool CameraRoleManager::isConfigurationComplete() const {
    return validateConfiguration() && !primary_camera_id_.empty();
}

bool CameraRoleManager::setZoom(const std::string& camera_id, double zoom_level) {
    auto it = camera_roles_.find(camera_id);
    if (it != camera_roles_.end()) {
        auto& config = it->second;
        if (config.role == CameraRole::ZOOM || config.role == CameraRole::ZOOM_VARIABLE) {
            config.current_zoom = std::clamp(zoom_level, config.min_zoom, config.max_zoom);
            return true;
        }
    }
    return false;
}

double CameraRoleManager::getZoom(const std::string& camera_id) const {
    auto it = camera_roles_.find(camera_id);
    if (it != camera_roles_.end()) {
        return it->second.current_zoom;
    }
    return 1.0;
}

bool CameraRoleManager::setColorMode(const std::string& camera_id, bool color_enabled) {
    auto it = camera_roles_.find(camera_id);
    if (it != camera_roles_.end()) {
        // Проверяем поддержку режима
        if (color_enabled && !it->second.supports_color) return false;
        if (!color_enabled && !it->second.supports_bw) return false;

        auto fmt = color_enabled ? CameraManager::PixelFormat::RGB
                                 : CameraManager::PixelFormat::GRAY;
        return g_camera_manager.setPixelFormat(camera_id, fmt);

    }
    return false;
}

bool CameraRoleManager::getColorMode(const std::string& camera_id) const {
    return g_camera_manager.getPixelFormat(camera_id) ==
           CameraManager::PixelFormat::RGB;
}

SystemConfiguration CameraRoleManager::getCurrentConfiguration() const {
    return current_config_;
}

std::string CameraRoleManager::getPrimaryCameraId() const {
    if (!primary_camera_id_.empty()) {
        return primary_camera_id_;
    }
    
    // Автоматически выбираем primary камеру
    auto wide_cameras = getCamerasByRole(CameraRole::WIDE_ANGLE);
    if (!wide_cameras.empty()) {
        return wide_cameras[0];
    }
    
    auto zoom_cameras = getCamerasByRole(CameraRole::ZOOM);
    if (!zoom_cameras.empty()) {
        return zoom_cameras[0];
    }
    
    return "";
}

std::vector<std::pair<std::string, std::string>> CameraRoleManager::getStereoPairs() const {
    std::vector<std::pair<std::string, std::string>> pairs;
    
    auto wide_cameras = getCamerasByRole(CameraRole::WIDE_ANGLE);
    
    // Для конфигурации SPHERE создаем стерео-пару из двух широкофокусных камер
    if (current_config_ == SystemConfiguration::SPHERE && wide_cameras.size() >= 2) {
        pairs.emplace_back(wide_cameras[0], wide_cameras[1]);
    }
    
    return pairs;
}

void CameraRoleManager::setupDefaultConfiguration() {
    camera_roles_.clear();
    
    switch (current_config_) {
        case SystemConfiguration::SPHERE: {
            // 2 широкофокусные + 1 зум
            assignRole("cam_wide_0", CameraRole::WIDE_ANGLE);
            assignRole("cam_wide_1", CameraRole::WIDE_ANGLE);
            assignRole("cam_zoom_0", CameraRole::ZOOM);
            primary_camera_id_ = "cam_wide_0";
            break;
        }
        case SystemConfiguration::HEMISPHERE: {
            // 1 широкофокусная + 1 зум
            assignRole("cam_wide_0", CameraRole::WIDE_ANGLE);
            assignRole("cam_zoom_0", CameraRole::ZOOM);
            primary_camera_id_ = "cam_wide_0";
            break;
        }
        case SystemConfiguration::HEMISPHERE_MULTI: {
            // 1 широкофокусная + N зум
            assignRole("cam_wide_0", CameraRole::WIDE_ANGLE);
            assignRole("cam_zoom_0", CameraRole::ZOOM);
            assignRole("cam_zoom_1", CameraRole::ZOOM_VARIABLE);
            primary_camera_id_ = "cam_wide_0";
            break;
        }
    }
}

} // namespace MultiCamera
