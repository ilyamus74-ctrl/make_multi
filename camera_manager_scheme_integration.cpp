#include <nlohmann/json.hpp>
using json = nlohmann::json;

// Новые структуры для схем камер
struct CameraRole {
    std::string role;        // "primary_wide", "secondary_wide", "zoom"
    int priority;           // 1 = высший приоритет
    std::string zone;       // "main_area", "backup_area", "detail_area"
    bool is_active;         // статус камеры
};

class CameraSchemeManager {
private:
    std::string current_scheme_;
    std::map<int, CameraRole> camera_roles_;
    std::map<std::string, std::string> failover_map_;
    
public:
    bool loadSchemeFromConfig(const std::string& config_path) {
        std::ifstream file(config_path);
        if (!file.is_open()) {
            std::cerr << "❌ Не удается загрузить конфиг: " << config_path << std::endl;
            return false;
        }
        
        json config;
        file >> config;
        
        // Загружаем схему
        current_scheme_ = config.value("setup_type", "hemisphere_zoom");
        std::cout << "🎯 Схема камер: " << current_scheme_ << std::endl;
        
        // Загружаем роли камер
        if (config.contains("cameras")) {
            for (const auto& cam : config["cameras"]) {
                int cam_id = std::stoi(cam["id"].get<std::string>().substr(4)); // cam_0 -> 0
                
                CameraRole role;
                role.role = cam.value("role", "disabled");
                role.priority = cam.value("priority", 99);
                role.zone = cam["coverage"].value("zone", "unknown");
                role.is_active = true;
                
                camera_roles_[cam_id] = role;
                
                std::cout << "📷 Camera " << cam_id 
                         << " -> роль: " << role.role 
                         << ", приоритет: " << role.priority << std::endl;
            }
        }
        
        // Загружаем логику failover
        if (config.contains("schemes") && config["schemes"].contains(current_scheme_)) {
            auto scheme = config["schemes"][current_scheme_];
            if (scheme.contains("failover")) {
                for (auto& [key, value] : scheme["failover"].items()) {
                    failover_map_[key] = value;
                }
            }
        }
        
        return true;
    }
    
    // Получить главную камеру по роли
    int getPrimaryCameraByRole(const std::string& role) {
        int best_camera = -1;
        int best_priority = 999;
        
        for (const auto& [cam_id, cam_role] : camera_roles_) {
            if (cam_role.role == role && cam_role.is_active && cam_role.priority < best_priority) {
                best_camera = cam_id;
                best_priority = cam_role.priority;
            }
        }
        
        return best_camera;
    }
    
    // Обработка отказа камеры
    bool handleCameraFailure(int failed_camera_id) {
        if (camera_roles_.find(failed_camera_id) == camera_roles_.end()) {
            return false;
        }
        
        std::string failed_role = camera_roles_[failed_camera_id].role;
        camera_roles_[failed_camera_id].is_active = false;
        
        std::cout << "⚠️  Отказ камеры " << failed_camera_id 
                  << " (роль: " << failed_role << ")" << std::endl;
        
        // Ищем резервную камеру
        if (failed_role == "primary_wide") {
            int backup = getPrimaryCameraByRole("secondary_wide");
            if (backup != -1) {
                std::cout << "🔄 Переключение на резервную камеру: " << backup << std::endl;
                return true;
            }
        }
        
        return false;
    }
    
    // Получить активные камеры по приоритету
    std::vector<int> getActiveCamerasByPriority() {
        std::vector<std::pair<int, int>> cam_priority; // camera_id, priority
        
        for (const auto& [cam_id, role] : camera_roles_) {
            if (role.is_active) {
                cam_priority.push_back({cam_id, role.priority});
            }
        }
        
        // Сортируем по приоритету
        std::sort(cam_priority.begin(), cam_priority.end(),
            [](const auto& a, const auto& b) { return a.second < b.second; });
        
        std::vector<int> result;
        for (const auto& [cam_id, priority] : cam_priority) {
            result.push_back(cam_id);
        }
        
        return result;
    }
    
    std::string getCameraRole(int camera_id) {
        if (camera_roles_.find(camera_id) != camera_roles_.end()) {
            return camera_roles_[camera_id].role;
        }
        return "unknown";
    }
};