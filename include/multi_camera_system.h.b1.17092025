
#pragma once

#include "camera_manager.h"
#include "camera_roles.h"
#include "global_tracker.h"
#include <memory>
#include <atomic>
#include <thread>

namespace MultiCamera {

class MultiCameraSystem {
public:
    MultiCameraSystem();
    ~MultiCameraSystem();
    
    // Инициализация системы
    bool initialize(const std::string& config_file = "");
    bool shutdown();
    
    // Управление конфигурацией
    bool setConfiguration(SystemConfiguration config);
    SystemConfiguration getCurrentConfiguration() const;
    
    // Управление камерами
    bool addCamera(const std::string& camera_id, const std::string& device_path, CameraRole role);
    bool removeCamera(const std::string& camera_id);
    bool activateCamera(const std::string& camera_id);
    bool deactivateCamera(const std::string& camera_id);
    
    // Калибровка системы
    bool startCalibration();
    bool stopCalibration();
    bool isCalibrationRunning() const;
    
    // Трекинг объектов
    bool startTracking();
    bool stopTracking();
    bool isTrackingRunning() const;
    
    // Получение данных
    std::vector<CameraRoleConfig> getCameraConfigurations() const;
    std::vector<std::string> getActiveCameras() const;
    
    // Статус системы
    bool isSystemReady() const;
    std::string getSystemStatus() const;
    
    // События системы
    struct SystemEvent {
        enum Type {
            CAMERA_CONNECTED,
            CAMERA_DISCONNECTED,
            CALIBRATION_STARTED,
            CALIBRATION_COMPLETED,
            CALIBRATION_FAILED,
            TRACKING_STARTED,
            TRACKING_STOPPED,
            CONFIGURATION_CHANGED
        };
        
        Type type;
        std::string camera_id;
        std::string message;
        std::chrono::steady_clock::time_point timestamp;
    };
    
    // Получение последних событий
    std::vector<SystemEvent> getRecentEvents(size_t max_count = 10) const;
    
private:
    std::unique_ptr<CameraManager> camera_manager_;
    std::unique_ptr<CameraRoleManager> role_manager_;
    std::unique_ptr<GlobalTracker> global_tracker_;
    
    std::atomic<bool> calibration_running_{false};
    std::atomic<bool> tracking_running_{false};
    std::atomic<bool> system_initialized_{false};
    
    std::thread calibration_thread_;
    std::thread tracking_thread_;
    
    mutable std::mutex events_mutex_;
    std::deque<SystemEvent> recent_events_;
    const size_t max_events_{100};
    
    void calibrationLoop();
    void trackingLoop();
    void addEvent(SystemEvent::Type type, const std::string& camera_id = "", 
                  const std::string& message = "");
    
    bool validateSystemConfiguration() const;
    void setupCameraCallbacks();
};

} // namespace MultiCamera
