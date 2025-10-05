
#pragma once

#include "camera_manager.h"
#include "camera_roles.h"
#include "global_tracker.h"
#include <memory>
#include <atomic>
#include <thread>
#include <stdexcept>
#include <chrono>
#include <deque>


namespace MultiCamera {

// Configuration validation results
struct ConfigValidationResult {
    bool is_valid = false;
    std::vector<std::string> errors;
    std::vector<std::string> warnings;
};

class MultiCameraSystem {
public:
    MultiCameraSystem();
    ~MultiCameraSystem();

    // Инициализация системы с улучшенной валидацией
    bool initialize(const std::string& config_file = "");
    bool shutdown();

    // Управление конфигурацией с валидацией
    bool setConfiguration(SystemConfiguration config);
    SystemConfiguration getCurrentConfiguration() const;
    ConfigValidationResult validateConfiguration(const SystemConfiguration& config) const;

    // Управление камерами с проверкой параметров
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

    // Методы для graceful shutdown
    void requestShutdown();
    bool isShutdownRequested() const;

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
            CONFIGURATION_CHANGED,
            SYSTEM_ERROR,
            PARAMETER_VALIDATION_FAILED
        };

        Type type;
        std::string camera_id;
        std::string message;
        std::chrono::steady_clock::time_point timestamp;
        // Add severity levels for better error handling  
        enum Severity {
            INFO,
            WARNING,
            ERROR,
            CRITICAL
        } severity = INFO;
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
    std::atomic<bool> shutdown_requested_{false};

    std::thread calibration_thread_;
    std::thread tracking_thread_;
    
    mutable std::mutex events_mutex_;
    std::deque<SystemEvent> recent_events_;
    const size_t max_events_{100};

    // Thread-safe configuration access
    mutable std::mutex config_mutex_;
    SystemConfiguration current_config_;

    void calibrationLoop();
    void trackingLoop();
    void addEvent(SystemEvent::Type type, const std::string& camera_id = "", 
                  const std::string& message = "", SystemEvent::Severity severity = SystemEvent::INFO);

    bool validateSystemConfiguration() const;
    void setupCameraCallbacks();

    // Enhanced error handling methods
    bool handleSystemError(const std::string& error_message, SystemEvent::Severity severity = SystemEvent::ERROR);
    void cleanupResources();
    bool validateCameraConfiguration(const std::string& camera_id, const std::string& device_path, CameraRole role) const;
};

} // namespace MultiCamera
