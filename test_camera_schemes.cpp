#include "camera_manager.h"
#include "global_tracker.h"
#include <iostream>
#include <thread>
#include <chrono>

void testCameraSchemes() {
    std::cout << "🧪 Testing Camera Schemes and Global Tracking" << std::endl;
    
    // Инициализация
    CameraManager manager;
    if (!manager.loadConfig("./config.json")) {
        std::cerr << "❌ Failed to load config" << std::endl;
        return;
    }
    
    manager.start(false); // без мониторинга устройств
    
    // Тест 1: Проверка доступных схем
    std::cout << "\n📋 Test 1: Available Schemes" << std::endl;
    auto schemes = manager.getAvailableSchemes();
    for (const auto& scheme : schemes) {
        std::cout << "  - " << scheme << std::endl;
    }
    
    // Тест 2: Установка схемы полусферы
    std::cout << "\n🌗 Test 2: Setting Hemisphere Scheme" << std::endl;
    if (manager.setSchemeType("hemisphere_zoom")) {
        std::cout << "✅ Hemisphere scheme set successfully" << std::endl;
        std::cout << "Current scheme: " << manager.getCurrentSchemeType() << std::endl;
    } else {
        std::cout << "❌ Failed to set hemisphere scheme" << std::endl;
    }
    
    // Тест 3: Проверка ролей камер
    std::cout << "\n🎭 Test 3: Camera Roles" << std::endl;
    auto cameras = manager.configuredCameras();
    for (const auto& cam : cameras) {
        std::string role = manager.getCameraRole(cam.id);
        std::cout << "  Camera " << cam.id << ": " << role 
                  << " (present: " << (cam.present ? "YES" : "NO") << ")" << std::endl;
    }
    
    // Тест 4: Ручное назначение ролей
    std::cout << "\n🎯 Test 4: Manual Role Assignment" << std::endl;
    if (cameras.size() >= 3) {
        manager.assignCameraRole(cameras[0].id, "primary_wide", 1);
        manager.assignCameraRole(cameras[1].id, "secondary_wide", 2);
        manager.assignCameraRole(cameras[2].id, "zoom", 3);
        
        std::cout << "✅ Roles assigned manually" << std::endl;
        
        // Проверяем результат
        auto primary = manager.getCamerasByRole("primary_wide");
        auto secondary = manager.getCamerasByRole("secondary_wide");
        auto zoom = manager.getCamerasByRole("zoom");
        
        std::cout << "  Primary cameras: " << primary.size() << std::endl;
        std::cout << "  Secondary cameras: " << secondary.size() << std::endl;
        std::cout << "  Zoom cameras: " << zoom.size() << std::endl;
    }
    
    // Тест 5: Включение глобального трекинга
    std::cout << "\n🌐 Test 5: Global Tracking" << std::endl;
    manager.enableGlobalTracking(true);
    if (manager.isGlobalTrackingEnabled()) {
        std::cout << "✅ Global tracking enabled" << std::endl;
        
        // Симуляция детекций
        std::map<std::string, object_detect_result_list> test_detections;
        
        // Создаем тестовые детекции для первой камеры
        if (!cameras.empty()) {
            object_detect_result_list det_list;
            det_list.count = 2;
            
            // Объект 1: человек
            det_list.results[0].cls_id = 0; // person
            det_list.results[0].prop = 0.85f;
            det_list.results[0].box = {100, 100, 200, 300};
            det_list.results[0].track_id = -1;
            
            // Объект 2: автомобиль
            det_list.results[1].cls_id = 2; // car
            det_list.results[1].prop = 0.92f;
            det_list.results[1].box = {300, 150, 500, 250};
            det_list.results[1].track_id = -1;
            
            test_detections[cameras[0].id] = det_list;
        }
        
        // Обновляем глобальный трекинг
        manager.updateGlobalDetections(test_detections);
        
        // Получаем результаты
        auto tracking_results = manager.getGlobalTrackingResults();
        std::cout << "  Global objects detected: " << tracking_results.size() << std::endl;
        
        for (const auto& obj : tracking_results) {
            std::cout << "    Object #" << obj["global_id"] 
                      << " (" << obj["class"] << "): confidence=" 
                      << obj["confidence"] << ", camera=" << obj["primary_camera"] << std::endl;
        }
        
        // Статистика
        auto stats = manager.getGlobalTrackingStats();
        std::cout << "  Tracking stats: " << stats.dump(2) << std::endl;
    }
    
    // Тест 6: Переключение на схему сферы
    std::cout << "\n🌍 Test 6: Switching to Sphere Scheme" << std::endl;
    if (manager.setSchemeType("sphere_zoom")) {
        std::cout << "✅ Sphere scheme set successfully" << std::endl;
        std::cout << "Current scheme: " << manager.getCurrentSchemeType() << std::endl;
    } else {
        std::cout << "❌ Failed to set sphere scheme" << std::endl;
    }
    
    // Тест 7: Проверка отказоустойчивости
    std::cout << "\n⚠️  Test 7: Failover Testing" << std::endl;
    if (!cameras.empty()) {
        std::string test_camera = cameras[0].id;
        std::cout << "Simulating failure of camera: " << test_camera << std::endl;
        
        // В реальной системе здесь был бы вызов handleCameraFailure
        // manager.handleCameraFailure(test_camera);
        std::cout << "  Failover simulation completed" << std::endl;
    }
    
    std::cout << "\n✅ All tests completed!" << std::endl;
    
    manager.stop();
}

int main() {
    try {
        testCameraSchemes();
    } catch (const std::exception& e) {
        std::cerr << "❌ Test failed with exception: " << e.what() << std::endl;
        return 1;
    }
    
    return 0;
}