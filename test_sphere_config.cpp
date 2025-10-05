#include "camera_scheme.h"
#include <iostream>
#include <iomanip>

void printSeparator(const std::string& title) {
    std::cout << "\n" << std::string(60, '=') << std::endl;
    std::cout << "  " << title << std::endl;
    std::cout << std::string(60, '=') << std::endl;
}

void testSphereConfiguration() {
    printSeparator("🌐 ТЕСТ СФЕРИЧЕСКОЙ КОНФИГУРАЦИИ");
    
    CameraSchemeManager manager;
    
    // Тест 1: Инициализация с конфигурационным файлом
    std::cout << "\n📋 Тест 1: Загрузка конфигурации" << std::endl;
    if (manager.initialize("config_sphere_example.json")) {
        std::cout << "✅ Конфигурация загружена успешно" << std::endl;
        std::cout << "🎯 Текущая схема: " << manager.schemeTypeToString(manager.getCurrentScheme()) << std::endl;
    } else {
        std::cout << "❌ Ошибка загрузки конфигурации" << std::endl;
        return;
    }
    
    // Тест 2: Проверка камер
    std::cout << "\n📷 Тест 2: Проверка камер" << std::endl;
    auto cameras = manager.getCameras();
    for (const auto& cam : cameras) {
        std::cout << "  📹 " << cam.id << " (" << manager.roleToString(cam.role) << ")" << std::endl;
        std::cout << "      Устройство: " << cam.device << std::endl;
        std::cout << "      Разрешение: " << cam.width << "x" << cam.height << "@" << cam.fps << "fps" << std::endl;
        std::cout << "      Покрытие: " << cam.coverage.azimuth_start << "° - " << cam.coverage.azimuth_end << "°" << std::endl;
        std::cout << "      PTZ: " << (cam.is_ptz ? "Да" : "Нет") << std::endl;
    }
    
    // Тест 3: Валидация сферической конфигурации
    std::cout << "\n🔍 Тест 3: Валидация сферы" << std::endl;
    auto sphere_config = manager.getSphereConfiguration();
    std::cout << "  🌐 Радиус сферы: " << sphere_config.sphere_radius << "м" << std::endl;
    std::cout << "  📍 Центр сферы: (" << sphere_config.sphere_center.x << ", " 
              << sphere_config.sphere_center.y << ", " << sphere_config.sphere_center.z << ")" << std::endl;
    std::cout << "  🔗 Угол перекрытия: " << sphere_config.overlap_angle << "°" << std::endl;
    
    double total_coverage = manager.calculateTotalCoverage();
    std::cout << "  📊 Общее покрытие: " << std::fixed << std::setprecision(1) << total_coverage << "°" << std::endl;
    
    // Тест 4: Проверка непокрытых зон
    std::cout << "\n🕳️ Тест 4: Непокрытые зоны" << std::endl;
    auto uncovered = manager.getUncoveredAreas();
    if (uncovered.empty()) {
        std::cout << "✅ Полное покрытие достигнуто!" << std::endl;
    } else {
        std::cout << "⚠️ Найдены непокрытые зоны:" << std::endl;
        for (const auto& [start, end] : uncovered) {
            std::cout << "    🔴 " << std::fixed << std::setprecision(1) 
                      << start << "° - " << end << "° (размер: " << (end - start) << "°)" << std::endl;
        }
    }
    
    // Тест 5: Управление зонами зума
    std::cout << "\n🔍 Тест 5: Зоны автоматического зума" << std::endl;
    std::cout << "  📊 Количество зон: " << sphere_config.zoom_zones.size() << std::endl;
    for (const auto& zone : sphere_config.zoom_zones) {
        std::cout << "    📍 " << zone.zone_name << ":" << std::endl;
        std::cout << "        Азимут: " << zone.min_azimuth << "° - " << zone.max_azimuth << "°" << std::endl;
        std::cout << "        Элевация: " << zone.min_elevation << "° - " << zone.max_elevation << "°" << std::endl;
        std::cout << "        Зум: " << zone.zoom_level << "x, Приоритет: " << zone.priority << std::endl;
    }
    
    // Тест 6: Добавление новой зоны зума
    std::cout << "\n➕ Тест 6: Добавление зоны зума" << std::endl;
    if (manager.addZoomZone("test_zone", -45, 45, -20, 20, 2.5, 3)) {
        std::cout << "✅ Зона успешно добавлена" << std::endl;
    }
    
    // Тест 7: Симуляция трекинга
    std::cout << "\n🎯 Тест 7: Симуляция трекинга" << std::endl;
    std::map<std::string, std::vector<Point2D>> test_detections;
    test_detections["wide_primary"] = {{320, 240}, {450, 180}};
    test_detections["wide_secondary"] = {{640, 360}};
    
    if (manager.updateSphereTracking(test_detections)) {
        auto tracks = manager.getActiveSphereTracks();
        std::cout << "  📈 Активных треков: " << tracks.size() << std::endl;
        for (const auto& track : tracks) {
            std::cout << "    🎯 " << track.object_id << " (уверенность: " 
                      << std::fixed << std::setprecision(2) << track.confidence << ")" << std::endl;
            std::cout << "        Позиция: (" << track.world_position.x << ", " 
                      << track.world_position.y << ", " << track.world_position.z << ")" << std::endl;
            std::cout << "        Камеры: ";
            for (const auto& cam : track.visible_cameras) {
                std::cout << cam << " ";
            }
            std::cout << std::endl;
        }
    }
    
    // Тест 8: Управление зум-камерой
    std::cout << "\n🔭 Тест 8: Управление зум-камерой" << std::endl;
    Point3D target{5.0, 2.5, 1.0};
    if (manager.setZoomTarget(target, 3.0)) {
        std::cout << "✅ Зум-камера наведена на цель" << std::endl;
    }
    
    // Тест 9: Статистика
    std::cout << "\n📊 Тест 9: Статистика системы" << std::endl;
    auto stats = manager.getSphereStats();
    std::cout << "  🎯 Отслеживаемых объектов: " << stats.total_objects_tracked << std::endl;
    std::cout << "  ✅ Успешных передач: " << stats.successful_handoffs << std::endl;
    std::cout << "  ❌ Неудачных передач: " << stats.failed_handoffs << std::endl;
    std::cout << "  📈 Средняя точность: " << std::fixed << std::setprecision(1) 
              << (stats.average_tracking_accuracy * 100) << "%" << std::endl;
    
    std::cout << "  📷 Использование камер:" << std::endl;
    for (const auto& [camera_id, usage] : stats.camera_usage_stats) {
        std::cout << "    " << camera_id << ": " << usage << " объектов" << std::endl;
    }
    
    printSeparator("🎉 ТЕСТЫ ЗАВЕРШЕНЫ");
}

int main() {
    try {
        testSphereConfiguration();
        return 0;
    } catch (const std::exception& e) {
        std::cerr << "❌ Ошибка: " << e.what() << std::endl;
        return 1;
    }
}
