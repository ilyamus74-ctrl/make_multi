#include "calibration_watcher.h"
#include <iostream>
#include <fstream>
#include <algorithm>
#include <chrono>
#include <thread>
#include <regex>
#include <ctime>
#include <iomanip>
#include <sstream>

CalibrationWatcher::CalibrationWatcher(
    const std::filesystem::path& record_dir,
    const std::filesystem::path& calib_dir
) : record_dir_(std::filesystem::absolute(record_dir)),
    calib_dir_(std::filesystem::absolute(calib_dir)),
    results_dir_(calib_dir_ / "results") {
    
    std::error_code ec;
    std::filesystem::create_directories(results_dir_, ec);
    if (ec) {
        logMessage("WARNING: Failed to create results directory: " + ec.message());
    }
}

CalibrationWatcher::~CalibrationWatcher() {
    stopCalibration();
}

bool CalibrationWatcher::startCalibration(const CalibrationParams& params) {
    if (processing_.load()) {
        logMessage("Calibration already in progress");
        return false;
    }
    
    processing_ = true;
    should_stop_ = false;
    // Используем все доступные ядра кроме одного для системы
    ///int num_threads = std::max(1, static_cast<int>(std::thread::hardware_concurrency()) - 1);
    ///cv::setNumThreads(num_threads);
    ///printf("Calibration using %d CPU threads\n", num_threads);
    // Оставляем настройки OpenCV по умолчанию для стабильности
    printf("Calibration using default OpenCV threading\n");
    progress_ = 0.0f;
    
    // Очистка предыдущих результатов
    {
        std::lock_guard<std::mutex> lock(results_mutex_);
        mono_results_.clear();
        stereo_results_.clear();
        camera_matrices_.clear();
        dist_coeffs_.clear();
    }
    
    updateStatus("Starting calibration process...", 0.0f);
    
    // Запуск рабочего потока
    worker_thread_ = std::thread(&CalibrationWatcher::calibrationWorker, this, params);
    
    return true;
}

bool CalibrationWatcher::startAutoCalibration(const std::string& scheme,
                                              const std::vector<std::string>& active_cameras,
                                              const CalibrationParams& params) {
    logMessage("Auto calibration requested for scheme: " + scheme);
    if (!active_cameras.empty()) {
        std::string cam_list;
        for (const auto& cam : active_cameras) {
            if (!cam_list.empty()) cam_list += ", ";
            cam_list += cam;
        }
        logMessage("Active cameras: " + cam_list);
    }
    return startCalibration(params);
}


void CalibrationWatcher::stopCalibration() {
    if (!processing_.load()) return;
    
    should_stop_ = true;
    updateStatus("Stopping calibration...", -1.0f);
    
    if (worker_thread_.joinable()) {
        worker_thread_.join();
    }
    
    processing_ = false;
    updateStatus("Calibration stopped", 100.0f);
}

void CalibrationWatcher::calibrationWorker(const CalibrationParams& params) {
    try {
        // 1. Сканируем записанные видео
        updateStatus("Scanning recorded videos...", 5.0f);
        auto video_files = scanRecordingFolder();
        
        if (video_files.empty()) {
            updateStatus("No video files found", 100.0f);
            processing_ = false;
            return;
        }
        
        logMessage("Found " + std::to_string(video_files.size()) + " video files");
        
        // 2. Извлекаем кадры из всех видео
        updateStatus("Extracting frames from videos...", 10.0f);
        auto processed_cameras = extractFramesFromAllVideos(params);
        
        if (processed_cameras.empty()) {
            updateStatus("No cameras processed successfully", 100.0f);
            processing_ = false;
            return;
        }
        
        if (should_stop_.load()) {
            updateStatus("Calibration stopped by user", 100.0f);
            processing_ = false;
            return;
        }
        
        // 3. Выполняем моно-калибровку для каждой камеры
        updateStatus("Performing mono calibrations...", 40.0f);
        std::vector<std::string> successful_cameras;
        
        for (size_t i = 0; i < processed_cameras.size() && !should_stop_.load(); ++i) {
            const auto& camera_id = processed_cameras[i];
            
            logMessage("Calibrating camera: " + camera_id);
            if (performMonoCalibration(camera_id, params)) {
                successful_cameras.push_back(camera_id);
                logMessage("Mono calibration successful for camera: " + camera_id);
            } else {
                logMessage("Mono calibration failed for camera: " + camera_id);
            }
            
            float progress = 40.0f + (30.0f * (i + 1)) / processed_cameras.size();
            updateStatus("Calibrating camera " + std::to_string(i + 1) + "/" + 
                        std::to_string(processed_cameras.size()), progress);
        }
        
        if (should_stop_.load()) {
            updateStatus("Calibration stopped by user", 100.0f);
            processing_ = false;
            return;
        }
        
        // 4. Выполняем стерео и мульти-камерную калибровку
        if (successful_cameras.size() >= 2) {
            updateStatus("Performing stereo/multi calibrations...", 75.0f);
            performMultiCameraCalibration(successful_cameras, params);
        }
        
        // 5. Сохраняем результаты
        updateStatus("Saving calibration results...", 95.0f);
        saveResults();
        
        updateStatus("Calibration completed successfully!", 100.0f);
        
    } catch (const std::exception& e) {
        updateStatus("Calibration failed: " + std::string(e.what()), 100.0f);
        logMessage("ERROR: " + std::string(e.what()));
    }
    
    processing_ = false;
}

std::vector<CalibrationWatcher::VideoFile> CalibrationWatcher::scanRecordingFolder() {
    std::vector<VideoFile> files;
    
    if (!std::filesystem::exists(record_dir_)) {
        logMessage("Recording directory does not exist: " + record_dir_.string());
        return files;
    }
    
    // Паттерн для файлов записи: ID_камеры_TIMESTAMP_разрешение.avi или просто ID_timestamp.avi
    std::regex video_pattern(R"(([^_]+)_.*\.avi)");
    
    for (const auto& entry : std::filesystem::directory_iterator(record_dir_)) {
        if (!entry.is_regular_file()) continue;
        
        auto path = entry.path();
        if (path.extension() != ".avi") continue;
        
        std::string filename = path.filename().string();
        std::smatch match;
        
        if (std::regex_match(filename, match, video_pattern)) {
            VideoFile video;
            video.camera_id = match[1].str();
            video.path = path;
            
            std::error_code ec;
            video.file_size = std::filesystem::file_size(path, ec);
            if (ec) continue;
            
            auto file_time = std::filesystem::last_write_time(path, ec);
            if (ec) continue;
            
            // Конвертируем в system_clock
            auto sctp = std::chrono::time_point_cast<std::chrono::system_clock::duration>(
                file_time - std::filesystem::file_time_type::clock::now() + 
                std::chrono::system_clock::now());
            video.last_modified = std::chrono::system_clock::to_time_t(sctp);
            
            files.push_back(video);
        }
    }
    
    return files;
}

bool CalibrationWatcher::isVideoComplete(const VideoFile& video) {
    // Проверяем, что файл не изменялся последние 3 секунды
    auto current_time = std::time(nullptr);
    if (current_time - video.last_modified < 3) {
        return false;
    }
    
    // Проверяем размер файла
    try {
        std::error_code ec;
        auto current_size = std::filesystem::file_size(video.path, ec);
        if (ec || current_size != video.file_size) {
            return false;
        }
        
        // Минимальный размер файла (1MB)
        if (current_size < 1024 * 1024) {
            return false;
        }
        
    } catch (const std::exception&) {
        return false;
    }
    
    return true;
}

std::vector<std::string> CalibrationWatcher::extractFramesFromAllVideos(const CalibrationParams& params) {
    auto video_files = scanRecordingFolder();
    std::vector<std::string> processed_cameras;
    
    for (size_t i = 0; i < video_files.size() && !should_stop_.load(); ++i) {
        const auto& video = video_files[i];
        
        if (!isVideoComplete(video)) {
            logMessage("Skipping incomplete video: " + video.path.string());
            continue;
        }
        
        logMessage("Processing video for camera: " + video.camera_id);
        
        if (extractFramesFromVideo(video, params)) {
            processed_cameras.push_back(video.camera_id);
            
            // Удаляем видеофайл если требуется
            if (params.delete_videos) {
                std::error_code ec;
                std::filesystem::remove(video.path, ec);
                if (!ec) {
                    logMessage("Deleted video file: " + video.path.string());
                }
            }
        }
        
        float progress = 10.0f + (25.0f * (i + 1)) / video_files.size();
        updateStatus("Processing video " + std::to_string(i + 1) + "/" + 
                    std::to_string(video_files.size()), progress);
    }
    
    return processed_cameras;
}

bool CalibrationWatcher::extractFramesFromVideo(const VideoFile& video, const CalibrationParams& params) {
    std::filesystem::path cam_dir = calib_dir_ / ("cam_" + video.camera_id) / "images";
    std::error_code ec;
    std::filesystem::create_directories(cam_dir, ec);
    
    if (ec) {
        logMessage("Failed to create camera directory: " + ec.message());
        return false;
    }
    
    cv::VideoCapture cap(video.path.string());
    if (!cap.isOpened()) {
        logMessage("Failed to open video file: " + video.path.string());
        return false;
    }
    
    cv::Mat frame;
    int frame_count = 0;
    int saved_count = 0;
    
    // Получаем общее количество кадров
    int total_frames = static_cast<int>(cap.get(cv::CAP_PROP_FRAME_COUNT));
    int skip_interval = std::max(1, total_frames / params.max_frames);
    
    logMessage("Processing video: " + std::to_string(total_frames) + 
               " frames, skip interval: " + std::to_string(skip_interval));
    
    while (cap.read(frame) && saved_count < params.max_frames && !should_stop_.load()) {
        // Пропускаем кадры для равномерного распределения
        if (frame_count % skip_interval != 0) {
            frame_count++;
            continue;
        }
        
        // Оцениваем качество кадра
        FrameQuality quality = evaluateFrameQuality(frame, params);
        
        if (quality.board_detected && quality.overall_score >= params.quality_threshold) {
            std::string filename = "frame_" + std::to_string(saved_count) + 
                                 "_score_" + std::to_string(static_cast<int>(quality.overall_score)) + ".png";
            
            std::filesystem::path filepath = cam_dir / filename;
            
            if (cv::imwrite(filepath.string(), frame)) {
                saved_count++;
                logMessage("Saved quality frame: " + filename + 
                          " (corners: " + std::to_string(quality.corners_found) + 
                          ", score: " + std::to_string(quality.overall_score) + ")");
            }
        }
        
        frame_count++;
    }
    
    cap.release();
    
    logMessage("Extracted " + std::to_string(saved_count) + " quality frames from " + 
               std::to_string(frame_count) + " total frames for camera " + video.camera_id);
    
    return saved_count >= params.min_frames;
}

CalibrationWatcher::FrameQuality CalibrationWatcher::evaluateFrameQuality(
    const cv::Mat& frame, const CalibrationParams& params) {
    
    FrameQuality quality;
    
    cv::Mat gray;
    cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
    
    // Оценка резкости
    quality.sharpness = calculateSharpness(gray);
    
    // Оценка контрастности
    quality.contrast = calculateContrast(gray);
    
    // Поиск шахматной доски
    std::vector<cv::Point2f> corners;
    cv::Size pattern_size = params.getPatternSize();
    
    quality.board_detected = cv::findChessboardCorners(
        gray, pattern_size, corners,
        cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_NORMALIZE_IMAGE | 
        cv::CALIB_CB_FAST_CHECK);
    
    if (quality.board_detected) {
        quality.corners_found = static_cast<int>(corners.size());
        
        // Улучшение точности углов
        cv::cornerSubPix(gray, corners, cv::Size(11, 11), cv::Size(-1, -1),
                        cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 30, 0.1));
        
        // Оценка качества углов (среднее значение интенсивности в окрестности углов)
        float corner_response = 0.0f;
        for (const auto& corner : corners) {
            if (corner.x >= 5 && corner.y >= 5 && 
                corner.x < gray.cols - 5 && corner.y < gray.rows - 5) {
                cv::Rect roi(corner.x - 5, corner.y - 5, 11, 11);
                cv::Scalar mean_val = cv::mean(gray(roi));
                corner_response += mean_val[0];
            }
        }
        quality.corner_response = corner_response / corners.size();
    }
    
    // Общая оценка качества
    if (quality.board_detected) {
        quality.overall_score = quality.sharpness * 0.4f + 
                               quality.contrast * 0.3f + 
                               (quality.corner_response / 255.0f) * 100.0f * 0.3f;
    }
    
    return quality;
}

std::vector<cv::Mat> CalibrationWatcher::loadAndSelectBestFrames(
    const std::string& camera_id, const CalibrationParams& params) {
    
    std::filesystem::path cam_dir = calib_dir_ / ("cam_" + camera_id) / "images";
    
    if (!std::filesystem::exists(cam_dir)) {
        logMessage("Camera directory does not exist: " + cam_dir.string());
        return {};
    }
    
    // Загружаем все изображения с их оценками качества
    std::vector<std::pair<cv::Mat, float>> scored_frames;
    
    for (const auto& entry : std::filesystem::directory_iterator(cam_dir)) {
        if (entry.path().extension() != ".png") continue;
        
        cv::Mat img = cv::imread(entry.path().string());
        if (img.empty()) continue;
        
        FrameQuality quality = evaluateFrameQuality(img, params);
        if (quality.board_detected && quality.overall_score >= params.quality_threshold) {
            scored_frames.emplace_back(img, quality.overall_score);
        }
    }
    
    // Сортируем по качеству (лучшие первыми)
    std::sort(scored_frames.begin(), scored_frames.end(),
              [](const auto& a, const auto& b) { return a.second > b.second; });
    
    // Берем только лучшие кадры
    std::vector<cv::Mat> best_frames;
    int max_frames = std::min(params.max_frames, static_cast<int>(scored_frames.size()));
    
    for (int i = 0; i < max_frames; ++i) {
        best_frames.push_back(scored_frames[i].first);
    }
    
    return best_frames;
}

bool CalibrationWatcher::performMonoCalibration(const std::string& camera_id, const CalibrationParams& params) {
    // Загружаем и выбираем лучшие кадры
    auto images = loadAndSelectBestFrames(camera_id, params);
    
    if (images.size() < params.min_frames) {
        logMessage("Not enough quality frames for camera " + camera_id + 
                  ": " + std::to_string(images.size()));
        return false;
    }
    
    // Подготовка данных для калибровки
    std::vector<std::vector<cv::Point3f>> object_points;
    std::vector<std::vector<cv::Point2f>> image_points;
    cv::Size image_size = images[0].size();
    
    auto world_points = generateChessboardPoints(params);
    
    for (const auto& img : images) {
        std::vector<cv::Point2f> corners;
        if (findChessboardInFrame(img, params, corners)) {
            object_points.push_back(world_points);
            image_points.push_back(corners);
        }
        
        if (should_stop_.load()) return false;
    }
    
    if (object_points.size() < params.min_frames) {
        logMessage("Not enough valid calibration frames for camera: " + camera_id);
        return false;
    }
    
    // Выполнение калибровки
    CameraCalibrationResult result;
    result.camera_id = camera_id;
    result.image_size = image_size;
    result.frames_used = static_cast<int>(object_points.size());
    
    // Получаем текущее время для записи
    auto now = std::time(nullptr);
    std::ostringstream time_stream;
    time_stream << std::put_time(std::localtime(&now), "%Y-%m-%d %H:%M:%S");
    result.calibration_time = time_stream.str();
    
    std::vector<cv::Mat> rvecs, tvecs;
    try {
        result.reprojection_error = cv::calibrateCamera(
            object_points, image_points, image_size,
            result.camera_matrix, result.dist_coeffs,
            rvecs, tvecs,
            cv::CALIB_FIX_PRINCIPAL_POINT);
        
        result.success = validateCalibrationResult(result);
        
        if (result.success) {
            // Вычисляем ошибки для каждого вида
            result.per_view_errors.resize(object_points.size());
            for (size_t i = 0; i < object_points.size(); ++i) {
                std::vector<cv::Point2f> projected_points;
                cv::projectPoints(object_points[i], rvecs[i], tvecs[i], 
                                result.camera_matrix, result.dist_coeffs, projected_points);
                
                double error = cv::norm(image_points[i], projected_points, cv::NORM_L2) / projected_points.size();
                result.per_view_errors[i] = error;
            }
        }
        
    } catch (const cv::Exception& e) {
        logMessage("OpenCV calibration error for camera " + camera_id + ": " + e.what());
        result.success = false;
    }
    
    // Сохраняем результат
    {
        std::lock_guard<std::mutex> lock(results_mutex_);
        mono_results_.push_back(result);
        
        if (result.success) {
            camera_matrices_[camera_id] = result.camera_matrix.clone();
            dist_coeffs_[camera_id] = result.dist_coeffs.clone();
        }
    }
    
    if (result.success) {
        logMessage("Mono calibration successful for " + camera_id + 
                  ": error = " + std::to_string(result.reprojection_error) +
                  ", frames = " + std::to_string(result.frames_used));
    }
    
    return result.success;
}

bool CalibrationWatcher::performStereoCalibration(const std::string& cam1, const std::string& cam2, 
                                                  const CalibrationParams& params) {
    // Проверяем наличие результатов моно-калибровки
    cv::Mat K1, K2, D1, D2;
    bool has_cam1 = getCameraMatrix(cam1, K1, D1);
    bool has_cam2 = getCameraMatrix(cam2, K2, D2);
    
    if (!has_cam1 || !has_cam2) {
        logMessage("Mono calibration results not found for stereo pair: " + cam1 + " + " + cam2);
        return false;
    }
    
    // Загружаем изображения для обеих камер
    auto images1 = loadAndSelectBestFrames(cam1, params);
    auto images2 = loadAndSelectBestFrames(cam2, params);
    
    if (images1.size() < params.min_frames || images2.size() < params.min_frames) {
        logMessage("Not enough images for stereo pair: " + cam1 + " + " + cam2);
        return false;
    }
    
    // Берем минимальное количество изображений
    size_t min_images = std::min(images1.size(), images2.size());
    images1.resize(min_images);
    images2.resize(min_images);
    
    // Подготовка данных для стерео калибровки
    std::vector<std::vector<cv::Point3f>> object_points;
    std::vector<std::vector<cv::Point2f>> image_points1, image_points2;
    
    auto world_points = generateChessboardPoints(params);
    
    for (size_t i = 0; i < min_images && !should_stop_.load(); ++i) {
        std::vector<cv::Point2f> corners1, corners2;
        
        if (findChessboardInFrame(images1[i], params, corners1) &&
            findChessboardInFrame(images2[i], params, corners2)) {
            
            object_points.push_back(world_points);
            image_points1.push_back(corners1);
            image_points2.push_back(corners2);
        }
    }
    
    if (object_points.size() < params.min_frames) {
        logMessage("Not enough valid stereo pairs for calibration: " + cam1 + " + " + cam2);
        return false;
    }
    
    // Выполнение стерео калибровки
    StereoCalibrationResult result;
    result.camera_pair = cam1 + "_" + cam2;
    
    auto now = std::time(nullptr);
    std::ostringstream time_stream;
    time_stream << std::put_time(std::localtime(&now), "%Y-%m-%d %H:%M:%S");
    result.calibration_time = time_stream.str();
    
    cv::Size image_size = images1[0].size();
    
    try {
        result.reprojection_error = cv::stereoCalibrate(
            object_points, image_points1, image_points2,
            K1, D1, K2, D2, image_size,
            result.R, result.T, result.E, result.F,
            cv::CALIB_FIX_INTRINSIC,
            cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 100, 1e-5));
        
        // Выполнение стерео ректификации
        cv::stereoRectify(K1, D1, K2, D2, image_size,
                         result.R, result.T,
                         result.R1, result.R2, result.P1, result.P2, result.Q);
        
        result.success = (result.reprojection_error < 1.0); // Порог ошибки
        
    } catch (const cv::Exception& e) {
        logMessage("Stereo calibration error for " + result.camera_pair + ": " + e.what());
        result.success = false;
    }
    
    // Сохраняем результат
    {
        std::lock_guard<std::mutex> lock(results_mutex_);
        stereo_results_.push_back(result);
    }
    
    if (result.success) {
        logMessage("Stereo calibration successful for " + result.camera_pair + 
                  ": error = " + std::to_string(result.reprojection_error));
    }
    
    return result.success;
}

void CalibrationWatcher::performMultiCameraCalibration(const std::vector<std::string>& camera_ids,
                                                       const CalibrationParams& params) {
    if (camera_ids.size() < 2) return;
    
    logMessage("Performing multi-camera calibration for " + std::to_string(camera_ids.size()) + " cameras");
    
    int successful_pairs = 0;
    int total_pairs = 0;
    
    // Выполняем все возможные стерео-пары
    for (size_t i = 0; i < camera_ids.size() && !should_stop_.load(); ++i) {
        for (size_t j = i + 1; j < camera_ids.size(); ++j) {
            total_pairs++;
            if (performStereoCalibration(camera_ids[i], camera_ids[j], params)) {
                successful_pairs++;
            }
        }
    }
    
    logMessage("Multi-camera calibration completed: " + 
               std::to_string(successful_pairs) + "/" + std::to_string(total_pairs) + " pairs successful");
}

std::vector<cv::Point3f> CalibrationWatcher::generateChessboardPoints(const CalibrationParams& params) {
    std::vector<cv::Point3f> points;
    
    cv::Size pattern_size = params.getPatternSize();
    
    for (int i = 0; i < pattern_size.height; ++i) {
        for (int j = 0; j < pattern_size.width; ++j) {
            points.emplace_back(j * params.square_size, i * params.square_size, 0.0f);
        }
    }
    
    return points;
}

bool CalibrationWatcher::findChessboardInFrame(const cv::Mat& frame, const CalibrationParams& params,
                                              std::vector<cv::Point2f>& corners) {
    cv::Mat gray;
    cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
    
    cv::Size pattern_size = params.getPatternSize();
    
    bool found = cv::findChessboardCorners(gray, pattern_size, corners,
        cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_NORMALIZE_IMAGE);
    
    if (found) {
        cv::cornerSubPix(gray, corners, cv::Size(11, 11), cv::Size(-1, -1),
                        cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 30, 0.1));
    }
    
    return found;
}

bool CalibrationWatcher::saveResults() const {
    try {
        nlohmann::json results;
        
        // Сохраняем результаты моно-калибровки
        nlohmann::json mono_json = nlohmann::json::array();
        {
            std::lock_guard<std::mutex> lock(results_mutex_);
            
            for (const auto& result : mono_results_) {
                nlohmann::json mono_result;
                mono_result["camera_id"] = result.camera_id;
                mono_result["success"] = result.success;
                mono_result["reprojection_error"] = result.reprojection_error;
                mono_result["frames_used"] = result.frames_used;
                mono_result["image_width"] = result.image_size.width;
                mono_result["image_height"] = result.image_size.height;
                mono_result["calibration_time"] = result.calibration_time;
                
                if (result.success) {
                    // Сохраняем матрицы в отдельный YAML файл
                    std::string yaml_filename = "cam_" + result.camera_id + "_calibration.yml";
                    std::filesystem::path yaml_path = results_dir_ / yaml_filename;
                    
                    cv::FileStorage fs(yaml_path.string(), cv::FileStorage::WRITE);
                    if (fs.isOpened()) {
                        fs << "camera_matrix" << result.camera_matrix;
                        fs << "distortion_coefficients" << result.dist_coeffs;
                        fs << "image_width" << result.image_size.width;
                        fs << "image_height" << result.image_size.height;
                        fs << "reprojection_error" << result.reprojection_error;
                        fs << "frames_used" << result.frames_used;
                        fs << "calibration_time" << result.calibration_time;
                        fs << "per_view_errors" << result.per_view_errors;
                        fs.release();
                        
                        mono_result["calibration_file"] = yaml_filename;
                    }
                }
                
                mono_json.push_back(mono_result);
            }
            
            // Сохраняем результаты стерео-калибровки
            nlohmann::json stereo_json = nlohmann::json::array();
            for (const auto& result : stereo_results_) {
                nlohmann::json stereo_result;
                stereo_result["camera_pair"] = result.camera_pair;

stereo_result["success"] = result.success;
                stereo_result["reprojection_error"] = result.reprojection_error;
                stereo_result["calibration_time"] = result.calibration_time;
                
                if (result.success) {
                    // Сохраняем стерео параметры в YAML
                    std::string yaml_filename = "stereo_" + result.camera_pair + "_calibration.yml";
                    std::filesystem::path yaml_path = results_dir_ / yaml_filename;
                    
                    cv::FileStorage fs(yaml_path.string(), cv::FileStorage::WRITE);
                    if (fs.isOpened()) {
                        fs << "R" << result.R;
                        fs << "T" << result.T;
                        fs << "E" << result.E;
                        fs << "F" << result.F;
                        fs << "R1" << result.R1;
                        fs << "R2" << result.R2;
                        fs << "P1" << result.P1;
                        fs << "P2" << result.P2;
                        fs << "Q" << result.Q;
                        fs << "reprojection_error" << result.reprojection_error;
                        fs << "calibration_time" << result.calibration_time;
                        fs.release();
                        
                        stereo_result["calibration_file"] = yaml_filename;
                    }
                }
                
                stereo_json.push_back(stereo_result);
            }
            
            results["mono_calibrations"] = mono_json;
            results["stereo_calibrations"] = stereo_json;
        }
        
        // Сохраняем JSON с общей информацией
        std::filesystem::path json_path = results_dir_ / "calibration_results.json";
        std::ofstream json_file(json_path);
        if (json_file.is_open()) {
            json_file << results.dump(2);
            json_file.close();
            
            logMessage("Calibration results saved to: " + json_path.string());
            return true;
        }
        
    } catch (const std::exception& e) {
        logMessage("Failed to save calibration results: " + std::string(e.what()));
    }
    
    return false;
}

bool CalibrationWatcher::loadResults() {
    try {
        std::filesystem::path json_path = results_dir_ / "calibration_results.json";
        
        if (!std::filesystem::exists(json_path)) {
            return false;
        }
        
        std::ifstream json_file(json_path);
        if (!json_file.is_open()) {
            return false;
        }
        
        nlohmann::json results;
        json_file >> results;
        json_file.close();
        
        std::lock_guard<std::mutex> lock(results_mutex_);
        
        mono_results_.clear();
        stereo_results_.clear();
        camera_matrices_.clear();
        dist_coeffs_.clear();
        
        // Загружаем результаты моно-калибровки
        if (results.contains("mono_calibrations")) {
            for (const auto& mono_json : results["mono_calibrations"]) {
                CameraCalibrationResult result;
                result.camera_id = mono_json["camera_id"];
                result.success = mono_json["success"];
                result.reprojection_error = mono_json["reprojection_error"];
                result.frames_used = mono_json["frames_used"];
                result.image_size = cv::Size(mono_json["image_width"], mono_json["image_height"]);
                result.calibration_time = mono_json.value("calibration_time", "");
                
                if (result.success && mono_json.contains("calibration_file")) {
                    std::string yaml_filename = mono_json["calibration_file"];
                    std::filesystem::path yaml_path = results_dir_ / yaml_filename;
                    
                    cv::FileStorage fs(yaml_path.string(), cv::FileStorage::READ);
                    if (fs.isOpened()) {
                        fs["camera_matrix"] >> result.camera_matrix;
                        fs["distortion_coefficients"] >> result.dist_coeffs;
                        if (fs["per_view_errors"].isSeq()) {
                            fs["per_view_errors"] >> result.per_view_errors;
                        }
                        fs.release();
                        
                        camera_matrices_[result.camera_id] = result.camera_matrix.clone();
                        dist_coeffs_[result.camera_id] = result.dist_coeffs.clone();
                    }
                }
                
                mono_results_.push_back(result);
            }
        }
        
        // Загружаем результаты стерео-калибровки
        if (results.contains("stereo_calibrations")) {
            for (const auto& stereo_json : results["stereo_calibrations"]) {
                StereoCalibrationResult result;
                result.camera_pair = stereo_json["camera_pair"];
                result.success = stereo_json["success"];
                result.reprojection_error = stereo_json["reprojection_error"];
                result.calibration_time = stereo_json.value("calibration_time", "");
                
                if (result.success && stereo_json.contains("calibration_file")) {
                    std::string yaml_filename = stereo_json["calibration_file"];
                    std::filesystem::path yaml_path = results_dir_ / yaml_filename;
                    
                    cv::FileStorage fs(yaml_path.string(), cv::FileStorage::READ);
                    if (fs.isOpened()) {
                        fs["R"] >> result.R;
                        fs["T"] >> result.T;
                        fs["E"] >> result.E;
                        fs["F"] >> result.F;
                        fs["R1"] >> result.R1;
                        fs["R2"] >> result.R2;
                        fs["P1"] >> result.P1;
                        fs["P2"] >> result.P2;
                        fs["Q"] >> result.Q;
                        fs.release();
                    }
                }
                
                stereo_results_.push_back(result);
            }
        }
        
        logMessage("Calibration results loaded from: " + json_path.string());
        return true;
        
    } catch (const std::exception& e) {
        logMessage("Failed to load calibration results: " + std::string(e.what()));
    }
    
    return false;
}

bool CalibrationWatcher::getCameraMatrix(const std::string& camera_id, 
                                        cv::Mat& camera_matrix, cv::Mat& dist_coeffs) const {
    std::lock_guard<std::mutex> lock(results_mutex_);
    
    auto it_k = camera_matrices_.find(camera_id);
    auto it_d = dist_coeffs_.find(camera_id);
    
    if (it_k != camera_matrices_.end() && it_d != dist_coeffs_.end()) {
        camera_matrix = it_k->second.clone();
        dist_coeffs = it_d->second.clone();
        return true;
    }
    
    return false;
}

bool CalibrationWatcher::getStereoParams(const std::string& camera_pair, 
                                        cv::Mat& R, cv::Mat& T, cv::Mat& Q) const {
    std::lock_guard<std::mutex> lock(results_mutex_);
    
    for (const auto& result : stereo_results_) {
        if (result.camera_pair == camera_pair && result.success) {
            R = result.R.clone();
            T = result.T.clone();
            Q = result.Q.clone();
            return true;
        }
    }
    
    return false;
}

void CalibrationWatcher::updateStatus(const std::string& message, float progress) {
    {
        std::lock_guard<std::mutex> lock(status_mutex_);
        status_message_ = message;
    }
    
    if (progress >= 0.0f) {
        progress_.store(progress);
    }
    
    logMessage("Status: " + message + " (" + std::to_string(progress_.load()) + "%)");
    
    // Вызываем callback если установлен
    if (status_callback_) {
        status_callback_(message, progress_.load());
    }
}

void CalibrationWatcher::logMessage(const std::string& message) const {
    auto now = std::time(nullptr);
    auto tm = *std::localtime(&now);
    
    char timestamp[32];
    std::strftime(timestamp, sizeof(timestamp), "%Y-%m-%d %H:%M:%S", &tm);
    
    std::string log_line = "[" + std::string(timestamp) + "] CalibrationWatcher: " + message;
    std::cout << log_line << std::endl;
    
    // Вызываем callback если установлен
    if (log_callback_) {
        log_callback_(log_line);
    }
}

float CalibrationWatcher::calculateSharpness(const cv::Mat& gray) {
    cv::Mat laplacian;
    cv::Laplacian(gray, laplacian, CV_64F);
    
    cv::Scalar mu, sigma;
    cv::meanStdDev(laplacian, mu, sigma);
    
    return static_cast<float>(sigma.val[0] * sigma.val[0]);
}

float CalibrationWatcher::calculateContrast(const cv::Mat& gray) {
    cv::Scalar mu, sigma;
    cv::meanStdDev(gray, mu, sigma);
    
    return static_cast<float>(sigma.val[0]);
}

bool CalibrationWatcher::validateCalibrationResult(const CameraCalibrationResult& result) {
    // Проверяем базовые условия успешной калибровки
    if (result.reprojection_error > 2.0) {
        return false;
    }
    
    if (result.camera_matrix.empty() || result.dist_coeffs.empty()) {
        return false;
    }
    
    // Проверяем разумность параметров камеры
    double fx = result.camera_matrix.at<double>(0, 0);
    double fy = result.camera_matrix.at<double>(1, 1);
    
    if (fx <= 0 || fy <= 0 || fx > 10000 || fy > 10000) {
        return false;
    }
    
    return true;
}

void CalibrationWatcher::cleanupTempFiles() {
    // Очистка временных файлов если необходимо
    // Может быть реализована для удаления промежуточных данных
}

// ===============================================================================
// Класс для измерения расстояний
// ===============================================================================

DistanceMeasurement::DistanceMeasurement(const CalibrationWatcher& calibration) {
    updateCalibrationData(calibration);
}

void DistanceMeasurement::updateCalibrationData(const CalibrationWatcher& calibration) {
    mono_calibrations_.clear();
    stereo_calibrations_.clear();
    
    // Копируем результаты калибровки
    for (const auto& mono_result : calibration.getMonoResults()) {
        if (mono_result.success) {
            mono_calibrations_[mono_result.camera_id] = mono_result;
        }
    }
    
    for (const auto& stereo_result : calibration.getStereoResults()) {
        if (stereo_result.success) {
            stereo_calibrations_[stereo_result.camera_pair] = stereo_result;
        }
    }
}

float DistanceMeasurement::measureDistance(const std::string& camera_id, const cv::Rect& bbox, 
                                         float real_object_height_mm) const {
    auto it = mono_calibrations_.find(camera_id);
    if (it == mono_calibrations_.end()) {
        return -1.0f; // Калибровка не найдена
    }
    
    return estimateObjectDistance(it->second.camera_matrix, bbox, real_object_height_mm);
}

float DistanceMeasurement::measureStereoDistance(const std::string& camera_pair, 
                                               const cv::Point2f& point1, const cv::Point2f& point2) const {
    auto it = stereo_calibrations_.find(camera_pair);
    if (it == stereo_calibrations_.end()) {
        return -1.0f; // Стерео калибровка не найдена
    }
    
    const auto& stereo_result = it->second;
    
    // Вычисляем диспарантность
    float disparity = std::abs(point1.x - point2.x);
    if (disparity < 1.0f) {
        return -1.0f; // Диспарантность слишком мала
    }
    
    // Извлекаем параметры из матрицы Q
    double Q03 = stereo_result.Q.at<double>(0, 3);
    double Q13 = stereo_result.Q.at<double>(1, 3);
    double Q23 = stereo_result.Q.at<double>(2, 3);
    double Q33 = stereo_result.Q.at<double>(3, 3);
    double Q32 = stereo_result.Q.at<double>(3, 2);
    
    if (std::abs(Q33) < 1e-6) {
        return -1.0f;
    }
    
    // Вычисляем глубину в мм
    double W = disparity * Q32 + Q33;
    if (std::abs(W) < 1e-6) {
        return -1.0f;
    }
    
    double Z = Q23 / W;
    
    return static_cast<float>(std::abs(Z));
}

cv::Point3f DistanceMeasurement::pixelToWorld(const std::string& camera_id, const cv::Point2f& pixel, 
                                            float distance_mm) const {
    auto it = mono_calibrations_.find(camera_id);
    if (it == mono_calibrations_.end()) {
        return cv::Point3f(0, 0, 0);
    }
    
    const auto& camera_matrix = it->second.camera_matrix;
    const auto& dist_coeffs = it->second.dist_coeffs;
    
    // Корректируем искажения
    std::vector<cv::Point2f> src_points = {pixel};
    std::vector<cv::Point2f> undist_points;
    cv::undistortPoints(src_points, undist_points, camera_matrix, dist_coeffs);
    
    const auto& undist_point = undist_points[0];
    
    // Переводим в мировые координаты
    float x = undist_point.x * distance_mm;
    float y = undist_point.y * distance_mm;
    float z = distance_mm;
    
    return cv::Point3f(x, y, z);
}

bool DistanceMeasurement::hasMonoCalibration(const std::string& camera_id) const {
    return mono_calibrations_.find(camera_id) != mono_calibrations_.end();
}

bool DistanceMeasurement::hasStereoCalibration(const std::string& camera_pair) const {
    return stereo_calibrations_.find(camera_pair) != stereo_calibrations_.end();
}

float DistanceMeasurement::estimateObjectDistance(const cv::Mat& camera_matrix, const cv::Rect& bbox,
                                               float real_height_mm) const {
    // Получаем фокусное расстояние
    double fy = camera_matrix.at<double>(1, 1);
    
    if (fy <= 0 || bbox.height <= 0) {
        return -1.0f;
    }
    
    // Формула: distance = (real_height * focal_length) / pixel_height
    float distance = static_cast<float>((real_height_mm * fy) / bbox.height);
    
    return distance;
}