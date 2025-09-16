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
#include <cmath>
#include <limits>
#include <array>

#include <utility>
extern "C" {
#include <libavcodec/avcodec.h>
#include <libavformat/avformat.h>
#include <libavutil/avutil.h>
}

namespace {

class FFmpegFramePTSReader {
public:
    FFmpegFramePTSReader() = default;
    ~FFmpegFramePTSReader() { close(); }

    bool open(const std::string& path) {
        close();

        int ret = avformat_open_input(&format_ctx_, path.c_str(), nullptr, nullptr);
        if (ret < 0) {
            close();
            return false;
        }

        ret = avformat_find_stream_info(format_ctx_, nullptr);
        if (ret < 0) {
            close();
            return false;
        }

        ret = av_find_best_stream(format_ctx_, AVMEDIA_TYPE_VIDEO, -1, -1, nullptr, 0);
        if (ret < 0) {
            close();
            return false;
        }

        video_stream_index_ = ret;
        video_stream_ = format_ctx_->streams[video_stream_index_];

        const AVCodec* decoder = avcodec_find_decoder(video_stream_->codecpar->codec_id);
        if (!decoder) {
            close();
            return false;
        }

        codec_ctx_ = avcodec_alloc_context3(decoder);
        if (!codec_ctx_) {
            close();
            return false;
        }

        ret = avcodec_parameters_to_context(codec_ctx_, video_stream_->codecpar);
        if (ret < 0) {
            close();
            return false;
        }

        ret = avcodec_open2(codec_ctx_, decoder, nullptr);
        if (ret < 0) {
            close();
            return false;
        }

        packet_ = av_packet_alloc();
        frame_ = av_frame_alloc();

        if (!packet_ || !frame_) {
            close();
            return false;
        }

        AVRational frame_rate = av_guess_frame_rate(format_ctx_, video_stream_, nullptr);
        double fps = av_q2d(frame_rate);
        frame_duration_ = (fps > 0.0) ? 1.0 / fps : 0.0;

        frame_index_ = 0;
        last_valid_pts_ = AV_NOPTS_VALUE;
        end_of_stream_ = false;

        return true;
    }

    bool nextTimestamp(double& timestamp_seconds) {
        if (!codec_ctx_ || !frame_) {
            return false;
        }

        while (true) {
            int ret = avcodec_receive_frame(codec_ctx_, frame_);
            if (ret >= 0) {
                int64_t pts = frame_->best_effort_timestamp;
                if (pts == AV_NOPTS_VALUE) {
                    pts = frame_->pts;
                }

                double seconds = 0.0;
                if (pts != AV_NOPTS_VALUE) {
                    last_valid_pts_ = pts;
                    seconds = pts * av_q2d(video_stream_->time_base);
                } else if (frame_duration_ > 0.0) {
                    seconds = frame_index_ * frame_duration_;
                } else if (last_valid_pts_ != AV_NOPTS_VALUE) {
                    double time_base = av_q2d(video_stream_->time_base);
                    seconds = (last_valid_pts_ + 1) * time_base;
                    last_valid_pts_ += 1;
                } else {
                    seconds = static_cast<double>(frame_index_);
                }

                ++frame_index_;
                av_frame_unref(frame_);
                timestamp_seconds = seconds;
                return true;
            }

            if (ret == AVERROR_EOF) {
                return false;
            }

            if (ret != AVERROR(EAGAIN)) {
                return false;
            }

            if (end_of_stream_) {
                return false;
            }

            ret = av_read_frame(format_ctx_, packet_);
            if (ret < 0) {
                avcodec_send_packet(codec_ctx_, nullptr);
                end_of_stream_ = true;
                continue;
            }

            if (packet_->stream_index == video_stream_index_) {
                ret = avcodec_send_packet(codec_ctx_, packet_);
                av_packet_unref(packet_);
                if (ret < 0) {
                    return false;
                }
            } else {
                av_packet_unref(packet_);
            }
        }
    }

    void close() {
        if (packet_) {
            av_packet_free(&packet_);
            packet_ = nullptr;
        }
        if (frame_) {
            av_frame_free(&frame_);
            frame_ = nullptr;
        }
        if (codec_ctx_) {
            avcodec_free_context(&codec_ctx_);
            codec_ctx_ = nullptr;
        }
        if (format_ctx_) {
            avformat_close_input(&format_ctx_);
            format_ctx_ = nullptr;
        }

        video_stream_index_ = -1;
        video_stream_ = nullptr;
        frame_duration_ = 0.0;
        frame_index_ = 0;
        last_valid_pts_ = AV_NOPTS_VALUE;
        end_of_stream_ = false;
    }

private:
    AVFormatContext* format_ctx_ = nullptr;
    AVCodecContext* codec_ctx_ = nullptr;
    AVPacket* packet_ = nullptr;
    AVFrame* frame_ = nullptr;
    AVStream* video_stream_ = nullptr;
    int video_stream_index_ = -1;
    double frame_duration_ = 0.0;
    size_t frame_index_ = 0;
    int64_t last_valid_pts_ = AV_NOPTS_VALUE;
    bool end_of_stream_ = false;
};

std::string makeStereoKey(const std::string& cam1, const std::string& cam2) {
    if (cam1 <= cam2) {
        return cam1 + "|" + cam2;
    }
    return cam2 + "|" + cam1;
}


} // namespace

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

        // 4. Выполняем синхронизацию и стерео/мульти-камерную калибровку
        if (successful_cameras.size() >= 2) {
            updateStatus("Synchronizing camera streams...", 72.5f);
            auto synchronization = synchronizeStreams(successful_cameras, params);

            if (should_stop_.load()) {
                updateStatus("Calibration stopped by user", 100.0f);
                processing_ = false;
                return;
            }

            if (!synchronization.multi_groups.empty()) {
                updateStatus("Performing stereo/multi calibrations...", 75.0f);
                performMultiCameraCalibration(successful_cameras, params, synchronization);
            } else {
                logMessage("No synchronized frame groups found. Skipping stereo calibration stage.");
            }
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

    FFmpegFramePTSReader pts_reader;
    bool pts_available = pts_reader.open(video.path.string());
    if (!pts_available) {
        logMessage("WARNING: Unable to initialise FFmpeg timestamp reader for " + video.path.string());
    }


    cv::Mat frame;
    int frame_count = 0;
    int saved_count = 0;

    // Получаем общее количество кадров
    int total_frames = static_cast<int>(cap.get(cv::CAP_PROP_FRAME_COUNT));
    int skip_interval = std::max(1, total_frames / std::max(1, params.max_frames));

    logMessage("Processing video: " + std::to_string(total_frames) +
               " frames, skip interval: " + std::to_string(skip_interval));

    while (cap.read(frame) && saved_count < params.max_frames && !should_stop_.load()) {
        double timestamp_seconds = 0.0;
        if (pts_available) {
            if (!pts_reader.nextTimestamp(timestamp_seconds)) {
                logMessage("WARNING: Failed to fetch PTS for frame " + std::to_string(frame_count) +
                           " in video " + video.path.string());
                pts_available = false;
                timestamp_seconds = cap.get(cv::CAP_PROP_POS_MSEC) / 1000.0;
            }
        } else {
            timestamp_seconds = cap.get(cv::CAP_PROP_POS_MSEC) / 1000.0;
        }

        // Пропускаем кадры для равномерного распределения
        if (frame_count % skip_interval != 0) {
            frame_count++;
            continue;
        }

        // Оцениваем качество кадра
        std::vector<cv::Point2f> corners;
        FrameQuality quality = evaluateFrameQuality(frame, params, &corners);

        if (quality.board_detected && quality.overall_score >= params.quality_threshold) {
            DetectedFrame detected_frame;
            detected_frame.timestamp = timestamp_seconds;
            detected_frame.corners = std::move(corners);
            detected_frame.pose = analyzeBoardPose(
                detected_frame.corners, frame.size(), params.getPatternSize());
            detected_frame.quality = quality;
            detected_frame.image = frame.clone();

            std::string filename = "frame_" + std::to_string(saved_count) +
                                 "_score_" + std::to_string(static_cast<int>(quality.overall_score)) + ".png";

            std::filesystem::path filepath = cam_dir / filename;
            detected_frame.image_path = filepath;

            if (cv::imwrite(filepath.string(), detected_frame.image)) {
                saved_count++;
                logMessage("Saved quality frame: " + filename +
                          " (corners: " + std::to_string(quality.corners_found) +
                          ", score: " + std::to_string(quality.overall_score) + ")");

                if (!saveFrameMetadata(filepath, detected_frame)) {
                    logMessage("WARNING: Failed to save metadata for frame " + filename);
                }
            }
        }

        frame_count++;
    }

    cap.release();

    logMessage("Extracted " + std::to_string(saved_count) + " quality frames from " +
               std::to_string(frame_count) + " total frames for camera " + video.camera_id);

    return saved_count >= params.min_frames;
}


bool CalibrationWatcher::saveFrameMetadata(const std::filesystem::path& image_path, const DetectedFrame& frame) const {
    try {
        nlohmann::json metadata;
        metadata["image"] = image_path.filename().string();
        metadata["timestamp"] = frame.timestamp;
        metadata["timestamp_units"] = "seconds";
        metadata["quality"] = {
            {"sharpness", frame.quality.sharpness},
            {"contrast", frame.quality.contrast},
            {"corner_response", frame.quality.corner_response},
            {"corners_found", frame.quality.corners_found},
            {"overall_score", frame.quality.overall_score},
            {"board_detected", frame.quality.board_detected}
        };

        nlohmann::json corners_json = nlohmann::json::array();
        for (const auto& pt : frame.corners) {
            corners_json.push_back({{"x", pt.x}, {"y", pt.y}});
        }
        metadata["corners"] = std::move(corners_json);


        metadata["pose"] = {
            {"tilt_degrees", frame.pose.tilt_degrees},
            {"scale", frame.pose.scale},
            {"center", {{"x", frame.pose.center.x}, {"y", frame.pose.center.y}}},
            {"normalized_center", {{"x", frame.pose.normalized_center.x},
                                     {"y", frame.pose.normalized_center.y}}}
        };

        std::filesystem::path metadata_path = image_path;
        metadata_path.replace_extension(".json");

        std::ofstream meta_file(metadata_path);
        if (!meta_file.is_open()) {
            logMessage("WARNING: Unable to open metadata file for writing: " + metadata_path.string());
            return false;
        }

        meta_file << metadata.dump(2);
        meta_file.close();
        return true;
    } catch (const std::exception& e) {
        logMessage("ERROR: Exception while saving metadata for " + image_path.string() + ": " + e.what());
        return false;
    }
}



CalibrationWatcher::FrameQuality CalibrationWatcher::evaluateFrameQuality(
    const cv::Mat& frame, const CalibrationParams& params,
    std::vector<cv::Point2f>* refined_corners) {

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
        if (refined_corners) {
            *refined_corners = corners;
        }

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
    } else if (refined_corners) {
        refined_corners->clear();
    }

    // Общая оценка качества
    if (quality.board_detected) {
        quality.overall_score = quality.sharpness * 0.4f +
                               quality.contrast * 0.3f +
                               (quality.corner_response / 255.0f) * 100.0f * 0.3f;
    }

    return quality;
}

BoardPose CalibrationWatcher::analyzeBoardPose(
    const std::vector<cv::Point2f>& corners,
    const cv::Size& image_size,
    const cv::Size& pattern_size) {

    BoardPose pose;

    if (corners.empty() || image_size.width <= 0 || image_size.height <= 0) {
        return pose;
    }

    cv::Point2f center(0.0f, 0.0f);
    for (const auto& pt : corners) {
        center += pt;
    }
    center *= (1.0f / static_cast<float>(corners.size()));
    pose.center = center;

    pose.normalized_center = cv::Point2f(
        center.x / static_cast<float>(image_size.width),
        center.y / static_cast<float>(image_size.height));
    pose.normalized_center.x = std::clamp(pose.normalized_center.x, 0.0f, 1.0f);
    pose.normalized_center.y = std::clamp(pose.normalized_center.y, 0.0f, 1.0f);

    float min_x = corners.front().x;
    float min_y = corners.front().y;
    float max_x = corners.front().x;
    float max_y = corners.front().y;
    for (const auto& pt : corners) {
        min_x = std::min(min_x, pt.x);
        min_y = std::min(min_y, pt.y);
        max_x = std::max(max_x, pt.x);
        max_y = std::max(max_y, pt.y);
    }

    float board_width = std::max(0.0f, max_x - min_x);
    float board_height = std::max(0.0f, max_y - min_y);
    float image_area = static_cast<float>(image_size.width) * static_cast<float>(image_size.height);
    if (image_area > 0.0f) {
        pose.scale = std::clamp((board_width * board_height) / image_area, 0.0f, 1.0f);
    }

    if (pattern_size.width > 1 && static_cast<int>(corners.size()) >= pattern_size.width) {
        int top_right_index = pattern_size.width - 1;
        if (top_right_index < static_cast<int>(corners.size())) {
            cv::Point2f horizontal_vec = corners[top_right_index] - corners[0];
            if (cv::norm(horizontal_vec) > 0.0f) {
                float angle_rad = std::atan2(horizontal_vec.y, horizontal_vec.x);
                pose.tilt_degrees = angle_rad * 180.0f / static_cast<float>(CV_PI);
            }
        }
    }

    return pose;
}

std::vector<CalibrationWatcher::DetectedFrame> CalibrationWatcher::loadAndSelectBestFrames(
    const std::string& camera_id, const CalibrationParams& params) {

    std::filesystem::path cam_dir = calib_dir_ / ("cam_" + camera_id) / "images";

    if (!std::filesystem::exists(cam_dir)) {
        logMessage("Camera directory does not exist: " + cam_dir.string());
        return {};
    }

    struct CandidateFrame {
        DetectedFrame frame;
        float score = 0.0f;
        std::array<float, 5> feature = {};
    };

    std::vector<CandidateFrame> candidates;

    for (const auto& entry : std::filesystem::directory_iterator(cam_dir)) {
        if (entry.path().extension() != ".png") {
            continue;
        }
        cv::Mat img = cv::imread(entry.path().string());
        if (img.empty()) {
            continue;
        }

        std::vector<cv::Point2f> corners;
        FrameQuality quality = evaluateFrameQuality(img, params, &corners);
        if (!quality.board_detected || quality.overall_score < params.quality_threshold || corners.empty()) {
            continue;
        }

        DetectedFrame detected;
        detected.image = img;
        detected.image_path = entry.path();
        detected.corners = std::move(corners);
        detected.quality = quality;
        detected.pose = analyzeBoardPose(detected.corners, detected.image.size(), params.getPatternSize());

        std::filesystem::path metadata_path = entry.path();
        metadata_path.replace_extension(".json");
        detected.timestamp = 0.0;
        if (std::filesystem::exists(metadata_path)) {
            try {
                std::ifstream meta_stream(metadata_path);
                if (meta_stream.is_open()) {
                    nlohmann::json metadata;
                    meta_stream >> metadata;
                    detected.timestamp = metadata.value("timestamp", 0.0);
                }
            } catch (const std::exception& e) {
                logMessage("WARNING: Failed to read metadata for frame " + entry.path().string() +
                           ": " + e.what());
            }
        }

        float angle_rad = detected.pose.tilt_degrees * static_cast<float>(CV_PI) / 180.0f;
        CandidateFrame candidate;
        candidate.frame = std::move(detected);
        candidate.score = quality.overall_score;
        candidate.feature = {
            candidate.frame.pose.normalized_center.x,
            candidate.frame.pose.normalized_center.y,
            candidate.frame.pose.scale,
            std::sin(angle_rad),
            std::cos(angle_rad)};

        candidates.push_back(std::move(candidate));
    }

    if (candidates.empty()) {
        return {};
    }

    std::sort(candidates.begin(), candidates.end(),
              [](const CandidateFrame& a, const CandidateFrame& b) {
                  return a.score > b.score;
              });

    const int max_frames_total = std::min<int>(params.max_frames, static_cast<int>(candidates.size()));
    const int max_frames_per_cluster = std::max(1, params.max_frames / 5);
    const float position_tolerance = 0.12f;
    const float scale_tolerance = 0.15f;
    const float tilt_tolerance_deg = 12.0f;

    auto angleDifference = [](const std::array<float, 5>& a, const std::array<float, 5>& b) {
        float angle_a = std::atan2(a[3], a[4]);
        float angle_b = std::atan2(b[3], b[4]);
        float diff = std::fabs(angle_a - angle_b);
        const float two_pi = 2.0f * static_cast<float>(CV_PI);
        if (diff > static_cast<float>(CV_PI)) {
            diff = two_pi - diff;
        }
        return diff * 180.0f / static_cast<float>(CV_PI);
    };

    struct Cluster {
        std::array<float, 5> center = {};
        int count = 0;
    };

    std::vector<Cluster> clusters;
    std::vector<DetectedFrame> selected_frames;
    selected_frames.reserve(max_frames_total);
    std::vector<bool> used(candidates.size(), false);

    for (size_t i = 0; i < candidates.size() && static_cast<int>(selected_frames.size()) < max_frames_total; ++i) {
        const auto& candidate = candidates[i];
        bool suppressed = false;
        bool assigned = false;

        for (auto& cluster : clusters) {
            if (std::fabs(candidate.feature[0] - cluster.center[0]) > position_tolerance ||
                std::fabs(candidate.feature[1] - cluster.center[1]) > position_tolerance ||
                std::fabs(candidate.feature[2] - cluster.center[2]) > scale_tolerance) {
                continue;
            }

            float diff = angleDifference(candidate.feature, cluster.center);
            if (diff > tilt_tolerance_deg) {
                continue;
            }

            if (cluster.count >= max_frames_per_cluster) {
                suppressed = true;
                break;
            }

            for (size_t j = 0; j < cluster.center.size(); ++j) {
                cluster.center[j] = (cluster.center[j] * cluster.count + candidate.feature[j]) /
                                    static_cast<float>(cluster.count + 1);
            }
            cluster.count++;
            selected_frames.push_back(candidate.frame);
            used[i] = true;
            assigned = true;
            break;
        }

        if (!assigned && !suppressed) {
            Cluster new_cluster;
            new_cluster.center = candidate.feature;
            new_cluster.count = 1;
            clusters.push_back(new_cluster);
            selected_frames.push_back(candidate.frame);
            used[i] = true;
        }
    }

    size_t initial_selection = selected_frames.size();

    if (static_cast<int>(selected_frames.size()) < std::min(params.min_frames, max_frames_total)) {
        for (size_t i = 0; i < candidates.size() && static_cast<int>(selected_frames.size()) < max_frames_total; ++i) {
            if (!used[i]) {
                selected_frames.push_back(candidates[i].frame);
                used[i] = true;
            }
        }
    }

    size_t fallback_added = selected_frames.size() - initial_selection;
    std::ostringstream cluster_message;
    cluster_message << "Selected " << selected_frames.size() << " frames across "
                    << clusters.size() << " pose clusters for camera " << camera_id;
    if (fallback_added > 0) {
        cluster_message << " (" << fallback_added << " added to satisfy minimum)";
    }
    logMessage(cluster_message.str());
    return selected_frames;
}

CalibrationWatcher::StreamSynchronization CalibrationWatcher::synchronizeStreams(
    const std::vector<std::string>& camera_ids,
    const CalibrationParams& params) const {

    StreamSynchronization sync;

    if (camera_ids.empty()) {
        return sync;
    }

    constexpr double kTimestampTolerance = 0.033; // ~33ms tolerance for 30 FPS streams
    std::map<std::string, double> first_timestamps;

    for (const auto& camera_id : camera_ids) {
        if (should_stop_.load()) {
            return sync;
        }

        std::filesystem::path cam_dir = calib_dir_ / ("cam_" + camera_id) / "images";
        if (!std::filesystem::exists(cam_dir)) {
            logMessage("Synchronization skipped for camera " + camera_id +
                       ": images directory not found at " + cam_dir.string());
            continue;
        }

        std::vector<std::filesystem::path> metadata_files;
        for (const auto& entry : std::filesystem::directory_iterator(cam_dir)) {
            if (entry.path().extension() == ".json") {
                metadata_files.push_back(entry.path());
            }
        }

        std::sort(metadata_files.begin(), metadata_files.end());

        std::vector<StreamSynchronization::FramePtr> frames;
        frames.reserve(metadata_files.size());

        for (const auto& metadata_path : metadata_files) {
            if (should_stop_.load()) {
                return sync;
            }

            try {
                std::ifstream meta_stream(metadata_path);
                if (!meta_stream.is_open()) {
                    logMessage("Unable to open metadata file " + metadata_path.string());
                    continue;
                }

                nlohmann::json metadata;
                meta_stream >> metadata;

                auto frame = std::make_shared<DetectedFrame>();
                frame->timestamp = metadata.value("timestamp", 0.0);

                const auto image_name = metadata.value("image", std::string());
                if (image_name.empty()) {
                    logMessage("Skipping metadata without image reference: " + metadata_path.string());
                    continue;
                }

                std::filesystem::path image_path = cam_dir / image_name;
                frame->image_path = image_path;
                frame->image = cv::imread(image_path.string(), cv::IMREAD_COLOR);
                if (frame->image.empty()) {
                    logMessage("Failed to load synchronized image: " + image_path.string());
                    continue;
                }

                if (metadata.contains("quality")) {
                    const auto& quality = metadata["quality"];
                    frame->quality.sharpness = quality.value("sharpness", 0.0f);
                    frame->quality.contrast = quality.value("contrast", 0.0f);
                    frame->quality.corner_response = quality.value("corner_response", 0.0f);
                    frame->quality.corners_found = quality.value("corners_found", 0);
                    frame->quality.board_detected = quality.value("board_detected", true);
                    frame->quality.overall_score = quality.value("overall_score", 0.0f);
                } else {
                    frame->quality.board_detected = true;
                }

                if (!frame->quality.board_detected) {
                    continue;
                }

                if (metadata.contains("corners") && metadata["corners"].is_array()) {
                    for (const auto& corner : metadata["corners"]) {
                        if (!corner.contains("x") || !corner.contains("y")) {
                            continue;
                        }
                        frame->corners.emplace_back(corner["x"].get<float>(),
                                                    corner["y"].get<float>());
                    }
                }

                if (frame->corners.empty()) {
                    logMessage("Skipping synchronized frame without stored chessboard corners: " +
                               image_path.string());
                    continue;
                }

                if (metadata.contains("pose") && metadata["pose"].is_object()) {
                    const auto& pose_json = metadata["pose"];
                    frame->pose.tilt_degrees = pose_json.value("tilt_degrees", 0.0f);
                    frame->pose.scale = pose_json.value("scale", 0.0f);

                    if (pose_json.contains("center") && pose_json["center"].is_object()) {
                        const auto& center_json = pose_json["center"];
                        frame->pose.center.x = center_json.value("x", 0.0f);
                        frame->pose.center.y = center_json.value("y", 0.0f);
                    }

                    if (pose_json.contains("normalized_center") && pose_json["normalized_center"].is_object()) {
                        const auto& normalized_json = pose_json["normalized_center"];
                        frame->pose.normalized_center.x = normalized_json.value("x", 0.0f);
                        frame->pose.normalized_center.y = normalized_json.value("y", 0.0f);
                    }
                } else {
                    frame->pose = analyzeBoardPose(frame->corners, frame->image.size(), params.getPatternSize());
                }


                frames.push_back(frame);
            } catch (const std::exception& e) {
                logMessage("Failed to parse metadata " + metadata_path.string() + ": " + e.what());
            }
        }

        std::sort(frames.begin(), frames.end(),
                  [](const auto& lhs, const auto& rhs) { return lhs->timestamp < rhs->timestamp; });

        size_t max_frames_per_camera = static_cast<size_t>(std::max(params.max_frames * 3, params.min_frames));
        if (frames.size() > max_frames_per_camera) {
            frames.resize(max_frames_per_camera);
        }

        if (!frames.empty()) {
            first_timestamps[camera_id] = frames.front()->timestamp;
            logMessage("Loaded " + std::to_string(frames.size()) +
                       " synchronized frames for camera " + camera_id);
            sync.mono_frames[camera_id] = std::move(frames);
        } else {
            logMessage("No synchronized frames available for camera " + camera_id);
        }
    }

    if (sync.mono_frames.size() < 2 || first_timestamps.empty()) {
        logMessage("Unable to synchronize streams: insufficient cameras with metadata");
        return sync;
    }

    double global_min_ts = std::numeric_limits<double>::max();
    for (const auto& [camera, ts] : first_timestamps) {
        global_min_ts = std::min(global_min_ts, ts);
    }

    for (const auto& [camera, ts] : first_timestamps) {
        sync.camera_offsets[camera] = ts - global_min_ts;
    }

    if (!sync.camera_offsets.empty()) {
        std::ostringstream oss;
        oss << "Estimated camera offsets (s): ";
        bool first = true;
        for (const auto& [camera, offset] : sync.camera_offsets) {
            if (!first) {
                oss << ", ";
            }
            oss << camera << '=' << std::fixed << std::setprecision(3) << offset;
            first = false;
        }
        logMessage(oss.str());
    }

    std::map<double, StreamSynchronization::TimestampGroup> grouped_frames;

    auto place_frame = [&](const std::string& camera, const StreamSynchronization::FramePtr& frame) {
        double offset = 0.0;
        auto offset_it = sync.camera_offsets.find(camera);
        if (offset_it != sync.camera_offsets.end()) {
            offset = offset_it->second;
        }

        double aligned_ts = frame->timestamp - offset;

        auto lower = grouped_frames.lower_bound(aligned_ts);

        auto try_place = [&](std::map<double, StreamSynchronization::TimestampGroup>::iterator it) {
            if (it != grouped_frames.end() && std::abs(it->first - aligned_ts) <= kTimestampTolerance) {
                it->second[camera] = frame;
                return true;
            }
            return false;
        };

        if (try_place(lower)) {
            return;
        }

        if (lower != grouped_frames.begin()) {
            auto prev = std::prev(lower);
            if (try_place(prev)) {
                return;
            }
        }

        grouped_frames[aligned_ts][camera] = frame;
    };

    for (const auto& [camera, frames] : sync.mono_frames) {
        for (const auto& frame : frames) {
            if (should_stop_.load()) {
                return sync;
            }
            place_frame(camera, frame);
        }
    }

    size_t stereo_match_count = 0;
    for (const auto& [timestamp, group] : grouped_frames) {
        if (group.size() < 2) {
            continue;
        }

        sync.multi_groups[timestamp] = group;

        for (auto it1 = group.begin(); it1 != group.end(); ++it1) {
            auto it2 = it1;
            ++it2;
            for (; it2 != group.end(); ++it2) {
                std::string key = makeStereoKey(it1->first, it2->first);
                sync.stereo_groups[key][timestamp] = {it1->second, it2->second};
                ++stereo_match_count;
            }
        }
    }

    logMessage("Stream synchronization prepared " + std::to_string(sync.multi_groups.size()) +
               " multi-camera timestamps and " + std::to_string(stereo_match_count) +
               " stereo matches");

    return sync;
}

bool CalibrationWatcher::performMonoCalibration(const std::string& camera_id, const CalibrationParams& params) {

    auto frames = loadAndSelectBestFrames(camera_id, params);

    if (frames.size() < static_cast<size_t>(params.min_frames)) {
        logMessage("Not enough quality frames for camera " + camera_id +
                  ": " + std::to_string(frames.size()));
        return false;
    }

    // Подготовка данных для калибровки
    std::vector<std::vector<cv::Point3f>> object_points;
    std::vector<std::vector<cv::Point2f>> image_points;
    cv::Size image_size = frames.front().image.size();

    auto world_points = generateChessboardPoints(params);

    for (const auto& frame : frames) {
        std::vector<cv::Point2f> corners = frame.corners;
        if (corners.empty() && !frame.image.empty()) {
            findChessboardInFrame(frame.image, params, corners);
        }

        if (!corners.empty()) {
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

bool CalibrationWatcher::performStereoCalibration(
    const std::string& cam1,
    const std::string& cam2,
    const StreamSynchronization::StereoGroup& synchronized_frames,
    const CalibrationParams& params) {

    cv::Mat K1, K2, D1, D2;
    bool has_cam1 = getCameraMatrix(cam1, K1, D1);
    bool has_cam2 = getCameraMatrix(cam2, K2, D2);

    if (!has_cam1 || !has_cam2) {
        logMessage("Mono calibration results not found for stereo pair: " + cam1 + " + " + cam2);
        return false;
    }


    if (synchronized_frames.empty()) {
        logMessage("No synchronized frames available for stereo pair: " + cam1 + " + " + cam2);
        return false;
    }


    struct Candidate {
        double timestamp;
        StreamSynchronization::FramePtr left;
        StreamSynchronization::FramePtr right;
        float score;
    };

    std::vector<Candidate> candidates;
    candidates.reserve(synchronized_frames.size());

    for (const auto& [timestamp, frame_pair] : synchronized_frames) {
        const auto& left = frame_pair.first;
        const auto& right = frame_pair.second;
        if (!left || !right) {
            continue;
        }
        if (left->image.empty() || right->image.empty()) {
            continue;
        }

        float pair_score = std::min(left->quality.overall_score, right->quality.overall_score);
        candidates.push_back({timestamp, left, right, pair_score});
    }

    if (candidates.size() < static_cast<size_t>(params.min_frames)) {
        logMessage("Not enough synchronized frames for stereo pair: " + cam1 + " + " + cam2 +
                   " (available " + std::to_string(candidates.size()) + ")");
        return false;
    }

    std::sort(candidates.begin(), candidates.end(), [](const Candidate& a, const Candidate& b) {
        if (std::abs(a.score - b.score) > 1e-3f) {
            return a.score > b.score;
        }
        return a.timestamp < b.timestamp;
    });

    auto world_points = generateChessboardPoints(params);
    std::vector<std::vector<cv::Point3f>> object_points;
    std::vector<std::vector<cv::Point2f>> image_points1;
    std::vector<std::vector<cv::Point2f>> image_points2;

    cv::Size image_size;
    bool image_size_initialized = false;

    size_t processed_candidates = 0;

    for (const auto& candidate : candidates) {
        if (should_stop_.load()) {
            return false;
        }

        const auto& left = candidate.left;
        const auto& right = candidate.right;

        if (!image_size_initialized) {
            image_size = left->image.size();
            image_size_initialized = true;
        }

        if (left->image.size() != image_size || right->image.size() != image_size) {
            logMessage("Skipping synchronized frame (timestamp " + std::to_string(candidate.timestamp) +
                       ") for pair " + cam1 + " + " + cam2 +
                       ": image size mismatch");
            continue;
        }

        std::vector<cv::Point2f> corners_left = left->corners;
        std::vector<cv::Point2f> corners_right = right->corners;

        if (corners_left.empty() && !left->image.empty()) {
            if (!findChessboardInFrame(left->image, params, corners_left)) {
                logMessage("Skipping timestamp " + std::to_string(candidate.timestamp) +
                           " for camera " + cam1 + ": chessboard not detected");
                continue;
            }
        }

        if (corners_right.empty() && !right->image.empty()) {
            if (!findChessboardInFrame(right->image, params, corners_right)) {
                logMessage("Skipping timestamp " + std::to_string(candidate.timestamp) +
                           " for camera " + cam2 + ": chessboard not detected");
                continue;
            }
        }

        if (corners_left.size() != corners_right.size() || corners_left.empty()) {
            logMessage("Skipping synchronized frame (timestamp " + std::to_string(candidate.timestamp) +
                       ") due to inconsistent corner detection for pair " + cam1 + " + " + cam2);
            continue;
        }

        object_points.push_back(world_points);
        image_points1.push_back(std::move(corners_left));
        image_points2.push_back(std::move(corners_right));
        processed_candidates++;

        if (object_points.size() >= static_cast<size_t>(params.max_frames)) {
            break;

        }
    }


    if (object_points.size() < static_cast<size_t>(params.min_frames)) {
        logMessage("Not enough valid synchronized stereo frames for " + cam1 + " + " + cam2 +
                   ": collected " + std::to_string(object_points.size()) +
                   " from " + std::to_string(candidates.size()) + " matches");
        return false;
    }

    StereoCalibrationResult result;
    result.camera_pair = cam1 + "_" + cam2;

    auto now = std::time(nullptr);
    std::ostringstream time_stream;
    time_stream << std::put_time(std::localtime(&now), "%Y-%m-%d %H:%M:%S");
    result.calibration_time = time_stream.str();


    try {
        result.reprojection_error = cv::stereoCalibrate(
            object_points, image_points1, image_points2,
            K1, D1, K2, D2, image_size,
            result.R, result.T, result.E, result.F,
            cv::CALIB_FIX_INTRINSIC,
            cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 100, 1e-5));

        cv::stereoRectify(K1, D1, K2, D2, image_size,
                          result.R, result.T,
                          result.R1, result.R2, result.P1, result.P2, result.Q);

        result.success = (result.reprojection_error < 1.0);


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
                  ": error = " + std::to_string(result.reprojection_error) +
                  ", frames used = " + std::to_string(object_points.size()) +
                  ", candidates processed = " + std::to_string(processed_candidates));
    }

    return result.success;
}

void CalibrationWatcher::performMultiCameraCalibration(const std::vector<std::string>& camera_ids,
                                                       const CalibrationParams& params,
                                                       const StreamSynchronization& sync_data) {
    if (camera_ids.size() < 2) {
        return;
    }

    size_t potential_pairs = camera_ids.size() * (camera_ids.size() - 1) / 2;
    logMessage("Performing multi-camera calibration for " + std::to_string(camera_ids.size()) +
               " cameras (" + std::to_string(potential_pairs) + " potential pairs)");

    int successful_pairs = 0;
    int attempted_pairs = 0;
    int skipped_pairs = 0;

    for (size_t i = 0; i < camera_ids.size(); ++i) {
        for (size_t j = i + 1; j < camera_ids.size(); ++j) {

            if (should_stop_.load()) {
                logMessage("Stereo calibration interrupted by stop request");
                break;
            }

            const auto& cam1 = camera_ids[i];
            const auto& cam2 = camera_ids[j];
            std::string key = makeStereoKey(cam1, cam2);

            auto group_it = sync_data.stereo_groups.find(key);
            if (group_it == sync_data.stereo_groups.end()) {
                logMessage("No synchronized frames for stereo pair: " + cam1 + " + " + cam2);
                skipped_pairs++;
                continue;
            }

            if (group_it->second.size() < static_cast<size_t>(params.min_frames)) {
                logMessage("Not enough synchronized matches (" +
                           std::to_string(group_it->second.size()) +
                           ") for stereo pair: " + cam1 + " + " + cam2);
                skipped_pairs++;
                continue;
            }

            attempted_pairs++;
            if (performStereoCalibration(cam1, cam2, group_it->second, params)) {
                successful_pairs++;
            }
        }
        if (should_stop_.load()) {
            break;
        }
    }


    logMessage("Multi-camera calibration completed: " +
               std::to_string(successful_pairs) + "/" + std::to_string(attempted_pairs) +
               " pairs successful, " + std::to_string(skipped_pairs) + " skipped due to missing data");
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