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
#include <set>
#include <cctype>
#include <numeric>


#include <utility>
#include <cassert>
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


std::string toLowerCopy(const std::string& value) {
    std::string result = value;
    std::transform(result.begin(), result.end(), result.begin(),
                   [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    return result;
}

bool containsAlpha(const std::string& token) {
    return std::any_of(token.begin(), token.end(), [](unsigned char c) {
        return std::isalpha(c) != 0;
    });
}

bool isResolutionToken(const std::string& token) {
    if (token.empty()) {
        return false;
    }

    return std::regex_match(token, std::regex(R"((\d+)[xX](\d+))"));
}

bool isMostlyNumeric(const std::string& token) {
    if (token.empty()) {
        return false;
    }

    size_t digit_count = std::count_if(token.begin(), token.end(), [](unsigned char c) {
        return std::isdigit(c) != 0;
    });

    return digit_count >= token.size() - 1;
}

std::string stripModePrefix(const std::string& token) {
    static const std::vector<std::string> prefixes = {"mode-", "mode_", "profile-", "profile_"};
    for (const auto& prefix : prefixes) {
        if (token.size() > prefix.size() &&
            std::equal(prefix.begin(), prefix.end(), token.begin(),
                       [](char a, char b) { return std::tolower(a) == std::tolower(b); })) {
            return token.substr(prefix.size());
        }
    }
    return token;
}

bool isLikelyModeToken(const std::string& token) {
    if (token.empty()) {
        return false;
    }

    std::string stripped = stripModePrefix(token);
    std::string lower = toLowerCopy(stripped);

    static const std::set<std::string> kKnownModes = {
        "wide", "wide_angle", "wide-angle", "wideangle", "zoom", "zoom_static", "zoom_variable",
        "zoomvariable", "tele", "telephoto", "ultrawide"
    };

    if (kKnownModes.count(lower)) {
        return true;
    }

    if (lower.rfind("zoom", 0) == 0 || lower.rfind("wide", 0) == 0 ||
        lower.rfind("tele", 0) == 0) {
        return true;
    }

    if (isResolutionToken(token) || isMostlyNumeric(token)) {
        return false;
    }

    return containsAlpha(token);
}

std::string sanitizePathComponent(const std::string& value) {
    std::string result;
    result.reserve(value.size());
    for (unsigned char c : value) {
        if (std::isalnum(c) || c == '_' || c == '-') {
            result.push_back(static_cast<char>(c));
        } else {
            result.push_back('_');
        }
    }
    return result;
}



double computeMean(const std::vector<double>& values) {
    if (values.empty()) {
        return 0.0;
    }
    double sum = std::accumulate(values.begin(), values.end(), 0.0);
    return sum / static_cast<double>(values.size());
}

double computeStandardDeviation(const std::vector<double>& values, double mean) {
    if (values.size() < 2) {
        return 0.0;
    }
    double accum = 0.0;
    for (double v : values) {
        double diff = v - mean;
        accum += diff * diff;
    }
    return std::sqrt(accum / static_cast<double>(values.size()));
}

double computePercentile(const std::vector<double>& sorted_values, double percentile) {
    if (sorted_values.empty()) {
        return 0.0;
    }

    if (percentile <= 0.0) {
        return sorted_values.front();
    }
    if (percentile >= 100.0) {
        return sorted_values.back();
    }

    double position = (percentile / 100.0) * (sorted_values.size() - 1);
    size_t index = static_cast<size_t>(position);
    double fraction = position - static_cast<double>(index);

    if (index + 1 < sorted_values.size()) {
        return sorted_values[index] +
               fraction * (sorted_values[index + 1] - sorted_values[index]);
    }

    return sorted_values[index];
}

double findNearestDifference(double value, const std::vector<double>& sorted_values) {
    if (sorted_values.empty()) {
        return std::numeric_limits<double>::quiet_NaN();
    }

    auto upper = std::lower_bound(sorted_values.begin(), sorted_values.end(), value);

    bool has_candidate = false;
    double best_diff = 0.0;

    if (upper != sorted_values.end()) {
        best_diff = *upper - value;
        has_candidate = true;
    }

    if (upper != sorted_values.begin()) {
        auto prev = upper;
        --prev;
        double diff = *prev - value;
        if (!has_candidate || std::abs(diff) < std::abs(best_diff)) {
            best_diff = diff;
            has_candidate = true;
        }
    }

    if (!has_candidate) {
        return std::numeric_limits<double>::quiet_NaN();
    }

    return best_diff;
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

std::string CalibrationWatcher::normalizeModeName(const std::string& mode) {
    if (mode.empty()) {
        return std::string();
    }

    std::string trimmed = mode;
    auto is_space = [](unsigned char c) { return std::isspace(c) != 0; };
    trimmed.erase(trimmed.begin(), std::find_if_not(trimmed.begin(), trimmed.end(), is_space));
    trimmed.erase(std::find_if_not(trimmed.rbegin(), trimmed.rend(), is_space).base(), trimmed.end());

    trimmed = stripModePrefix(trimmed);

    std::string normalized;
    normalized.reserve(trimmed.size());
    for (unsigned char c : trimmed) {
        if (std::isalnum(c)) {
            normalized.push_back(static_cast<char>(std::tolower(c)));
        } else if (c == '-' || c == ' ' || c == '\t' || c == '\n') {
            normalized.push_back('_');
        } else {
            normalized.push_back('_');
        }
    }

    while (!normalized.empty() && normalized.front() == '_') {
        normalized.erase(normalized.begin());
    }
    while (!normalized.empty() && normalized.back() == '_') {
        normalized.pop_back();
    }

    return normalized;
}

std::string CalibrationWatcher::makeProfileKey(const std::string& camera_id, const std::string& mode) {
    std::string normalized_mode = normalizeModeName(mode);
    if (normalized_mode.empty()) {
        return camera_id;
    }
    return camera_id + "@" + normalized_mode;
}

std::pair<std::string, std::string> CalibrationWatcher::splitProfileKey(const std::string& profile_key) {
    auto pos = profile_key.find('@');
    if (pos == std::string::npos) {
        return {profile_key, std::string()};
    }

    std::string camera = profile_key.substr(0, pos);
    std::string mode = profile_key.substr(pos + 1);
    return {camera, normalizeModeName(mode)};
}

std::filesystem::path CalibrationWatcher::cameraModeRootDir(const std::string& camera_id,
                                                            const std::string& mode) const {
    std::filesystem::path root = calib_dir_ / ("cam_" + sanitizePathComponent(camera_id));
    std::string normalized_mode = normalizeModeName(mode);
    if (!normalized_mode.empty()) {
        root /= "mode_" + sanitizePathComponent(normalized_mode);
    }
    return root;
}

std::filesystem::path CalibrationWatcher::cameraModeImagesDir(const std::string& camera_id,
                                                              const std::string& mode) const {
    return cameraModeRootDir(camera_id, mode) / "images";
}

CalibrationWatcher::~CalibrationWatcher() {
    stopCalibration();
}


void CalibrationWatcher::setLifecycleCallbacks(CalibrationWatcher::LifecycleCallback on_begin,
                                               CalibrationWatcher::LifecycleCallback on_finish) {
    std::lock_guard<std::mutex> lock(lifecycle_mutex_);
    pre_calibration_callback_ = std::move(on_begin);
    post_calibration_callback_ = std::move(on_finish);
}

bool CalibrationWatcher::invokePreCalibrationCallback() {
    CalibrationWatcher::LifecycleCallback callback;
    {
        std::lock_guard<std::mutex> lock(lifecycle_mutex_);
        callback = pre_calibration_callback_;
    }

    if (!callback) {
        return true;
    }

    try {
        callback();
        return true;
    } catch (const std::exception& e) {
        logMessage(std::string("Pre-calibration hook failed: ") + e.what());
    } catch (...) {
        logMessage("Pre-calibration hook failed with unknown error");
    }
    return false;
}

void CalibrationWatcher::invokePostCalibrationCallback() {
    CalibrationWatcher::LifecycleCallback callback;
    {
        std::lock_guard<std::mutex> lock(lifecycle_mutex_);
        callback = post_calibration_callback_;
    }

    if (!callback) {
        return;
    }

    try {
        callback();
    } catch (const std::exception& e) {
        logMessage(std::string("Post-calibration hook failed: ") + e.what());
    } catch (...) {
        logMessage("Post-calibration hook failed with unknown error");
    }
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

    if (!invokePreCalibrationCallback()) {
        processing_ = false;
        updateStatus("Calibration aborted before start", 100.0f);
        invokePostCalibrationCallback();
        return false;
    }

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
    try {
        worker_thread_ = std::thread([this, params]() {
            struct CallbackGuard {
                CalibrationWatcher* watcher;
                ~CallbackGuard() {
                    watcher->invokePostCalibrationCallback();
                }
            } guard{this};

            this->calibrationWorker(params);
        });
    } catch (const std::exception& e) {
        logMessage(std::string("Failed to start calibration thread: ") + e.what());
        invokePostCalibrationCallback();
        processing_ = false;
        updateStatus("Failed to start calibration", 100.0f);
        return false;
    } catch (...) {
        logMessage("Failed to start calibration thread: unknown error");
        invokePostCalibrationCallback();
        processing_ = false;
        updateStatus("Failed to start calibration", 100.0f);
        return false;
    }
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


CalibrationWatcher::RecordingInventory CalibrationWatcher::getRecordingInventory() {
    RecordingInventory inventory;

    auto video_files = scanRecordingFolder();
    std::set<std::string> seen_pairs;

    for (const auto& video : video_files) {
        if (!isVideoComplete(video)) {
            continue;
        }

        RecordingInventory::VideoInfo info;
        info.camera_id = video.camera_id;
        info.mode = video.mode;
        info.capture_type = video.capture_type;
        info.capture_group = video.capture_group;
        info.path = video.path;
        info.last_modified = video.last_modified;
        info.file_size = video.file_size;
        inventory.videos.push_back(std::move(info));

        std::vector<std::string> group = video.capture_group;
        if (group.empty() && !video.camera_id.empty()) {
            group.push_back(video.camera_id);
        }

        if (group.empty()) {
            continue;
        }

        std::sort(group.begin(), group.end());
        group.erase(std::unique(group.begin(), group.end()), group.end());

        if (group.empty()) {
            continue;
        }

        if ((video.capture_type == "stereo" || group.size() == 2) && group.size() >= 2) {
            std::string key = group.front();
            for (size_t i = 1; i < group.size(); ++i) {
                key.append(",").append(group[i]);
            }
            if (seen_pairs.insert(key).second) {
                inventory.stereo_pairs.push_back(group);
            }
        }

        for (const auto& camera : group) {
            inventory.mono_cameras.insert(camera);
        }
    }

    return inventory;
}


void CalibrationWatcher::calibrationWorker(const CalibrationParams& params) {
    auto stopRequested = [this]() { return should_stop_.load(); };
    try {
        {
            std::lock_guard<std::mutex> lock(camera_offsets_mutex_);
            camera_time_offsets_.clear();
        }
        // 1. Сканируем записанные видео
        updateStatus("Scanning recorded videos...", 5.0f);
        auto video_files = scanRecordingFolder();

        if (video_files.empty()) {
            updateStatus("No video files found", 100.0f);
            processing_ = false;
            return;
        }

        logMessage("Found " + std::to_string(video_files.size()) + " video files");


        // 2. Индексация видео (PTS)
        const float index_start = 10.0f;
        const float index_end = 30.0f;
        updateStatus("Indexing video timestamps (PTS)...", index_start);

        std::map<std::string, std::vector<double>> timestamp_index;
        const size_t total_videos = video_files.size();

        for (size_t i = 0; i < total_videos; ++i) {
            if (stopRequested()) {
                updateStatus("Calibration stopped by user", 100.0f);
                processing_ = false;
                return;
            }

            const auto& video = video_files[i];
            const std::string profile_key = video.profileKey();
            bool skipped = false;
            bool indexed = false;

            if (!isVideoComplete(video)) {
                skipped = true;
                logMessage("Skipping timestamp indexing for incomplete video: " +
                           video.path.string());
            } else {
                std::vector<double> timestamps;
                if (indexVideoPTS(video, timestamps) && !timestamps.empty()) {
                    timestamp_index.emplace(profile_key, std::move(timestamps));
                    indexed = true;
                }
            }

            float progress = index_start;
            if (total_videos > 0) {
                progress += (index_end - index_start) *
                            static_cast<float>(i + 1) / static_cast<float>(total_videos);
            }

            std::ostringstream message;
            message << "Indexing video timestamps (" << (i + 1) << "/" << total_videos
                    << ") for camera " << profile_key;
            if (skipped) {
                message << " (skipped)";
            } else if (!indexed) {
                message << " (fallback)";
            }
            updateStatus(message.str(), progress);
        }

        if (stopRequested()) {
            updateStatus("Calibration stopped by user", 100.0f);
            processing_ = false;
            return;
        }


        updateStatus("Analyzing frame timing...", 32.0f);
        analyzeTimestampAlignment(video_files, timestamp_index, params);

        if (stopRequested()) {
            updateStatus("Calibration stopped by user", 100.0f);
            processing_ = false;
            return;
        }


        // 3. Детекция шахматки и сохранение углов
        const float detect_start = 35.0f;
        const float detect_end = 60.0f;
        updateStatus("Detecting chessboard patterns...", detect_start);

        auto processed_cameras = extractFramesFromAllVideos(
            video_files, params, timestamp_index, detect_start, detect_end);

        auto deduplicate_cameras = [](std::vector<std::string>& cameras) {
            std::sort(cameras.begin(), cameras.end());
            cameras.erase(std::unique(cameras.begin(), cameras.end()), cameras.end());
        };

        deduplicate_cameras(processed_cameras);
        if (!processed_cameras.empty()) {
            logMessage("Proceeding with calibration of " +
                       std::to_string(processed_cameras.size()) +
                       " unique cameras.");
        }

        if (processed_cameras.empty()) {
            updateStatus("No cameras processed successfully", 100.0f);
            processing_ = false;
            return;
        }

        if (stopRequested()) {
            updateStatus("Calibration stopped by user", 100.0f);
            processing_ = false;
            return;
        }

        updateStatus("Estimating camera offsets...", 62.0f);
        computeCameraOffsets(processed_cameras, params, timestamp_index);

        if (stopRequested()) {
            updateStatus("Calibration stopped by user", 100.0f);
            processing_ = false;
            return;
        }

        // 4. Синхронизация потоков
        updateStatus("Synchronizing camera streams...", 65.0f);
        auto synchronization = synchronizeStreams(processed_cameras, params);

        if (stopRequested()) {
            updateStatus("Calibration stopped by user", 100.0f);
            processing_ = false;
            return;
        }

        // 5. Отбор моно-кадров и моно-калибровка
        const float mono_start = 70.0f;
        const float mono_end = 90.0f;
        std::vector<std::string> successful_cameras;

        successful_cameras.reserve(processed_cameras.size());

        for (size_t i = 0; i < processed_cameras.size(); ++i) {
            if (stopRequested()) {
                updateStatus("Calibration stopped by user", 100.0f);
                processing_ = false;
                return;
            }
            const auto& camera_id = processed_cameras[i];
            std::ostringstream message;
            message << "Mono calibration (" << (i + 1) << "/"
                    << processed_cameras.size() << ") for camera " << camera_id;

            float progress = mono_start;
            if (!processed_cameras.empty()) {
                progress += (mono_end - mono_start) *
                            static_cast<float>(i) / static_cast<float>(processed_cameras.size());
            }
            updateStatus(message.str(), progress);

            logMessage("Calibrating camera: " + camera_id);
            if (performMonoCalibration(camera_id, params)) {
                successful_cameras.push_back(camera_id);
                logMessage("Mono calibration successful for camera: " + camera_id);
            } else {
                logMessage("Mono calibration failed for camera: " + camera_id);
            }

            progress = mono_start;
            if (!processed_cameras.empty()) {
                progress += (mono_end - mono_start) *
                            static_cast<float>(i + 1) /
                            static_cast<float>(processed_cameras.size());
            }
            updateStatus(message.str(), progress);

        }

        if (stopRequested()) {
            updateStatus("Calibration stopped by user", 100.0f);
            processing_ = false;
            return;
        }

        // 6. Стерео/мульти-калибровка
        deduplicate_cameras(successful_cameras);
        if (!successful_cameras.empty()) {
            logMessage("Mono calibration succeeded for " +
                       std::to_string(successful_cameras.size()) +
                       " unique cameras.");
        }
        if (successful_cameras.size() >= 2) {
            if (!synchronization.multi_groups.empty()) {
                updateStatus("Performing stereo/multi calibrations...", 92.0f);
                logMessage("Starting multi-camera calibration for " +
                           std::to_string(successful_cameras.size()) +
                           " unique cameras.");
                performMultiCameraCalibration(successful_cameras, params, synchronization);

                if (stopRequested()) {
                    updateStatus("Calibration stopped by user", 100.0f);
                    processing_ = false;
                    return;
                }
            } else {
                logMessage("No synchronized frame groups found. Skipping stereo calibration stage.");
            }
        } else {
            logMessage("Not enough unique cameras passed mono calibration for stereo stage");
        }


        if (stopRequested()) {
            updateStatus("Calibration stopped by user", 100.0f);
            processing_ = false;
            return;
        }

        // 7. Сохранение результатов
        updateStatus("Saving calibration results...", 97.0f);
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

    for (const auto& entry : std::filesystem::directory_iterator(record_dir_)) {
        if (!entry.is_regular_file()) continue;

        auto path = entry.path();
        if (path.extension() != ".avi") continue;


        std::string stem = path.stem().string();

        std::vector<std::string> tokens;
        tokens.reserve(8);
        std::stringstream ss(stem);
        std::string token;
        while (std::getline(ss, token, '_')) {
            if (!token.empty()) {
                tokens.push_back(token);
            }
        }

        if (tokens.empty()) {
            continue;
        }
        VideoFile video;
        video.path = path;

        size_t prefix_tokens = 0;
        if (!tokens.empty()) {
            struct PrefixPattern {
                const char* label;
                size_t payload_count;
            };
            static const std::vector<PrefixPattern> kPatterns = {
                {"mono", 1},
                {"stereo", 2},
            };

            for (const auto& pattern : kPatterns) {
                if (tokens[0] == pattern.label && tokens.size() > pattern.payload_count) {
                    video.capture_type = normalizeModeName(pattern.label);
                    video.capture_group.assign(tokens.begin() + 1,
                                               tokens.begin() + 1 + pattern.payload_count);
                    prefix_tokens = 1 + pattern.payload_count;
                    break;
                }
            }
        }

        if (prefix_tokens >= tokens.size()) {
            prefix_tokens = tokens.size() - 1;
        }

        video.camera_id = tokens[prefix_tokens];

        size_t mode_token_index = prefix_tokens + 1;
        if (mode_token_index < tokens.size() && isLikelyModeToken(tokens[mode_token_index])) {
            video.mode = normalizeModeName(tokens[mode_token_index]);
        }

        if (video.capture_type == "mono" && video.capture_group.empty()) {
            video.capture_group.push_back(video.camera_id);
        }

        if (!video.capture_group.empty()) {
            if (std::find(video.capture_group.begin(), video.capture_group.end(), video.camera_id) ==
                video.capture_group.end()) {
                video.capture_group.push_back(video.camera_id);
            }
            std::sort(video.capture_group.begin(), video.capture_group.end());
            video.capture_group.erase(std::unique(video.capture_group.begin(), video.capture_group.end()),
                                      video.capture_group.end());
        }

        std::error_code ec;
        video.file_size = std::filesystem::file_size(path, ec);
        if (ec) continue;

        auto file_time = std::filesystem::last_write_time(path, ec);
        if (ec) continue;

        auto sctp = std::chrono::time_point_cast<std::chrono::system_clock::duration>(
            file_time - std::filesystem::file_time_type::clock::now() +
            std::chrono::system_clock::now());
        video.last_modified = std::chrono::system_clock::to_time_t(sctp);

        files.push_back(video);
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


bool CalibrationWatcher::indexVideoPTS(const VideoFile& video, std::vector<double>& timestamps) {
    timestamps.clear();

    FFmpegFramePTSReader reader;
    if (!reader.open(video.path.string())) {
        logMessage("Failed to open video for timestamp indexing: " + video.path.string());
        return false;
    }

    double timestamp = 0.0;
    while (!should_stop_.load()) {
        if (!reader.nextTimestamp(timestamp)) {
            break;
        }
        timestamps.push_back(timestamp);
    }

    if (!timestamps.empty()) {
        logMessage("Indexed " + std::to_string(timestamps.size()) +
                   " timestamps for camera " + video.profileKey());
        return true;
    }

    logMessage("No timestamps indexed for camera " + video.profileKey() +
               "; fallback timestamps will be used");
    return false;
}


void CalibrationWatcher::analyzeTimestampAlignment(
    const std::vector<VideoFile>& video_files,
    const std::map<std::string, std::vector<double>>& timestamp_index,
    const CalibrationParams& params) const {

    if (video_files.empty()) {
        return;
    }

    std::map<std::string, VideoFile> unique_videos;
    for (const auto& video : video_files) {
        unique_videos.try_emplace(video.profileKey(), video);
    }

    nlohmann::json analysis;

    auto now = std::time(nullptr);
    std::tm local_tm{};
#if defined(_WIN32)
    localtime_s(&local_tm, &now);
#else
    localtime_r(&now, &local_tm);
#endif
    std::ostringstream timestamp_stream;
    timestamp_stream << std::put_time(&local_tm, "%Y-%m-%d %H:%M:%S");

    analysis["generated_at"] = timestamp_stream.str();
    analysis["time_tolerance_ms"] = params.time_tolerance_ms;
    analysis["video_count"] = unique_videos.size();

    nlohmann::json cameras_json = nlohmann::json::object();

    std::map<std::string, std::vector<double>> sorted_timestamps;

    for (const auto& [profile, video] : unique_videos) {
        nlohmann::json camera_json;
        camera_json["camera_id"] = video.camera_id;
        camera_json["mode"] = video.mode;
        if (!video.capture_type.empty()) {
            camera_json["capture_type"] = video.capture_type;
        }
        if (!video.capture_group.empty()) {
            camera_json["capture_group"] = video.capture_group;
        }
        camera_json["profile"] = profile;
        camera_json["file_size_bytes"] = video.file_size;
        camera_json["has_pts"] = false;
        camera_json["frame_count"] = 0;
        camera_json["duration_s"] = 0.0;
        camera_json["fps_estimate"] = 0.0;

        auto ts_it = timestamp_index.find(profile);
        if (ts_it != timestamp_index.end() && !ts_it->second.empty()) {
            std::vector<double> timestamps = ts_it->second;
            std::sort(timestamps.begin(), timestamps.end());

            camera_json["has_pts"] = true;
            camera_json["frame_count"] = timestamps.size();
            sorted_timestamps.emplace(profile, timestamps);

            if (timestamps.size() >= 2) {
                double duration = timestamps.back() - timestamps.front();
                camera_json["duration_s"] = duration;
                camera_json["fps_estimate"] =
                    (duration > 0.0) ? static_cast<double>(timestamps.size() - 1) / duration : 0.0;

                std::vector<double> intervals_ms;
                intervals_ms.reserve(timestamps.size() - 1);
                for (size_t i = 1; i < timestamps.size(); ++i) {
                    intervals_ms.push_back((timestamps[i] - timestamps[i - 1]) * 1000.0);
                }

                if (!intervals_ms.empty()) {
                    std::vector<double> sorted_intervals = intervals_ms;
                    std::sort(sorted_intervals.begin(), sorted_intervals.end());

                    double mean_interval = computeMean(intervals_ms);
                    double std_interval = computeStandardDeviation(intervals_ms, mean_interval);

                    camera_json["interval_ms"] = {
                        {"min", sorted_intervals.front()},
                        {"max", sorted_intervals.back()},
                        {"mean", mean_interval},
                        {"median", computePercentile(sorted_intervals, 50.0)},
                        {"p90", computePercentile(sorted_intervals, 90.0)},
                        {"p95", computePercentile(sorted_intervals, 95.0)},
                        {"std", std_interval}
                    };
                }
            }
        }

        cameras_json[profile] = std::move(camera_json);
    }

    analysis["cameras"] = std::move(cameras_json);

    nlohmann::json pairs_json = nlohmann::json::object();

    std::vector<std::string> profiles;
    profiles.reserve(unique_videos.size());
    for (const auto& [profile, _] : unique_videos) {
        profiles.push_back(profile);
    }

    for (size_t i = 0; i < profiles.size(); ++i) {
        for (size_t j = i + 1; j < profiles.size(); ++j) {
            const auto& profile_left = profiles[i];
            const auto& profile_right = profiles[j];
            const auto& video_left = unique_videos.at(profile_left);
            const auto& video_right = unique_videos.at(profile_right);

            const std::string pair_key = makeStereoKey(profile_left, profile_right);

            nlohmann::json pair_json;
            pair_json["profile_left"] = profile_left;
            pair_json["profile_right"] = profile_right;
            pair_json["camera_left"] = video_left.camera_id;
            pair_json["camera_right"] = video_right.camera_id;
            pair_json["mode_left"] = video_left.mode;
            pair_json["mode_right"] = video_right.mode;
            if (!video_left.capture_type.empty()) {
                pair_json["capture_type_left"] = video_left.capture_type;
            }
            if (!video_right.capture_type.empty()) {
                pair_json["capture_type_right"] = video_right.capture_type;
            }
            if (!video_left.capture_group.empty()) {
                pair_json["capture_group_left"] = video_left.capture_group;
            }
            if (!video_right.capture_group.empty()) {
                pair_json["capture_group_right"] = video_right.capture_group;
            }
            if (!video_left.capture_group.empty() && !video_right.capture_group.empty()) {
                pair_json["shared_capture_group"] =
                    (video_left.capture_group == video_right.capture_group);
            }

            auto ts_left_it = sorted_timestamps.find(profile_left);
            auto ts_right_it = sorted_timestamps.find(profile_right);

            bool has_left = ts_left_it != sorted_timestamps.end();
            bool has_right = ts_right_it != sorted_timestamps.end();
            pair_json["has_pts_left"] = has_left;
            pair_json["has_pts_right"] = has_right;

            if (has_left && has_right) {
                const auto& ts_left = ts_left_it->second;
                const auto& ts_right = ts_right_it->second;

                std::vector<double> signed_diffs_ms;
                std::vector<double> abs_diffs_ms;
                signed_diffs_ms.reserve(ts_left.size());
                abs_diffs_ms.reserve(ts_left.size());

                for (double timestamp_left : ts_left) {
                    double diff_seconds = findNearestDifference(timestamp_left, ts_right);
                    if (!std::isfinite(diff_seconds)) {
                        continue;
                    }
                    double diff_ms = diff_seconds * 1000.0;
                    signed_diffs_ms.push_back(diff_ms);
                    abs_diffs_ms.push_back(std::abs(diff_ms));
                }

                const size_t match_count = signed_diffs_ms.size();
                pair_json["match_count"] = match_count;
                pair_json["frames_left"] = ts_left.size();
                pair_json["frames_right"] = ts_right.size();

                if (!signed_diffs_ms.empty()) {
                    std::vector<double> signed_sorted = signed_diffs_ms;
                    std::sort(signed_sorted.begin(), signed_sorted.end());

                    std::vector<double> abs_sorted = abs_diffs_ms;
                    std::sort(abs_sorted.begin(), abs_sorted.end());

                    double mean_signed = computeMean(signed_diffs_ms);
                    double std_signed = computeStandardDeviation(signed_diffs_ms, mean_signed);
                    double mean_abs = computeMean(abs_diffs_ms);
                    double std_abs = computeStandardDeviation(abs_diffs_ms, mean_abs);

                    double median_signed = computePercentile(signed_sorted, 50.0);
                    double median_abs = computePercentile(abs_sorted, 50.0);
                    double p90_abs = computePercentile(abs_sorted, 90.0);
                    double p95_abs = computePercentile(abs_sorted, 95.0);

                    pair_json["offset_ms"] = {
                        {"min", signed_sorted.front()},
                        {"max", signed_sorted.back()},
                        {"mean", mean_signed},
                        {"median", median_signed},
                        {"std", std_signed}
                    };

                    pair_json["abs_diff_ms"] = {
                        {"min", abs_sorted.front()},
                        {"max", abs_sorted.back()},
                        {"mean", mean_abs},
                        {"median", median_abs},
                        {"p90", p90_abs},
                        {"p95", p95_abs},
                        {"std", std_abs}
                    };

                    size_t within_tolerance = std::count_if(
                        abs_diffs_ms.begin(), abs_diffs_ms.end(),
                        [tol = params.time_tolerance_ms](double value) {
                            return value <= static_cast<double>(tol);
                        });

                    pair_json["matches_within_tolerance"] = within_tolerance;
                    pair_json["within_tolerance_ratio"] =
                        match_count > 0 ? static_cast<double>(within_tolerance) /
                                              static_cast<double>(match_count)
                                        : 0.0;
                    pair_json["coverage_vs_shorter_stream"] =
                        std::min<size_t>(ts_left.size(), ts_right.size()) > 0 ?
                            static_cast<double>(match_count) /
                                static_cast<double>(std::min(ts_left.size(), ts_right.size()))
                            : 0.0;

                    double recommended_tolerance_ms = std::max<double>(
                        params.time_tolerance_ms,
                        std::ceil(p95_abs + 1.0));
                    pair_json["recommended_tolerance_ms"] = recommended_tolerance_ms;
                    pair_json["needs_more_tolerance"] =
                        recommended_tolerance_ms > params.time_tolerance_ms;

                    std::ostringstream summary;
                    summary << "Timestamp analysis for pair "
                            << video_left.camera_id;
                    if (!video_left.mode.empty()) {
                        summary << " (" << video_left.mode << ")";
                    }
                    summary << " + " << video_right.camera_id;
                    if (!video_right.mode.empty()) {
                        summary << " (" << video_right.mode << ")";
                    }
                    summary << ": median offset " << std::fixed << std::setprecision(2)
                            << median_signed << " ms, 95th percentile difference "
                            << p95_abs << " ms (" << within_tolerance << "/"
                            << match_count << " within " << params.time_tolerance_ms
                            << " ms).";
                    if (recommended_tolerance_ms > params.time_tolerance_ms) {
                        summary << " Consider increasing tolerance to ~"
                                << static_cast<int>(recommended_tolerance_ms) << " ms.";
                    }
                    logMessage(summary.str());
                } else {
                    pair_json["match_count"] = 0;
                    logMessage("Timestamp analysis for pair " + video_left.camera_id +
                               " + " + video_right.camera_id +
                               ": insufficient overlapping timestamps to compute statistics.");
                }
            } else {
                logMessage("Timestamp analysis for pair " + video_left.camera_id + " + " +
                           video_right.camera_id +
                           ": missing timestamp information for one of the cameras.");
            }

            pairs_json[pair_key] = std::move(pair_json);
        }
    }

    analysis["pairs"] = std::move(pairs_json);

    try {
        std::error_code ec;
        std::filesystem::create_directories(results_dir_, ec);
        if (ec) {
            logMessage("WARNING: Failed to ensure results directory exists for timestamp analysis: " +
                       ec.message());
            return;
        }

        const auto analysis_path = results_dir_ / "timestamp_analysis.json";
        std::ofstream file(analysis_path);
        if (!file.is_open()) {
            logMessage("WARNING: Unable to write timestamp analysis to " + analysis_path.string());
            return;
        }
        file << analysis.dump(4);
        file.close();

        logMessage("Timestamp analysis saved to: " + analysis_path.string());
    } catch (const std::exception& e) {
        logMessage(std::string("WARNING: Exception while saving timestamp analysis: ") + e.what());
    }
}


std::vector<std::string> CalibrationWatcher::extractFramesFromAllVideos(
    const std::vector<VideoFile>& video_files,
    const CalibrationParams& params,
    const std::map<std::string, std::vector<double>>& timestamp_index,
    float progress_start,
    float progress_end) {
    std::vector<std::string> processed_cameras;

    if (video_files.empty()) {
        return processed_cameras;
    }

    const size_t total_videos = video_files.size();
    const float progress_range = std::max(0.0f, progress_end - progress_start);

    struct StageInfo {
        std::string text;
        float progress_before = 0.0f;
        float progress_after = 0.0f;
    };

    struct VideoProcessResult {
        std::string profile_key;
        bool skipped = false;
        bool success = false;
    };

    std::vector<StageInfo> stages(total_videos);
    for (size_t i = 0; i < total_videos; ++i) {
        const auto& video = video_files[i];
        const std::string profile_key = video.profileKey();

        StageInfo info;
        std::ostringstream stage_message;
        stage_message << "Detecting chessboard patterns (" << (i + 1) << "/"
                      << total_videos << ") for camera " << profile_key;
        info.text = stage_message.str();
        info.progress_before = progress_start + progress_range *
            (total_videos > 0 ? static_cast<float>(i) / static_cast<float>(total_videos) : 0.0f);
        info.progress_after = progress_start + progress_range *
            (total_videos > 0 ? static_cast<float>(i + 1) / static_cast<float>(total_videos) : 0.0f);

        stages[i] = std::move(info);
    }

    auto process_video = [&](size_t index) -> VideoProcessResult {
        VideoProcessResult result;
        if (index >= video_files.size() || should_stop_.load()) {
            return result;
        }

        const auto& video = video_files[index];
        result.profile_key = video.profileKey();

        if (!isVideoComplete(video)) {
            result.skipped = true;
            logMessage("Skipping incomplete video: " + video.path.string());
            return result;
        }

        logMessage("Processing video for camera: " + result.profile_key);

        const std::vector<double>* camera_index = nullptr;
        auto index_it = timestamp_index.find(result.profile_key);
        if (index_it != timestamp_index.end()) {
            camera_index = &index_it->second;
        }

        if (extractFramesFromVideo(video, params, camera_index)) {
            result.success = true;

            if (params.delete_videos) {
                std::error_code ec;
                std::filesystem::remove(video.path, ec);
                if (!ec) {
                    logMessage("Deleted video file: " + video.path.string());
                }
            }
        }

        return result;
    };

    auto build_completed_message = [](const StageInfo& stage, const VideoProcessResult& result) {

        std::ostringstream completed_message;
        completed_message << stage.text;
        if (result.skipped) {
            completed_message << " (skipped)";
        } else if (!result.success) {
            completed_message << " (no detections)";
        }
        return completed_message.str();
    };

    auto finalize_camera_list = [](std::vector<std::string>& cameras) {
        std::sort(cameras.begin(), cameras.end());
        cameras.erase(std::unique(cameras.begin(), cameras.end()), cameras.end());
    };

    if (!params.enable_multithreading || total_videos == 1) {
        for (size_t i = 0; i < total_videos && !should_stop_.load(); ++i) {
            const auto& stage = stages[i];
            updateStatus(stage.text, stage.progress_before);

            auto result = process_video(i);
            if (result.success) {
                processed_cameras.push_back(result.profile_key);
            }

            updateStatus(build_completed_message(stage, result), stage.progress_after);
        }
        finalize_camera_list(processed_cameras);
        if (!processed_cameras.empty()) {
            logMessage("Successfully processed frames for " +
                       std::to_string(processed_cameras.size()) +
                       " unique cameras.");
        }
        return processed_cameras;
    }

    // Многопоточная обработка видео
    std::mutex status_mutex;
    std::vector<VideoProcessResult> results(total_videos);
    std::atomic<size_t> next_index{0};

    unsigned int hw_threads = std::thread::hardware_concurrency();
    size_t thread_count = hw_threads > 0 ? static_cast<size_t>(hw_threads) : 2;
    thread_count = std::clamp<size_t>(thread_count, 2, total_videos);

    logMessage("Multithreaded calibration enabled: using " + std::to_string(thread_count) +
               " threads for video processing");

    auto worker = [&]() {
        while (!should_stop_.load()) {
            size_t index = next_index.fetch_add(1);
            if (index >= total_videos) {
                break;
            }

            const auto& stage = stages[index];
            {
                std::lock_guard<std::mutex> lock(status_mutex);
                updateStatus(stage.text, stage.progress_before);
            }

            auto result = process_video(index);
            results[index] = result;

            {
                std::lock_guard<std::mutex> lock(status_mutex);
                updateStatus(build_completed_message(stage, result), stage.progress_after);
            }

            if (should_stop_.load()) {
                break;
            }
        }
    };

    std::vector<std::thread> workers;
    workers.reserve(thread_count);
    for (size_t i = 0; i < thread_count; ++i) {
        workers.emplace_back(worker);
    }
    for (auto& worker_thread : workers) {
        if (worker_thread.joinable()) {
            worker_thread.join();
        }
    }

    if (should_stop_.load()) {
        finalize_camera_list(processed_cameras);
        return processed_cameras;
    }

    for (size_t i = 0; i < results.size(); ++i) {
        if (results[i].success) {
            processed_cameras.push_back(results[i].profile_key);
        }

    }

    finalize_camera_list(processed_cameras);
    if (!processed_cameras.empty()) {
        logMessage("Successfully processed frames for " +
                   std::to_string(processed_cameras.size()) +
                   " unique cameras.");
    }
    return processed_cameras;
}

bool CalibrationWatcher::extractFramesFromVideo(const VideoFile& video,
                                                const CalibrationParams& params,
                                                const std::vector<double>* timestamp_index) {
    auto profile = splitProfileKey(video.profileKey());
    const std::string& base_camera_id = profile.first;
    const std::string& mode = profile.second;

    std::filesystem::path cam_dir = cameraModeImagesDir(base_camera_id, mode);
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

    cv::Mat camera_matrix;
    cv::Mat dist_coeffs;
    bool has_intrinsics = getCameraMatrixForProfile(video.profileKey(), camera_matrix, dist_coeffs);

    // Получаем общее количество кадров
    bool using_indexed_timestamps = timestamp_index && !timestamp_index->empty();
    int total_frames = 0;
    if (using_indexed_timestamps) {
        total_frames = static_cast<int>(timestamp_index->size());
    }

    if (total_frames <= 0) {
        total_frames = static_cast<int>(cap.get(cv::CAP_PROP_FRAME_COUNT));
    }

    if (total_frames <= 0) {
        total_frames = params.max_frames;
    }

    int skip_interval = std::max(1, total_frames / std::max(1, params.max_frames));

    logMessage("Processing video: " + std::to_string(total_frames) +
               " frames, skip interval: " + std::to_string(skip_interval));

    if (!using_indexed_timestamps) {
        logMessage("PTS index unavailable for camera " + video.profileKey() +
                   ", using capture timestamps as fallback");
    }

    while (cap.read(frame) && saved_count < params.max_frames && !should_stop_.load()) {
        double timestamp_seconds = 0.0;
        if (using_indexed_timestamps &&
            frame_count < static_cast<int>(timestamp_index->size())) {
            timestamp_seconds = (*timestamp_index)[frame_count];
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
                detected_frame.corners,
                frame.size(),
                params.getPatternSize(),
                has_intrinsics ? camera_matrix : cv::Mat(),
                has_intrinsics ? dist_coeffs : cv::Mat());
            detected_frame.quality = quality;
            detected_frame.image = frame.clone();

            std::string filename = "frame_" + std::to_string(saved_count) +
                                 "_score_" + std::to_string(static_cast<int>(quality.overall_score)) + ".png";

            std::filesystem::path filepath = cam_dir / filename;
            detected_frame.image_path = filepath;

            if (cv::imwrite(filepath.string(), detected_frame.image)) {
                saved_count++;
                logMessage("Saved quality frame for " + video.profileKey() + ": " + filename +
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
               std::to_string(frame_count) + " total frames for camera " + video.profileKey());

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
                                     {"y", frame.pose.normalized_center.y}}},
            {"pixel_spans", {
                {"horizontal", frame.pose.pixel_span_horizontal},
                {"vertical", frame.pose.pixel_span_vertical},
                {"diagonal", frame.pose.pixel_span_diagonal}
            }},
            {"normalized_spans", {
                {"horizontal", frame.pose.normalized_span_horizontal},
                {"vertical", frame.pose.normalized_span_vertical},
                {"diagonal", frame.pose.normalized_span_diagonal}
            }}
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
    const cv::Size& pattern_size,
    const cv::Mat& camera_matrix,
    const cv::Mat& dist_coeffs) {

    BoardPose pose;

    if (corners.empty() || image_size.width <= 0 || image_size.height <= 0) {
        return pose;
    }


    const int cols = pattern_size.width;
    const int rows = pattern_size.height;

    auto computeAverageSpan = [&](const std::vector<cv::Point2f>& pts, bool horizontal) {
        if (cols <= 0 || rows <= 0 || static_cast<int>(pts.size()) < cols * rows) {
            return 0.0f;
        }

        float accum = 0.0f;
        int count = 0;

        if (horizontal) {
            if (cols < 2) {
                return 0.0f;
            }
            for (int r = 0; r < rows; ++r) {
                int idx0 = r * cols;
                int idx1 = idx0 + (cols - 1);
                const cv::Point2f vec = pts[idx1] - pts[idx0];
                accum += std::sqrt(vec.x * vec.x + vec.y * vec.y);
                ++count;
            }
        } else {
            if (rows < 2) {
                return 0.0f;
            }
            for (int c = 0; c < cols; ++c) {
                int idx0 = c;
                int idx1 = idx0 + (rows - 1) * cols;
                const cv::Point2f vec = pts[idx1] - pts[idx0];
                accum += std::sqrt(vec.x * vec.x + vec.y * vec.y);
                ++count;
            }
        }

        if (count == 0) {
            return 0.0f;
        }
        return accum / static_cast<float>(count);
    };


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

    pose.pixel_span_horizontal = computeAverageSpan(corners, true);
    pose.pixel_span_vertical = computeAverageSpan(corners, false);
    pose.pixel_span_diagonal = std::sqrt(
        pose.pixel_span_horizontal * pose.pixel_span_horizontal +
        pose.pixel_span_vertical * pose.pixel_span_vertical);

    bool normalized_success = false;
    if (!camera_matrix.empty() && camera_matrix.total() >= 9 && cols > 0 && rows > 0) {
        try {
            std::vector<cv::Point2f> undistorted;
            cv::undistortPoints(corners, undistorted, camera_matrix, dist_coeffs);
            pose.normalized_span_horizontal = computeAverageSpan(undistorted, true);
            pose.normalized_span_vertical = computeAverageSpan(undistorted, false);
            pose.normalized_span_diagonal = std::sqrt(
                pose.normalized_span_horizontal * pose.normalized_span_horizontal +
                pose.normalized_span_vertical * pose.normalized_span_vertical);
            normalized_success = pose.normalized_span_diagonal > 0.0f;
        } catch (const cv::Exception&) {
            normalized_success = false;
        }
    }

    if (normalized_success) {
        pose.scale = pose.normalized_span_diagonal;
    } else {
        const float width_f = static_cast<float>(image_size.width);
        const float height_f = static_cast<float>(image_size.height);
        float image_diag = std::sqrt(width_f * width_f + height_f * height_f);
        if (image_diag > 0.0f) {
            pose.scale = pose.pixel_span_diagonal / image_diag;
        } else {
            pose.scale = 0.0f;
        }
    }

    if (cols > 1 && static_cast<int>(corners.size()) >= cols) {
        int top_right_index = cols - 1;
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

    auto profile = splitProfileKey(camera_id);
    std::string label = makeProfileKey(profile.first, profile.second);
    std::filesystem::path cam_dir = cameraModeImagesDir(profile.first, profile.second);

    cv::Mat camera_matrix;
    cv::Mat dist_coeffs;
    bool has_intrinsics = getCameraMatrixForProfile(label, camera_matrix, dist_coeffs);

    if (!std::filesystem::exists(cam_dir)) {
        logMessage("Camera directory does not exist for " + label + ": " + cam_dir.string());
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
        detected.pose = analyzeBoardPose(detected.corners,
                                         detected.image.size(),
                                         params.getPatternSize(),
                                         has_intrinsics ? camera_matrix : cv::Mat(),
                                         has_intrinsics ? dist_coeffs : cv::Mat());

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
                    << clusters.size() << " pose clusters for camera " << label;
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
    const cv::Size pattern_size = params.getPatternSize();

    if (camera_ids.empty()) {
        return sync;
    }

    const double timestamp_tolerance = std::max(params.time_tolerance_ms, 1) / 1000.0;
    std::map<std::string, double> first_timestamps;

    for (const auto& camera_id : camera_ids) {
        if (should_stop_.load()) {
            return sync;
        }

        auto profile = splitProfileKey(camera_id);
        std::string label = makeProfileKey(profile.first, profile.second);
        std::filesystem::path cam_dir = cameraModeImagesDir(profile.first, profile.second);
        if (!std::filesystem::exists(cam_dir)) {
            logMessage("Synchronization skipped for camera " + label +
                       ": images directory not found at " + cam_dir.string());
            continue;
        }

        cv::Mat camera_matrix;
        cv::Mat dist_coeffs;
        bool has_intrinsics = getCameraMatrixForProfile(label, camera_matrix, dist_coeffs);


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

                BoardPose metadata_pose;
                bool metadata_pose_available = false;

                if (metadata.contains("pose") && metadata["pose"].is_object()) {
                    metadata_pose_available = true;
                    const auto& pose_json = metadata["pose"];
                    metadata_pose.tilt_degrees = pose_json.value("tilt_degrees", 0.0f);
                    metadata_pose.scale = pose_json.value("scale", 0.0f);

                    if (pose_json.contains("center") && pose_json["center"].is_object()) {
                        const auto& center_json = pose_json["center"];
                        metadata_pose.center.x = center_json.value("x", 0.0f);
                        metadata_pose.center.y = center_json.value("y", 0.0f);
                    }

                    if (pose_json.contains("normalized_center") && pose_json["normalized_center"].is_object()) {
                        const auto& normalized_json = pose_json["normalized_center"];
                        metadata_pose.normalized_center.x = normalized_json.value("x", 0.0f);
                        metadata_pose.normalized_center.y = normalized_json.value("y", 0.0f);
                    }

                    if (pose_json.contains("pixel_spans") && pose_json["pixel_spans"].is_object()) {
                        const auto& spans_json = pose_json["pixel_spans"];
                        metadata_pose.pixel_span_horizontal = spans_json.value("horizontal", 0.0f);
                        metadata_pose.pixel_span_vertical = spans_json.value("vertical", 0.0f);
                        metadata_pose.pixel_span_diagonal = spans_json.value("diagonal", 0.0f);
                    }

                    if (pose_json.contains("normalized_spans") && pose_json["normalized_spans"].is_object()) {
                        const auto& spans_json = pose_json["normalized_spans"];
                        metadata_pose.normalized_span_horizontal = spans_json.value("horizontal", 0.0f);
                        metadata_pose.normalized_span_vertical = spans_json.value("vertical", 0.0f);
                        metadata_pose.normalized_span_diagonal = spans_json.value("diagonal", 0.0f);
                    }
                }
                BoardPose computed_pose = analyzeBoardPose(
                    frame->corners,
                    frame->image.size(),
                    pattern_size,
                    has_intrinsics ? camera_matrix : cv::Mat(),
                    has_intrinsics ? dist_coeffs : cv::Mat());

                if (metadata_pose_available) {
                    if (computed_pose.normalized_span_diagonal <= 0.0f) {
                        computed_pose.normalized_span_horizontal = metadata_pose.normalized_span_horizontal;
                        computed_pose.normalized_span_vertical = metadata_pose.normalized_span_vertical;
                        computed_pose.normalized_span_diagonal = metadata_pose.normalized_span_diagonal;
                    }
                    if (computed_pose.pixel_span_diagonal <= 0.0f) {
                        computed_pose.pixel_span_horizontal = metadata_pose.pixel_span_horizontal;
                        computed_pose.pixel_span_vertical = metadata_pose.pixel_span_vertical;
                        computed_pose.pixel_span_diagonal = metadata_pose.pixel_span_diagonal;
                    }
                    if (computed_pose.scale <= 0.0f) {
                        computed_pose.scale = metadata_pose.scale;
                    }
                    if (computed_pose.tilt_degrees == 0.0f && metadata_pose.tilt_degrees != 0.0f) {
                        computed_pose.tilt_degrees = metadata_pose.tilt_degrees;
                    }
                    if (computed_pose.center == cv::Point2f() && metadata_pose.center != cv::Point2f()) {
                        computed_pose.center = metadata_pose.center;
                    }
                    if (computed_pose.normalized_center == cv::Point2f() &&
                        metadata_pose.normalized_center != cv::Point2f()) {
                        computed_pose.normalized_center = metadata_pose.normalized_center;
                    }
                }

                frame->pose = computed_pose;

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
                       " synchronized frames for camera " + label);
            sync.mono_frames[camera_id] = std::move(frames);
        } else {
            logMessage("No synchronized frames available for camera " + label);
        }
    }

    if (sync.mono_frames.size() < 2) {
        logMessage("Unable to synchronize streams: insufficient cameras with metadata");
        return sync;
    }

    double fallback_min_ts = std::numeric_limits<double>::max();
    for (const auto& [camera, ts] : first_timestamps) {
        fallback_min_ts = std::min(fallback_min_ts, ts);
    }

    {
        std::lock_guard<std::mutex> lock(camera_offsets_mutex_);
        for (const auto& camera : camera_ids) {
            auto it = camera_time_offsets_.find(camera);
            if (it != camera_time_offsets_.end()) {
                sync.camera_offsets[camera] = it->second;
            }
        }
    }

    if (sync.camera_offsets.size() >= 2) {
        double min_offset = std::numeric_limits<double>::max();
        for (const auto& [camera, offset] : sync.camera_offsets) {
            min_offset = std::min(min_offset, offset);
        }
        if (min_offset != std::numeric_limits<double>::max()) {
            for (auto& [camera, offset] : sync.camera_offsets) {
                offset -= min_offset;
            }
        }
        if (fallback_min_ts != std::numeric_limits<double>::max()) {
            for (const auto& [camera, ts] : first_timestamps) {
                if (sync.camera_offsets.find(camera) == sync.camera_offsets.end()) {
                    sync.camera_offsets[camera] = ts - fallback_min_ts;
                }
            }
        }
    } else {
        sync.camera_offsets.clear();
        if (first_timestamps.size() < 2 || fallback_min_ts == std::numeric_limits<double>::max()) {
            logMessage("Unable to derive synchronization offsets from metadata");
            return sync;
        }
        for (const auto& [camera, ts] : first_timestamps) {
            sync.camera_offsets[camera] = ts - fallback_min_ts;
        }
        logMessage("Using fallback synchronization offsets derived from first timestamps");
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
            if (it != grouped_frames.end() && std::abs(it->first - aligned_ts) <= timestamp_tolerance) {
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
    auto profile = splitProfileKey(camera_id);
    std::string label = makeProfileKey(profile.first, profile.second);

    auto frames = loadAndSelectBestFrames(label, params);

    if (frames.size() < static_cast<size_t>(params.min_frames)) {
        logMessage("Not enough quality frames for camera " + label +
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
        logMessage("Not enough valid calibration frames for camera: " + label);
        return false;
    }

    // Выполнение калибровки
    CameraCalibrationResult result;
    result.camera_id = profile.first;
    result.mode = profile.second;
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
        logMessage("OpenCV calibration error for camera " + label + ": " + e.what());
        result.success = false;
    }
    
    // Сохраняем результат
    {
        std::lock_guard<std::mutex> lock(results_mutex_);
        mono_results_.push_back(result);
        
        if (result.success) {
            camera_matrices_[result.camera_id][result.mode] = result.camera_matrix.clone();
            dist_coeffs_[result.camera_id][result.mode] = result.dist_coeffs.clone();
        }
    }
    
    if (result.success) {
        logMessage("Mono calibration successful for " + label +
                  ": error = " + std::to_string(result.reprojection_error) +
                  ", frames = " + std::to_string(result.frames_used));
    }
    
    return result.success;
}

bool CalibrationWatcher::performStereoCalibration(
    const std::string& cam_first,
    const std::string& cam_second,
    const StreamSynchronization::StereoGroup& synchronized_frames,
    const CalibrationParams& params) {

    cv::Mat K1, K2, D1, D2;

    bool has_cam_first = getCameraMatrixForProfile(cam_first, K1, D1);
    bool has_cam_second = getCameraMatrixForProfile(cam_second, K2, D2);

    std::string pair_label = cam_first + " + " + cam_second;

    if (!has_cam_first || !has_cam_second) {
        logMessage("Mono calibration results not found for stereo pair: " + pair_label);
        return false;
    }


    if (synchronized_frames.empty()) {
        logMessage("No synchronized frames available for stereo pair: " + pair_label);
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
        auto left = frame_pair.first;
        auto right = frame_pair.second;

        if (!left || !right) {
            continue;
        }
        if (left->image.empty() || right->image.empty()) {
            continue;
        }

        float pair_score = std::min(left->quality.overall_score, right->quality.overall_score);
        candidates.push_back({timestamp, left, right, pair_score});
    }

    if (candidates.size() < static_cast<size_t>(params.min_stereo_matches)) {
        logMessage("Not enough synchronized frames for stereo pair: " + pair_label +
                   " (available " + std::to_string(candidates.size()) +
                   ", required " + std::to_string(params.min_stereo_matches) + ")");
        return false;
    }

    std::sort(candidates.begin(), candidates.end(), [](const Candidate& a, const Candidate& b) {
        if (std::abs(a.score - b.score) > 1e-3f) {
            return a.score > b.score;
        }
        return a.timestamp < b.timestamp;
    });

    auto world_points = generateChessboardPoints(params);
    cv::Size pattern_size = params.getPatternSize();
    std::vector<std::vector<cv::Point3f>> object_points;
    std::vector<std::vector<cv::Point2f>> image_points1;
    std::vector<std::vector<cv::Point2f>> image_points2;

    cv::Size image_size;
    bool image_size_initialized = false;

    size_t processed_candidates = 0;
    size_t pose_mismatch_rejections = 0;

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
                       ") for pair " + pair_label +
                       ": image size mismatch");
            continue;
        }

        std::vector<cv::Point2f> corners_left = left->corners;
        std::vector<cv::Point2f> corners_right = right->corners;

        if (corners_left.empty() && !left->image.empty()) {
            if (!findChessboardInFrame(left->image, params, corners_left)) {
                logMessage("Skipping timestamp " + std::to_string(candidate.timestamp) +
                           " for camera " + cam_first + ": chessboard not detected");
                continue;
            }
        }

        if (corners_right.empty() && !right->image.empty()) {
            if (!findChessboardInFrame(right->image, params, corners_right)) {
                logMessage("Skipping timestamp " + std::to_string(candidate.timestamp) +
                           " for camera " + cam_second + ": chessboard not detected");
                continue;
            }
        }

        if (corners_left.size() != corners_right.size() || corners_left.empty()) {
            logMessage("Skipping synchronized frame (timestamp " + std::to_string(candidate.timestamp) +
                       ") due to inconsistent corner detection for pair " + pair_label);
            continue;
        }


        std::filesystem::path left_metadata_path;
        const std::filesystem::path* left_metadata_ptr = nullptr;
        if (!left->image_path.empty()) {
            left_metadata_path = left->image_path;
            left_metadata_path.replace_extension(".json");
            left_metadata_ptr = &left_metadata_path;
        }

        std::filesystem::path right_metadata_path;
        const std::filesystem::path* right_metadata_ptr = nullptr;
        if (!right->image_path.empty()) {
            right_metadata_path = right->image_path;
            right_metadata_path.replace_extension(".json");
            right_metadata_ptr = &right_metadata_path;
        }

        if (ensureConsistentCornerOrder(corners_left, pattern_size, candidate.timestamp,
                                        pair_label + " [left]", left_metadata_ptr)) {
            left->corners = corners_left;
            left->pose = analyzeBoardPose(left->corners, left->image.size(), pattern_size, K1, D1);
        }

        if (ensureConsistentCornerOrder(corners_right, pattern_size, candidate.timestamp,
                                        pair_label + " [right]", right_metadata_ptr)) {
            right->corners = corners_right;
            right->pose = analyzeBoardPose(right->corners, right->image.size(), pattern_size, K2, D2);
        }

        const auto& left_pose = left->pose;
        const auto& right_pose = right->pose;

        const float dx = std::abs(left_pose.normalized_center.x - right_pose.normalized_center.x);
        const float dy = std::abs(left_pose.normalized_center.y - right_pose.normalized_center.y);

        auto relativeDifference = [](float a, float b) {
            const float max_val = std::max(std::max(std::abs(a), std::abs(b)), 1e-6f);
            return std::abs(a - b) / max_val;
        };

        const bool spans_available =
            left_pose.normalized_span_diagonal > 0.0f && right_pose.normalized_span_diagonal > 0.0f;
        const float normalized_span_diff = spans_available
            ? relativeDifference(left_pose.normalized_span_diagonal, right_pose.normalized_span_diagonal)
            : 0.0f;
        const float normalized_horizontal_diff = spans_available
            ? relativeDifference(left_pose.normalized_span_horizontal, right_pose.normalized_span_horizontal)
            : 0.0f;
        const float normalized_vertical_diff = spans_available
            ? relativeDifference(left_pose.normalized_span_vertical, right_pose.normalized_span_vertical)
            : 0.0f;

        float delta_tilt = left_pose.tilt_degrees - right_pose.tilt_degrees;
        float tilt_diff = std::fabs(std::remainder(delta_tilt, 360.0f));
        if (tilt_diff > 180.0f) {
            tilt_diff = 360.0f - tilt_diff;
        }

#ifndef NDEBUG
        {
            std::ostringstream debug_ss;
            debug_ss << "Stereo tilt check: left=" << left_pose.tilt_degrees
                     << " right=" << right_pose.tilt_degrees
                     << " delta=" << delta_tilt
                     << " normalized=" << tilt_diff;
            logMessage(debug_ss.str());
        }
#endif


        bool pose_valid = true;
        std::string rejection_reason;

        if (dy > params.stereo_pose_max_center_diff_vertical) {
            std::ostringstream ss;
            ss << "pose mismatch: vertical center diff " << dy
               << " > " << params.stereo_pose_max_center_diff_vertical;
            rejection_reason = ss.str();
            pose_valid = false;
        } else if (dx > params.stereo_pose_max_center_diff_horizontal) {
            std::ostringstream ss;
            ss << "pose mismatch: horizontal center diff " << dx
               << " > " << params.stereo_pose_max_center_diff_horizontal;

            rejection_reason = ss.str();
            pose_valid = false;
        } else if (spans_available &&
                   normalized_span_diff > params.stereo_pose_max_normalized_span_ratio) {
            std::ostringstream ss;
            ss << "pose mismatch: normalized span diff " << normalized_span_diff
               << " > " << params.stereo_pose_max_normalized_span_ratio
               << " (h=" << normalized_horizontal_diff
               << ", v=" << normalized_vertical_diff << ")";
            rejection_reason = ss.str();
            pose_valid = false;
        } else if (tilt_diff > params.stereo_pose_max_tilt_diff) {
            std::ostringstream ss;
            ss << "pose mismatch: tilt diff " << tilt_diff
               << " > " << params.stereo_pose_max_tilt_diff;
            rejection_reason = ss.str();
            pose_valid = false;
        }

        if (!pose_valid) {
            auto formatSpans = [](const BoardPose& pose) {
                std::ostringstream oss;
                oss << "px[h=" << pose.pixel_span_horizontal
                    << ", v=" << pose.pixel_span_vertical
                    << ", d=" << pose.pixel_span_diagonal
                    << "], norm[h=" << pose.normalized_span_horizontal
                    << ", v=" << pose.normalized_span_vertical
                    << ", d=" << pose.normalized_span_diagonal << "]";
                return oss.str();
            };

            std::ostringstream msg;
            msg << "Skipping synchronized frame (timestamp " << candidate.timestamp
                << ") for pair " << pair_label << ": " << rejection_reason
                << "; left spans=" << formatSpans(left_pose)
                << ", right spans=" << formatSpans(right_pose);
            logMessage(msg.str());
            pose_mismatch_rejections++;
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


    if (object_points.size() < static_cast<size_t>(params.min_stereo_matches)) {
        logMessage("Not enough valid synchronized stereo frames for " + pair_label +
                   ": collected " + std::to_string(object_points.size()) +
                   " from " + std::to_string(candidates.size()) + " matches (required " +
                   std::to_string(params.min_stereo_matches) + ")" +
                   ", pose-filter rejects = " + std::to_string(pose_mismatch_rejections));
        return false;
    }

    StereoCalibrationResult result;
    result.camera_pair = cam_first + "_" + cam_second;

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
                  ", candidates processed = " + std::to_string(processed_candidates) +
                  ", pose-filter rejects = " + std::to_string(pose_mismatch_rejections));
    } else {
        logMessage("Stereo calibration failed for " + result.camera_pair +
                  ": error = " + std::to_string(result.reprojection_error) +
                  ", frames used = " + std::to_string(object_points.size()) +
                  ", candidates processed = " + std::to_string(processed_candidates) +
                  ", pose-filter rejects = " + std::to_string(pose_mismatch_rejections));
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
    std::set<std::string> processed_pairs;

    for (size_t i = 0; i < camera_ids.size(); ++i) {
        for (size_t j = i + 1; j < camera_ids.size(); ++j) {

            if (should_stop_.load()) {
                logMessage("Stereo calibration interrupted by stop request");
                break;
            }

            const auto& cam1 = camera_ids[i];
            const auto& cam2 = camera_ids[j];

            std::string first = std::min(cam1, cam2);
            std::string second = std::max(cam1, cam2);

            auto profile1 = splitProfileKey(first);
            auto profile2 = splitProfileKey(second);

            if (profile1.first == profile2.first) {
                skipped_pairs++;
                continue;
            }

            std::string key = makeStereoKey(first, second);

            if (!processed_pairs.insert(key).second) {
                continue;
            }
            auto group_it = sync_data.stereo_groups.find(key);
            if (group_it == sync_data.stereo_groups.end()) {
                logMessage("No synchronized frames for stereo pair: " + first + " + " + second);
                skipped_pairs++;
                continue;
            }

            if (group_it->second.size() < static_cast<size_t>(params.min_stereo_matches)) {
                logMessage("Not enough synchronized matches (" +
                           std::to_string(group_it->second.size()) +
                           ") for stereo pair: " + first + " + " + second +
                           " (required " + std::to_string(params.min_stereo_matches) + ")");
                skipped_pairs++;
                continue;
            }

            attempted_pairs++;
            if (performStereoCalibration(first, second, group_it->second, params)) {
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


bool CalibrationWatcher::ensureConsistentCornerOrder(std::vector<cv::Point2f>& corners,
                                                     const cv::Size& pattern_size,
                                                     double timestamp,
                                                     const std::string& context_label,
                                                     const std::filesystem::path* metadata_path) {
    const int width = pattern_size.width;
    const int height = pattern_size.height;
    const size_t expected_corners = static_cast<size_t>(width) * static_cast<size_t>(height);

    if (width < 2 || height < 2 || corners.size() < expected_corners) {
        return false;
    }

    auto compute_basis = [&](const std::vector<cv::Point2f>& pts) {
        cv::Point2f horizontal = pts[1] - pts[0];
        cv::Point2f vertical = pts[width] - pts[0];
        return std::make_pair(horizontal, vertical);
    };

    auto compute_cross = [&](const std::vector<cv::Point2f>& pts) {
        const auto [horizontal, vertical] = compute_basis(pts);
        return static_cast<double>(horizontal.x) * static_cast<double>(vertical.y)
             - static_cast<double>(horizontal.y) * static_cast<double>(vertical.x);
    };

    auto flip_horizontal = [&](std::vector<cv::Point2f>& pts) {
        for (int r = 0; r < height; ++r) {
            auto row_begin = pts.begin() + static_cast<size_t>(r) * width;
            std::reverse(row_begin, row_begin + width);
        }
    };

    auto flip_vertical = [&](std::vector<cv::Point2f>& pts) {
        for (int r = 0; r < height / 2; ++r) {
            auto top_begin = pts.begin() + static_cast<size_t>(r) * width;
            auto bottom_begin = pts.begin() + static_cast<size_t>(height - 1 - r) * width;
            for (int c = 0; c < width; ++c) {
                std::swap(*(top_begin + c), *(bottom_begin + c));
            }
        }
    };
    auto update_metadata = [&]() {
        if (metadata_path && !metadata_path->empty() && std::filesystem::exists(*metadata_path)) {
            try {
                nlohmann::json metadata;
                {
                    std::ifstream in(*metadata_path);
                    if (in.is_open()) {
                        in >> metadata;
                    } else {
                        logMessage("WARNING: Unable to open metadata for updating corners: "
                                   + metadata_path->string());
                        return true;
                    }
                }

                nlohmann::json corners_json = nlohmann::json::array();
                for (const auto& pt : corners) {
                    corners_json.push_back({{"x", pt.x}, {"y", pt.y}});
                }
                metadata["corners"] = std::move(corners_json);

                std::ofstream out(*metadata_path);
                if (out.is_open()) {
                    out << metadata.dump(2);
                } else {
                    logMessage("WARNING: Unable to write updated metadata for "
                               + metadata_path->string());
                }
            } catch (const std::exception& e) {
                logMessage("WARNING: Failed to update metadata corners for " + metadata_path->string()
                           + ": " + e.what());
            }
        }
        return true;
    };

    auto apply_left_handed_fix = [&]() {

        const std::vector<cv::Point2f> original_corners = corners;
        bool flipped_horizontal = false;
        bool flipped_vertical = false;

        std::vector<cv::Point2f> candidate = original_corners;
        flip_horizontal(candidate);
        if (compute_cross(candidate) > 0.0) {
            corners = std::move(candidate);
            flipped_horizontal = true;
        } else {
            candidate = original_corners;
            flip_vertical(candidate);
            if (compute_cross(candidate) > 0.0) {
                corners = std::move(candidate);
                flipped_vertical = true;
            } else {
                candidate = original_corners;
                flip_horizontal(candidate);
                flip_vertical(candidate);
                corners = std::move(candidate);
                flipped_horizontal = true;
                flipped_vertical = true;
            }
        }

        const double corrected_cross = compute_cross(corners);
        if (corrected_cross <= 0.0) {
            logMessage("ERROR: Chessboard basis remains left-handed after reordering corners for "
                       + context_label + " at timestamp " + std::to_string(timestamp));
        }
        assert(corrected_cross > 0.0 &&
               "Chessboard basis should be right-handed after reordering corners");

        if (flipped_horizontal || flipped_vertical) {
            std::ostringstream msg_stream;
            msg_stream << "Flipped chessboard corner order";
            if (flipped_horizontal && flipped_vertical) {
                msg_stream << " horizontally and vertically";
            } else if (flipped_horizontal) {
                msg_stream << " horizontally";
            } else if (flipped_vertical) {
                msg_stream << " vertically";
            }
            msg_stream << " for " << context_label << " at timestamp "
                       << std::fixed << std::setprecision(6) << timestamp;
            logMessage(msg_stream.str());
        }

        return update_metadata();
    };

    const auto [dir_horizontal, dir_vertical] = compute_basis(corners);
    const double cross = static_cast<double>(dir_horizontal.x) * static_cast<double>(dir_vertical.y)
                       - static_cast<double>(dir_horizontal.y) * static_cast<double>(dir_vertical.x);

    if (cross < 0.0) {
        return apply_left_handed_fix();
    }

    const double dot_horizontal = static_cast<double>(dir_horizontal.x);
    const double dot_vertical = static_cast<double>(dir_vertical.y);

    if (dot_horizontal < 0.0 && dot_vertical < 0.0) {
        std::vector<cv::Point2f> corrected_corners(expected_corners);
        for (int r = 0; r < height; ++r) {
            for (int c = 0; c < width; ++c) {
                corrected_corners[static_cast<size_t>(r) * width + c] =
                    corners[static_cast<size_t>(height - 1 - r) * width + (width - 1 - c)];
            }
        }
        corners = std::move(corrected_corners);


        bool post_rotation_horizontal_flip = false;
        bool post_rotation_vertical_flip = false;

        auto [rotated_horizontal, rotated_vertical] = compute_basis(corners);
        if (rotated_horizontal.x < 0.0f) {
            flip_horizontal(corners);
            post_rotation_horizontal_flip = true;
            std::tie(rotated_horizontal, rotated_vertical) = compute_basis(corners);
        }

        if (rotated_vertical.y < 0.0f) {
            flip_vertical(corners);
            post_rotation_vertical_flip = true;
            std::tie(rotated_horizontal, rotated_vertical) = compute_basis(corners);
        }

        const double rotated_cross = compute_cross(corners);
        if (rotated_cross <= 0.0) {
            logMessage("ERROR: Chessboard basis non-positive after 180-degree remapping for "
                       + context_label + " at timestamp " + std::to_string(timestamp)
                       + ", applying left-handed correction");
            return apply_left_handed_fix();
        }

        std::ostringstream msg_stream;
        msg_stream << "Rotated chessboard corner order by 180 degrees for " << context_label
                   << " at timestamp " << std::fixed << std::setprecision(6) << timestamp;
        logMessage(msg_stream.str());

        if (post_rotation_horizontal_flip || post_rotation_vertical_flip) {
            std::ostringstream flip_stream;
            flip_stream << "Adjusted chessboard corner order after 180-degree rotation";
            if (post_rotation_horizontal_flip && post_rotation_vertical_flip) {
                flip_stream << " with additional horizontal and vertical flips";
            } else if (post_rotation_horizontal_flip) {
                flip_stream << " with an additional horizontal flip";
            } else if (post_rotation_vertical_flip) {
                flip_stream << " with an additional vertical flip";
            }
            flip_stream << " for " << context_label << " at timestamp "
                        << std::fixed << std::setprecision(6) << timestamp;
            logMessage(flip_stream.str());
        }

        return update_metadata();
    }

    return false;
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
                mono_result["mode"] = result.mode;
                mono_result["success"] = result.success;
                mono_result["reprojection_error"] = result.reprojection_error;
                mono_result["frames_used"] = result.frames_used;
                mono_result["image_width"] = result.image_size.width;
                mono_result["image_height"] = result.image_size.height;
                mono_result["calibration_time"] = result.calibration_time;

                if (result.success) {
                    // Сохраняем матрицы в отдельный YAML файл
                    std::string yaml_filename = "cam_" + result.camera_id;
                    if (!result.mode.empty()) {
                        yaml_filename += "__" + result.mode;
                    }
                    yaml_filename += "_calibration.yml";
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
                result.mode = normalizeModeName(mono_json.value("mode", std::string()));
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

                        camera_matrices_[result.camera_id][result.mode] = result.camera_matrix.clone();
                        dist_coeffs_[result.camera_id][result.mode] = result.dist_coeffs.clone();
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
    auto outer_k = camera_matrices_.find(camera_id);
    auto outer_d = dist_coeffs_.find(camera_id);
    if (outer_k == camera_matrices_.end() || outer_d == dist_coeffs_.end() || outer_k->second.empty()) {
        return false;
    }

    const auto& mode_map = outer_k->second;
    auto mode_it = mode_map.find(std::string());
    if (mode_it == mode_map.end()) {
        mode_it = mode_map.begin();
    }

    const auto& dist_mode_map = outer_d->second;
    auto dist_it = dist_mode_map.find(mode_it->first);
    if (dist_it == dist_mode_map.end()) {
        return false;
    }

    camera_matrix = mode_it->second.clone();
    dist_coeffs = dist_it->second.clone();
    return true;
}

bool CalibrationWatcher::getCameraMatrix(const std::string& camera_id, const std::string& mode,
                                        cv::Mat& camera_matrix, cv::Mat& dist_coeffs) const {
    std::lock_guard<std::mutex> lock(results_mutex_);
    auto outer_k = camera_matrices_.find(camera_id);
    auto outer_d = dist_coeffs_.find(camera_id);
    if (outer_k == camera_matrices_.end() || outer_d == dist_coeffs_.end()) {
        return false;
    }

    std::string normalized_mode = normalizeModeName(mode);
    auto mode_it = outer_k->second.find(normalized_mode);
    if (mode_it == outer_k->second.end()) {
        return false;
    }

    auto dist_it = outer_d->second.find(normalized_mode);
    if (dist_it == outer_d->second.end()) {
        return false;
    }

    camera_matrix = mode_it->second.clone();
    dist_coeffs = dist_it->second.clone();
    return true;
}

bool CalibrationWatcher::getCameraMatrixForProfile(const std::string& profile_key,
                                                   cv::Mat& camera_matrix, cv::Mat& dist_coeffs) const {
    auto profile = splitProfileKey(profile_key);
    return getCameraMatrix(profile.first, profile.second, camera_matrix, dist_coeffs);
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

void CalibrationWatcher::computeCameraOffsets(const std::vector<std::string>& camera_ids,
                                              const CalibrationParams& params,
                                              const std::map<std::string, std::vector<double>>& timestamp_index) {
    if (camera_ids.size() < 2) {
        std::lock_guard<std::mutex> lock(camera_offsets_mutex_);
        camera_time_offsets_.clear();
        return;
    }

    std::map<std::string, double> offsets;

    if (!loadExternalCameraOffsets(offsets)) {
        const double tolerance = std::max(params.time_tolerance_ms, 1) / 1000.0;
        std::map<std::string, std::vector<double>> chessboard_timestamps;
        size_t cameras_with_timestamps = 0;

        for (const auto& camera : camera_ids) {
            auto timestamps = loadChessboardTimestamps(camera);
            if (!timestamps.empty()) {
                cameras_with_timestamps++;
                chessboard_timestamps.emplace(camera, std::move(timestamps));
            }
        }

        if (cameras_with_timestamps >= 2) {
            std::string reference_camera;
            for (const auto& camera : camera_ids) {
                if (chessboard_timestamps.find(camera) != chessboard_timestamps.end()) {
                    reference_camera = camera;
                    offsets[reference_camera] = 0.0;
                    break;
                }
            }

            if (!reference_camera.empty()) {
                const auto& reference = chessboard_timestamps[reference_camera];
                for (const auto& camera : camera_ids) {
                    if (camera == reference_camera) {
                        continue;
                    }

                    double offset_value = 0.0;
                    auto target_it = chessboard_timestamps.find(camera);
                    if (target_it != chessboard_timestamps.end() && !target_it->second.empty()) {
                        offset_value = estimateOffsetFromTimestamps(reference, target_it->second, tolerance);
                    } else {
                        offset_value = estimateOffsetFromIndex(reference_camera, camera, timestamp_index);
                        logMessage("Falling back to indexed timestamps for camera " + camera +
                                   " when estimating offsets");
                    }
                    offsets[camera] = offset_value;
                }
            }
        }

        if (offsets.size() < camera_ids.size()) {
            if (offsets.empty() && !camera_ids.empty()) {
                offsets[camera_ids.front()] = 0.0;
            }

            for (const auto& camera : camera_ids) {
                if (offsets.find(camera) == offsets.end()) {
                    double fallback = estimateOffsetFromIndex(camera_ids.front(), camera, timestamp_index);
                    offsets[camera] = fallback;
                    logMessage("Offset for camera " + camera +
                               " estimated using recording timestamps only");
                }
            }
        }

        if (!offsets.empty()) {
            double min_offset = std::numeric_limits<double>::max();
            for (const auto& [camera, value] : offsets) {
                min_offset = std::min(min_offset, value);
            }

            if (min_offset != std::numeric_limits<double>::max()) {
                for (auto& [camera, value] : offsets) {
                    value -= min_offset;
                }
            }
        }
    } else {
        for (const auto& camera : camera_ids) {
            if (offsets.find(camera) == offsets.end()) {
                offsets[camera] = 0.0;
            }
        }
    }

    {
        std::lock_guard<std::mutex> lock(camera_offsets_mutex_);
        camera_time_offsets_ = offsets;
    }

    if (!offsets.empty()) {
        std::ostringstream oss;
        oss << "Camera time offsets (s): ";
        bool first = true;
        for (const auto& camera : camera_ids) {
            auto it = offsets.find(camera);
            if (it == offsets.end()) {
                continue;
            }
            if (!first) {
                oss << ", ";
            }
            oss << camera << '=' << std::fixed << std::setprecision(4) << it->second;
            first = false;
        }
        logMessage(oss.str());
    } else {
        logMessage("Camera time offsets were not determined; default synchronization will be used");
    }
}

bool CalibrationWatcher::loadExternalCameraOffsets(std::map<std::string, double>& offsets) const {
    const std::array<std::filesystem::path, 2> candidates = {
        record_dir_ / "sync_offsets.json",
        calib_dir_ / "sync_offsets.json"
    };

    auto appendOffset = [&offsets](const std::string& camera, double value_seconds) {
        offsets[camera] = value_seconds;
    };

    auto parseOffsetObject = [&appendOffset](const nlohmann::json& node, double scale) {
        if (!node.is_object()) {
            return false;
        }
        bool added = false;
        for (auto it = node.begin(); it != node.end(); ++it) {
            const auto& value = it.value();
            if (value.is_number_float() || value.is_number_integer()) {
                appendOffset(it.key(), value.get<double>() * scale);
                added = true;
            } else if (value.is_object()) {
                if (value.contains("seconds") && value["seconds"].is_number()) {
                    appendOffset(it.key(), value["seconds"].get<double>());
                    added = true;
                } else if (value.contains("milliseconds") && value["milliseconds"].is_number()) {
                    appendOffset(it.key(), value["milliseconds"].get<double>() / 1000.0);
                    added = true;
                } else if (value.contains("offset_seconds") && value["offset_seconds"].is_number()) {
                    appendOffset(it.key(), value["offset_seconds"].get<double>());
                    added = true;
                } else if (value.contains("offset_ms") && value["offset_ms"].is_number()) {
                    appendOffset(it.key(), value["offset_ms"].get<double>() / 1000.0);
                    added = true;
                }
            }
        }
        return added;
    };

    auto parseOffsetArray = [&appendOffset](const nlohmann::json& node) {
        if (!node.is_array()) {
            return false;
        }
        bool added = false;
        for (const auto& entry : node) {
            if (!entry.is_object() || !entry.contains("camera")) {
                continue;
            }
            const auto camera = entry["camera"].get<std::string>();
            if (entry.contains("seconds") && entry["seconds"].is_number()) {
                appendOffset(camera, entry["seconds"].get<double>());
                added = true;
            } else if (entry.contains("offset_seconds") && entry["offset_seconds"].is_number()) {
                appendOffset(camera, entry["offset_seconds"].get<double>());
                added = true;
            } else if (entry.contains("milliseconds") && entry["milliseconds"].is_number()) {
                appendOffset(camera, entry["milliseconds"].get<double>() / 1000.0);
                added = true;
            } else if (entry.contains("offset_ms") && entry["offset_ms"].is_number()) {
                appendOffset(camera, entry["offset_ms"].get<double>() / 1000.0);
                added = true;
            }
        }
        return added;
    };

    for (const auto& candidate : candidates) {
        if (!std::filesystem::exists(candidate)) {
            continue;
        }

        try {
            std::ifstream file(candidate);
            if (!file.is_open()) {
                continue;
            }

            nlohmann::json data;
            file >> data;

            offsets.clear();

            bool parsed = false;
            if (data.contains("camera_offsets")) {
                parsed |= parseOffsetObject(data["camera_offsets"], 1.0);
            }
            if (data.contains("camera_offsets_seconds")) {
                parsed |= parseOffsetObject(data["camera_offsets_seconds"], 1.0);
            }
            if (data.contains("camera_offsets_ms")) {
                parsed |= parseOffsetObject(data["camera_offsets_ms"], 1.0 / 1000.0);
            }
            if (!parsed && data.is_object()) {
                parsed |= parseOffsetObject(data, 1.0);
            }
            if (!parsed && data.is_array()) {
                parsed |= parseOffsetArray(data);
            }

            if (parsed && !offsets.empty()) {
                logMessage("Loaded camera offsets from external synchronization file: " + candidate.string());
                return true;
            }

            offsets.clear();
        } catch (const std::exception& e) {
            logMessage("Failed to load synchronization offsets from " + candidate.string() + ": " + e.what());
        }
    }

    offsets.clear();
    return false;
}

std::vector<double> CalibrationWatcher::loadChessboardTimestamps(const std::string& camera_id) const {
    std::vector<double> timestamps;
    auto profile = splitProfileKey(camera_id);
    std::filesystem::path cam_dir = cameraModeImagesDir(profile.first, profile.second);

    if (!std::filesystem::exists(cam_dir)) {
        return timestamps;
    }

    for (const auto& entry : std::filesystem::directory_iterator(cam_dir)) {
        if (entry.path().extension() != ".json") {
            continue;
        }

        try {
            std::ifstream meta_stream(entry.path());
            if (!meta_stream.is_open()) {
                continue;
            }

            nlohmann::json metadata;
            meta_stream >> metadata;

            bool board_detected = true;
            if (metadata.contains("quality") && metadata["quality"].is_object()) {
                const auto& quality = metadata["quality"];
                board_detected = quality.value("board_detected", true);
            }

            if (!board_detected) {
                continue;
            }

            timestamps.push_back(metadata.value("timestamp", 0.0));
        } catch (const std::exception& e) {
            logMessage("Failed to read chessboard timestamp from " + entry.path().string() + ": " + e.what());
        }
    }

    std::sort(timestamps.begin(), timestamps.end());
    return timestamps;
}

double CalibrationWatcher::estimateOffsetFromTimestamps(const std::vector<double>& reference,
                                                        const std::vector<double>& target,
                                                        double tolerance) const {
    if (reference.empty() || target.empty()) {
        return 0.0;
    }

    const double min_tolerance = std::max(tolerance, 0.001);
    const double max_window = 5.0; // 5 секунд максимум для оценки

    std::map<long long, std::vector<double>> buckets;
    for (double ref_ts : reference) {
        for (double tgt_ts : target) {
            double diff = tgt_ts - ref_ts;
            if (std::abs(diff) > max_window) {
                continue;
            }
            long long key = static_cast<long long>(std::llround(diff / min_tolerance));
            buckets[key].push_back(diff);
        }
    }

    if (buckets.empty()) {
        return target.front() - reference.front();
    }

    size_t best_count = 0;
    double best_offset = target.front() - reference.front();

    for (auto& [key, values] : buckets) {
        if (values.empty()) {
            continue;
        }
        if (values.size() > best_count) {
            best_count = values.size();
            auto median_it = values.begin() + values.size() / 2;
            std::nth_element(values.begin(), median_it, values.end());
            best_offset = *median_it;
        }
    }

    return best_offset;
}

double CalibrationWatcher::estimateOffsetFromIndex(
    const std::string& reference_camera,
    const std::string& target_camera,
    const std::map<std::string, std::vector<double>>& timestamp_index) const {

    auto ref_it = timestamp_index.find(reference_camera);
    auto target_it = timestamp_index.find(target_camera);

    if (ref_it != timestamp_index.end() && target_it != timestamp_index.end() &&
        !ref_it->second.empty() && !target_it->second.empty()) {
        return target_it->second.front() - ref_it->second.front();
    }

    return 0.0;
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
            std::string key = CalibrationWatcher::makeProfileKey(mono_result.camera_id, mono_result.mode);
            mono_calibrations_[key] = mono_result;
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
    const CameraCalibrationResult* calib = findMonoCalibration(camera_id);
    if (!calib) {
        return -1.0f;
    }
    return estimateObjectDistance(calib->camera_matrix, bbox, real_object_height_mm);
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
    const CameraCalibrationResult* calib = findMonoCalibration(camera_id);
    if (!calib) {
        return cv::Point3f(0, 0, 0);
    }

    const auto& camera_matrix = calib->camera_matrix;
    const auto& dist_coeffs = calib->dist_coeffs;

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
    return findMonoCalibration(camera_id) != nullptr;
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

const CameraCalibrationResult* DistanceMeasurement::findMonoCalibration(const std::string& requested) const {
    auto profile = CalibrationWatcher::splitProfileKey(requested);
    std::string base_id = profile.first;
    std::string mode = profile.second;

    std::string key = CalibrationWatcher::makeProfileKey(base_id, mode);
    auto it = mono_calibrations_.find(key);
    if (it != mono_calibrations_.end()) {
        return &it->second;
    }

    if (!mode.empty()) {
        std::string fallback = CalibrationWatcher::makeProfileKey(base_id, std::string());
        it = mono_calibrations_.find(fallback);
        if (it != mono_calibrations_.end()) {
            return &it->second;
        }
    }

    for (const auto& [profile_key, calibration] : mono_calibrations_) {
        auto split = CalibrationWatcher::splitProfileKey(profile_key);
        if (split.first == base_id) {
            return &calibration;
        }
    }

    return nullptr;
}
