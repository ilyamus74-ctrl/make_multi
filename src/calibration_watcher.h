#pragma once

#include <filesystem>
#include <string>
#include <vector>
#include <map>
#include <memory>
#include <atomic>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <functional>
#include <opencv2/opencv.hpp>
#include <nlohmann/json.hpp>

struct CalibrationParams {
    int board_cols = 10;       // количество столбцов клеток (не углов!)
    int board_rows = 7;        // количество строк клеток (не углов!)  
    float square_size = 30.0f; // размер клетки в мм
    int min_frames = 15;       // минимум качественных кадров для калибровки
    int max_frames = 50;       // максимум кадров для обработки
    float quality_threshold = 50.0f; // порог качества изображения
    bool delete_videos = true; // удалять видео после обработки
    
    // Вычисляемые поля (внутренние углы)
    int getInnerCols() const { return board_cols - 1; }
    int getInnerRows() const { return board_rows - 1; }
    cv::Size getPatternSize() const { return cv::Size(getInnerCols(), getInnerRows()); }
};

struct CameraCalibrationResult {
    std::string camera_id;
    cv::Mat camera_matrix;
    cv::Mat dist_coeffs;
    cv::Size image_size;
    double reprojection_error = 0.0;
    int frames_used = 0;
    bool success = false;
    
    // Дополнительная информация
    std::vector<double> per_view_errors;
    std::string calibration_time;
};

struct StereoCalibrationResult {
    std::string camera_pair;
    cv::Mat R, T, E, F;
    cv::Mat R1, R2, P1, P2, Q;
    double reprojection_error = 0.0;
    bool success = false;
    std::string calibration_time;
};

struct BoardPose {
    float tilt_degrees = 0.0f;                // Rotation of the board in image plane
    cv::Point2f center = {};                  // Chessboard center in pixels
    cv::Point2f normalized_center = {};       // Center normalised to [0,1]
    float scale = 0.0f;                       // Relative size of the board
};

class CalibrationWatcher {
public:
    // Callback для обновления статуса
    using StatusCallback = std::function<void(const std::string&, float)>;
    using LogCallback = std::function<void(const std::string&)>;

    explicit CalibrationWatcher(
        const std::filesystem::path& record_dir = "./rec",
        const std::filesystem::path& calib_dir = "./calibration"
    );
    ~CalibrationWatcher();

    // Основной метод - запуск процесса калибровки
    bool startCalibration(const CalibrationParams& params);
    // Запуск автоматической калибровки для указанной схемы и активных камер
    bool startAutoCalibration(const std::string& scheme,
                              const std::vector<std::string>& active_cameras,
                              const CalibrationParams& params = CalibrationParams());
    void stopCalibration();
    
    // Установка callbacks
    void setStatusCallback(StatusCallback callback) { status_callback_ = callback; }
    void setLogCallback(LogCallback callback) { log_callback_ = callback; }

    // Получение результатов
    std::vector<CameraCalibrationResult> getMonoResults() const { 
        std::lock_guard<std::mutex> lock(results_mutex_);
        return mono_results_; 
    }
    std::vector<StereoCalibrationResult> getStereoResults() const { 
        std::lock_guard<std::mutex> lock(results_mutex_);
        return stereo_results_; 
    }
    
    // Проверка статуса
    bool isProcessing() const { return processing_.load(); }
    std::string getStatus() const { 
        std::lock_guard<std::mutex> lock(status_mutex_);
        return status_message_; 
    }
    float getProgress() const { return progress_.load(); }

    // Сохранение/загрузка результатов
    bool saveResults() const;
    bool loadResults();

    // Получение матриц калибровки для измерения расстояний
    bool getCameraMatrix(const std::string& camera_id, cv::Mat& camera_matrix, cv::Mat& dist_coeffs) const;
    bool getStereoParams(const std::string& camera_pair, cv::Mat& R, cv::Mat& T, cv::Mat& Q) const;

private:
    struct VideoFile {
        std::string camera_id;
        std::filesystem::path path;
        std::time_t last_modified;
        uintmax_t file_size;
    };

    struct FrameQuality {
        float sharpness = 0.0f;
        float contrast = 0.0f;
        float corner_response = 0.0f;
        int corners_found = 0;
        bool board_detected = false;
        float overall_score = 0.0f;
    };

public:
    struct DetectedFrame {
        double timestamp = 0.0;                      // Presentation timestamp in seconds
        std::vector<cv::Point2f> corners;            // Refined chessboard corners
        FrameQuality quality;                        // Calculated quality metrics
        cv::Mat image;                               // Captured image for persistence
        std::filesystem::path image_path;            // Path to stored image on disk
        BoardPose pose;                              // Estimated board pose parameters
    };

private:
    struct StreamSynchronization {
        using FramePtr = std::shared_ptr<DetectedFrame>;
        using FramePair = std::pair<FramePtr, FramePtr>;
        using TimestampGroup = std::map<std::string, FramePtr>;
        using StereoGroup = std::map<double, FramePair>;

        std::map<std::string, std::vector<FramePtr>> mono_frames;   // Individual camera timelines
        std::map<std::string, StereoGroup> stereo_groups;           // Pair-wise synchronized frames
        std::map<double, TimestampGroup> multi_groups;              // Multi-camera synchronized sets
        std::map<std::string, double> camera_offsets;               // Estimated timestamp offsets
    };

    // Пути
    std::filesystem::path record_dir_;
    std::filesystem::path calib_dir_;
    std::filesystem::path results_dir_;
    
    // Состояние обработки
    std::atomic<bool> processing_{false};
    std::atomic<bool> should_stop_{false};
    std::atomic<float> progress_{0.0f};
    mutable std::mutex status_mutex_;
    std::string status_message_;
    
    // Результаты калибровки
    mutable std::mutex results_mutex_;
    std::vector<CameraCalibrationResult> mono_results_;
    std::vector<StereoCalibrationResult> stereo_results_;
    std::map<std::string, cv::Mat> camera_matrices_;
    std::map<std::string, cv::Mat> dist_coeffs_;
    
    // Callbacks
    StatusCallback status_callback_;
    LogCallback log_callback_;
    
    // Рабочий поток
    std::thread worker_thread_;

    // Внутренние методы
    void calibrationWorker(const CalibrationParams& params);
    
    std::vector<VideoFile> scanRecordingFolder();
    bool isVideoComplete(const VideoFile& video);


    BoardPose analyzeBoardPose(const std::vector<cv::Point2f>& corners,
                               const cv::Size& image_size,
                               const cv::Size& pattern_size);


    std::vector<std::string> extractFramesFromAllVideos(const CalibrationParams& params);
    bool extractFramesFromVideo(const VideoFile& video, const CalibrationParams& params);
    
    FrameQuality evaluateFrameQuality(const cv::Mat& frame, const CalibrationParams& params,
                                      std::vector<cv::Point2f>* refined_corners = nullptr);


    std::vector<DetectedFrame> loadAndSelectBestFrames(const std::string& camera_id,
                                                       const CalibrationParams& params);

    StreamSynchronization synchronizeStreams(const std::vector<std::string>& camera_ids,
                                             const CalibrationParams& params) const;


    bool performMonoCalibration(const std::string& camera_id, const CalibrationParams& params);
    bool performStereoCalibration(const std::string& cam1, const std::string& cam2,
                                 const StreamSynchronization::StereoGroup& synchronized_frames,
                                 const CalibrationParams& params);

    void performMultiCameraCalibration(const std::vector<std::string>& camera_ids,
                                      const CalibrationParams& params,
                                      const StreamSynchronization& sync_data);

    std::vector<cv::Point3f> generateChessboardPoints(const CalibrationParams& params);
    bool findChessboardInFrame(const cv::Mat& frame, const CalibrationParams& params,
                              std::vector<cv::Point2f>& corners);

    void updateStatus(const std::string& message, float progress = -1.0f);
    void logMessage(const std::string& message) const;

    bool saveFrameMetadata(const std::filesystem::path& image_path, const DetectedFrame& frame) const;

    // Утилиты для оценки качества
    static float calculateSharpness(const cv::Mat& gray);
    static float calculateContrast(const cv::Mat& gray);
    static bool validateCalibrationResult(const CameraCalibrationResult& result);
    
    // Очистка временных файлов
    void cleanupTempFiles();
};

// Класс для измерения расстояний
class DistanceMeasurement {
public:
    explicit DistanceMeasurement(const CalibrationWatcher& calibration);
    
    // Обновление калибровочных данных
    void updateCalibrationData(const CalibrationWatcher& calibration);
    
    // Измерение расстояния до объекта по монокамере
    float measureDistance(const std::string& camera_id, const cv::Rect& bbox, 
                         float real_object_height_mm = 1700.0f) const;
    
    // Измерение расстояния по стереопаре
    float measureStereoDistance(const std::string& camera_pair, 
                               const cv::Point2f& point1, const cv::Point2f& point2) const;
    
    // Преобразование пикселей в мировые координаты
    cv::Point3f pixelToWorld(const std::string& camera_id, const cv::Point2f& pixel, 
                            float distance_mm) const;
    
    // Проверка доступности калибровки
    bool hasMonoCalibration(const std::string& camera_id) const;
    bool hasStereoCalibration(const std::string& camera_pair) const;

private:
    std::map<std::string, CameraCalibrationResult> mono_calibrations_;
    std::map<std::string, StereoCalibrationResult> stereo_calibrations_;
    
    float estimateObjectDistance(const cv::Mat& camera_matrix, const cv::Rect& bbox,
                               float real_height_mm) const;
};
