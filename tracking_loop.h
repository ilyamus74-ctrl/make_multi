#pragma once
#include <vector>
#include <mutex>
#include <limits>
#include <atomic>
#include "yolov8.h"
#include "image_utils.h"
#include "nlohmann/json.hpp"

struct Track {
    int id;
    image_rect_t box;
    int misses;
    float color[3];
    int cls;
};

class SimpleTracker {
public:
    // Main tracking update method with thread safety and improved algorithms
    void update(object_detect_result_list* dets, const image_buffer_t* img);

    // Thread-safe color retrieval with improved search order
    bool getColor(int id, float out[3]) const;

    // Add cleanup method for proper shutdown (RAII pattern)
    void cleanup();

    // Add parameter validation with bounds checking
    bool setParameters(float max_dist, float reid_dist, float color_thresh, 
                      int max_misses, int max_lost_age);

private:

    // Use atomic for thread-safe ID generation with overflow protection
    std::atomic<int> next_id{1};

    // Add mutex for thread safety of shared data structures
    mutable std::mutex tracker_mutex_;
    std::vector<Track> tracks;
    std::vector<Track> lost;
    float max_dist = 100.0f;
    float reid_dist = 120.0f;
    float color_thresh = 40.0f;
    int max_misses = 30;
    int max_lost_age = 150;

    // Improved color calculation with bounds checking
    static void avgColor(const image_buffer_t* img, const image_rect_t& b, float out[3]);

    // Enhanced color difference using normalized Euclidean distance
    static float colorDiff(const float a[3], const float b[3]);

    // Helper methods for better algorithm design
    int getNextTrackId();                    // Thread-safe ID generation with overflow protection
    bool validateParameters() const;         // Configuration validation
};

nlohmann::json formatDetectionResults(object_detect_result_list* results, int img_w, int img_h, const SimpleTracker& tracker);
