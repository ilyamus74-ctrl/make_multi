#pragma once
#include <vector>
#include <mutex>
#include <limits>
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
    void update(object_detect_result_list* dets, const image_buffer_t* img);
    bool getColor(int id, float out[3]) const;
    
    // Add cleanup method for proper shutdown
    void cleanup();
    
    // Add parameter validation
    bool setParameters(float max_dist, float reid_dist, float color_thresh, 
                      int max_misses, int max_lost_age);
    
private:
    // Use atomic for thread-safe ID generation with overflow protection
    std::atomic<int> next_id{1};
    
    // Add mutex for thread safety
    mutable std::mutex tracker_mutex_;
    
    std::vector<Track> tracks;
    std::vector<Track> lost;
    float max_dist = 100.0f;
    float reid_dist = 120.0f;
    float color_thresh = 40.0f;
    int max_misses = 30;
    int max_lost_age = 150;
    
    static void avgColor(const image_buffer_t* img, const image_rect_t& b, float out[3]);
    static float colorDiff(const float a[3], const float b[3]);
    
    // Add helper methods for better algorithm
    int getNextTrackId();
    bool validateParameters() const;
};

nlohmann::json formatDetectionResults(object_detect_result_list* results, int img_w, int img_h, const SimpleTracker& tracker);
