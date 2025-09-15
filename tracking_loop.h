#pragma once
#include <vector>
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
private:
    int next_id = 0;
    std::vector<Track> tracks;
    std::vector<Track> lost;
    float max_dist = 100.0f;
    float reid_dist = 120.0f;
    float color_thresh = 40.0f;
    int max_misses = 30;
    int max_lost_age = 150;
    static void avgColor(const image_buffer_t* img, const image_rect_t& b, float out[3]);
    static float colorDiff(const float a[3], const float b[3]);
};

nlohmann::json formatDetectionResults(object_detect_result_list* results, int img_w, int img_h, const SimpleTracker& tracker);
