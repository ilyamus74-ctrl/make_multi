#include "global_tracker.h"
#include "camera_scheme.h"
#include <cassert>
#include <iostream>
#include <vector>

class GlobalTrackerTestAdapter : public GlobalTracker {
public:
    using GlobalTracker::GlobalTracker;
    using GlobalTracker::passesSpatialGate;
};

void testSpatialGateRejectsDistantCandidates() {
    CameraSchemeManager manager;
    GlobalTrackerTestAdapter tracker(&manager, nullptr);

    GlobalObject obj{};
    obj.global_id = 1;
    obj.world_position = cv::Point3f(0.0f, 0.0f, 0.0f);
    obj.velocity = cv::Point3f(0.0f, 0.0f, 0.0f);
    obj.last_seen_timestamp = 1000;

    auto far_result = tracker.passesSpatialGate(obj,
                                                "cam1",
                                                cv::Point3f(0.0f, 0.0f, 0.0f),
                                                cv::Point3f(10.0f, 0.0f, 0.0f),
                                                1500);
    assert(!far_result.has_value());

    auto near_result = tracker.passesSpatialGate(obj,
                                                 "cam1",
                                                 cv::Point3f(0.0f, 0.0f, 0.0f),
                                                 cv::Point3f(0.5f, 0.0f, 0.0f),
                                                 1500);
    assert(near_result.has_value());

    std::cout << "Spatial gating distance rejection test passed" << std::endl;
}

void testSparseCandidatesHighDensity() {
    CameraSchemeManager manager;
    GlobalTrackerTestAdapter tracker(&manager, nullptr);

    std::vector<GlobalObject> objects(10);
    for (size_t i = 0; i < objects.size(); ++i) {
        objects[i].global_id = static_cast<int>(i + 1);
        objects[i].world_position = cv::Point3f(static_cast<float>(i) * 0.5f, 0.0f, 0.0f);
        objects[i].velocity = cv::Point3f(0.2f, 0.0f, 0.0f);
        objects[i].last_seen_timestamp = 1000;
    }

    std::vector<cv::Point3f> detections;
    for (int j = 0; j < 40; ++j) {
        detections.emplace_back(static_cast<float>(j) * 0.4f, 0.0f, 0.0f);
    }

    const uint64_t timestamp = 1300;

    size_t total_pairs = objects.size() * detections.size();
    size_t gated_pairs = 0;

    for (const auto& obj : objects) {
        cv::Point3f predicted = obj.world_position + obj.velocity *
                                static_cast<float>((timestamp - obj.last_seen_timestamp) / 1000.0);
        for (const auto& detection : detections) {
            if (tracker.passesSpatialGate(obj, "cam1", predicted, detection, timestamp)) {
                ++gated_pairs;
            }
        }
    }

    assert(gated_pairs > 0);
    assert(gated_pairs < total_pairs / 3);

    std::cout << "Spatial gating high density test passed with "
              << gated_pairs << " of " << total_pairs << " pairs" << std::endl;
}

int main() {
    testSpatialGateRejectsDistantCandidates();
    testSparseCandidatesHighDensity();
    std::cout << "All spatial gating tests passed" << std::endl;
    return 0;
}