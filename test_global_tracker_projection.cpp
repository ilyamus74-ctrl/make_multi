#include "global_tracker.h"
#include "camera_scheme.h"
#include <algorithm>
#include <cassert>
#include <iostream>
#include <vector>

class GlobalTrackerTestHelper {
public:
    static void addTrackedObject(GlobalTracker& tracker, const GlobalObject& object) {
        std::lock_guard<std::recursive_mutex> lock(tracker.mutex_);
        tracker.tracked_objects_[object.global_id] = object;
        tracker.next_global_id_ = std::max(tracker.next_global_id_, object.global_id + 1);
    }

    static size_t trackedObjectCount(const GlobalTracker& tracker) {
        std::lock_guard<std::recursive_mutex> lock(tracker.mutex_);
        return tracker.tracked_objects_.size();
    }

    static GlobalObject getTrackedObject(const GlobalTracker& tracker, int global_id) {
        std::lock_guard<std::recursive_mutex> lock(tracker.mutex_);
        auto it = tracker.tracked_objects_.find(global_id);
        if (it == tracker.tracked_objects_.end()) {
            return GlobalObject{};
        }
        return it->second;
    }
};

void testInvalidProjectionDetectionIsSkipped() {
    CameraSchemeManager manager;
    GlobalTracker tracker(&manager, nullptr);

    GlobalObject existing{};
    existing.global_id = 1;
    existing.world_position = cv::Point3f(0.0f, 0.0f, 0.0f);
    existing.velocity = cv::Point3f(0.0f, 0.0f, 0.0f);
    existing.residual = cv::Point3f(0.0f, 0.0f, 0.0f);
    existing.residual_magnitude = 0.0f;
    existing.residual_speed = 0.0f;
    existing.confidence = 0.8;
    existing.last_seen_timestamp = 1000;
    existing.primary_camera_id = "cam_test";
    existing.camera_last_seen["cam_test"] = 1000;
    existing.camera_miss_counts["cam_test"] = 0;

    LocalDetection previous_detection{};
    previous_detection.box = cv::Rect(10, 10, 20, 20);
    previous_detection.track_id = 77;
    existing.camera_detections["cam_test"] = previous_detection;

    GlobalTrackerTestHelper::addTrackedObject(tracker, existing);

    LocalDetection invalid_detection{};
    invalid_detection.box = cv::Rect(12, 12, 18, 18);
    invalid_detection.track_id = previous_detection.track_id;

    tracker.updateDetections("cam_test", {invalid_detection}, 2000);

    assert(GlobalTrackerTestHelper::trackedObjectCount(tracker) == 1);
    GlobalObject stored = GlobalTrackerTestHelper::getTrackedObject(tracker, existing.global_id);
    assert(stored.last_seen_timestamp == 1000);

    auto last_seen_it = stored.camera_last_seen.find("cam_test");
    if (last_seen_it != stored.camera_last_seen.end()) {
        assert(last_seen_it->second == 1000);
    }

    auto active_objects = tracker.getActiveObjects();
    assert(active_objects.size() == 1);
    assert(active_objects[0].global_id == existing.global_id);

    std::cout << "Invalid projection detection regression test passed" << std::endl;
}

int main() {
    testInvalidProjectionDetectionIsSkipped();
    std::cout << "All invalid projection tests passed" << std::endl;
    return 0;
}