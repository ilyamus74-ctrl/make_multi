#include "global_tracker.h"
#include "camera_scheme.h"
#include <algorithm>
#include <cassert>
#include <iostream>

class GlobalTrackerTestHelper {
public:
    static void addTrackedObject(GlobalTracker& tracker, const GlobalObject& object) {
        std::lock_guard<std::recursive_mutex> lock(tracker.mutex_);
        tracker.tracked_objects_[object.global_id] = object;
        tracker.next_global_id_ = std::max(tracker.next_global_id_, object.global_id + 1);
    }

    static void cleanup(GlobalTracker& tracker, uint64_t timestamp) {
        tracker.cleanupOldObjects(timestamp);
    }

    static size_t trackedObjectCount(const GlobalTracker& tracker) {
        std::lock_guard<std::recursive_mutex> lock(tracker.mutex_);
        return tracker.tracked_objects_.size();
    }

    static size_t pendingCount(const GlobalTracker& tracker) {
        std::lock_guard<std::recursive_mutex> lock(tracker.mutex_);
        return tracker.pending_reid_.size();
    }

    static int revivePending(GlobalTracker& tracker,
                              const std::string& camera_id,
                              const LocalDetection& detection,
                              const cv::Point3f& world,
                              uint64_t timestamp) {
        return tracker.tryRevivePending(camera_id, detection, world, timestamp);
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

void testRevivalAfterDormantPeriod() {
    CameraSchemeManager manager;
    auto config = manager.getTrackingConfig();
    config.id_retention_seconds = 2.0;
    config.dormant_id_retention_seconds = 8.0;
    config.pending_reid_descriptor_threshold = 0.05;
    manager.setTrackingConfig(config);

    GlobalTracker tracker(&manager, nullptr);

    GlobalObject dormant{};
    dormant.global_id = 42;
    dormant.world_position = cv::Point3f(1.0f, 2.0f, 0.0f);
    dormant.velocity = cv::Point3f(0.0f, 0.0f, 0.0f);
    dormant.residual = cv::Point3f(0.0f, 0.0f, 0.0f);
    dormant.residual_magnitude = 0.0f;
    dormant.residual_speed = 0.0f;
    dormant.confidence = 0.6;
    dormant.last_seen_timestamp = 1000;
    dormant.last_descriptor.has_color = true;
    dormant.last_descriptor.color = {100.0f, 150.0f, 200.0f};
    dormant.has_last_descriptor = true;
    dormant.position_history.push_back(dormant.world_position);

    GlobalTrackerTestHelper::addTrackedObject(tracker, dormant);

    GlobalTrackerTestHelper::cleanup(tracker, 5000);

    assert(GlobalTrackerTestHelper::trackedObjectCount(tracker) == 0);
    assert(GlobalTrackerTestHelper::pendingCount(tracker) == 1);

    LocalDetection returning_detection{};
    returning_detection.box = cv::Rect(0, 0, 10, 20);
    returning_detection.track_id = 7;
    returning_detection.descriptor.has_color = true;
    returning_detection.descriptor.color = {102.0f, 149.0f, 198.0f};

    int revived_id = GlobalTrackerTestHelper::revivePending(
        tracker,
        "cam_test",
        returning_detection,
        cv::Point3f(1.05f, 2.02f, 0.0f),
        5200);

    assert(revived_id == dormant.global_id);
    assert(GlobalTrackerTestHelper::trackedObjectCount(tracker) == 1);
    assert(GlobalTrackerTestHelper::pendingCount(tracker) == 0);

    GlobalObject revived = GlobalTrackerTestHelper::getTrackedObject(tracker, revived_id);
    assert(revived.confidence >= 0.5);
    assert(revived.has_last_descriptor);
    assert(revived.last_descriptor.color[0] == returning_detection.descriptor.color[0]);

    std::cout << "Dormant revival test passed" << std::endl;
}

int main() {
    testRevivalAfterDormantPeriod();
    std::cout << "All dormant revival tests passed" << std::endl;
    return 0;
}