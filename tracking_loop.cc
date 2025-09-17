#include "tracking_loop.h"
#include <algorithm>
#include <cmath>
#include <atomic>
#include <mutex>
#include <limits>

void SimpleTracker::avgColor(const image_buffer_t* img, const image_rect_t& b, float out[3]) {
    // Input validation
    if (!img || !img->virt_addr || !out) {
        if (out) {
            out[0] = out[1] = out[2] = 0.0f;
        }
        return;
    }
    
    int x1 = std::max(0, b.left);
    int y1 = std::max(0, b.top);
    int x2 = std::min(img->width - 1, b.right - 1);
    int y2 = std::min(img->height - 1, b.bottom - 1);
    
    // Validate bounds
    if (x1 >= x2 || y1 >= y2 || x2 >= img->width || y2 >= img->height) {
        out[0] = out[1] = out[2] = 0.0f;
        return;
    }
    
    long r = 0, g = 0, bsum = 0; 
    int cnt = 0;
    
    for (int y = y1; y <= y2; ++y) {
        unsigned char* row = img->virt_addr + y * img->width * 3;
        for (int x = x1; x <= x2; ++x) {
            unsigned char* px = row + x * 3;
            r += px[0]; g += px[1]; bsum += px[2];
            cnt++;
        }
    }
    if (cnt == 0) cnt = 1;
    
    // Normalize colors to 0-255 range
    out[0] = static_cast<float>(r) / cnt; 
    out[1] = static_cast<float>(g) / cnt; 
    out[2] = static_cast<float>(bsum) / cnt;
}

float SimpleTracker::colorDiff(const float a[3], const float b[3]) {
    // Better color distance using normalized Euclidean distance
    if (!a || !b) return std::numeric_limits<float>::max();
    
    // Normalize to 0-1 range for better comparison
    float diff_r = (a[0] - b[0]) / 255.0f;
    float diff_g = (a[1] - b[1]) / 255.0f;
    float diff_b = (a[2] - b[2]) / 255.0f;
    
    return std::sqrt(diff_r * diff_r + diff_g * diff_g + diff_b * diff_b) * 255.0f;
}

int SimpleTracker::getNextTrackId() {
    // Prevent overflow by resetting to 1 when approaching max
    int current_id = next_id.fetch_add(1);
    if (current_id >= std::numeric_limits<int>::max() - 1000) {
        next_id.store(1);
        return 1;
    }
    return current_id;
}

bool SimpleTracker::validateParameters() const {
    return max_dist > 0 && reid_dist > 0 && color_thresh > 0 && 
           max_misses > 0 && max_lost_age > 0;
}

bool SimpleTracker::setParameters(float max_dist, float reid_dist, float color_thresh, 
                                 int max_misses, int max_lost_age) {
    if (max_dist <= 0 || reid_dist <= 0 || color_thresh <= 0 || 
        max_misses <= 0 || max_lost_age <= 0) {
        return false;
    }
    
    std::lock_guard<std::mutex> lock(tracker_mutex_);
    this->max_dist = max_dist;
    this->reid_dist = reid_dist;
    this->color_thresh = color_thresh;
    this->max_misses = max_misses;
    this->max_lost_age = max_lost_age;
    return true;
}

void SimpleTracker::cleanup() {
    std::lock_guard<std::mutex> lock(tracker_mutex_);
    tracks.clear();
    lost.clear();
    next_id.store(1);
}
void SimpleTracker::update(object_detect_result_list* dets, const image_buffer_t* img) {
    // Input validation
    if (!dets || !img || !validateParameters()) {
        return;
    }
    
    std::lock_guard<std::mutex> lock(tracker_mutex_);
    
    // Pre-allocate to avoid reallocations during tracking
    std::vector<bool> assigned;
    assigned.reserve(dets->count);
    assigned.assign(dets->count, false);
    
    // Initialize track IDs
    for (int i = 0; i < dets->count; ++i) {
        dets->results[i].track_id = -1;
    }

    // Phase 1: Associate existing tracks with detections
    for (auto& t : tracks) {
        int best = -1;
        float best_d = max_dist;
        float tcx = (t.box.left + t.box.right) * 0.5f;
        float tcy = (t.box.top + t.box.bottom) * 0.5f;
        
        for (int i = 0; i < dets->count; ++i) {
            if (assigned[i]) continue;
            
            const auto& b = dets->results[i].box;
            float dcx = (b.left + b.right) * 0.5f;
            float dcy = (b.top + b.bottom) * 0.5f;
            
            // Use more robust distance calculation including size similarity
            float dist = std::hypot(tcx - dcx, tcy - dcy);
            
            // Add size consistency check
            float track_area = (t.box.right - t.box.left) * (t.box.bottom - t.box.top);
            float det_area = (b.right - b.left) * (b.bottom - b.top);
            if (track_area > 0 && det_area > 0) {
                float size_ratio = std::max(track_area, det_area) / std::min(track_area, det_area);
                if (size_ratio > 3.0f) { // Skip if size changed too much
                    continue;
                }
            }
            
            if (dist < best_d) {
                best_d = dist;
                best = i;
            }
        }
        
        if (best != -1) {
            auto& det = dets->results[best];
            t.box = det.box;
            t.misses = 0;
            t.cls = det.cls_id;
            avgColor(img, det.box, t.color);
            det.track_id = t.id;
            assigned[best] = true;
        } else {
            t.misses++;
        }
    }

    // Phase 2: Move tracks with too many misses to lost tracks
    auto it = tracks.begin();
    while (it != tracks.end()) {
        if (it->misses > max_misses) {
            it->misses = 0;
            lost.push_back(std::move(*it)); // Use move for efficiency
            it = tracks.erase(it);
        } else {
            ++it;
        }
    }

    // Phase 3: Try to re-identify lost tracks using unassigned detections
    for (int i = 0; i < dets->count; ++i) {
        if (assigned[i]) continue;
        
        auto& det = dets->results[i];
        float col[3];
        avgColor(img, det.box, col);
        
        int best = -1;
        float best_d = reid_dist;
        float best_c = color_thresh;
        float dcx = (det.box.left + det.box.right) * 0.5f;
        float dcy = (det.box.top + det.box.bottom) * 0.5f;
        
        for (size_t j = 0; j < lost.size(); ++j) {
            auto& lt = lost[j];
            if (lt.cls != det.cls_id) continue;
            
            float tcx = (lt.box.left + lt.box.right) * 0.5f;
            float tcy = (lt.box.top + lt.box.bottom) * 0.5f;
            float dist = std::hypot(tcx - dcx, tcy - dcy);
            float cdist = colorDiff(lt.color, col);
            
            // Weighted combination of distance and color
            float combined_score = dist + cdist * 0.5f;
            float combined_thresh = best_d + best_c * 0.5f;
            
            if (combined_score < combined_thresh) {
                best_d = dist;
                best_c = cdist;
                best = static_cast<int>(j);
            }
        }
        
        if (best != -1) {
            Track t = std::move(lost[best]); // Use move for efficiency
            t.box = det.box;
            t.cls = det.cls_id;
            std::copy(col, col + 3, t.color);
            t.misses = 0;
            det.track_id = t.id;
            tracks.push_back(std::move(t));
            lost.erase(lost.begin() + best);
            assigned[i] = true;
        }
    }

    // Phase 4: Create new tracks for remaining unassigned detections
    for (int i = 0; i < dets->count; ++i) {
        if (assigned[i]) continue;
        
        auto& det = dets->results[i];
        Track t{};
        t.id = getNextTrackId();
        t.box = det.box;
        t.misses = 0;
        t.cls = det.cls_id;
        avgColor(img, det.box, t.color);
        det.track_id = t.id;
        tracks.push_back(std::move(t));
    }

    // Phase 5: Clean up old lost tracks
    auto lit = lost.begin();
    while (lit != lost.end()) {
        lit->misses++;
        if (lit->misses > max_lost_age) {
            lit = lost.erase(lit);
        } else {
            ++lit;
        }
    }
}

bool SimpleTracker::getColor(int id, float out[3]) const {
    if (!out) return false;
    
    std::lock_guard<std::mutex> lock(tracker_mutex_);
    
    // Search active tracks first (more likely to be found)
    for (const auto& t : tracks) {
        if (t.id == id) {
            out[0] = t.color[0];
            out[1] = t.color[1];
            out[2] = t.color[2];
            return true;
        }
    }
    
    // Search lost tracks
    for (const auto& t : lost) {
        if (t.id == id) {
            out[0] = t.color[0];
            out[1] = t.color[1];
            out[2] = t.color[2];
            return true;
        }
    }
    
    return false;
}

nlohmann::json formatDetectionResults(object_detect_result_list* results, int img_w, int img_h, const SimpleTracker& tracker) {
    nlohmann::json arr = nlohmann::json::array();
    for (int i = 0; i < results->count; ++i) {
        auto* d = &results->results[i];
        nlohmann::json j;
        j["track_id"] = d->track_id;
        j["class_id"] = d->cls_id;
        j["score"] = d->prop;
        j["box"] = {d->box.left, d->box.top, d->box.right, d->box.bottom};
        float col[3];
        if (tracker.getColor(d->track_id, col)) {
            j["color"] = {col[0], col[1], col[2]};
        }
        arr.push_back(j);
    }
    nlohmann::json out;
    out["detections"] = arr;
    out["image_w"] = img_w;
    out["image_h"] = img_h;
    return out;
}