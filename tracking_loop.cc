#include "tracking_loop.h"
#include <algorithm>
#include <cmath>
#include <atomic>
#include <mutex>
#include <limits>
#include <vector>

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
//    long r = 0, g = 0, bsum = 0; int cnt = 0;
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

void SimpleTracker::setGrayscaleMode(bool use_grayscale) {
    std::lock_guard<std::mutex> lock(tracker_mutex_);
    use_grayscale_tracking = use_grayscale;
}

bool SimpleTracker::isGrayscaleMode() const {
    std::lock_guard<std::mutex> lock(tracker_mutex_);
    return use_grayscale_tracking;
}

void SimpleTracker::convertToGrayscale(const image_buffer_t* img, const image_rect_t& b, float& intensity) {
    // Input validation
    if (!img || !img->virt_addr) {
        intensity = 0.0f;
        return;
    }

    int x1 = std::max(0, b.left);
    int y1 = std::max(0, b.top);
    int x2 = std::min(img->width - 1, b.right - 1);
    int y2 = std::min(img->height - 1, b.bottom - 1);

    // Validate bounds
    if (x1 >= x2 || y1 >= y2 || x2 >= img->width || y2 >= img->height) {
        intensity = 0.0f;
        return;
    }

    long gray_sum = 0;
    int cnt = 0;

    for (int y = y1; y <= y2; ++y) {
        unsigned char* row = img->virt_addr + y * img->width * 3;
        for (int x = x1; x <= x2; ++x) {
            unsigned char* px = row + x * 3;
            // Use standard grayscale conversion: 0.299*R + 0.587*G + 0.114*B
            float gray = 0.299f * px[0] + 0.587f * px[1] + 0.114f * px[2];
            gray_sum += static_cast<long>(gray);
            cnt++;
        }
    }

    if (cnt == 0) cnt = 1;
    intensity = static_cast<float>(gray_sum) / cnt;
}

void SimpleTracker::extractTextureFeatures(const image_buffer_t* img, const image_rect_t& b, float features[4]) {
    // Input validation
    if (!img || !img->virt_addr || !features) {
        if (features) {
            features[0] = features[1] = features[2] = features[3] = 0.0f;
        }
        return;
    }

    int x1 = std::max(1, b.left);       // Need padding for gradient calculation
    int y1 = std::max(1, b.top);
    int x2 = std::min(img->width - 2, b.right - 1);
    int y2 = std::min(img->height - 2, b.bottom - 1);

    // Validate bounds
    if (x1 >= x2 || y1 >= y2 || x2 >= img->width || y2 >= img->height) {
        features[0] = features[1] = features[2] = features[3] = 0.0f;
        return;
    }

    std::vector<float> gradients;
    std::vector<int> lbp_values;
    gradients.reserve((y2 - y1 + 1) * (x2 - x1 + 1));
    lbp_values.reserve((y2 - y1 + 1) * (x2 - x1 + 1));

    for (int y = y1; y <= y2; ++y) {
        for (int x = x1; x <= x2; ++x) {
            // Convert to grayscale for this pixel
            unsigned char* px = img->virt_addr + y * img->width * 3 + x * 3;
            float center_gray = 0.299f * px[0] + 0.587f * px[1] + 0.114f * px[2];

            // Calculate gradient magnitude using Sobel-like operator
            unsigned char* px_left = img->virt_addr + y * img->width * 3 + (x-1) * 3;
            unsigned char* px_right = img->virt_addr + y * img->width * 3 + (x+1) * 3;
            unsigned char* px_up = img->virt_addr + (y-1) * img->width * 3 + x * 3;
            unsigned char* px_down = img->virt_addr + (y+1) * img->width * 3 + x * 3;

            float left_gray = 0.299f * px_left[0] + 0.587f * px_left[1] + 0.114f * px_left[2];
            float right_gray = 0.299f * px_right[0] + 0.587f * px_right[1] + 0.114f * px_right[2];
            float up_gray = 0.299f * px_up[0] + 0.587f * px_up[1] + 0.114f * px_up[2];
            float down_gray = 0.299f * px_down[0] + 0.587f * px_down[1] + 0.114f * px_down[2];

            float gx = right_gray - left_gray;
            float gy = down_gray - up_gray;
            float gradient_mag = std::sqrt(gx * gx + gy * gy);
            gradients.push_back(gradient_mag);

            // Simple 8-point Local Binary Pattern
            int lbp = 0;
            float neighbors[8];
            
            // Get 8 neighbors in clockwise order
            unsigned char* n1 = img->virt_addr + (y-1) * img->width * 3 + (x-1) * 3;  // top-left
            unsigned char* n2 = img->virt_addr + (y-1) * img->width * 3 + x * 3;      // top
            unsigned char* n3 = img->virt_addr + (y-1) * img->width * 3 + (x+1) * 3;  // top-right
            unsigned char* n4 = img->virt_addr + y * img->width * 3 + (x+1) * 3;      // right
            unsigned char* n5 = img->virt_addr + (y+1) * img->width * 3 + (x+1) * 3;  // bottom-right
            unsigned char* n6 = img->virt_addr + (y+1) * img->width * 3 + x * 3;      // bottom
            unsigned char* n7 = img->virt_addr + (y+1) * img->width * 3 + (x-1) * 3;  // bottom-left
            unsigned char* n8 = img->virt_addr + y * img->width * 3 + (x-1) * 3;      // left

            neighbors[0] = 0.299f * n1[0] + 0.587f * n1[1] + 0.114f * n1[2];
            neighbors[1] = 0.299f * n2[0] + 0.587f * n2[1] + 0.114f * n2[2];
            neighbors[2] = 0.299f * n3[0] + 0.587f * n3[1] + 0.114f * n3[2];
            neighbors[3] = 0.299f * n4[0] + 0.587f * n4[1] + 0.114f * n4[2];
            neighbors[4] = 0.299f * n5[0] + 0.587f * n5[1] + 0.114f * n5[2];
            neighbors[5] = 0.299f * n6[0] + 0.587f * n6[1] + 0.114f * n6[2];
            neighbors[6] = 0.299f * n7[0] + 0.587f * n7[1] + 0.114f * n7[2];
            neighbors[7] = 0.299f * n8[0] + 0.587f * n8[1] + 0.114f * n8[2];

            for (int i = 0; i < 8; ++i) {
                if (neighbors[i] >= center_gray) {
                    lbp |= (1 << i);
                }
            }
            lbp_values.push_back(lbp);
        }
    }

    // Calculate statistics
    if (gradients.empty()) {
        features[0] = features[1] = features[2] = features[3] = 0.0f;
        return;
    }

    // Gradient mean and variance
    float grad_sum = 0.0f;
    for (float g : gradients) grad_sum += g;
    features[0] = grad_sum / gradients.size();  // mean_gradient

    float grad_var_sum = 0.0f;
    for (float g : gradients) {
        float diff = g - features[0];
        grad_var_sum += diff * diff;
    }
    features[1] = grad_var_sum / gradients.size();  // gradient_variance

    // LBP mean and variance
    float lbp_sum = 0.0f;
    for (int lbp : lbp_values) lbp_sum += lbp;
    features[2] = lbp_sum / lbp_values.size();  // lbp_mean

    float lbp_var_sum = 0.0f;
    for (int lbp : lbp_values) {
        float diff = lbp - features[2];
        lbp_var_sum += diff * diff;
    }
    features[3] = lbp_var_sum / lbp_values.size();  // lbp_variance
}

float SimpleTracker::compareGrayscaleFeatures(const float intensity_a, const float features_a[4], 
                                             const float intensity_b, const float features_b[4]) {
    if (!features_a || !features_b) return std::numeric_limits<float>::max();

    // Weighted combination of intensity and texture features
    float intensity_diff = std::abs(intensity_a - intensity_b) / 255.0f;
    
    float texture_diff = 0.0f;
    // Compare each texture feature with appropriate weighting
    texture_diff += std::abs(features_a[0] - features_b[0]) / 100.0f;  // gradient mean (normalized)
    texture_diff += std::abs(features_a[1] - features_b[1]) / 10000.0f; // gradient variance (normalized)
    texture_diff += std::abs(features_a[2] - features_b[2]) / 255.0f;   // LBP mean (normalized)
    texture_diff += std::abs(features_a[3] - features_b[3]) / 65535.0f; // LBP variance (normalized)
    
    // Combine intensity and texture with appropriate weights
    return intensity_diff * 0.4f + texture_diff * 0.6f;
}

float SimpleTracker::adaptiveThreshold(float base_threshold, const image_buffer_t* img, const image_rect_t& b) {
    // Simple adaptive threshold based on local contrast
    if (!img || !img->virt_addr) return base_threshold;

    int x1 = std::max(0, b.left);
    int y1 = std::max(0, b.top);
    int x2 = std::min(img->width - 1, b.right - 1);
    int y2 = std::min(img->height - 1, b.bottom - 1);

    if (x1 >= x2 || y1 >= y2) return base_threshold;

    // Calculate local variance as a measure of contrast
    float mean = 0.0f;
    int cnt = 0;
    
    for (int y = y1; y <= y2; ++y) {
        unsigned char* row = img->virt_addr + y * img->width * 3;
        for (int x = x1; x <= x2; ++x) {
            unsigned char* px = row + x * 3;
            float gray = 0.299f * px[0] + 0.587f * px[1] + 0.114f * px[2];
            mean += gray;
            cnt++;
        }
    }
    if (cnt > 0) mean /= cnt;

    float variance = 0.0f;
    for (int y = y1; y <= y2; ++y) {
        unsigned char* row = img->virt_addr + y * img->width * 3;
        for (int x = x1; x <= x2; ++x) {
            unsigned char* px = row + x * 3;
            float gray = 0.299f * px[0] + 0.587f * px[1] + 0.114f * px[2];
            float diff = gray - mean;
            variance += diff * diff;
        }
    }
    if (cnt > 0) variance /= cnt;

    // Adapt threshold based on local contrast
    float contrast_factor = std::sqrt(variance) / 50.0f; // Normalize variance
    contrast_factor = std::max(0.5f, std::min(2.0f, contrast_factor)); // Clamp to reasonable range
    
    return base_threshold * contrast_factor;
}
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
            
            if (use_grayscale_tracking) {
                convertToGrayscale(img, det.box, t.grayscale_intensity);
                extractTextureFeatures(img, det.box, t.texture_features);
            } else {
                avgColor(img, det.box, t.color);
            }
            
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
        
        float col[3] = {0.0f, 0.0f, 0.0f};
        float gray_intensity = 0.0f;
        float texture_features[4] = {0.0f, 0.0f, 0.0f, 0.0f};
        
        if (use_grayscale_tracking) {
            convertToGrayscale(img, det.box, gray_intensity);
            extractTextureFeatures(img, det.box, texture_features);
        } else {
            avgColor(img, det.box, col);
        }

        int best = -1;
        float best_d = reid_dist;
        float best_c = use_grayscale_tracking ? 
                       adaptiveThreshold(color_thresh * 0.01f, img, det.box) : // Normalize for grayscale comparison
                       color_thresh;
        float dcx = (det.box.left + det.box.right) * 0.5f;
        float dcy = (det.box.top + det.box.bottom) * 0.5f;

        for (size_t j = 0; j < lost.size(); ++j) {
            auto& lt = lost[j];
            if (lt.cls != det.cls_id) continue;
            float tcx = (lt.box.left + lt.box.right) * 0.5f;
            float tcy = (lt.box.top + lt.box.bottom) * 0.5f;
            float dist = std::hypot(tcx - dcx, tcy - dcy);
            
            float cdist;
            if (use_grayscale_tracking) {
                cdist = compareGrayscaleFeatures(lt.grayscale_intensity, lt.texture_features,
                                               gray_intensity, texture_features);
            } else {
                cdist = colorDiff(lt.color, col);
            }
            
            // Weighted combination of distance and color/texture
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
            
            if (use_grayscale_tracking) {
                t.grayscale_intensity = gray_intensity;
                std::copy(texture_features, texture_features + 4, t.texture_features);
            } else {
                std::copy(col, col + 3, t.color);
            }
            
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
        
        if (use_grayscale_tracking) {
            convertToGrayscale(img, det.box, t.grayscale_intensity);
            extractTextureFeatures(img, det.box, t.texture_features);
            // Initialize color values to zero when using grayscale mode
            t.color[0] = t.color[1] = t.color[2] = 0.0f;
        } else {
            avgColor(img, det.box, t.color);
            // Initialize grayscale values to zero when using color mode
            t.grayscale_intensity = 0.0f;
            t.texture_features[0] = t.texture_features[1] = t.texture_features[2] = t.texture_features[3] = 0.0f;
        }
        
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