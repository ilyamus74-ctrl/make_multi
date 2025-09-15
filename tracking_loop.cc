#include "tracking_loop.h"
#include <algorithm>
#include <cmath>

void SimpleTracker::avgColor(const image_buffer_t* img, const image_rect_t& b, float out[3]) {
    int x1 = std::max(0, b.left);
    int y1 = std::max(0, b.top);
    int x2 = std::min(img->width - 1, b.right - 1);
    int y2 = std::min(img->height - 1, b.bottom - 1);
    long r = 0, g = 0, bsum = 0; int cnt = 0;
    for (int y = y1; y <= y2; ++y) {
        unsigned char* row = img->virt_addr + y * img->width * 3;
        for (int x = x1; x <= x2; ++x) {
            unsigned char* px = row + x * 3;
            r += px[0]; g += px[1]; bsum += px[2];
            cnt++;
        }
    }
    if (cnt == 0) cnt = 1;
    out[0] = r / (float)cnt; out[1] = g / (float)cnt; out[2] = bsum / (float)cnt;
}

float SimpleTracker::colorDiff(const float a[3], const float b[3]) {
    return std::fabs(a[0]-b[0]) + std::fabs(a[1]-b[1]) + std::fabs(a[2]-b[2]);
}

void SimpleTracker::update(object_detect_result_list* dets, const image_buffer_t* img) {
    std::vector<bool> assigned(dets->count, false);
    for (int i = 0; i < dets->count; ++i) dets->results[i].track_id = -1;

    for (auto& t : tracks) {
        int best = -1; float best_d = max_dist;
        float tcx = (t.box.left + t.box.right) / 2.0f;
        float tcy = (t.box.top + t.box.bottom) / 2.0f;
        for (int i = 0; i < dets->count; ++i) if (!assigned[i]) {
            auto& b = dets->results[i].box;
            float dcx = (b.left + b.right) / 2.0f;
            float dcy = (b.top + b.bottom) / 2.0f;
            float d = std::hypot(tcx - dcx, tcy - dcy);
            if (d < best_d) { best_d = d; best = i; }
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

    auto it = tracks.begin();
    while (it != tracks.end()) {
        if (it->misses > max_misses) {
            it->misses = 0;
            lost.push_back(*it);
            it = tracks.erase(it);
        } else {
            ++it;
        }
    }

    for (int i = 0; i < dets->count; ++i) if (!assigned[i]) {
        auto& det = dets->results[i];
        float col[3];
        avgColor(img, det.box, col);
        int best = -1; float best_d = reid_dist; float best_c = color_thresh;
        float dcx = (det.box.left + det.box.right) / 2.0f;
        float dcy = (det.box.top + det.box.bottom) / 2.0f;
        for (size_t j = 0; j < lost.size(); ++j) {
            auto& lt = lost[j];
            if (lt.cls != det.cls_id) continue;
            float tcx = (lt.box.left + lt.box.right) / 2.0f;
            float tcy = (lt.box.top + lt.box.bottom) / 2.0f;
            float dist = std::hypot(tcx - dcx, tcy - dcy);
            float cdist = colorDiff(lt.color, col);
            if (dist < best_d && cdist < best_c) { best_d = dist; best_c = cdist; best = j; }
        }
        if (best != -1) {
            Track t = lost[best];
            t.box = det.box;
            t.cls = det.cls_id;
            std::copy(col, col+3, t.color);
            t.misses = 0;
            det.track_id = t.id;
            tracks.push_back(t);
            lost.erase(lost.begin() + best);
            assigned[i] = true;
        }
    }

    for (int i = 0; i < dets->count; ++i) if (!assigned[i]) {
        auto& det = dets->results[i];
        Track t{};
        t.id = next_id++;
        t.box = det.box;
        t.misses = 0;
        t.cls = det.cls_id;
        avgColor(img, det.box, t.color);
        det.track_id = t.id;
        tracks.push_back(t);
    }

    auto lit = lost.begin();
    while (lit != lost.end()) {
        lit->misses++;
        if (lit->misses > max_lost_age) lit = lost.erase(lit);
        else ++lit;
    }
}

bool SimpleTracker::getColor(int id, float out[3]) const {
    for (auto& t : tracks) if (t.id == id) { out[0]=t.color[0]; out[1]=t.color[1]; out[2]=t.color[2]; return true; }
    for (auto& t : lost)   if (t.id == id) { out[0]=t.color[0]; out[1]=t.color[1]; out[2]=t.color[2]; return true; }
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