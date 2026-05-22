#!/usr/bin/env python3
from pathlib import Path
import sys
import shutil

path = Path(sys.argv[1]) if len(sys.argv) > 1 else Path("mjpeg_gst_http.cpp")
if not path.exists():
    raise SystemExit(f"file not found: {path}")

s = path.read_text(encoding="utf-8")
orig = s

backup = path.with_suffix(path.suffix + ".bak_roi_filter")
if not backup.exists():
    shutil.copy2(path, backup)
    print(f"backup: {backup}")

def must_replace_once(text, old, new, name):
    count = text.count(old)
    if count != 1:
        raise SystemExit(f"[FAIL] {name}: expected 1 occurrence, found {count}")
    return text.replace(old, new, 1)

def replace_once_if_present(text, old, new, name):
    count = text.count(old)
    if count == 0:
        print(f"[SKIP] {name}: pattern not found, maybe already patched")
        return text
    if count > 1:
        raise SystemExit(f"[FAIL] {name}: expected <=1 occurrence, found {count}")
    print(f"[PATCH] {name}")
    return text.replace(old, new, 1)

# 1) Function signature: add ROI-specific class filter pointer.
old_sig = "static bool run_inference_on_bgr(const cv::Mat& bgr, int offsetX, int offsetY, std::vector<DetectionBox>& outBoxes) {"
new_sig = "static bool run_inference_on_bgr(const cv::Mat& bgr, int offsetX, int offsetY, const std::set<int>* roiClassFilter, std::vector<DetectionBox>& outBoxes) {"
if old_sig in s:
    s = s.replace(old_sig, new_sig, 1)
    print("[PATCH] run_inference_on_bgr signature")
else:
    print("[SKIP] run_inference_on_bgr signature: maybe already patched")

# 2) Apply ROI-specific class filter inside detection loop.
old_loop = """  for (int i = 0; i < od.count; ++i) {
    const auto& d = od.results[i];
    DetectionBox box;
"""
new_loop = """  for (int i = 0; i < od.count; ++i) {
    const auto& d = od.results[i];
    if (roiClassFilter && !roiClassFilter->empty() &&
        roiClassFilter->count(d.cls_id) == 0) {
      continue;
    }
    DetectionBox box;
"""
if "roiClassFilter->count(d.cls_id)" not in s:
    s = must_replace_once(s, old_loop, new_loop, "ROI class filter in loop")
else:
    print("[SKIP] ROI class filter in loop: already present")

# 3) Add inferenceAttempted flag.
old_flags = "          bool anyInfer = false;"
new_flags = """          bool inferenceAttempted = false;
          bool anyInfer = false;"""
if "bool inferenceAttempted = false;" not in s:
    s = must_replace_once(s, old_flags, new_flags, "inferenceAttempted flag")
else:
    print("[SKIP] inferenceAttempted flag: already present")

# 4) Full-frame call.
old = "            anyInfer = run_inference_on_bgr(bgr, 0, 0, collectedBoxes) || anyInfer;"
new = """            inferenceAttempted = true;
            anyInfer = run_inference_on_bgr(bgr, 0, 0, nullptr, collectedBoxes) || anyInfer;"""
if old in s:
    s = s.replace(old, new, 1)
    print("[PATCH] full_frame run_inference call")
else:
    print("[SKIP] full_frame run_inference call: maybe already patched")

# 5) Tiled call: first bgr(rc) call with 14-space indent belongs to tiled loop.
old = "              anyInfer = run_inference_on_bgr(bgr(rc), x, y, collectedBoxes) || anyInfer;"
new = """              inferenceAttempted = true;
              anyInfer = run_inference_on_bgr(bgr(rc), x, y, nullptr, collectedBoxes) || anyInfer;"""
if old in s:
    s = s.replace(old, new, 1)
    print("[PATCH] tiled run_inference call")
else:
    print("[SKIP] tiled run_inference call: maybe already patched")

# 6) Hybrid full-frame ROI call: pass roi.class_filter.
old = "                anyInfer = run_inference_on_bgr(bgr, 0, 0, collectedBoxes) || anyInfer;"
new = """                inferenceAttempted = true;
                anyInfer = run_inference_on_bgr(bgr, 0, 0, &roi.class_filter, collectedBoxes) || anyInfer;"""
if old in s:
    s = s.replace(old, new, 1)
    print("[PATCH] hybrid full-frame ROI run_inference call")
else:
    print("[SKIP] hybrid full-frame ROI run_inference call: maybe already patched")

# 7) ROI crop call: pass roi.class_filter.
old = "                anyInfer = run_inference_on_bgr(bgr(rc), x, y, collectedBoxes) || anyInfer;"
new = """                inferenceAttempted = true;
                anyInfer = run_inference_on_bgr(bgr(rc), x, y, &roi.class_filter, collectedBoxes) || anyInfer;"""
if old in s:
    s = s.replace(old, new, 1)
    print("[PATCH] ROI crop run_inference call")
else:
    print("[SKIP] ROI crop run_inference call: maybe already patched")

# 8) Do not clear detections if no inference was attempted due to every_n_frames scheduling.
old = """            draw_detection_boxes(bgr, boxes);
          } else {
            g_lastDetections = 0;"""
new = """            draw_detection_boxes(bgr, boxes);
          } else if (inferenceAttempted) {
            g_lastDetections = 0;"""
if old in s:
    s = s.replace(old, new, 1)
    print("[PATCH] no-clear on scheduled skip")
else:
    print("[SKIP] no-clear on scheduled skip: maybe already patched")

# 9) Basic validation.
if "run_inference_on_bgr(bgr, 0, 0, collectedBoxes)" in s:
    raise SystemExit("[FAIL] stale full-frame call remains")
if "run_inference_on_bgr(bgr(rc), x, y, collectedBoxes)" in s:
    raise SystemExit("[FAIL] stale ROI/tile call remains")
if "static bool run_inference_on_bgr(const cv::Mat& bgr, int offsetX, int offsetY, std::vector<DetectionBox>& outBoxes)" in s:
    raise SystemExit("[FAIL] stale function signature remains")
if "roiClassFilter->count(d.cls_id)" not in s:
    raise SystemExit("[FAIL] ROI class filter was not inserted")
if "else if (inferenceAttempted)" not in s:
    raise SystemExit("[FAIL] scheduled-skip clear guard was not inserted")

if s == orig:
    print("no changes written")
else:
    path.write_text(s, encoding="utf-8")
    print(f"patched: {path}")
