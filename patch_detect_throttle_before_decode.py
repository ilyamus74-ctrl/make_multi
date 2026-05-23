from pathlib import Path
import shutil

p = Path("mjpeg_gst_http.cpp")
s = p.read_text(encoding="utf-8")

backup = p.with_suffix(".cpp.bak_throttle_before_decode")
if not backup.exists():
    shutil.copy2(p, backup)
    print(f"backup: {backup}")

# 1. Move throttle check before cv::imdecode.
old = '''      if (g_detectEnabled.load()) {
        std::vector<uint8_t> inJpeg(map.data, map.data + map.size);
        cv::Mat bgr = cv::imdecode(inJpeg, cv::IMREAD_COLOR);

        if (!bgr.empty()) {'''

new = '''      if (g_detectEnabled.load()) {
        const uint64_t frameId = g_frameId.load();
        const int detectEvery = std::max(1, g_detectEveryNFrames.load());
        const bool runDetectorThisFrame =
            (frameId % static_cast<uint64_t>(detectEvery)) == 0;

        if (!runDetectorThisFrame) {
          g_lastInferenceSkipped = true;
          // Keep outJpeg empty so the final block stores original JPEG from GStreamer.
          // Do not decode, do not encode, do not run YOLO, do not clear tracker.
        } else {
        std::vector<uint8_t> inJpeg(map.data, map.data + map.size);
        cv::Mat bgr = cv::imdecode(inJpeg, cv::IMREAD_COLOR);

        if (!bgr.empty()) {'''

if old not in s:
    raise SystemExit("pattern 1 not found: detect block header")
s = s.replace(old, new, 1)

# 2. Remove duplicate throttle declarations inside decoded branch.
old = '''          const uint64_t frameId = g_frameId.load();
          const int detectEvery = std::max(1, g_detectEveryNFrames.load());
          const bool runDetectorThisFrame = (frameId % static_cast<uint64_t>(detectEvery)) == 0;
          bool inferenceAttempted = false;'''

new = '''          bool inferenceAttempted = false;'''

if old not in s:
    raise SystemExit("pattern 2 not found: duplicate throttle declarations")
s = s.replace(old, new, 1)

# 3. Since we are inside runDetectorThisFrame branch, remove runDetectorThisFrame conditions.
s = s.replace(
    '          if (runDetectorThisFrame && dm == DetectionMode::FullFrame) {',
    '          if (dm == DetectionMode::FullFrame) {',
    1
)

s = s.replace(
    '          } else if (runDetectorThisFrame && dm == DetectionMode::Tiled) {',
    '          } else if (dm == DetectionMode::Tiled) {',
    1
)

s = s.replace(
    '          } else if (runDetectorThisFrame) {',
    '          } else {',
    1
)

# 4. Remove old skipped-frame branch after inference.
old = '''          if (!runDetectorThisFrame) {
            g_lastInferenceSkipped = true;
          } else if (anyInfer) {'''

new = '''          if (anyInfer) {'''

if old not in s:
    raise SystemExit("pattern 4 not found: old skip branch")
s = s.replace(old, new, 1)

# 5. Add one closing brace for the new else block after imencode.
old = '''          std::vector<int> p = {cv::IMWRITE_JPEG_QUALITY, o.jpegq};
          cv::imencode(".jpg", bgr, outJpeg, p);
        }
      }'''

new = '''          std::vector<int> p = {cv::IMWRITE_JPEG_QUALITY, o.jpegq};
          cv::imencode(".jpg", bgr, outJpeg, p);
        }
        }
      }'''

if old not in s:
    raise SystemExit("pattern 5 not found: imencode close")
s = s.replace(old, new, 1)

p.write_text(s, encoding="utf-8")
print("patched mjpeg_gst_http.cpp")
