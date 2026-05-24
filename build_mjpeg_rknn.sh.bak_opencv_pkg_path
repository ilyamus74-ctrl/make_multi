#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$SCRIPT_DIR"
BUILD_DIR="${BUILD_DIR:-$ROOT_DIR/.build}"
mkdir -p "$BUILD_DIR"

cd "$ROOT_DIR"

pick_existing_file() {
  for path in "$@"; do
    if [[ -f "$path" ]]; then
      echo "$path"
      return 0
    fi
  done
  return 1
}

MJPEG_SRC="$(pick_existing_file "$ROOT_DIR/mjpeg_gst_http.cpp" "$ROOT_DIR/new_yolo8/mjpeg_gst_http.cpp" || true)"
POSTPROCESS_SRC="$(pick_existing_file "$ROOT_DIR/postprocess.cc" "$ROOT_DIR/new_yolo8/postprocess.cc" || true)"
IMAGE_UTILS_SRC="$(pick_existing_file "$ROOT_DIR/include/utils/image_utils.c" "$ROOT_DIR/utils/image_utils.c" || true)"
FILE_UTILS_SRC="$(pick_existing_file "$ROOT_DIR/include/utils/file_utils.c" "$ROOT_DIR/utils/file_utils.c" || true)"

missing=()
[[ -n "$MJPEG_SRC" ]] || missing+=("mjpeg_gst_http.cpp")
[[ -n "$POSTPROCESS_SRC" ]] || missing+=("postprocess.cc")
[[ -n "$IMAGE_UTILS_SRC" ]] || missing+=("image_utils.c")
[[ -n "$FILE_UTILS_SRC" ]] || missing+=("file_utils.c")
[[ -f "$ROOT_DIR/rknpu2/yolov8.cc" ]] || missing+=("rknpu2/yolov8.cc")

if (( ${#missing[@]} > 0 )); then
  echo "Missing required source files: ${missing[*]}" >&2
  echo "Please run this script in a full RKNN source tree (or restore missing files)." >&2
  exit 1
fi

CFLAGS="-O2 -I$ROOT_DIR -I$ROOT_DIR/include -I$ROOT_DIR/include/utils -I$ROOT_DIR/include/3rdparty/rknpu2/include -I$ROOT_DIR/include/3rdparty/librga/include"
CXXFLAGS="-O2 -std=c++17 $CFLAGS"

gcc $CFLAGS -c "$IMAGE_UTILS_SRC" -o "$BUILD_DIR/image_utils.o"
gcc $CFLAGS -c "$FILE_UTILS_SRC" -o "$BUILD_DIR/file_utils.o"

EXTRA_LINK_LIBS="${EXTRA_LINK_LIBS:-}"

# image_utils.c depends on TurboJPEG (tj*) and librga (wrapbuffer*/improcess).
# Keep these defaults for RKNN demo environments; allow overrides via EXTRA_LINK_LIBS.
DEFAULT_RK_LINK_LIBS="-lturbojpeg -lrga"

# shellcheck disable=SC2046
g++ $CXXFLAGS \
  "$MJPEG_SRC" \
  "$POSTPROCESS_SRC" \
  rknpu2/yolov8.cc \
  "$BUILD_DIR/image_utils.o" \
  "$BUILD_DIR/file_utils.o" \
  -o "$BUILD_DIR/mjpeg_rknn_http" \
  $(pkg-config --cflags --libs gstreamer-1.0 gstreamer-app-1.0 opencv4) \
  -Llib -lrknnrt $DEFAULT_RK_LINK_LIBS $EXTRA_LINK_LIBS -lpthread -ldl

echo "Built: $BUILD_DIR/mjpeg_rknn_http"


# Build PTZ autopilot
g++ -O2 -std=c++17 ptz_autopilot.cpp -o "$BUILD_DIR/ptz_autopilot" -lpthread
echo "Built: $BUILD_DIR/ptz_autopilot"
