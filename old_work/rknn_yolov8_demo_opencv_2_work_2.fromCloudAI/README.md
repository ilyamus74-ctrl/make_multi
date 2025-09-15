# make_multi

C++ utilities for working with multiple cameras, including a calibration
tool and YOLOv8-based demos.


## Configuration

The utilities look for a `config.json` file located in the same directory as
the executable. This path can be overridden with the `--config` flag where
available.


### Camera Modes

Each camera entry in `config.json` uses a `mode` field to control how the
device operates:

- `preview` – capture frames for viewing only, detection is disabled.
- `detect` – run the detection pipeline for the camera.
- `calibration` – reserve the camera for calibration tasks.

Older configurations that used the `preview` boolean are still accepted but
should be migrated to the new `mode` field.


## Calibration Flow

The `yolov8_web_server_calibration` utility streams raw camera frames for
collecting calibration images. Detection and bounding-box drawing are disabled
when the server is started with the `--no-draw` flag, and the web UI fetches the
undistorted preview via `/api/preview.mjpg`. While calibration is running the
video feed shows no detections or overlays.