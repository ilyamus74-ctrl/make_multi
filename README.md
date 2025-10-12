# make_multi

C++ utilities for working with multiple cameras, including a calibration
tool and YOLOv8-based demos.


## Configuration

`config/config.json` contains an example configuration. At runtime the
utilities look for a `config.json` file located in the same directory as the
executable. This path can be overridden with the `--config` flag where
available. If the file is missing the program will generate a template with the
basic fields, including the new `scheme_type` and per-camera `role`.

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


### Identifying identical USB cameras

Some USB webcams report the same USB identifiers and serial number, which
previously caused them to collapse into a single `/dev/v4l/by-id/…` entry. The
camera manager now looks at both `/dev/v4l/by-id` and `/dev/v4l/by-path`, and
the web UI lets you choose which identifier to use when adding a new camera.

- When the manager discovers a camera it exposes every available identifier in
  the **Add Camera** card. If multiple identifiers are available you can pick
  the desired one from the drop-down.
- Selecting a *by-path* identifier anchors the configuration to the physical
  USB port, which keeps otherwise identical cameras distinct.
- The preview endpoints (`/api/preview` and `/api/preview.mjpg`) now accept
  `?path=` and `?dev=` query parameters in addition to the existing
  `?by=` option, allowing previews of devices referenced by port.

Existing configurations are backwards compatible and will continue to match by
their stored `by-id` substring. Editing and saving a camera through the web UI
will persist both identifiers when available, so future restarts can fall back
to the USB path if the by-id symlink is not unique.


## Calibration Flow

The `yolov8_web_server_calibration` utility streams raw camera frames for
collecting calibration images. Detection and bounding-box drawing are disabled
when the server is started with the `--no-draw` flag, and the web UI fetches the
undistorted preview via `/api/preview.mjpg`. While calibration is running the
video feed shows no detections or overlays.
