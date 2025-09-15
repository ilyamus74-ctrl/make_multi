# Calibration Scenarios

This guide covers two ways to choose cameras for stereo calibration.

## Manual camera selection

If you already know which devices should form a stereo pair, call the
`/api/calibration/start-auto` endpoint with the camera identifiers:

```json
POST /api/calibration/start-auto
{
  "camera_a": "cam0",
  "camera_b": "cam1",
  "board_w": 9,
  "board_h": 6,
  "square_size": 30.0
}
```

The server captures images from the specified cameras and runs OpenCV's
`opencv_calib_stereo` using the provided chessboard dimensions.

## Automatic compatibility detection

When unsure which cameras pair well, use `/api/calibration/analyze-compatibility`.
Provide the list of connected cameras and chessboard size:

```json
POST /api/calibration/analyze-compatibility
{
  "cameras": ["cam0", "cam1", "cam2"],
  "board_w": 9,
  "board_h": 6,
  "quality_threshold": 30,
  "wide_angle_threshold": 80.0
}
```

A typical response contains characteristics for each camera and suggested pairs:

```json
{
  "status": "ok",
  "camera_characteristics": {
    "cam0": {
      "estimated_fov": 75.2,
      "quality_score": 28,
      "is_wide_angle": false,
      "resolution": [1920, 1080]
    },
    "cam1": {
      "estimated_fov": 120.5,
      "quality_score": 35,
      "is_wide_angle": true,
      "resolution": [1280, 720]
    }
  },
  "compatible_groups": [["cam0", "cam1"]],
  "suggested_pair": ["cam1", "cam0"]
}
```

### Interpreting results

- `estimated_fov` – approximate field of view in degrees. Values above the
  `wide_angle_threshold` indicate wide‑angle lenses.
- `quality_score` – higher values mean sharper, well‑exposed frames, making them
  better candidates for calibration.
- `suggested_pair` – the analyzer's recommended camera pair based on similar FOV
  and adequate image quality.

## Chessboard and lighting tips

- Fill most of the frame with the chessboard while keeping all corners visible.
- Hold the board flat and avoid motion blur; capture from multiple positions and
  angles.
- Use even, diffuse lighting. Avoid strong reflections or deep shadows on the
  board.
- Ensure the pattern has good contrast and remains in focus across frames.

These practices improve corner detection and lead to more accurate calibration
results.
