# make_multi

## Calibration utilities

The repository provides a C++ command‑line utility built from
`calibration_main.cc` and `calibration/session.cpp`. It records videos for
each camera defined in `config.json` (placed alongside the executable),
streams the live feed over HTTP and extracts the best frames for mono and
stereo camera calibration.
### Usage

Build the utility with your C++ toolchain (example using g++):

```bash
g++ calibration_main.cc calibration/session.cpp -o calibration_main \
    $(pkg-config --cflags --libs opencv4) -pthread
```

Run one of the following commands:

```bash
./calibration_main record_mono <device> <out>
./calibration_main record_stereo <devL> <devR> <outL> <outR>
./calibration_main extract_mono <video> <outdir>
./calibration_main extract_stereo <videoL> <videoR> <outdir>
```
```

Running the program will:

1. Record a 30‑second video for each camera individually while serving the
   stream at `http://localhost:8080`.
2. Select high‑quality frames from each video based on sharpness, brightness,
   color and chessboard detection, saving only the chosen frames and deleting
   the raw video.
3. Capture a synchronized 30‑second video for all cameras and perform the same
   frame selection for stereo calibration.

The resulting images are stored under `calibration/mono/<id>` and
`calibration/stereo/<id>`
