#include "calibration/calibration.h"
#include <iostream>
#include <filesystem>

int main(int argc, char **argv) {
  if (argc < 2) {
    std::cout << "Usage: " << argv[0]
              << " <record_mono|record_stereo|extract_mono|extract_stereo>"
              << std::endl;
    return 0;
  }
  std::string cmd = argv[1];
  std::filesystem::path exe_dir = std::filesystem::canonical(argv[0]).parent_path();
  std::filesystem::path cfg_path = exe_dir / "config.json";
  CalibSettings cfg = loadCalibSettings(cfg_path.string());
  if (cmd == "record_mono") {
    if (argc < 4) {
      std::cout << "record_mono <device> <out>" << std::endl;
      return 1;
    }
    return recordMono(argv[2], argv[3], cfg.width, cfg.height, cfg.duration,
                       cfg.fps, 8080) ? 0 : 1;
  } else if (cmd == "record_stereo") {
    if (argc < 6) {
      std::cout << "record_stereo <devL> <devR> <outL> <outR>" << std::endl;
      return 1;
    }
    return recordStereo(argv[2], argv[3], argv[4], argv[5], cfg.width,
                         cfg.height, cfg.duration, cfg.fps, 8080) ? 0 : 1;
  } else if (cmd == "extract_mono") {
    if (argc < 4) {
      std::cout << "extract_mono <video> <outdir>" << std::endl;
      return 1;
    }
    extractGoodFramesMono(argv[2], argv[3], cv::Size(9, 6));
    return 0;
  } else if (cmd == "extract_stereo") {
    if (argc < 5) {
      std::cout << "extract_stereo <videoL> <videoR> <outdir>" << std::endl;
      return 1;
    }
    extractGoodFramesStereo(argv[2], argv[3], argv[4], cv::Size(9, 6));
    return 0;
  }
  std::cout << "Unknown command" << std::endl;
  return 1;
}