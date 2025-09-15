#pragma once
#include <vector>
#include <string>
#include <filesystem>
#include "nlohmann/json.hpp"

namespace stereo {

struct StereoPairCfg { int a=0; int b=0; std::string file; };

void loadConfig(const std::filesystem::path& config_path);
void saveConfig(const std::filesystem::path& config_path);
void updateConfig(const nlohmann::json& cfg);
const nlohmann::json& getConfig();
const std::vector<StereoPairCfg>& getPairs();

} // namespace stereo
