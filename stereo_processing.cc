#include "stereo_processing.h"
#include <fstream>

namespace stereo {

static nlohmann::json stereo_cfg = nlohmann::json::object();
static std::vector<StereoPairCfg> stereo_pairs;

static void parse(){
    stereo_pairs.clear();
    if(stereo_cfg.contains("pairs") && stereo_cfg["pairs"].is_array()){
        for(auto &p : stereo_cfg["pairs"]) {
            StereoPairCfg sp;
            sp.a = p.value("a",0);
            sp.b = p.value("b",0);
            sp.file = p.value("file", std::string());
            stereo_pairs.push_back(sp);
        }
    }
}

void loadConfig(const std::filesystem::path& config_path){
    auto file = config_path / "stereo_config.json";
    std::ifstream f(file);
    if(f){ try{ f >> stereo_cfg; } catch(...) { stereo_cfg = nlohmann::json::object(); } }
    else stereo_cfg = nlohmann::json::object();
    parse();
}

void saveConfig(const std::filesystem::path& config_path){
    auto file = config_path / "stereo_config.json";
    std::error_code ec;
    std::filesystem::create_directories(file.parent_path(), ec);
    std::ofstream f(file);
    if(f) f << stereo_cfg.dump(2);
}

void updateConfig(const nlohmann::json& cfg){
    stereo_cfg = cfg;
    parse();
}

const nlohmann::json& getConfig(){ return stereo_cfg; }
const std::vector<StereoPairCfg>& getPairs(){ return stereo_pairs; }

} // namespace stereo
