#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <csignal>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <unordered_map>
#include <unordered_set>
#include <vector>
#include <cstdio>
#include <ctime>

#include "httplib.h"
#include "nlohmann/json.hpp"

using json = nlohmann::json;

struct Options {
    std::string config_path = "./config.json";
    std::string host = "127.0.0.1";
    int base_port = 5000;
    int pull_interval_ms = 250;
    int http_port = 7090;
    int max_tracks_per_cam = 50;
};

struct CamInfo {
    std::string id;
    std::string host;
    int port = 0;
    bool up = false;
    uint64_t last_pull_ms = 0;
};

struct TrackView {
    int local_id = -1;
    int global_id = -1;
    std::string cls;
    float conf = 0.f;
    uint64_t age_ms = 0;
    std::array<int, 4> bbox{0, 0, 0, 0};
    uint64_t ts_ms = 0;
};

struct CamState {
    CamInfo info;
    bool up = false;
    uint64_t last_pull_ms = 0;
    uint64_t last_success_ms = 0;
    uint64_t last_remote_ts = 0;
    std::vector<TrackView> tracks;
};

struct TrackLogInfo {
    int global_id = -1;
    uint64_t last_ts_ms = 0;
    uint64_t last_age_ms = 0;
};

static uint64_t now_ms() {
    return std::chrono::duration_cast<std::chrono::milliseconds>(
               std::chrono::system_clock::now().time_since_epoch())
        .count();
}

static std::string formatUtc(uint64_t ts_ms) {
    if (ts_ms == 0) {
        return "-";
    }
    std::time_t sec = static_cast<std::time_t>(ts_ms / 1000);
    std::tm tm{};
#if defined(_WIN32)
    gmtime_s(&tm, &sec);
#else
    gmtime_r(&sec, &tm);
#endif
    char buf[64];
    std::snprintf(buf, sizeof(buf), "%04d-%02d-%02dT%02d:%02d:%02d.%03uZ",
                  tm.tm_year + 1900,
                  tm.tm_mon + 1,
                  tm.tm_mday,
                  tm.tm_hour,
                  tm.tm_min,
                  tm.tm_sec,
                  static_cast<unsigned>(ts_ms % 1000));
    return buf;
}

static std::string escapeHtml(const std::string& in) {
    std::string out;
    out.reserve(in.size());
    for (char c : in) {
        switch (c) {
            case '&': out += "&amp;"; break;
            case '<': out += "&lt;"; break;
            case '>': out += "&gt;"; break;
            case '"': out += "&quot;"; break;
            case '\'': out += "&#39;"; break;
            default: out.push_back(c); break;
        }
    }
    return out;
}

static void printUsage(const char* argv0) {
    std::cout << "Usage: " << argv0 << " [--config file] [--host 127.0.0.1] [--base-port 5000]\n"
              << "              [--pull-interval-ms 250] [--http-port 7090] [--max-tracks-per-cam 50]\n";
}

enum class ParseResult { kOk, kShowUsage, kError };

static bool parseInt(const std::string& value, int& out) {
    try {
        size_t idx = 0;
        int parsed = std::stoi(value, &idx);
        if (idx != value.size()) return false;
        out = parsed;
        return true;
    } catch (...) {
        return false;
    }
}

static ParseResult parseArgs(int argc, char** argv, Options& opts) {
    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        auto need_value = [&](const char* name) -> std::string {
            if (i + 1 >= argc) {
                std::cerr << "Missing value for " << name << "\n";
                printUsage(argv[0]);
                throw std::runtime_error("missing value");
            }
            return argv[++i];
        };
        try {
            if (arg == "--config") {
                opts.config_path = need_value(arg.c_str());
            } else if (arg == "--host") {
                opts.host = need_value(arg.c_str());
            } else if (arg == "--base-port") {
                std::string v = need_value(arg.c_str());
                if (!parseInt(v, opts.base_port)) {
                    std::cerr << "Invalid --base-port value" << std::endl;
                    return ParseResult::kError;
                }
            } else if (arg == "--pull-interval-ms") {
                std::string v = need_value(arg.c_str());
                if (!parseInt(v, opts.pull_interval_ms)) {
                    std::cerr << "Invalid --pull-interval-ms value" << std::endl;
                    return ParseResult::kError;
                }
            } else if (arg == "--http-port") {
                std::string v = need_value(arg.c_str());
                if (!parseInt(v, opts.http_port)) {
                    std::cerr << "Invalid --http-port value" << std::endl;
                    return ParseResult::kError;
                }
            } else if (arg == "--max-tracks-per-cam") {
                std::string v = need_value(arg.c_str());
                if (!parseInt(v, opts.max_tracks_per_cam)) {
                    std::cerr << "Invalid --max-tracks-per-cam value" << std::endl;
                    return ParseResult::kError;
                }
            } else if (arg == "--help" || arg == "-h") {
                printUsage(argv[0]);
                return ParseResult::kShowUsage;
            } else {
                std::cerr << "Unknown argument: " << arg << "\n";
                printUsage(argv[0]);
                return ParseResult::kError;
            }
        } catch (const std::runtime_error&) {
            return ParseResult::kError;
        }
    }
    return ParseResult::kOk;
}

static std::vector<CamInfo> loadCameras(const Options& opts) {
    std::vector<CamInfo> cams;
    std::ifstream f(opts.config_path);
    if (!f) {
        std::cerr << "Failed to open config: " << opts.config_path << "\n";
        return cams;
    }
    json cfg;
    try {
        f >> cfg;
    } catch (const std::exception& e) {
        std::cerr << "Failed to parse config: " << e.what() << "\n";
        return cams;
    }
    auto it = cfg.find("cameras");
    if (it == cfg.end() || !it->is_array()) {
        std::cerr << "Config does not contain cameras array" << std::endl;
        return cams;
    }
    int idx = 0;
    for (const auto& cam : *it) {
        if (!cam.is_object()) continue;
        CamInfo info;
        info.id = cam.value("id", std::string());
        if (info.id.empty()) {
            info.id = "cam" + std::to_string(idx);
        }
        info.host = cam.value("host", opts.host);
        if (cam.contains("http_port")) {
            info.port = cam.value("http_port", opts.base_port + idx);
        } else if (cam.contains("det_port")) {
            info.port = cam.value("det_port", opts.base_port + idx);
        } else {
            info.port = opts.base_port + idx;
        }
        cams.push_back(info);
        ++idx;
    }
    return cams;
}

class Logger {
public:
    explicit Logger(std::string path) : path_(std::move(path)) {
        std::filesystem::path p(path_);
        std::error_code ec;
        std::filesystem::create_directories(p.parent_path(), ec);
    }

    void log(const std::string& line) {
        std::lock_guard<std::mutex> lk(mu_);
        std::ofstream ofs(path_, std::ios::app);
        if (ofs) {
            ofs << line << '\n';
        }
    }

private:
    std::string path_;
    std::mutex mu_;
};

class GlobalDiagApp {
public:
    GlobalDiagApp(Options opts, std::vector<CamInfo> cams)
        : opts_(std::move(opts)),
          cams_(std::move(cams)),
          states_(cams_.size()),
          logger_("logs/global_diag.log") {
        if (opts_.max_tracks_per_cam <= 0) {
            opts_.max_tracks_per_cam = 1;
        }
        for (size_t i = 0; i < cams_.size(); ++i) {
            states_[i].info = cams_[i];
        }
    }

    ~GlobalDiagApp() { stop(); }

    void start() {
        poll_thread_ = std::thread(&GlobalDiagApp::pollLoop, this);
    }

    void run() {
        setupRoutes();
        server_.listen("0.0.0.0", opts_.http_port);
    }

    void stop() {
        bool expected = false;
        if (stop_flag_.compare_exchange_strong(expected, true)) {
            server_.stop();
        } else {
            stop_flag_.store(true);
            server_.stop();
        }
        if (poll_thread_.joinable()) {
            poll_thread_.join();
        }
    }

private:
    void setupRoutes() {
        server_.Get("/", [this](const httplib::Request&, httplib::Response& res) {
            auto snapshot = getStateSnapshot();
            res.set_content(buildHtml(snapshot), "text/html; charset=utf-8");
        });

        server_.Get("/state", [this](const httplib::Request&, httplib::Response& res) {
            auto snapshot = getStateSnapshot();
            json j;
            j["ts_ms"] = now_ms();
            auto cams = json::array();
            for (const auto& cs : snapshot) {
                json jc;
                jc["id"] = cs.info.id;
                jc["status"] = cs.up ? "UP" : "DOWN";
                jc["tracks"] = cs.tracks.size();
                size_t with_global = 0;
                for (const auto& t : cs.tracks) {
                    if (t.global_id >= 0) ++with_global;
                }
                jc["with_global"] = with_global;
                jc["without_global"] = cs.tracks.size() - with_global;
                jc["last_pull_ms"] = cs.last_pull_ms;
                jc["last_success_ms"] = cs.last_success_ms;
                jc["last_remote_ts"] = cs.last_remote_ts;
                cams.push_back(std::move(jc));
            }
            j["cams"] = std::move(cams);
            res.set_content(j.dump(), "application/json");
        });
    }

    std::vector<CamState> getStateSnapshot() const {
        std::lock_guard<std::mutex> lk(state_mx_);
        return states_;
    }

    std::string buildHtml(const std::vector<CamState>& snapshot) const {
        std::ostringstream oss;
        oss << "<!DOCTYPE html><html><head><meta charset='utf-8'>"
            << "<meta http-equiv='refresh' content='1'>"
            << "<title>Global diagnostics</title>"
            << "<style>body{font-family:Arial,Helvetica,sans-serif;background:#111;color:#eee;padding:16px;}"
            << "table{width:100%;border-collapse:collapse;}"
            << "th,td{border:1px solid #333;padding:6px;text-align:left;}"
            << "th{background:#222;}"
            << ".status-up{color:#4caf50;font-weight:bold;}"
            << ".status-down{color:#f44336;font-weight:bold;}"
            << ".objects div{white-space:nowrap;overflow:hidden;text-overflow:ellipsis;}"
            << "</style></head><body>";
        oss << "<h1>Global diagnostics</h1>";
        oss << "<table><thead><tr><th>Camera</th><th>Status</th><th>Tracks"
            << "</th><th>With G#</th><th>Without G#</th><th>Last update (UTC)</th><th>Последние объекты</th></tr></thead><tbody>";
        if (snapshot.empty()) {
            oss << "<tr><td colspan='7'>No cameras configured</td></tr>";
        } else {
            for (const auto& cs : snapshot) {
                std::vector<TrackView> tracks = cs.tracks;
                std::sort(tracks.begin(), tracks.end(), [](const TrackView& a, const TrackView& b) {
                    if (a.ts_ms == b.ts_ms) return a.local_id < b.local_id;
                    return a.ts_ms > b.ts_ms;
                });
                size_t with_global = 0;
                for (const auto& t : tracks) {
                    if (t.global_id >= 0) ++with_global;
                }
                oss << "<tr>";
                oss << "<td>" << escapeHtml(cs.info.id) << "</td>";
                const char* status_class = cs.up ? "status-up" : "status-down";
                const char* status_text = cs.up ? "UP" : "DOWN";
                oss << "<td class='" << status_class << "'>" << status_text << "</td>";
                oss << "<td>" << tracks.size() << "</td>";
                oss << "<td>" << with_global << "</td>";
                oss << "<td>" << (tracks.size() - with_global) << "</td>";
                oss << "<td>" << escapeHtml(formatUtc(cs.last_success_ms)) << "</td>";
                oss << "<td><div class='objects'>";
                size_t limit = std::min<size_t>(tracks.size(), 10);
                for (size_t i = 0; i < limit; ++i) {
                    const auto& t = tracks[i];
                    std::ostringstream row;
                    row << "L#" << t.local_id << " -> G#" << t.global_id
                        << " | " << escapeHtml(t.cls)
                        << " | age " << t.age_ms << "ms"
                        << " | conf " << std::fixed << std::setprecision(2) << t.conf;
                    oss << "<div>" << row.str() << "</div>";
                }
                if (limit == 0) {
                    oss << "<div>-</div>";
                }
                oss << "</div></td></tr>";
            }
        }
        oss << "</tbody></table>";
        oss << "<p>Server time: " << escapeHtml(formatUtc(now_ms())) << "</p>";
        oss << "</body></html>";
        return oss.str();
    }

    void pollLoop() {
        while (!stop_flag_.load()) {
            uint64_t cycle_start = now_ms();
            for (size_t i = 0; i < cams_.size(); ++i) {
                pollCamera(i);
                if (stop_flag_.load()) break;
            }
            maybeLogHeartbeat(now_ms());
            uint64_t elapsed = now_ms() - cycle_start;
            if (elapsed < static_cast<uint64_t>(opts_.pull_interval_ms)) {
                std::this_thread::sleep_for(std::chrono::milliseconds(opts_.pull_interval_ms - elapsed));
            }
        }
    }

    void pollCamera(size_t idx) {
        if (idx >= cams_.size()) return;
        const auto& cam = cams_[idx];
        std::vector<TrackView> tracks;
        uint64_t remote_ts = 0;
        bool ok = fetchTracks(cam, tracks, remote_ts);
        uint64_t now = now_ms();
        {
            std::lock_guard<std::mutex> lk(state_mx_);
            auto& state = states_[idx];
            state.last_pull_ms = now;
            if (ok) {
                state.up = true;
                state.last_success_ms = now;
                state.last_remote_ts = remote_ts;
                state.tracks = tracks;
            } else {
                state.up = false;
                state.tracks.clear();
            }
        }
        if (ok) {
            logTrackEvents(cam.id, tracks);
        } else {
            handleCameraDown(cam.id);
        }
    }

    bool fetchTracks(const CamInfo& cam, std::vector<TrackView>& out, uint64_t& remote_ts) {
        httplib::Client cli(cam.host.c_str(), cam.port);
        cli.set_connection_timeout(0, 200000);
        cli.set_read_timeout(0, 200000);
        cli.set_write_timeout(0, 200000);
        cli.set_keep_alive(false);
        if (auto res = cli.Get("/tracks")) {
            if (res->status != 200) return false;
            auto parsed = json::parse(res->body, nullptr, false);
            if (parsed.is_discarded()) return false;
            remote_ts = parsed.value("ts_ms", now_ms());
            auto tracks_it = parsed.find("tracks");
            if (tracks_it == parsed.end() || !tracks_it->is_array()) return false;
            out.clear();
            for (const auto& t : *tracks_it) {
                if (!t.is_object()) continue;
                TrackView tv;
                tv.local_id = t.value("local_id", -1);
                if (tv.local_id < 0) continue;
                tv.global_id = t.value("global_id", -1);
                tv.cls = t.value("class", std::string());
                tv.conf = t.value("conf", 0.0f);
                tv.age_ms = t.value("age_ms", 0ULL);
                tv.ts_ms = t.value("ts_ms", remote_ts);
                auto bbox_it = t.find("bbox");
                if (bbox_it != t.end()) {
                    if (bbox_it->is_array() && bbox_it->size() >= 4) {
                        for (size_t i = 0; i < 4; ++i) {
                            tv.bbox[i] = static_cast<int>((*bbox_it)[i].get<int>());
                        }
                    } else if (bbox_it->is_object()) {
                        tv.bbox[0] = bbox_it->value("x", 0);
                        tv.bbox[1] = bbox_it->value("y", 0);
                        tv.bbox[2] = bbox_it->value("w", 0);
                        tv.bbox[3] = bbox_it->value("h", 0);
                    }
                }
                out.push_back(std::move(tv));
            }
            std::sort(out.begin(), out.end(), [](const TrackView& a, const TrackView& b) {
                if (a.ts_ms == b.ts_ms) return a.local_id < b.local_id;
                return a.ts_ms > b.ts_ms;
            });
            if (out.size() > static_cast<size_t>(opts_.max_tracks_per_cam)) {
                out.resize(opts_.max_tracks_per_cam);
            }
            return true;
        }
        return false;
    }

    void logTrackEvents(const std::string& cam_id, const std::vector<TrackView>& tracks) {
        auto& prev = track_log_state_[cam_id];
        std::unordered_set<int> seen;
        for (const auto& tv : tracks) {
            seen.insert(tv.local_id);
            auto it = prev.find(tv.local_id);
            if (it == prev.end()) {
                std::ostringstream oss;
                oss << formatUtc(tv.ts_ms) << ' ' << cam_id
                    << " L#" << tv.local_id << "->G#" << tv.global_id
                    << " class=" << tv.cls
                    << " conf=" << std::fixed << std::setprecision(2) << tv.conf
                    << " age=" << tv.age_ms << "ms";
                logger_.log(oss.str());
                prev.emplace(tv.local_id, TrackLogInfo{tv.global_id, tv.ts_ms, tv.age_ms});
            } else {
                if (it->second.global_id != tv.global_id) {
                    std::ostringstream oss;
                    oss << formatUtc(tv.ts_ms) << ' ' << cam_id
                        << " L#" << tv.local_id << "->G#" << tv.global_id
                        << " class=" << tv.cls
                        << " conf=" << std::fixed << std::setprecision(2) << tv.conf
                        << " age=" << tv.age_ms << "ms";
                    logger_.log(oss.str());
                }
                it->second.global_id = tv.global_id;
                it->second.last_ts_ms = tv.ts_ms;
                it->second.last_age_ms = tv.age_ms;
            }
        }
        uint64_t ts = now_ms();
        for (auto it = prev.begin(); it != prev.end();) {
            if (!seen.count(it->first)) {
                std::ostringstream oss;
                oss << formatUtc(ts) << ' ' << cam_id
                    << " L#" << it->first << " end age=" << it->second.last_age_ms << "ms";
                logger_.log(oss.str());
                it = prev.erase(it);
            } else {
                ++it;
            }
        }
    }

    void handleCameraDown(const std::string& cam_id) {
        auto it = track_log_state_.find(cam_id);
        if (it == track_log_state_.end()) return;
        uint64_t ts = now_ms();
        for (const auto& kv : it->second) {
            std::ostringstream oss;
            oss << formatUtc(ts) << ' ' << cam_id
                << " L#" << kv.first << " end age=" << kv.second.last_age_ms << "ms";
            logger_.log(oss.str());
        }
        track_log_state_.erase(it);
    }

    void maybeLogHeartbeat(uint64_t ts) {
        if (ts - last_heartbeat_ms_ < 5000) {
            return;
        }
        auto snapshot = getStateSnapshot();
        size_t up = 0;
        size_t total_tracks = 0;
        size_t with_global = 0;
        for (const auto& cs : snapshot) {
            if (cs.up) ++up;
            total_tracks += cs.tracks.size();
            for (const auto& t : cs.tracks) {
                if (t.global_id >= 0) ++with_global;
            }
        }
        std::ostringstream oss;
        oss << formatUtc(ts) << " HEARTBEAT cams=" << snapshot.size()
            << " up=" << up
            << " down=" << (snapshot.size() >= up ? snapshot.size() - up : 0)
            << " total_tracks=" << total_tracks
            << " withG=" << with_global;
        logger_.log(oss.str());
        last_heartbeat_ms_ = ts;
    }

    Options opts_;
    std::vector<CamInfo> cams_;
    mutable std::mutex state_mx_;
    std::vector<CamState> states_;
    httplib::Server server_;
    std::thread poll_thread_;
    std::atomic<bool> stop_flag_{false};
    Logger logger_;
    std::unordered_map<std::string, std::unordered_map<int, TrackLogInfo>> track_log_state_;
    uint64_t last_heartbeat_ms_ = 0;
};

static GlobalDiagApp* g_app = nullptr;

static void handleSignal(int) {
    if (g_app) {
        g_app->stop();
    }
}

int main(int argc, char** argv) {
    Options opts;
    ParseResult pr = parseArgs(argc, argv, opts);
    if (pr == ParseResult::kShowUsage) {
        return 0;
    }
    if (pr == ParseResult::kError) {
        return 1;
    }
    auto cams = loadCameras(opts);
    if (cams.empty()) {
        std::cerr << "No cameras found in config" << std::endl;
    } else {
        std::cout << "Loaded " << cams.size() << " cameras" << std::endl;
    }
    GlobalDiagApp app(opts, std::move(cams));
    g_app = &app;
    std::signal(SIGINT, handleSignal);
    std::signal(SIGTERM, handleSignal);
    app.start();
    app.run();
    app.stop();
    return 0;
}
