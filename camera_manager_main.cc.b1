#include "camera_manager.h"
#include <iostream>
#include <csignal>
#include <fstream>
#include <vector>

#include "nlohmann/json.hpp"
#include "httplib.h"

static CameraManager g_mgr;
static httplib::Server g_server;

static void sigint(int){ g_mgr.stop(); g_server.stop(); }

int main(int argc, char** argv){
    std::string cfg = "config.json";
    if (argc > 1) cfg = argv[1];
    if (!g_mgr.loadConfig(cfg)) return 1;
    std::ifstream jf(cfg);
    nlohmann::json j; jf >> j;
    int port = j.value("http", nlohmann::json::object()).value("port", 8080);

    std::signal(SIGINT, sigint);
    g_mgr.start();

    g_server.set_mount_point("/", "./web");

    g_server.Get("/api/configured", [](const httplib::Request&, httplib::Response& res){
        auto cams = g_mgr.configuredCameras();
        nlohmann::json out = nlohmann::json::array();
        for (auto& c : cams)
            out.push_back({{"id", c.id}, {"present", c.present}});
        res.set_content(out.dump(), "application/json");
    });

    g_server.Get("/api/new", [](const httplib::Request&, httplib::Response& res){
        nlohmann::json out = g_mgr.unconfiguredCameras();
        res.set_content(out.dump(), "application/json");
    });

    g_server.Post("/api/add", [](const httplib::Request& req, httplib::Response& res){
        try {
            auto j = nlohmann::json::parse(req.body);
            std::string id = j.at("id").get<std::string>();
            std::string by = j.at("by_id").get<std::string>();
            if (!g_mgr.addCamera(id, by)) res.status = 400;
        } catch (...) {
            res.status = 400;
        }
    });

    std::thread http_thr([&]{ g_server.listen("0.0.0.0", port); });
    std::cout << "CameraManager running. Press Ctrl+C to exit." << std::endl;
    while(g_server.is_running()){
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }
    if (http_thr.joinable()) http_thr.join();
    return 0;
}