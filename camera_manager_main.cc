#include "camera_manager.h"
#include <iostream>
#include <csignal>

static CameraManager g_mgr;
static void sigint(int){ g_mgr.stop(); }

int main(int argc, char** argv){
    std::string cfg = "config.json";
    if (argc > 1) cfg = argv[1];
    if (!g_mgr.loadConfig(cfg)) return 1;
    std::signal(SIGINT, sigint);
    g_mgr.start();
    std::cout << "CameraManager running. Press Ctrl+C to exit." << std::endl;
    while(true){ std::this_thread::sleep_for(std::chrono::seconds(1)); }
    return 0;
}
