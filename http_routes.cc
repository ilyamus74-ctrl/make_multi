#include "http_routes.h"

void setup_http_routes(httplib::Server& server) {
    server.set_pre_routing_handler([](const httplib::Request&, httplib::Response& res) {
        res.set_header("Access-Control-Allow-Origin", "*");
        res.set_header("Access-Control-Allow-Methods", "GET, POST, OPTIONS");
        res.set_header("Access-Control-Allow-Headers", "Content-Type, Authorization");
        return httplib::Server::HandlerResponse::Unhandled;
    });
    server.Options(".*", [](const httplib::Request&, httplib::Response&){ });
    server.set_mount_point("/", "./web");
}
