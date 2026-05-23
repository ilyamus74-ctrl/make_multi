#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <iostream>
#include <iterator>
#include <mutex>
#include <netdb.h>
#include <netinet/in.h>
#include <sstream>
#include <string>
#include <sys/socket.h>
#include <thread>
#include <unistd.h>
#include <vector>
#include <arpa/inet.h>

struct Config {std::string mjpeg_url="http://127.0.0.1:8080"; std::string tracker_host="127.0.0.1"; int tracker_port=8080; std::string bridge_host="127.0.0.1"; int bridge_port=8765; int control_port=8090; int width=1920; int height=1080; double hz=20, kp=20, ki=0, kd=3, deadzone=0.05; int max_pan=20,max_tilt=20,max_accel=4; bool invert_pan=false; bool invert_tilt=true; double target_x=0.5,target_y=0.5; int min_pan=0,min_tilt=0; bool enabled=false;};
struct TrackerState {std::string mode="IDLE"; int track_id=0; bool valid=false; int l=0,t=0,r=0,b=0;};
struct Runtime {std::atomic<bool> run{true}; std::atomic<bool> enabled{false}; std::atomic<int> seq{1}; std::mutex m; std::string last_tracker_mode="IDLE"; int last_track_id=0; double errx=0,erry=0; int cmdp=0,cmdt=0; std::string last_error; std::string mode="IDLE";};
static Runtime g; static Config cfg;
static std::string g_ptzConfigFile = "ptz_autopilot_config.json";

static std::string trim(const std::string&s){size_t a=s.find_first_not_of(" \t\r\n"); if(a==std::string::npos) return ""; size_t b=s.find_last_not_of(" \t\r\n"); return s.substr(a,b-a+1);} 
static bool json_bool(const std::string&s,const std::string&k,bool &v){auto p=s.find("\""+k+"\""); if(p==std::string::npos) return false; p=s.find(':',p); if(p==std::string::npos) return false; auto t=trim(s.substr(p+1,6)); if(t.rfind("true",0)==0){v=true;return true;} if(t.rfind("false",0)==0){v=false;return true;} return false;}
static bool json_int(const std::string&s,const std::string&k,int &v){auto p=s.find("\""+k+"\""); if(p==std::string::npos) return false; p=s.find(':',p); if(p==std::string::npos) return false; char*e=nullptr; long x=strtol(s.c_str()+p+1,&e,10); if(e==s.c_str()+p+1) return false; v=(int)x; return true;}
static bool json_num(const std::string&s,const std::string&k,double &v){auto p=s.find("\""+k+"\""); if(p==std::string::npos) return false; p=s.find(':',p); if(p==std::string::npos) return false; char*e=nullptr; double x=strtod(s.c_str()+p+1,&e); if(e==s.c_str()+p+1) return false; v=x; return true;}
static std::string json_str(const std::string&s,const std::string&k){auto p=s.find("\""+k+"\""); if(p==std::string::npos) return ""; p=s.find(':',p); p=s.find('"',p); if(p==std::string::npos) return ""; auto e=s.find('"',p+1); if(e==std::string::npos) return ""; return s.substr(p+1,e-p-1);} 

static void clamp_config() {
  cfg.hz = std::max(1.0, std::min(50.0, cfg.hz));
  cfg.kp = std::max(0.0, std::min(100.0, cfg.kp));
  cfg.ki = std::max(0.0, std::min(20.0, cfg.ki));
  cfg.kd = std::max(0.0, std::min(50.0, cfg.kd));
  cfg.deadzone = std::max(0.0, std::min(0.5, cfg.deadzone));

  cfg.max_pan = std::max(1, std::min(100, cfg.max_pan));
  cfg.max_tilt = std::max(1, std::min(100, cfg.max_tilt));
  cfg.max_accel = std::max(1, std::min(100, cfg.max_accel));

  cfg.target_x = std::max(0.1, std::min(0.9, cfg.target_x));
  cfg.target_y = std::max(0.1, std::min(0.9, cfg.target_y));

  cfg.min_pan = std::max(0, std::min(cfg.max_pan, cfg.min_pan));
  cfg.min_tilt = std::max(0, std::min(cfg.max_tilt, cfg.min_tilt));
}
static void save_ptz_config() {
  std::ofstream f(g_ptzConfigFile);
  if (!f) return;

  f << "{\n"
    << "  \"kp\": " << cfg.kp << ",\n"
    << "  \"ki\": " << cfg.ki << ",\n"
    << "  \"kd\": " << cfg.kd << ",\n"
    << "  \"deadzone\": " << cfg.deadzone << ",\n"
    << "  \"max_pan\": " << cfg.max_pan << ",\n"
    << "  \"max_tilt\": " << cfg.max_tilt << ",\n"
    << "  \"max_accel\": " << cfg.max_accel << ",\n"
    << "  \"hz\": " << cfg.hz << ",\n"
    << "  \"invert_pan\": " << (cfg.invert_pan ? "true" : "false") << ",\n"
    << "  \"invert_tilt\": " << (cfg.invert_tilt ? "true" : "false") << ",\n"
    << "  \"target_x\": " << cfg.target_x << ",\n"
    << "  \"target_y\": " << cfg.target_y << ",\n"
    << "  \"min_pan\": " << cfg.min_pan << ",\n"
    << "  \"min_tilt\": " << cfg.min_tilt << "\n"
    << "}\n";
}
static void load_ptz_config() {
  std::ifstream f(g_ptzConfigFile);
  if (!f) return;

  std::string body((std::istreambuf_iterator<char>(f)), std::istreambuf_iterator<char>());

  json_num(body, "kp", cfg.kp);
  json_num(body, "ki", cfg.ki);
  json_num(body, "kd", cfg.kd);
  json_num(body, "deadzone", cfg.deadzone);

  json_int(body, "max_pan", cfg.max_pan);
  json_int(body, "max_tilt", cfg.max_tilt);
  json_int(body, "max_accel", cfg.max_accel);

  json_num(body, "hz", cfg.hz);

  bool b = false;
  if (json_bool(body, "invert_pan", b)) cfg.invert_pan = b;
  if (json_bool(body, "invert_tilt", b)) cfg.invert_tilt = b;

  json_num(body, "target_x", cfg.target_x);
  json_num(body, "target_y", cfg.target_y);

  json_int(body, "min_pan", cfg.min_pan);
  json_int(body, "min_tilt", cfg.min_tilt);

  clamp_config();
}

static int tcp_connect(const std::string& host,int port){int fd=socket(AF_INET,SOCK_STREAM,0); if(fd<0)return -1; sockaddr_in a{}; a.sin_family=AF_INET; a.sin_port=htons(port); if(inet_pton(AF_INET,host.c_str(),&a.sin_addr)<=0){close(fd); return -1;} if(connect(fd,(sockaddr*)&a,sizeof(a))<0){close(fd);return -1;} return fd;}
static bool http_get(const std::string&host,int port,const std::string&path,std::string&body){int fd=tcp_connect(host,port); if(fd<0) return false; std::ostringstream req; req<<"GET "<<path<<" HTTP/1.1\r\nHost: "<<host<<":"<<port<<"\r\nConnection: close\r\n\r\n"; std::string r=req.str(); send(fd,r.data(),r.size(),0); std::string resp; char buf[4096]; ssize_t n; while((n=recv(fd,buf,sizeof(buf),0))>0) resp.append(buf,n); close(fd); auto p=resp.find("\r\n\r\n"); if(p==std::string::npos) return false; body=resp.substr(p+4); return resp.find(" 200 ")!=std::string::npos;}

class WsClient{int fd=-1; public: bool connect_ws(const std::string&h,int p){if(fd>=0) return true; fd=tcp_connect(h,p); if(fd<0) return false; std::string req="GET /ws HTTP/1.1\r\nHost: "+h+":"+std::to_string(p)+"\r\nUpgrade: websocket\r\nConnection: Upgrade\r\nSec-WebSocket-Key: dGhlIHNhbXBsZSBub25jZQ==\r\nSec-WebSocket-Version: 13\r\n\r\n"; send(fd,req.data(),req.size(),0); char b[1024]; int n=recv(fd,b,sizeof(b),0); if(n<=0){close(fd);fd=-1; return false;} std::string s(b,n); if(s.find("101")==std::string::npos){close(fd);fd=-1; return false;} return true;} bool send_text(const std::string&m){ if(fd<0) return false; std::vector<uint8_t> fr; fr.push_back(0x81); size_t len=m.size(); if(len<126){fr.push_back(0x80|uint8_t(len));} else return false; uint8_t mk[4]={0x12,0x34,0x56,0x78}; fr.insert(fr.end(),mk,mk+4); for(size_t i=0;i<len;i++) fr.push_back(uint8_t(m[i])^mk[i%4]); if(::send(fd,fr.data(),fr.size(),0)<0){close(fd);fd=-1; return false;} return true;} void close_ws(){if(fd>=0) close(fd); fd=-1;} ~WsClient(){close_ws();}};

static bool parse_tracker(const std::string&j,TrackerState&ts){
  ts.mode=json_str(j,"mode");
  if(ts.mode.empty()) return false;

  json_int(j,"selected_track_id",ts.track_id);
  json_bool(j,"selected_box_valid",ts.valid);

  // /api/tracker/state returns: "box": {...}, not "selected_box".
  auto p=j.find("\"box\"");
  if(p!=std::string::npos){
    auto sub=j.substr(p);
    json_int(sub,"left",ts.l);
    json_int(sub,"top",ts.t);
    json_int(sub,"right",ts.r);
    json_int(sub,"bottom",ts.b);
  }

  // Safety: if tracker says valid but box was not parsed or is degenerate,
  // do not allow autopilot to move from fake 0,0,0,0 coordinates.
  if(ts.valid && (ts.r <= ts.l || ts.b <= ts.t)){
    ts.valid=false;
  }

  return true;
}

static void autopilot_loop(){WsClient ws; auto last=std::chrono::steady_clock::now(); double ix=0,iy=0,pex=0,pey=0; int prevp=0,prevt=0; bool sent_stop=false; while(g.run){auto start=std::chrono::steady_clock::now(); TrackerState ts; std::string body; bool ok=http_get(cfg.tracker_host,cfg.tracker_port,"/api/tracker/state",body)&&parse_tracker(body,ts); int pan=0,tilt=0; double ex=0,ey=0; bool active=g.enabled.load(); bool needSend=false; if(active && ok && ts.mode=="TRACKING" && ts.valid){double cx=(ts.l+ts.r)*0.5, cy=(ts.t+ts.b)*0.5; const double nx=cx/std::max(1,cfg.width); const double ny=cy/std::max(1,cfg.height); ex=(nx-cfg.target_x)*2.0; ey=(ny-cfg.target_y)*2.0; if(std::abs(ex)<cfg.deadzone) ex=0; if(std::abs(ey)<cfg.deadzone) ey=0; auto now=std::chrono::steady_clock::now(); double dt=std::chrono::duration<double>(now-last).count(); last=now; dt=std::max(0.02,std::min(0.2,dt)); ix+=ex*dt; iy+=ey*dt; double dx=(ex-pex)/dt, dy=(ey-pey)/dt; pex=ex; pey=ey; pan=int(std::lround(cfg.kp*ex+cfg.ki*ix+cfg.kd*dx)); tilt=int(std::lround(cfg.kp*ey+cfg.ki*iy+cfg.kd*dy)); pan=std::max(-cfg.max_pan,std::min(cfg.max_pan,pan)); tilt=std::max(-cfg.max_tilt,std::min(cfg.max_tilt,tilt)); if(ex!=0.0 && cfg.min_pan>0 && std::abs(pan)<cfg.min_pan) pan=(ex>0)?cfg.min_pan:-cfg.min_pan; if(ey!=0.0 && cfg.min_tilt>0 && std::abs(tilt)<cfg.min_tilt) tilt=(ey>0)?cfg.min_tilt:-cfg.min_tilt; if(cfg.invert_pan) pan=-pan; if(cfg.invert_tilt) tilt=-tilt; pan=std::max(prevp-cfg.max_accel,std::min(prevp+cfg.max_accel,pan)); tilt=std::max(prevt-cfg.max_accel,std::min(prevt+cfg.max_accel,tilt)); prevp=pan; prevt=tilt; sent_stop=false; needSend=true;} else {ix=iy=pex=pey=0; prevp=prevt=0; if(!sent_stop){pan=0;tilt=0; sent_stop=true; needSend=true;} else {std::this_thread::sleep_for(std::chrono::milliseconds(10));}}
    {std::lock_guard<std::mutex> lk(g.m); g.last_tracker_mode=ok?ts.mode:"UNREACHABLE"; g.last_track_id=ts.track_id; g.errx=ex; g.erry=ey; g.cmdp=pan; g.cmdt=tilt; g.mode=(active?"ACTIVE":"IDLE"); if(!ok) g.last_error="tracker_unreachable"; else g.last_error.clear();}
    if(needSend){ if(!ws.connect_ws(cfg.bridge_host,cfg.bridge_port)) {std::lock_guard<std::mutex> lk(g.m); g.last_error="bridge_unreachable";} else {int seq=g.seq.fetch_add(1); std::ostringstream cmd; cmd<<"J "<<seq<<" "<<pan<<" "<<tilt; if(!ws.send_text(cmd.str())) ws.close_ws(); }}
    auto elapsed=std::chrono::duration<double>(std::chrono::steady_clock::now()-start).count(); double sleep_s=std::max(0.001,(1.0/cfg.hz)-elapsed); std::this_thread::sleep_for(std::chrono::duration<double>(sleep_s)); }
}

static std::string state_json(){std::lock_guard<std::mutex>lk(g.m); std::ostringstream os; os<<"{\"ok\":true,\"enabled\":"<<(g.enabled?"true":"false")<<",\"mode\":\""<<g.mode<<"\",\"last_tracker_mode\":\""<<g.last_tracker_mode<<"\",\"last_track_id\":"<<g.last_track_id<<",\"err_x\":"<<g.errx<<",\"err_y\":"<<g.erry<<",\"cmd_pan\":"<<g.cmdp<<",\"cmd_tilt\":"<<g.cmdt<<",\"invert_pan\":"<<(cfg.invert_pan?"true":"false")<<",\"invert_tilt\":"<<(cfg.invert_tilt?"true":"false")<<",\"target_x\":"<<cfg.target_x<<",\"target_y\":"<<cfg.target_y<<",\"min_pan\":"<<cfg.min_pan<<",\"min_tilt\":"<<cfg.min_tilt<<",\"seq\":"<<g.seq.load()<<",\"last_error\":\""<<g.last_error<<"\"}"; return os.str();}

static void control_server(){int s=socket(AF_INET,SOCK_STREAM,0); int on=1; setsockopt(s,SOL_SOCKET,SO_REUSEADDR,&on,sizeof(on)); sockaddr_in a{}; a.sin_family=AF_INET;a.sin_port=htons(cfg.control_port); a.sin_addr.s_addr=htonl(INADDR_ANY); bind(s,(sockaddr*)&a,sizeof(a)); listen(s,16); while(g.run){int c=accept(s,nullptr,nullptr); if(c<0) continue; std::string req; char b[4096]; int n=recv(c,b,sizeof(b),0); if(n>0) req.assign(b,n); std::string method=req.substr(0,req.find(' ')); auto p2=req.find(' ',req.find(' ')+1); std::string path=req.substr(req.find(' ')+1,p2-req.find(' ')-1); auto bp=req.find("\r\n\r\n"); std::string body=(bp==std::string::npos)?"":req.substr(bp+4); std::string out="{\"ok\":false}"; int status=200; std::string status_text="OK";
 if(method=="OPTIONS"){status=204; status_text="No Content"; out.clear();}
 else if(method=="GET" && path=="/api/autopilot/state") out=state_json();
 else if(method=="POST" && path=="/api/autopilot/start"){g.enabled=true; out=state_json();}
 else if(method=="POST" && path=="/api/autopilot/stop"){g.enabled=false; out=state_json();}
 else if(method=="POST" && path=="/api/autopilot/config"){json_num(body,"kp",cfg.kp);json_num(body,"ki",cfg.ki);json_num(body,"kd",cfg.kd);json_num(body,"deadzone",cfg.deadzone);json_int(body,"max_pan",cfg.max_pan);json_int(body,"max_tilt",cfg.max_tilt);json_int(body,"max_accel",cfg.max_accel);json_num(body,"hz",cfg.hz); bool invp=false; if(json_bool(body,"invert_pan",invp)) cfg.invert_pan=invp; bool invt=false; if(json_bool(body,"invert_tilt",invt)) cfg.invert_tilt=invt; json_num(body,"target_x",cfg.target_x); json_num(body,"target_y",cfg.target_y); json_int(body,"min_pan",cfg.min_pan); json_int(body,"min_tilt",cfg.min_tilt); clamp_config(); save_ptz_config(); out=state_json();}
 std::string hdr="HTTP/1.1 "+std::to_string(status)+" "+status_text+"\r\nContent-Type: application/json\r\nAccess-Control-Allow-Origin: *\r\nAccess-Control-Allow-Methods: GET, POST, OPTIONS\r\nAccess-Control-Allow-Headers: Content-Type\r\nContent-Length: "+std::to_string(out.size())+"\r\nConnection: close\r\n\r\n"; send(c,hdr.data(),hdr.size(),0); if(!out.empty()) send(c,out.data(),out.size(),0); close(c);} close(s);} 
static void on_sig(int){g.enabled=false; g.run=false;}
int main(int argc,char**argv){for(int i=1;i<argc;i++){std::string a=argv[i]; auto next=[&](auto &v){if(i+1<argc){std::istringstream ss(argv[++i]); ss>>v;}}; if(a=="--mjpeg-url") next(cfg.mjpeg_url); else if(a=="--bridge-host") next(cfg.bridge_host); else if(a=="--bridge-port") next(cfg.bridge_port); else if(a=="--control-port") next(cfg.control_port); else if(a=="--width") next(cfg.width); else if(a=="--height") next(cfg.height); else if(a=="--hz") next(cfg.hz); else if(a=="--kp") next(cfg.kp); else if(a=="--ki") next(cfg.ki); else if(a=="--kd") next(cfg.kd); else if(a=="--deadzone") next(cfg.deadzone); else if(a=="--max-pan") next(cfg.max_pan); else if(a=="--max-tilt") next(cfg.max_tilt); else if(a=="--max-accel") next(cfg.max_accel); else if(a=="--invert-pan") {int v=0; next(v); cfg.invert_pan=(v!=0);} else if(a=="--invert-tilt") {int v=1; next(v); cfg.invert_tilt=(v!=0);} else if(a=="--target-x") next(cfg.target_x); else if(a=="--target-y") next(cfg.target_y); else if(a=="--min-pan") next(cfg.min_pan); else if(a=="--min-tilt") next(cfg.min_tilt); else if(a=="--enable"){int v=0; next(v); cfg.enabled=(v!=0);} }
 auto p=cfg.mjpeg_url.find("://"); auto hostport=(p==std::string::npos)?cfg.mjpeg_url:cfg.mjpeg_url.substr(p+3); auto slash=hostport.find('/'); if(slash!=std::string::npos) hostport=hostport.substr(0,slash); auto colon=hostport.find(':'); if(colon!=std::string::npos){cfg.tracker_host=hostport.substr(0,colon); cfg.tracker_port=std::stoi(hostport.substr(colon+1));}
 clamp_config();
 load_ptz_config();
 clamp_config();
 g.enabled=cfg.enabled; std::signal(SIGINT,on_sig); std::signal(SIGTERM,on_sig); std::thread t1(autopilot_loop), t2(control_server); t1.join(); t2.join(); return 0;}
