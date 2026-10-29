// APP-001-R10：只验证静止入口的真实HTTP/WS请求，不发送速度、初始位姿或舵机命令。
// 运行前必须由操作手册35.8启动速度入口隔离且禁用导航的静止组合。
#include <boost/asio.hpp>
#include <boost/beast.hpp>
#include <boost/beast/websocket.hpp>
#include <nlohmann/json.hpp>
#include <chrono>
#include <fstream>
#include <iostream>
#include <poll.h>
#include <stdexcept>
#include <thread>
namespace asio=boost::asio;namespace beast=boost::beast;namespace http=beast::http;
using tcp=asio::ip::tcp;using json=nlohmann::json;
void check(bool passed,const std::string & reason){if(!passed)throw std::runtime_error(reason);}
double seconds(){return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();}
int main(int argc,char ** argv){try{
  check(argc==3&&std::string(argv[1])=="--live-static","仅允许显式 --live-static 地图ID");
  asio::io_context io;
  auto request=[&](const std::string & target,const json & body=json()){
    tcp::socket socket(io);socket.connect({asio::ip::make_address("127.0.0.1"),8080});
    timeval limit{};limit.tv_sec=2;
    ::setsockopt(socket.native_handle(),SOL_SOCKET,SO_RCVTIMEO,&limit,sizeof(limit));
    ::setsockopt(socket.native_handle(),SOL_SOCKET,SO_SNDTIMEO,&limit,sizeof(limit));
    http::request<http::string_body> req(body.is_null()?http::verb::get:http::verb::post,"/inspect"+target,11);
    req.set(http::field::host,"127.0.0.1:8080");
    if(!body.is_null()){req.set("X-Mapping-Console","1");req.set(http::field::content_type,"application/json");req.body()=body.dump();req.prepare_payload();}
    http::write(socket,req);beast::flat_buffer buffer;http::response<http::string_body> response;
    http::read(socket,buffer,response);return response;
  };
  auto state=[&]{const auto res=request("/api/state");check(res.result_int()==200,"网页状态读取失败");return json::parse(res.body());};
  const auto before=state();
  check(before.at("hardware_enabled")==true&&before.at("navigation_enabled")==false&&before.at("goal_navigation_enabled")==false,
    "实机必须处于硬件启用、单点与整轮导航均关闭的静止模式");
  check(before.at("navigation_trial").at("phase")=="IDLE","已有导航试验，不接管会话");
  beast::websocket::stream<tcp::socket> ws(io);ws.next_layer().connect({asio::ip::make_address("127.0.0.1"),8080});
  ws.handshake("127.0.0.1:8080","/inspect/ws/control");beast::flat_buffer welcome;ws.read(welcome);
  const auto session=json::parse(beast::buffers_to_string(welcome.data())).at("session");
  auto drain=[&]{pollfd fd{ws.next_layer().native_handle(),POLLIN,0};while(poll(&fd,1,0)>0){beast::flat_buffer message;ws.read(message);}};
  const auto selected=request("/api/map/select",{{"session",session},{"map_id",argv[2]}});
  check(selected.result_int()==200,"载入地图失败："+selected.body());
  // 该目标取自昨晚同一342×299保存图的空闲栅格；导航禁用会先拒绝，不进行实车行驶。
  const json target{{"x",-4.99},{"y",-.12},{"yaw",1.733}};
  double maximum=0;const auto started=seconds();
  for(int i=0;i<20;++i){drain();const auto current=state();
    check(current.at("navigation_enabled")==false&&current.at("goal_navigation_enabled")==false,"静止开关发生变化，立即停止探针");
    const auto at=seconds();const auto res=request("/api/teach/navigate",{{"session",session},
      {"request_id","static_loop_20261022_"+std::to_string(i)},{"pose",target}});
    const auto elapsed=seconds()-at;maximum=std::max(maximum,elapsed);
    const auto result=json::parse(res.body());
    std::cout<<json{{"sample",i+1},{"target","/api/teach/navigate"},{"status",res.result_int()},
      {"elapsed_ms",elapsed*1000},{"reason",result.value("error","")}}.dump(-1,' ',false)<<'\n';
    check(res.result_int()==409&&result.value("error","")=="当前是静止测试，尚未启用单点导航","未得到预期的静止拒绝："+res.body());
    check(elapsed<1,"请求超过1秒，仍存在处理阻塞");
    std::this_thread::sleep_for(std::chrono::milliseconds(150));
  }
  const auto after=state();const auto health=after.at("health");
  for(const auto * name:{"odom","scan","rgb","still","hardware_interface"})check(health.at(name)==true,std::string("真实数据未就绪：")+name);
  check(after.at("navigation_trial").at("phase")=="IDLE","静止拒绝改变了导航阶段");
  std::cout<<json{{"result","PASS"},{"requests",20},{"maximum_ms",maximum*1000},{"duration_s",seconds()-started},
    {"health",health},{"pose",after.at("pose")},{"navigation_trial",after.at("navigation_trial")}}.dump(-1,' ',false)<<'\n';
  ws.next_layer().close();return 0;
}catch(const std::exception & e){std::cerr<<"APP-001-R10 静止闭环失败："<<e.what()<<'\n';return 1;}}
