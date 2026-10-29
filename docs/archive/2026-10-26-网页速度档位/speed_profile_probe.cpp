// APP-001-R15实机静止探针：只切档和读状态，绝不提交定位、目标、舵机或速度。
#include <boost/asio.hpp>
#include <boost/beast.hpp>
#include <boost/beast/websocket.hpp>
#include <nlohmann/json.hpp>
#include <chrono>
#include <iostream>
#include <stdexcept>
#include <thread>
namespace asio=boost::asio;namespace beast=boost::beast;
namespace http=beast::http;namespace ws=beast::websocket;
using tcp=asio::ip::tcp;using json=nlohmann::json;
int main() {
  try {
    asio::io_context io;
    auto request=[&](const std::string & target,const json & body=json(nullptr)) {
      tcp::socket connection(io);connection.connect({asio::ip::make_address("127.0.0.1"),8080});
      http::request<http::string_body> req(body.is_null()?http::verb::get:http::verb::post,"/inspect"+target,11);
      req.set(http::field::host,"127.0.0.1:8080");
      if(!body.is_null()){req.set("X-Mapping-Console","1");req.body()=body.dump();req.prepare_payload();}
      http::write(connection,req);beast::flat_buffer buffer;http::response<http::string_body> result;
      http::read(connection,buffer,result);
      if(result.result_int()!=200)throw std::runtime_error(result.body());
      return json::parse(result.body());
    };
    auto check=[](const json & value) {
      if(value["navigation_trial"]["active"]!=false||value["health"]["still"]!=true||
        value["motion"]["command_linear_mps"]!=0||value["navigation_trial"]["phase"]!="IDLE")
        throw std::runtime_error("实机不满足静止、无目标条件，停止切档探针");
    };
    check(request("/api/state"));
    ws::stream<tcp::socket> socket(io);socket.next_layer().connect({asio::ip::make_address("127.0.0.1"),8080});
    socket.handshake("127.0.0.1:8080","/inspect/ws/control");
    beast::flat_buffer buffer;socket.read(buffer);
    const auto session=json::parse(beast::buffers_to_string(buffer.data())).at("session");
    for(const double speed:{.10,.18,.25,.30,.18}) {
      check(request("/api/state"));
      request("/api/navigation/speed",{{"session",session},{"max_linear_mps",speed}});
      bool confirmed=false;const auto until=std::chrono::steady_clock::now()+std::chrono::seconds(4);
      while(std::chrono::steady_clock::now()<until) {
        const auto value=request("/api/state");check(value);
        if(value["navigation_speed"]["phase"]=="FAILED")throw std::runtime_error(value["navigation_speed"]["message"]);
        if(value["navigation_speed"]["ready"]==true&&value["navigation_speed"]["applied_mps"]==speed){confirmed=true;break;}
        std::this_thread::sleep_for(std::chrono::milliseconds(40));
      }
      if(!confirmed)throw std::runtime_error("切档读回等待超时");
      std::cout<<"PASS："<<speed<<"m/s控制器读回一致；目标IDLE、下发0、软件停稳\n";
    }
    socket.close(ws::close_code::normal);
  }catch(const std::exception & error){std::cerr<<error.what()<<'\n';return 1;}
}
