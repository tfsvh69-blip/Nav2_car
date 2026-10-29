#include <gtest/gtest.h>
#include "carcar_mapping_console/http_server.hpp"
#include <boost/asio.hpp>
#include <boost/beast.hpp>
#include <boost/beast/websocket.hpp>
#include <algorithm>
#include <atomic>
#include <thread>
#include <mutex>
#include <poll.h>
#include "carcar_mapping_console/rtc_control.hpp"
#include "carcar_mapping_console/safety.hpp"
#ifdef MAPPING_ENABLE_WEBRTC
#include <rtc/rtc.hpp>
#endif
namespace asio=boost::asio;namespace beast=boost::beast;namespace http=beast::http;namespace websocket=beast::websocket;using tcp=asio::ip::tcp;
using namespace carcar_mapping_console;
TEST(HttpTransport, WebRtcCapabilityRetainsExistingControllerSession) {
  asio::io_context io;HttpServer server("127.0.0.1",18118,[](const HttpRequest&){return HttpResponse{};},
    [](const std::string&,const std::string&){});
  websocket::stream<tcp::socket> ws(io);ws.next_layer().connect({asio::ip::make_address("127.0.0.1"),18118});
  ws.handshake("127.0.0.1:18118","/ws/control");beast::flat_buffer buffer;ws.read(buffer);
  const auto welcome=nlohmann::json::parse(beast::buffers_to_string(buffer.data()));
  EXPECT_FALSE(welcome.value("session",std::string()).empty());
  EXPECT_EQ(welcome.value("rtc_protocol",0),RtcControl::available()?1:0);
  ws.next_layer().close();
}
#ifdef MAPPING_ENABLE_WEBRTC
namespace {
using json=nlohmann::json;
struct RtcProbe {
  std::mutex mutex;std::vector<json> outgoing,received;std::atomic<bool> open{false};
};
struct NativeRtc {
  asio::io_context io;websocket::stream<tcp::socket> ws{io};beast::flat_buffer input;
  std::shared_ptr<RtcProbe> probe=std::make_shared<RtcProbe>();
  std::shared_ptr<rtc::PeerConnection> peer;std::shared_ptr<rtc::DataChannel> channel;
  std::string session;
  explicit NativeRtc(unsigned short port,bool reliable=false) {
    ws.next_layer().connect({asio::ip::make_address("127.0.0.1"),port});ws.handshake("127.0.0.1:"+std::to_string(port),"/ws/control");
    ws.read(input);session=json::parse(beast::buffers_to_string(input.data())).at("session");input.consume(input.size());
    rtc::Configuration config;config.bindAddress="127.0.0.1";config.maxMessageSize=2048;
    peer=std::make_shared<rtc::PeerConnection>(config);const auto p=probe;
    peer->onLocalDescription([p](rtc::Description d){std::lock_guard<std::mutex> lock(p->mutex);
      p->outgoing.push_back({{"action","rtc_offer"},{"sdp",std::string(d)}});});
    peer->onLocalCandidate([p](rtc::Candidate c){std::lock_guard<std::mutex> lock(p->mutex);
      p->outgoing.push_back({{"action","rtc_candidate"},{"candidate",c.candidate()},{"mid",c.mid()}});});
    rtc::DataChannelInit init;if(!reliable){init.reliability.unordered=true;init.reliability.maxRetransmits=0;}
    channel=peer->createDataChannel("mapping-control",init);
    channel->onOpen([p]{p->open=true;});
    channel->onMessage([p](rtc::message_variant data){if(!std::holds_alternative<std::string>(data))return;
      auto value=json::parse(std::get<std::string>(data),nullptr,false);
      if(value.is_object()){std::lock_guard<std::mutex> lock(p->mutex);p->received.push_back(std::move(value));}});
  }
  ~NativeRtc(){channel->resetCallbacks();peer->resetCallbacks();channel->close();peer->close();beast::error_code ec;ws.next_layer().close(ec);}
  void pump() {
    std::vector<json> pending;{std::lock_guard<std::mutex> lock(probe->mutex);pending.swap(probe->outgoing);}
    for(const auto & value:pending){const auto text=value.dump();ws.write(asio::buffer(text));}
    pollfd fd{ws.next_layer().native_handle(),POLLIN,0};
    if(poll(&fd,1,10)>0){ws.read(input);const auto value=json::parse(beast::buffers_to_string(input.data()));input.consume(input.size());
      if(value.contains("rtc_description"))peer->setRemoteDescription(rtc::Description(value.at("sdp").get<std::string>(),value.at("type").get<std::string>()));
      else if(value.contains("rtc_candidate"))peer->addRemoteCandidate(rtc::Candidate(value.at("candidate").get<std::string>(),value.at("mid").get<std::string>()));}
  }
  bool wait(const std::function<bool()> & predicate,double seconds=3) {
    const auto until=std::chrono::steady_clock::now()+std::chrono::duration<double>(seconds);
    while(std::chrono::steady_clock::now()<until){if(predicate())return true;pump();}return predicate();
  }
  void tcp_send(const json & value){const auto text=value.dump();ws.write(asio::buffer(text));}
};
}
TEST(HttpTransport, RealUdpChannelSharesOwnerAndDropsOldTcpMotionAfterSwitch) {
  std::mutex mutex;std::vector<json> requests;std::string owner;
  HttpServer server("127.0.0.1",18119,[](const HttpRequest&){return HttpResponse{};},[&](const std::string & id,const std::string & body){
    std::lock_guard<std::mutex> lock(mutex);owner=id;if(!body.empty())requests.push_back(json::parse(body));});
  NativeRtc client(18119);
  ASSERT_TRUE(client.wait([&]{return server.diagnostics().value("control_transport",std::string())=="webrtc_udp";}));
  ASSERT_TRUE(client.wait([&]{return client.probe->open.load();}));EXPECT_EQ(owner,client.session);
  const json request{{"action","drive"},{"sequence",1},{"challenge","fresh"},{"x",1},{"yaw",0}};
  client.channel->send(request.dump());
  ASSERT_TRUE(client.wait([&]{std::lock_guard<std::mutex> lock(mutex);return std::any_of(requests.begin(),requests.end(),[](const json & v){return v.value("action","")=="drive";});}));
  client.tcp_send({{"action","drive"},{"sequence",2},{"challenge","old_tcp"},{"x",1},{"yaw",0}});
  client.tcp_send({{"action","drive"},{"sequence",3},{"challenge","fresh_backup"},{"backup",true},{"x",1},{"yaw",0}});
  client.tcp_send({{"action","stop"},{"cause","test_marker"}});
  ASSERT_TRUE(client.wait([&]{std::lock_guard<std::mutex> lock(mutex);return std::any_of(requests.begin(),requests.end(),[](const json & v){return v.value("cause","")=="test_marker";});}));
  {std::lock_guard<std::mutex> lock(mutex);
    EXPECT_FALSE(std::any_of(requests.begin(),requests.end(),[](const json & v){return v.value("challenge","")=="old_tcp";}));
    EXPECT_TRUE(std::any_of(requests.begin(),requests.end(),[](const json & v){return v.value("challenge","")=="fresh_backup"&&v.value("backup",false);}));}
  const auto now=std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
  server.broadcast(json{{"challenge","state"},{"control_revision",2},{"issued_steady_ms",now*1000},{"armed",false}}.dump());
  ASSERT_TRUE(client.wait([&]{std::lock_guard<std::mutex> lock(client.probe->mutex);return !client.probe->received.empty();}));
  // RTC详细快照每100ms采一次；等待采样，不把即时路由变化和上一次快照混为一时刻。
  ASSERT_TRUE(client.wait([&]{return server.diagnostics().value("rtc",json::object()).value("active",false);}));
  const auto stops_before_close=[&]{std::lock_guard<std::mutex> lock(mutex);return std::count_if(requests.begin(),requests.end(),[](const json & v){return v.value("action","")=="stop";});}();
  client.channel->close();ASSERT_TRUE(client.wait([&]{return server.diagnostics().value("control_transport",std::string())=="websocket";}));
  {std::lock_guard<std::mutex> lock(mutex);EXPECT_EQ(std::count_if(requests.begin(),requests.end(),[](const json & v){return v.value("action","")=="stop";}),stops_before_close);}
  client.ws.next_layer().close();
  const auto until=std::chrono::steady_clock::now()+std::chrono::seconds(1);
  while(server.diagnostics().value("connected",false)&&std::chrono::steady_clock::now()<until)
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  const auto closed=server.diagnostics();EXPECT_FALSE(closed.value("connected",true));
  EXPECT_EQ(closed.value("control_transport",std::string()),"disconnected");
  EXPECT_FALSE(closed.value("rtc",json::object()).value("active",true));
  EXPECT_EQ(closed["last_disconnect"]["stage"],"read");
  EXPECT_FALSE(closed["last_disconnect"].value("error_message",std::string()).empty());
}
TEST(HttpTransport, ReliableOrderedChannelCannotSilentlyReplaceRealtimeProfile) {
  HttpServer server("127.0.0.1",18120,[](const HttpRequest&){return HttpResponse{};},[](const std::string&,const std::string&){});
  NativeRtc client(18120,true);
  EXPECT_FALSE(client.wait([&]{return server.diagnostics().value("control_transport",std::string())=="webrtc_udp";},.5));
  EXPECT_FALSE(server.diagnostics().value("rtc",json::object()).value("active",false));
}
TEST(HttpTransport, BriefTcpSilenceDoesNotTearDownHealthyUdpController) {
  std::atomic<int> disconnected{0},received{0};
  HttpServer server("127.0.0.1",18121,[](const HttpRequest&){return HttpResponse{};},
    [&](const std::string&,const std::string & body){if(body.empty())++disconnected;
      else if(json::parse(body).value("action","")=="heartbeat")++received;});
  NativeRtc client(18121);
  ASSERT_TRUE(client.wait([&]{return server.diagnostics().value("rtc",json::object()).value("active",false);}));
  // 只暂停TCP读取/自动pong，真实UDP仍双向流动；模拟信令TCP短暂积压。
  // 不读TCP，避免测试框架替客户端回复ping、掩盖2秒超时问题。
  const auto until=std::chrono::steady_clock::now()+std::chrono::milliseconds(3400);
  uint64_t revision=0;
  while(std::chrono::steady_clock::now()<until){
    if(client.channel->isOpen())client.channel->send(json{{"action","heartbeat"},{"sequence",++revision}}.dump());
    const auto now=std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
    server.broadcast(json{{"armed",false},{"control_revision",revision},{"issued_steady_ms",now*1000}}.dump());
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }
  EXPECT_EQ(disconnected.load(),0)<<"健康UDP被短暂TCP空闲超时一并关闭";
  EXPECT_TRUE(server.diagnostics().value("connected",false));
  EXPECT_GT(received.load(),50);
  std::lock_guard<std::mutex> lock(client.probe->mutex);EXPECT_GT(client.probe->received.size(),50u);
}
#endif
TEST(HttpTransport, SlowHttpHandlerDoesNotDelayStop) {
  asio::io_context io; std::atomic<bool> entered{false}, stopped{false};
  HttpServer server("127.0.0.1",18115,[&](const HttpRequest&){
    entered=true;std::this_thread::sleep_for(std::chrono::milliseconds(400));return HttpResponse{};
  },[&](const std::string&,const std::string&body){if(body.find("stop")!=std::string::npos)stopped=true;});
  websocket::stream<tcp::socket> ws(io);ws.next_layer().connect({asio::ip::make_address("127.0.0.1"),18115});
  ws.handshake("127.0.0.1:18115","/ws/control");
  tcp::socket slow(io);slow.connect({asio::ip::make_address("127.0.0.1"),18115});
  const std::string request="GET /slow HTTP/1.1\r\nHost: 127.0.0.1:18115\r\n\r\n";
  asio::write(slow,asio::buffer(request));
  auto until=std::chrono::steady_clock::now()+std::chrono::seconds(1);
  while(!entered&&std::chrono::steady_clock::now()<until)std::this_thread::sleep_for(std::chrono::milliseconds(1));
  ASSERT_TRUE(entered);const std::string stop="{\"action\":\"stop\"}";ws.write(asio::buffer(stop));
  until=std::chrono::steady_clock::now()+std::chrono::milliseconds(150);
  while(!stopped&&std::chrono::steady_clock::now()<until)std::this_thread::sleep_for(std::chrono::milliseconds(1));
  EXPECT_TRUE(stopped)<<"同步HTTP处理阻塞了停车请求";ws.next_layer().close();slow.close();
}
TEST(HttpTransport, BrowserCanReuseHttpConnection) {
  asio::io_context io;HttpServer server("127.0.0.1",18116,
    [](const HttpRequest&){return HttpResponse{200,"text/plain",{'o','k'},{}};},[](const std::string&,const std::string&){});
  beast::tcp_stream socket(io);socket.connect({asio::ip::make_address("127.0.0.1"),18116});beast::flat_buffer buffer;
  for(int i=0;i<3;++i){http::request<http::empty_body> request(http::verb::get,"/status",11);request.set(http::field::host,"127.0.0.1:18116");
    http::write(socket,request);http::response<http::string_body> response;http::read(socket,buffer,response);
    ASSERT_TRUE(response.keep_alive())<<"每次状态查询仍强制重新建立TCP连接";EXPECT_EQ(response.body(),"ok");}
}
TEST(HttpTransport, CompactControlKeepsStopAndDropsExpiredPendingState) {
  asio::io_context io;HttpServer server("127.0.0.1",18117,[](const HttpRequest&){return HttpResponse{};},
    [](const std::string&,const std::string&){});
  websocket::stream<tcp::socket> ws(io);ws.next_layer().connect({asio::ip::make_address("127.0.0.1"),18117});
  ws.handshake("127.0.0.1:18117","/ws/control");beast::flat_buffer buffer;ws.read(buffer);buffer.consume(buffer.size());
  const std::string profile="{\"action\":\"transport\",\"profile\":\"compact\"}";ws.write(asio::buffer(profile));
  auto until=std::chrono::steady_clock::now()+std::chrono::seconds(1);
  while(!server.diagnostics().value("compact_profile",false)&&std::chrono::steady_clock::now()<until)
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  ASSERT_TRUE(server.diagnostics().value("compact_profile",false));EXPECT_TRUE(server.diagnostics().value("tcp_nodelay",false));
  const auto now=std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
  nlohmann::json state{{"challenge","expired"},{"issued_steady_ms",(now-.4)*1000},{"control_lease_ms",300},
    {"armed",false},{"reason","操纵授权超时"},{"timing",{{"large",std::string(5000,'x')}}},
    {"last_stop",{{"server_steady",now},{"reason","操纵授权超时"},{"timing",{{"detail",1}}}}}};
  server.broadcast(state.dump());until=std::chrono::steady_clock::now()+std::chrono::seconds(1);
  while(server.diagnostics().value("states_expired_before_write",0)==0&&std::chrono::steady_clock::now()<until)
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  ASSERT_EQ(server.diagnostics().value("states_expired_before_write",0),1);
  state["challenge"]="fresh";state["issued_steady_ms"]=now*1000;server.broadcast(state.dump());ws.read(buffer);
  const auto received=nlohmann::json::parse(beast::buffers_to_string(buffer.data()));
  EXPECT_EQ(received["challenge"],"fresh");
  EXPECT_EQ(received["issued_steady_ms"],state["issued_steady_ms"])<<"过期后补发的状态被改写了签发时间";
  EXPECT_FALSE(received.contains("timing"));
  EXPECT_EQ(received["last_stop"]["reason"],"操纵授权超时");EXPECT_FALSE(received["last_stop"].contains("timing"));
  EXPECT_LT(received.dump().size(),500u);ws.next_layer().close();
}
TEST(HttpTransport, SlowMapReadersDoNotBlockControlChannel) {
  std::atomic<int> stopped{0};asio::io_context io;
  HttpServer server("127.0.0.1",18111,[](const HttpRequest&){return HttpResponse{200,"application/octet-stream",std::vector<unsigned char>(5*1024*1024,42),{}};},
    [&](const std::string&,const std::string&body){if(body.find("stop")!=std::string::npos)++stopped;});
  std::vector<std::unique_ptr<tcp::socket>> slow;
  for(int i=0;i<3;++i){auto socket=std::make_unique<tcp::socket>(io);socket->connect(tcp::endpoint(asio::ip::make_address("127.0.0.1"),18111));
    const std::string req="GET /api/map.png HTTP/1.1\r\nHost: 127.0.0.1:18111\r\n\r\n";asio::write(*socket,asio::buffer(req));slow.push_back(std::move(socket));}
  websocket::stream<tcp::socket> ws(io);ws.next_layer().connect(tcp::endpoint(asio::ip::make_address("127.0.0.1"),18111));ws.handshake("127.0.0.1:18111","/ws/control");
  const std::string stop="{\"action\":\"stop\"}";ws.write(asio::buffer(stop));
  const auto deadline=std::chrono::steady_clock::now()+std::chrono::milliseconds(500);
  while(stopped.load()==0&&std::chrono::steady_clock::now()<deadline)std::this_thread::sleep_for(std::chrono::milliseconds(5));
  EXPECT_EQ(stopped.load(),1);
  ws.next_layer().close();for(auto&socket:slow)socket->close();
}
TEST(HttpTransport, SecondControllerCannotTakeOver) {
  asio::io_context io;HttpServer server("127.0.0.1",18112,[](const HttpRequest&){return HttpResponse{};},[](const std::string&,const std::string&){});
  websocket::stream<tcp::socket> first(io);first.next_layer().connect(tcp::endpoint(asio::ip::make_address("127.0.0.1"),18112));first.handshake("127.0.0.1:18112","/ws/control");
  websocket::stream<tcp::socket> other(io);other.next_layer().connect(tcp::endpoint(asio::ip::make_address("127.0.0.1"),18112));
  EXPECT_THROW(other.handshake("127.0.0.1:18112","/ws/control"),boost::system::system_error);
  // 失败升级连接必须结束，不能当成普通HTTP keep-alive挂住客户端。
  other.next_layer().non_blocking(true);beast::error_code ec;char byte;
  const auto until=std::chrono::steady_clock::now()+std::chrono::milliseconds(500);
  do{other.next_layer().read_some(asio::buffer(&byte,1),ec);
    if(ec==asio::error::eof)break;
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }while(std::chrono::steady_clock::now()<until);
  EXPECT_EQ(ec,asio::error::eof);
  first.next_layer().close();
}
TEST(HttpTransport, InspectPrefixReachesUpstreamAndMissingUpstreamIsVisible) {
  asio::io_context io;
  HttpServer upstream("127.0.0.1",18119,[](const HttpRequest& request){
    const auto body=request.target=="/api/state"?std::string("{\"ok\":true}"):std::string("no");
    return HttpResponse{200,"application/json",{body.begin(),body.end()},
      {{"X-Navigation-Layer","{\"source_frame\":\"odom\",\"sequence\":17}"},{"X-Private-Upstream","not-forwarded"}}};
  },[](const std::string&,const std::string&){});
  HttpServer server("127.0.0.1",18118,[](const HttpRequest&){return HttpResponse{};},[](const std::string&,const std::string&){});
  server.set_inspect_upstream("127.0.0.1",18119);
  beast::tcp_stream socket(io);socket.connect({asio::ip::make_address("127.0.0.1"),18118});
  http::request<http::empty_body> request{http::verb::get,"/inspect/api/state",11};
  request.set(http::field::host,"127.0.0.1:18118");
  http::write(socket,request);
  beast::flat_buffer buffer;http::response<http::string_body> response;http::read(socket,buffer,response);
  EXPECT_EQ(response.result_int(),200);
  EXPECT_NE(response.body().find("ok"),std::string::npos);
  EXPECT_EQ(response["X-Navigation-Layer"],"{\"source_frame\":\"odom\",\"sequence\":17}");
  EXPECT_EQ(response.find("X-Private-Upstream"),response.end());
  socket.close();
  server.set_inspect_upstream("127.0.0.1",1);
  beast::tcp_stream missing(io);missing.connect({asio::ip::make_address("127.0.0.1"),18118});
  http::write(missing,request);buffer.consume(buffer.size());http::read(missing,buffer,response);
  EXPECT_EQ(response.result_int(),502);
  EXPECT_NE(response.body().find("巡检页面未启动"),std::string::npos);
}
TEST(HttpTransport, CrossOriginWriteIsDenied) {
  asio::io_context io;HttpServer server("127.0.0.1",18113,[](const HttpRequest&){return HttpResponse{};},[](const std::string&,const std::string&){});
  beast::tcp_stream socket(io);socket.connect(tcp::endpoint(asio::ip::make_address("127.0.0.1"),18113));
  http::request<http::string_body> request(http::verb::post,"/api/save",11);request.set(http::field::host,"127.0.0.1:18113");request.set(http::field::origin,"http://unrelated.example");request.set("X-Mapping-Console","1");request.body()="{}";request.prepare_payload();http::write(socket,request);
  beast::flat_buffer buffer;http::response<http::string_body> response;http::read(socket,buffer,response);EXPECT_EQ(response.result_int(),403);
}
TEST(HttpTransport, SilentOldControllerReleasesOwnershipForReconnect) {
  asio::io_context io;std::atomic<int> disconnected{0};
  HttpServer server("127.0.0.1",18114,[](const HttpRequest&){return HttpResponse{};},[&](const std::string&,const std::string&body){if(body.empty())++disconnected;});
  websocket::stream<tcp::socket> old(io);old.next_layer().connect(tcp::endpoint(asio::ip::make_address("127.0.0.1"),18114));old.handshake("127.0.0.1:18114","/ws/control");
  // 故意不再读取，不回复服务端ping；模拟WiFi中断但尚未收到TCP关闭。
  const auto started=std::chrono::steady_clock::now();
  const auto deadline=started+std::chrono::seconds(12);
  while(disconnected.load()==0&&std::chrono::steady_clock::now()<deadline)std::this_thread::sleep_for(std::chrono::milliseconds(10));
  ASSERT_EQ(disconnected.load(),1);
  EXPECT_GE(std::chrono::duration<double>(std::chrono::steady_clock::now()-started).count(),9.5);
  EXPECT_EQ(server.diagnostics()["last_disconnect"]["reason"],"websocket_idle_timeout");
  EXPECT_EQ(server.diagnostics()["last_disconnect"]["stage"],"read");
  EXPECT_GE(server.diagnostics()["last_disconnect"].value("ws_last_rx_age_ms",0.0),9500);
  EXPECT_EQ(server.diagnostics()["last_disconnect"]["ws_pongs_received"],0);
  EXPECT_FALSE(server.diagnostics().value("rtc",nlohmann::json::object()).value("active",true));
  websocket::stream<tcp::socket> reconnect(io);reconnect.next_layer().connect(tcp::endpoint(asio::ip::make_address("127.0.0.1"),18114));
  EXPECT_NO_THROW(reconnect.handshake("127.0.0.1:18114","/ws/control"));reconnect.next_layer().close();old.next_layer().close();
}

TEST(HttpTransport, InspectPrefixOnSameServerStaysLocalForHttpAndWebSocket) {
  asio::io_context io;
  HttpServer server("127.0.0.1",18117,[](const HttpRequest& request){
    const std::string body=request.target=="/api/state"?"local-isolated-console":"wrong-path";
    return HttpResponse{200,"text/plain",{body.begin(),body.end()},{}};
  },[](const std::string&,const std::string&){});
  server.set_inspect_upstream("127.0.0.1",18117);
  beast::tcp_stream socket(io);socket.connect({asio::ip::make_address("127.0.0.1"),18117});
  http::request<http::empty_body> request{http::verb::get,"/inspect/api/state",11};
  request.set(http::field::host,"127.0.0.1:18117");http::write(socket,request);
  beast::flat_buffer buffer;http::response<http::string_body> response;http::read(socket,buffer,response);
  EXPECT_EQ(response.result_int(),200);EXPECT_EQ(response.body(),"local-isolated-console");
  websocket::stream<tcp::socket> control(io);control.next_layer().connect({asio::ip::make_address("127.0.0.1"),18117});
  EXPECT_NO_THROW(control.handshake("127.0.0.1:18117","/inspect/ws/control"));
  control.next_layer().close();
}
