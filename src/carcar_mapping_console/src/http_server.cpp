#include "carcar_mapping_console/http_server.hpp"
#include "carcar_mapping_console/common.hpp"
#include "carcar_mapping_console/data_flow.hpp"
#include "carcar_mapping_console/rtc_control.hpp"
#include <boost/asio.hpp>
#include <boost/beast.hpp>
#include <boost/beast/websocket.hpp>
#include <atomic>
#include <array>
#include <thread>
#include <optional>
#include <mutex>
#include <future>
#include <deque>
#include <netinet/tcp.h>
#include <sys/ioctl.h>
#include <linux/sockios.h>
namespace carcar_mapping_console {
namespace asio = boost::asio;
namespace beast = boost::beast;
namespace http = beast::http;
namespace websocket = beast::websocket;
using tcp = asio::ip::tcp;
struct HttpServer::Impl {
  struct Ws;
  struct Http;
  asio::io_context io{1}; tcp::acceptor acceptor;
  asio::executor_work_guard<asio::io_context::executor_type> work{asio::make_work_guard(io)};
  // 只有一个HTTP工作线程，保持原接口串行语义；磁盘/JSON工作不占控制事件循环。
  asio::thread_pool http_worker{1};
  Handler handler; Control control; std::thread thread;
  std::weak_ptr<Ws> controller;
  std::atomic<unsigned> clients{0};
  std::vector<std::weak_ptr<Http>> connections;
  std::string inspect_host = "127.0.0.1";
  unsigned short inspect_port = 8081;
  std::mutex broadcast_mutex;
  std::string broadcast_value;
  double broadcast_at = 0;
  bool broadcast_pending = false;
  bool stopped = false;
  mutable std::mutex stats_mutex;
  // 连接回收允许短暂重传；车辆运动期限由独立Safety的300ms签发截止执行。
  static constexpr auto session_idle_timeout=std::chrono::seconds(10);
  json stats{{"connected",false},{"tcp_nodelay",false},{"tcp_sample_valid",false},
    {"session_idle_timeout_ms",std::chrono::duration_cast<std::chrono::milliseconds>(session_idle_timeout).count()},{"disconnects",0},
    {"broadcast_replaced",0},{"pending_replaced",0},{"states_expired_before_write",0},
    {"writes",0},{"last_write_bytes",0},{"last_queue_wait_ms",0.0},{"max_queue_wait_ms",0.0},
    {"last_write_ms",0.0},{"max_write_ms",0.0},{"last_rx_gap_ms",0.0}};
  void count(const char * key) {std::lock_guard<std::mutex> lock(stats_mutex);stats[key]=stats.value(key,uint64_t{0})+1;}
  void duration(const char * last, const char * maximum, double ms) {
    std::lock_guard<std::mutex> lock(stats_mutex);stats[last]=ms;stats[maximum]=std::max(stats.value(maximum,0.0),ms);
  }
  struct Ws : std::enable_shared_from_this<Ws> {
    websocket::stream<beast::tcp_stream> stream;
    beast::flat_buffer input; Impl & owner; std::string id = nonce();
    struct State {std::string data;double queued_at;bool reliable=false;};
    std::string sending; std::optional<State> pending;
    std::deque<State> signaling;
    std::optional<State> rtc_deferred;
    bool flush_armed=false;
    asio::steady_timer rtc_flush;
    std::shared_ptr<RtcControl> rtc;
    bool rtc_active=false;uint64_t rtc_generation=0;double rtc_rx_at=0;
    std::optional<http::request<http::string_body>> handshake;
    asio::steady_timer stats_timer;
    bool closed = false, compact = false;double last_rx = 0, wire_rx = 0;
    uint64_t pings_received=0,pongs_received=0;
    explicit Ws(tcp::socket socket, Impl & impl) : stream(std::move(socket)), owner(impl), rtc_flush(stream.get_executor()), stats_timer(stream.get_executor()) {}
    ~Ws() {owner.clients--; if (!closed) owner.control(id, "");}
    void accept(http::request<http::string_body> request) {
      beast::error_code ec;auto & socket=beast::get_lowest_layer(stream).socket();
      socket.set_option(tcp::no_delay(true),ec);if(ec)return close("tcp_setup",ec);
      tcp::no_delay option;socket.get_option(option,ec);
      {std::lock_guard<std::mutex> lock(owner.stats_mutex);owner.stats["tcp_nodelay"]=!ec&&option.value();}
      // SDP信令可比操纵包大；进入门控的操纵消息仍限制为2048字节。
      stream.read_message_max(16384);
      auto timeout = websocket::stream_base::timeout::suggested(beast::role_type::server);
      timeout.handshake_timeout = std::chrono::seconds(3);
      timeout.idle_timeout = session_idle_timeout;
      timeout.keep_alive_pings = true;
      stream.set_option(timeout);
      stream.control_callback([weak=weak_from_this()](websocket::frame_type kind,beast::string_view){
        if(auto self=weak.lock()){
          self->wire_rx=steady();
          if(kind==websocket::frame_type::ping)++self->pings_received;
          if(kind==websocket::frame_type::pong)++self->pongs_received;
        }
      });
      handshake.emplace(std::move(request));
      stream.async_accept(*handshake, [self = shared_from_this()](beast::error_code ec) {
        if (ec||self->closed) return self->close("handshake",ec);
        self->handshake.reset();self->wire_rx=steady();
        {std::lock_guard<std::mutex> lock(self->owner.stats_mutex);self->owner.stats["connected"]=true;
          self->owner.stats["compact_profile"]=false;
          self->owner.stats["last_rx_gap_ms"]=0;self->owner.stats["control_last_rx_gap_ms"]=0;}
        self->owner.count("connection_generation");
        self->owner.control(self->id, "{\"action\":\"connect\"}");
        self->signal({{"session",self->id},{"transport_protocol",1},{"rtc_protocol",RtcControl::available()?1:0}});
        self->sample_tcp();self->read();
      });
    }
    void signal(json value){send({value.dump(),steady(),true});}
    void rtc_state(bool active,const std::string & why) {
      if(closed)return;
      // 接通或回退都不发停车。运动仍由门控按签发后300ms截止；这里停车会把使能打成来回跳动。
      rtc_active=active;
      if(!active)++rtc_generation;
      {std::lock_guard<std::mutex> lock(owner.stats_mutex);
        owner.stats["control_transport"]=active?"webrtc_udp":"websocket";}
      signal({{"rtc_status",active?"active":"fallback"},{"reason",why}});
      if(!active&&rtc){rtc->close();rtc.reset();}
    }
    void rtc_offer(const std::string & sdp) {
      if(!RtcControl::available())return;
      if(rtc){rtc->close();rtc.reset();}
      if(rtc_active)rtc_state(false,"rtc_restart");
      const auto generation=++rtc_generation;
      std::weak_ptr<Ws> weak=shared_from_this();
      rtc=std::make_shared<RtcControl>(
        [weak,generation](json value){if(auto self=weak.lock())asio::post(self->owner.io,[weak,generation,value=std::move(value)]()mutable{
          if(auto self=weak.lock())if(!self->closed&&self->rtc_generation==generation)self->signal(std::move(value));});},
        [weak,generation](std::string body){if(auto self=weak.lock())asio::post(self->owner.io,[weak,generation,body=std::move(body)]{
          if(auto self=weak.lock())if(!self->closed&&self->rtc_active&&self->rtc_generation==generation){
            const auto value=json::parse(body,nullptr,false);if(!value.is_object())return;
            if(!value.contains("action")||!value["action"].is_string())return;
            const auto action=value["action"].get<std::string>();
            if(action!="drive"&&action!="intent"&&action!="heartbeat"&&action!="release"&&action!="stop")return;
            const auto now=steady();
            {std::lock_guard<std::mutex> lock(self->owner.stats_mutex);
              self->owner.stats["control_last_rx_gap_ms"]=self->rtc_rx_at?(now-self->rtc_rx_at)*1000:0;}
            self->rtc_rx_at=now;self->owner.control(self->id,body);
          }});},
        [weak,generation](bool active,std::string why){if(auto self=weak.lock())asio::post(self->owner.io,[weak,generation,active,why=std::move(why)]{
          if(auto self=weak.lock())if(!self->closed&&self->rtc_generation==generation)self->rtc_state(active,why);});},
        beast::get_lowest_layer(stream).socket().local_endpoint().address().to_string());
      rtc->offer(sdp);
    }
    void sample_tcp() {
      if(closed)return;
      auto & socket=beast::get_lowest_layer(stream).socket();tcp_info info{};socklen_t size=sizeof(info);
      const bool valid=getsockopt(socket.native_handle(),IPPROTO_TCP,TCP_INFO,&info,&size)==0;
      int queued=-1,not_sent=-1;ioctl(socket.native_handle(),TIOCOUTQ,&queued);
#ifdef SIOCOUTQNSD
      ioctl(socket.native_handle(),SIOCOUTQNSD,&not_sent);
#endif
      const auto rtc_stats=rtc?rtc->diagnostics():json{{"available",RtcControl::available()},{"active",false}};
      {std::lock_guard<std::mutex> lock(owner.stats_mutex);owner.stats["tcp_sample_valid"]=valid;
        owner.stats["ws_last_rx_age_ms"]=wire_rx?(steady()-wire_rx)*1000:-1;
        owner.stats["ws_pings_received"]=pings_received;owner.stats["ws_pongs_received"]=pongs_received;
        owner.stats["rtc"]=rtc_stats;owner.stats["control_transport"]=rtc_active?"webrtc_udp":"websocket";
        owner.stats["tcp_send_queue_bytes"]=queued;owner.stats["tcp_notsent_bytes"]=not_sent;
        if(valid){owner.stats["tcp_rtt_ms"]=info.tcpi_rtt/1000.0;owner.stats["tcp_unacked"]=info.tcpi_unacked;
          owner.stats["tcp_retransmits"]=info.tcpi_total_retrans;}}
      stats_timer.expires_after(std::chrono::milliseconds(100));
      stats_timer.async_wait([self=shared_from_this()](beast::error_code ec){if(!ec)self->sample_tcp();});
    }
    void close(const std::string & stage="server_close",beast::error_code error={}) {
      if (closed) return;
      closed = true;
      auto rtc_stats=rtc?rtc->diagnostics():json{{"available",RtcControl::available()},{"active",false}};
      const auto was_rtc_active=rtc_active;rtc_active=false;++rtc_generation;
      if(rtc){rtc->close();rtc.reset();}
      if(auto current=owner.controller.lock();current.get()==this)owner.controller.reset();
      std::string reason=stage;
      if(error==beast::error::timeout)reason=stage=="handshake"?"websocket_handshake_timeout":"websocket_idle_timeout";
      else if(error==websocket::error::closed)reason="peer_close";
      else if(error)reason="transport_error";
      const auto peer_reason=stream.reason();
      rtc_stats["active"]=false;rtc_stats["buffered_bytes"]=0;rtc_stats["reason"]=reason;
      {std::lock_guard<std::mutex> lock(owner.stats_mutex);owner.stats["connected"]=false;
        owner.stats["control_transport"]="disconnected";owner.stats["rtc"]=std::move(rtc_stats);
        owner.stats["tcp_sample_valid"]=false;
        owner.stats["disconnects"]=owner.stats.value("disconnects",uint64_t{0})+1;
        owner.stats["last_disconnect"]={{"reason",reason},{"stage",stage},{"error_value",error.value()},
          {"error_category",error.category().name()},{"error_message",error.message()},
          {"peer_close_code",static_cast<unsigned>(peer_reason.code)},{"rtc_was_active",was_rtc_active},
          {"ws_last_rx_age_ms",wire_rx?(steady()-wire_rx)*1000:-1},
          {"ws_pings_received",pings_received},{"ws_pongs_received",pongs_received},
          {"server_steady",steady()},{"connection_generation",owner.stats.value("connection_generation",uint64_t{0})}};}
      // 先完成诊断快照，网关的断开事件才能保存本次首个错误，而不是上一次断开原因。
      owner.control(id, "");
      beast::error_code ec;stats_timer.cancel(ec);rtc_flush.cancel(ec);rtc_deferred.reset();beast::get_lowest_layer(stream).socket().close(ec);
    }
    void read() {
      stream.async_read(input, [self = shared_from_this()](beast::error_code ec, std::size_t) {
        if (ec) return self->close("read",ec);
        if (!self->stream.got_text()) return self->close("binary_frame");
        const auto now=steady();
        {std::lock_guard<std::mutex> lock(self->owner.stats_mutex);
          self->owner.stats["last_rx_gap_ms"]=self->last_rx==0?0:(now-self->last_rx)*1000;}
        self->last_rx=now;self->wire_rx=now;
        const auto body=beast::buffers_to_string(self->input.data());
        const auto value=json::parse(body,nullptr,false);
        if(value.is_object()&&value.contains("action")&&value["action"]=="rtc_offer") {
          try{self->rtc_offer(value.at("sdp").get<std::string>());}catch(const std::exception&){self->rtc_state(false,"rtc_negotiation_failed");}
        }
        else if(value.is_object()&&value.contains("action")&&value["action"]=="rtc_candidate") {
          try{if(self->rtc)self->rtc->candidate(value.at("candidate").get<std::string>(),value.value("mid",std::string()));}
          catch(const std::exception&){self->rtc_state(false,"rtc_candidate_failed");}
        }
        else if(value.is_object()&&value.contains("action")&&value["action"]=="rtc_cancel")self->rtc_state(false,"rtc_cancelled");
        else if(value.is_object()&&value.contains("action")&&value["action"]=="transport"&&
           value.contains("profile")&&value["profile"]=="compact") {
          self->compact=true;
          std::lock_guard<std::mutex> lock(self->owner.stats_mutex);self->owner.stats["compact_profile"]=true;
        }
        // 没有backup标记的旧TCP运动/心跳仍丢弃，避免切换前排队的旧动作晚到。
        // 同一条新指令的可靠副本带backup，UDP丢包时由它在原300ms内续租。
        else if(!(self->rtc_active&&value.is_object()&&value.contains("action")&&
          (value["action"]=="drive"||value["action"]=="intent"||value["action"]=="heartbeat")&&
          !value.value("backup",false)))
          self->owner.control(self->id,body);
        self->input.consume(self->input.size()); self->read();
      });
    }
    bool control_expired(const State & state) const {
      const auto value=json::parse(state.data,nullptr,false);
      if(!value.is_object()||!value.contains("challenge")||!value.contains("issued_steady_ms"))return false;
      const auto now=steady();
      return now*1000-value.value("issued_steady_ms",now*1000)>=value.value("control_lease_ms",300.0);
    }
    void arm_flush() {
      if(closed||flush_armed||!rtc_deferred)return;
      flush_armed=true;
      rtc_flush.expires_after(std::chrono::milliseconds(20));
      rtc_flush.async_wait([self=shared_from_this()](beast::error_code ec){
        if(ec||self->closed)return;
        self->flush_armed=false;self->flush_deferred();
        if(self->rtc_deferred)self->arm_flush();
      });
    }
    void flush_deferred() {
      if(closed||!rtc_deferred||!rtc_active){rtc_deferred.reset();return;}
      if(control_expired(*rtc_deferred)){rtc_deferred.reset();owner.count("states_expired_before_write");return;}
      const auto value=json::parse(rtc_deferred->data,nullptr,false);
      if(!value.is_object()||!rtc)return;
      // 仍使用原签发时间。缓冲排空时已经超过300ms的状态直接丢弃，不改时间后冒充新挑战。
      if(rtc->send(compact_control(value).dump()))rtc_deferred.reset();
    }
    void send(State state) {
      if (closed) return;
      // 过期挑战不能占住发送位，也不能覆盖已经排队的更新挑战。
      if(!state.reliable&&control_expired(state)){
        owner.count("states_expired_before_write");
        if(!sending.empty()||!pending||pending->reliable||control_expired(*pending))return;
        auto next=std::move(*pending);pending.reset();send(std::move(next));return;
      }
      // 数据通道不能等待TCP信令的异步写。缓冲未空时保留最新未过期状态，排空后按原签发时间补发。
      if(rtc_active&&!state.reliable){
        const auto value=json::parse(state.data,nullptr,false);const auto now=steady();
        if(!value.is_object())return;
        owner.duration("last_queue_wait_ms","max_queue_wait_ms",(now-state.queued_at)*1000);
        {std::lock_guard<std::mutex> lock(owner.stats_mutex);owner.stats["last_control_revision"]=value.value("control_revision",uint64_t{0});}
        if(rtc&&rtc->send(compact_control(value).dump())){rtc_deferred.reset();return;}
        rtc_deferred=std::move(state);arm_flush();return;
      }
      if (!sending.empty()) {
        if(state.reliable){if(signaling.size()>=16)return close("signaling_overflow");signaling.push_back(std::move(state));}
        else{if(pending)owner.count("pending_replaced");pending=std::move(state);}return;
      }
      auto value=json::parse(state.data,nullptr,false);const auto now=steady();
      if((compact||rtc_active)&&value.is_object()&&value.contains("challenge"))state.data=compact_control(value).dump();
      owner.duration("last_queue_wait_ms","max_queue_wait_ms",(now-state.queued_at)*1000);
      {std::lock_guard<std::mutex> lock(owner.stats_mutex);owner.stats["last_write_bytes"]=state.data.size();
        owner.stats["last_control_revision"]=value.is_object()?value.value("control_revision",uint64_t{0}):0;}
      if(rtc_active&&!state.reliable){if(rtc)rtc->send(state.data);return;}
      sending = std::move(state.data); stream.text(true);
      stream.async_write(asio::buffer(sending), [self = shared_from_this(),now](beast::error_code ec, std::size_t) {
        if (ec) return self->close("write",ec);
        self->owner.count("writes");self->owner.duration("last_write_ms","max_write_ms",(steady()-now)*1000);
        self->sending.clear();
        if(!self->signaling.empty()){auto next=std::move(self->signaling.front());self->signaling.pop_front();self->send(std::move(next));}
        else if (self->pending) {auto next = std::move(*self->pending); self->pending.reset(); self->send(std::move(next));}
      });
    }
  };
  struct Http : std::enable_shared_from_this<Http> {
    beast::tcp_stream stream; beast::flat_buffer buffer;
    std::optional<http::request_parser<http::string_body>> parser;
    std::shared_ptr<http::response<http::vector_body<unsigned char>>> response;
    Impl & owner;
    bool counted = true, keep_alive = false;
    Http(tcp::socket socket, Impl & impl) : stream(std::move(socket)), owner(impl) {}
    ~Http() {if (counted) owner.clients--;}
    static std::string rewrite_inspect(const std::string & target) {
      const auto query_at = target.find('?');
      const auto path = query_at == std::string::npos ? target : target.substr(0, query_at);
      const auto query = query_at == std::string::npos ? std::string() : target.substr(query_at);
      const std::string prefix = "/inspect";
      if (path != prefix && path.rfind(prefix + "/", 0) != 0) return {};
      auto upstream = path.size() <= prefix.size() ? std::string("/") : path.substr(prefix.size());
      if (upstream.empty()) upstream = "/";
      return upstream + query;
    }
    void proxy_inspect_http(http::request<http::string_body> request) {
      const auto upstream_target = rewrite_inspect(std::string(request.target()));
      const auto method = request.method();
      const auto body = request.body();
      const auto content_type = std::string(request[http::field::content_type]);
      const auto console = std::string(request["X-Mapping-Console"]);
      std::string host; unsigned short port;
      {std::lock_guard<std::mutex> lock(owner.stats_mutex); host = owner.inspect_host; port = owner.inspect_port;}
      asio::post(owner.http_worker, [self = shared_from_this(), method, upstream_target, body, content_type, console, host, port] {
        HttpResponse result{502, "text/plain; charset=utf-8", {}, {}};
        const std::string unavailable = "巡检页面未启动";
        result.body.assign(unavailable.begin(), unavailable.end());
        try {
          asio::io_context local;
          beast::tcp_stream upstream(local);
          upstream.expires_after(std::chrono::seconds(3));
          upstream.connect({asio::ip::make_address(host), port});
          http::request<http::string_body> forwarded(method, upstream_target, 11);
          forwarded.set(http::field::host, host + ":" + std::to_string(port));
          if (!console.empty()) forwarded.set("X-Mapping-Console", console);
          if (!content_type.empty()) forwarded.set(http::field::content_type, content_type);
          forwarded.body() = body;
          forwarded.prepare_payload();
          http::write(upstream, forwarded);
          beast::flat_buffer storage;
          http::response<http::vector_body<unsigned char>> received;
          http::read(upstream, storage, received);
          result.status = received.result_int();
          result.type = std::string(received[http::field::content_type]);
          if (result.type.empty()) result.type = "application/octet-stream";
          result.body = std::move(received.body());
          for (const auto name : {"X-Frame-Sequence", "X-Source-Stamp", "X-Map-Meta"})
            if (received.find(name) != received.end()) result.headers[name] = std::string(received[name]);
        } catch (const std::exception &) {}
        asio::post(self->owner.io, [self, result = std::move(result)]() mutable {self->reply(std::move(result));});
      });
    }
    void proxy_inspect_ws(http::request<http::string_body> request) {
      request.target(rewrite_inspect(std::string(request.target())));
      auto client = stream.release_socket();
      std::string host; unsigned short port;
      {std::lock_guard<std::mutex> lock(owner.stats_mutex); host = owner.inspect_host; port = owner.inspect_port;}
      std::thread([client = std::move(client), request = std::move(request), host, port]() mutable {
        beast::error_code ec;
        asio::io_context local;
        tcp::socket upstream(local);
        upstream.connect({asio::ip::make_address(host), port}, ec);
        if (ec) {client.close(ec); return;}
        http::write(upstream, request, ec);
        if (ec) {client.close(ec); upstream.close(ec); return;}
        beast::flat_buffer storage;
        http::response<http::string_body> handshake;
        http::read(upstream, storage, handshake, ec);
        if (ec) {client.close(ec); upstream.close(ec); return;}
        http::write(client, handshake, ec);
        if (ec) {client.close(ec); upstream.close(ec); return;}
        if (storage.size()) asio::write(client, storage.data(), ec);
        auto pump = [](tcp::socket & from, tcp::socket & to) {
          std::array<char, 4096> data;
          beast::error_code read_error, write_error;
          while (!read_error && !write_error) {
            const auto count = from.read_some(asio::buffer(data), read_error);
            if (read_error || !count) break;
            asio::write(to, asio::buffer(data.data(), count), write_error);
          }
          from.shutdown(tcp::socket::shutdown_receive, read_error);
          to.shutdown(tcp::socket::shutdown_send, write_error);
        };
        std::thread reverse([&] {pump(upstream, client);});
        pump(client, upstream);
        reverse.join();
        client.close(ec); upstream.close(ec);
      }).detach();
    }
    void read() {
      stream.expires_after(std::chrono::seconds(3));
      parser.emplace();parser->body_limit(4096);
      http::async_read(stream, buffer, *parser, [self = shared_from_this()](beast::error_code ec, std::size_t) {
        if (ec||!self->stream.socket().is_open()) return;
        auto request = self->parser->release();self->keep_alive=request.keep_alive();
        const std::string host(request[http::field::host]);
        const std::string origin(request[http::field::origin]);
        const bool same_origin = origin.empty() || origin == "http://" + host;
        const auto inspect_target = rewrite_inspect(std::string(request.target()));
        if (!inspect_target.empty()) {
          if (!same_origin) return self->reply({403, "text/plain", {'d','e','n','i','e','d'}, {}});
          if (websocket::is_upgrade(request)) return self->proxy_inspect_ws(std::move(request));
          return self->proxy_inspect_http(std::move(request));
        }
        if (websocket::is_upgrade(request) && request.target() == "/ws/control" && same_origin) {
          if (self->owner.controller.expired()) {
            auto ws = std::make_shared<Ws>(self->stream.release_socket(), self->owner);
            self->counted = false;
            self->owner.controller = ws; ws->accept(std::move(request)); return;
          }
          // 升级失败不复用HTTP连接，让客户端及时结束失败握手并重试。
          self->keep_alive=false;
          return self->reply({409, "text/plain; charset=utf-8", {'b','u','s','y'}, {}});
        }
        if(websocket::is_upgrade(request))self->keep_alive=false;
        if (!same_origin || (request.method() == http::verb::post && request["X-Mapping-Console"] != "1"))
          return self->reply({403, "text/plain", {'d','e','n','i','e','d'}, {}});
        HttpRequest input{std::string(request.method_string()),std::string(request.target()),request.body()};
        asio::post(self->owner.http_worker,[self,input=std::move(input)]{
          HttpResponse response;
          try{response=self->owner.handler(input);}
          catch(const std::exception & error){auto text=json{{"error",error.what()}}.dump();
            response={400,"application/json; charset=utf-8",{text.begin(),text.end()},{}};}
          asio::post(self->owner.io,[self,response=std::move(response)]()mutable{self->reply(std::move(response));});
        });
      });
    }
    void reply(HttpResponse value) {
      if(!stream.socket().is_open())return;
      response = std::make_shared<http::response<http::vector_body<unsigned char>>>(static_cast<http::status>(value.status), 11);
      response->set(http::field::content_type, value.type);
      response->set(http::field::cache_control, "no-store");
      response->set("X-Content-Type-Options", "nosniff");
      response->set("Content-Security-Policy", "default-src 'self'; img-src 'self' blob:; connect-src 'self'; style-src 'self'; script-src 'self'; frame-ancestors 'none'");
      for (const auto & field : value.headers) response->set(field.first, field.second);
      response->body() = std::move(value.body); response->keep_alive(keep_alive); response->prepare_payload();
      stream.expires_after(std::chrono::seconds(3));
      http::async_write(stream, *response, [self = shared_from_this()](beast::error_code ec, std::size_t) {
        if(!ec&&self->keep_alive){self->response.reset();self->read();return;}
        self->stream.socket().shutdown(tcp::socket::shutdown_send, ec);
      });
    }
  };
  Impl(const std::string & address, unsigned short port, Handler h, Control c)
    : acceptor(io), handler(std::move(h)), control(std::move(c)) {
    const tcp::endpoint endpoint(asio::ip::make_address(address), port);
    acceptor.open(endpoint.protocol()); acceptor.set_option(asio::socket_base::reuse_address(true));
    acceptor.bind(endpoint); acceptor.listen(); accept(); thread = std::thread([this] {io.run();});
  }
  void accept() {
    acceptor.async_accept([this](beast::error_code ec, tcp::socket socket) {
      if (!ec && clients.load() < 16) {
        clients++;
        auto connection = std::make_shared<Http>(std::move(socket), *this);
        connections.erase(std::remove_if(connections.begin(), connections.end(), [](auto & v) {return v.expired();}), connections.end());
        connections.push_back(connection); connection->read();
      }
      if (acceptor.is_open()) accept();
    });
  }
  void shutdown() {
    if(stopped)return;
    stopped=true;
    std::promise<void> closed;auto done=closed.get_future();
    asio::post(io, [this,&closed] {
      if (auto ws = controller.lock()) ws->close("server_shutdown");
      beast::error_code ec; acceptor.close(ec);
      for (auto & connection : connections) if (auto client = connection.lock()) client->stream.socket().close(ec);
      closed.set_value();
    });
    done.wait();http_worker.join();work.reset();io.stop(); if (thread.joinable()) thread.join();
    // 清理未完成 handlers 时 owner 仍然存在。
    io.restart(); io.run();
  }
  ~Impl() {shutdown();}
};
HttpServer::HttpServer(const std::string & a, unsigned short p, Handler h, Control c)
  : impl_(std::make_unique<Impl>(a, p, std::move(h), std::move(c))) {}
HttpServer::~HttpServer() = default;
void HttpServer::stop(){impl_->shutdown();}
void HttpServer::set_inspect_upstream(const std::string & host, unsigned short port) {
  std::lock_guard<std::mutex> lock(impl_->stats_mutex);
  impl_->inspect_host = host; impl_->inspect_port = port;
}
json HttpServer::diagnostics()const{std::lock_guard<std::mutex> lock(impl_->stats_mutex);return impl_->stats;}
void HttpServer::broadcast(const std::string & value) {
  std::lock_guard<std::mutex> lock(impl_->broadcast_mutex);
  impl_->broadcast_value = value;
  impl_->broadcast_at = steady();
  if (impl_->broadcast_pending) {impl_->count("broadcast_replaced");return;}
  impl_->broadcast_pending = true;
  asio::post(impl_->io, [owner = impl_.get()] {
    std::string latest;double queued_at;
    {std::lock_guard<std::mutex> guard(owner->broadcast_mutex); latest = owner->broadcast_value;
      queued_at=owner->broadcast_at;owner->broadcast_pending = false;}
    if (auto ws = owner->controller.lock()) ws->send({std::move(latest),queued_at});
  });
}
}  // namespace carcar_mapping_console
