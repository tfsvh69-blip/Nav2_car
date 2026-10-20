#include "carcar_mapping_console/rtc_control.hpp"
#include <mutex>
#include <stdexcept>
#ifdef MAPPING_ENABLE_WEBRTC
#include <rtc/rtc.hpp>
#endif

namespace carcar_mapping_console {
using json = nlohmann::json;
struct RtcControl::Impl : std::enable_shared_from_this<Impl> {
  mutable std::mutex mutex;
  Signal signal; Receive receive; State state;
  bool closed = false, opened = false, failed = false;
  json stats{{"available",RtcControl::available()},{"active",false},{"sent",0},
    {"received",0},{"backpressure_dropped",0},{"buffered_sends",0},{"buffered_bytes",0}};
#ifdef MAPPING_ENABLE_WEBRTC
  std::shared_ptr<rtc::PeerConnection> peer;
  std::shared_ptr<rtc::DataChannel> channel;
  void notify(bool active, const std::string & why) {
    std::lock_guard<std::mutex> lock(mutex);
    if(closed||(active&&opened)||(!active&&failed))return;
    if(active)opened=true;else failed=true;
    stats["active"]=active;stats["reason"]=why;
    if(state)state(active,why);
  }
  void start(const std::string & bind_address) {
    rtc::Configuration config;
    config.enableIceTcp=false; config.maxMessageSize=2048; config.mtu=1200;
    if(!bind_address.empty())config.bindAddress=bind_address;
    // 本轮只做局域网host候选，不访问外部STUN/TURN或传输视频。
    peer=std::make_shared<rtc::PeerConnection>(config);
    const std::weak_ptr<Impl> weak=shared_from_this();
    peer->onLocalDescription([weak](rtc::Description d){if(auto self=weak.lock()){
      std::lock_guard<std::mutex> lock(self->mutex);if(!self->closed&&self->signal)
        self->signal({{"rtc_description",true},{"type",d.typeString()},{"sdp",std::string(d)}});}});
    peer->onLocalCandidate([weak](rtc::Candidate c){if(auto self=weak.lock()){
      std::lock_guard<std::mutex> lock(self->mutex);if(!self->closed&&self->signal)
        self->signal({{"rtc_candidate",true},{"candidate",c.candidate()},{"mid",c.mid()}});}});
    peer->onStateChange([weak](rtc::PeerConnection::State s){if(auto self=weak.lock())
      if(s==rtc::PeerConnection::State::Failed||s==rtc::PeerConnection::State::Disconnected||s==rtc::PeerConnection::State::Closed)
        self->notify(false,"rtc_connection_closed");});
    peer->onDataChannel([weak](std::shared_ptr<rtc::DataChannel> dc){
      auto self=weak.lock();if(!self)return;
      const auto r=dc->reliability();
      if(dc->label()!="mapping-control"||!r.unordered||!r.maxRetransmits||*r.maxRetransmits!=0||r.maxPacketLifeTime){
        dc->close();self->notify(false,"rtc_reliability_mismatch");return;
      }
      {std::lock_guard<std::mutex> lock(self->mutex);if(self->closed||self->channel){dc->close();return;}self->channel=dc;}
      dc->onOpen([weak]{if(auto self=weak.lock())self->notify(true,"webrtc_udp");});
      dc->onClosed([weak]{if(auto self=weak.lock())self->notify(false,"rtc_channel_closed");});
      dc->onError([weak](std::string){if(auto self=weak.lock())self->notify(false,"rtc_channel_error");});
      dc->onMessage([weak](rtc::message_variant data){if(auto self=weak.lock()){
        if(!std::holds_alternative<std::string>(data))return;
        auto text=std::get<std::string>(std::move(data));if(text.empty()||text.size()>2048)return;
        std::lock_guard<std::mutex> lock(self->mutex);
        if(self->closed||!self->opened||self->failed)return;
        self->stats["received"]=self->stats["received"].get<uint64_t>()+1;
        if(self->receive)self->receive(std::move(text));
      }});
      if(dc->isOpen())self->notify(true,"webrtc_udp");
    });
  }
#endif
};
bool RtcControl::available() {
#ifdef MAPPING_ENABLE_WEBRTC
  return true;
#else
  return false;
#endif
}
RtcControl::RtcControl(Signal signal,Receive receive,State state,const std::string & bind_address):impl_(std::make_shared<Impl>()) {
  impl_->signal=std::move(signal);impl_->receive=std::move(receive);impl_->state=std::move(state);
#ifdef MAPPING_ENABLE_WEBRTC
  impl_->start(bind_address);
#else
  (void)bind_address;
#endif
}
RtcControl::~RtcControl(){close();}
void RtcControl::offer(const std::string & sdp) {
  if(sdp.empty()||sdp.size()>12000)throw std::runtime_error("RTC offer size");
#ifdef MAPPING_ENABLE_WEBRTC
  impl_->peer->setRemoteDescription(rtc::Description(sdp,"offer"));
#else
  throw std::runtime_error("WebRTC disabled");
#endif
}
void RtcControl::candidate(const std::string & value,const std::string & mid) {
  if(value.size()>2048||mid.size()>64)throw std::runtime_error("RTC candidate size");
#ifdef MAPPING_ENABLE_WEBRTC
  impl_->peer->addRemoteCandidate(rtc::Candidate(value,mid));
#else
  (void)value;(void)mid;
#endif
}
bool RtcControl::send(const std::string & message) {
#ifdef MAPPING_ENABLE_WEBRTC
  std::shared_ptr<rtc::DataChannel> dc;
  {std::lock_guard<std::mutex> lock(impl_->mutex);if(impl_->closed||impl_->failed||!impl_->opened)return false;dc=impl_->channel;}
  if(!dc||!dc->isOpen()||message.size()>2048)return false;
  const auto buffered=dc->bufferedAmount();
  {std::lock_guard<std::mutex> lock(impl_->mutex);impl_->stats["buffered_bytes"]=buffered;
    if(buffered){impl_->stats["backpressure_dropped"]=impl_->stats["backpressure_dropped"].get<uint64_t>()+1;return false;}}
  try{
    const bool sent=dc->send(message);
    std::lock_guard<std::mutex> lock(impl_->mutex);
    impl_->stats["sent"]=impl_->stats["sent"].get<uint64_t>()+1;
    if(!sent)impl_->stats["buffered_sends"]=impl_->stats["buffered_sends"].get<uint64_t>()+1;
    return sent;
  }catch(const std::exception&){impl_->notify(false,"rtc_send_error");return false;}
#else
  (void)message;return false;
#endif
}
json RtcControl::diagnostics() const {
  std::lock_guard<std::mutex> lock(impl_->mutex);auto result=impl_->stats;
#ifdef MAPPING_ENABLE_WEBRTC
  if(impl_->channel)result["buffered_bytes"]=impl_->channel->bufferedAmount();
  rtc::Candidate local,remote;
  if(impl_->peer&&impl_->peer->getSelectedCandidatePair(&local,&remote)){
    result["local_candidate"]=local.candidate();result["remote_candidate"]=remote.candidate();}
#endif
  return result;
}
void RtcControl::close() {
#ifdef MAPPING_ENABLE_WEBRTC
  std::shared_ptr<rtc::DataChannel> dc;std::shared_ptr<rtc::PeerConnection> pc;
#endif
  {std::lock_guard<std::mutex> lock(impl_->mutex);if(impl_->closed)return;
    impl_->closed=true;impl_->signal={};impl_->receive={};impl_->state={};impl_->stats["active"]=false;
#ifdef MAPPING_ENABLE_WEBRTC
    dc=std::move(impl_->channel);pc=std::move(impl_->peer);
#endif
  }
#ifdef MAPPING_ENABLE_WEBRTC
  if(dc){dc->resetCallbacks();dc->close();}if(pc){pc->resetCallbacks();pc->close();}
#endif
}
}  // namespace carcar_mapping_console
