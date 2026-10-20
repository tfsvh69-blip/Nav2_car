'use strict';
// 仅管理数据通道；使能、按压和车端授权不由传输层自动恢复。
class MappingRtcTransport {
  constructor(signal,onMessage,onState) {
    this.signal=signal;this.onMessage=onMessage;this.onState=onState;
    this.closed=false;this.opened=false;this.offerSent=false;
    this.localCandidates=[];this.remoteCandidates=[];
    this.pc=new RTCPeerConnection({iceServers:[]});
    this.channel=this.pc.createDataChannel('mapping-control',{ordered:false,maxRetransmits:0});
    this.channel.onmessage=event=>{if(!this.closed&&this.opened&&typeof event.data==='string')this.onMessage(event.data);};
    this.channel.onopen=()=>{if(this.closed)return;this.opened=true;clearTimeout(this.timer);this.onState('open');};
    this.channel.onclose=()=>this.fail('rtc_channel_closed');
    this.channel.onerror=()=>this.fail('rtc_channel_error');
    this.pc.onconnectionstatechange=()=>{
      const state=this.pc.connectionState;
      if(state==='connected'||state==='connecting'){clearTimeout(this.disconnectTimer);return;}
      // disconnected 在无线上会短暂出现并恢复。立刻拆通道会把一次使能打成停车。
      if(state==='disconnected'){
        clearTimeout(this.disconnectTimer);
        this.disconnectTimer=setTimeout(()=>{if(!this.closed&&this.pc.connectionState==='disconnected')this.fail('rtc_connection_closed');},1000);
        return;
      }
      if(state==='failed'||state==='closed')this.fail('rtc_connection_closed');
    };
    this.pc.onicecandidate=event=>{
      if(this.closed||!event.candidate)return;
      const value={action:'rtc_candidate',candidate:event.candidate.candidate,mid:event.candidate.sdpMid||'0'};
      if(this.offerSent)this.signal(value);else if(this.localCandidates.length<32)this.localCandidates.push(value);
    };
    this.timer=setTimeout(()=>this.fail('rtc_connect_timeout'),5000);
  }
  async start() {
    try{
      const offer=await this.pc.createOffer();if(this.closed)return;
      await this.pc.setLocalDescription(offer);if(this.closed)return;
      this.signal({action:'rtc_offer',sdp:this.pc.localDescription.sdp});this.offerSent=true;
      for(const value of this.localCandidates)this.signal(value);this.localCandidates=[];
    }catch(_){this.fail('rtc_offer_failed');}
  }
  async handle(value) {
    if(this.closed)return;
    try{
      if(value.rtc_description){
        await this.pc.setRemoteDescription({type:value.type,sdp:value.sdp});if(this.closed)return;
        for(const candidate of this.remoteCandidates)await this.pc.addIceCandidate(candidate);
        this.remoteCandidates=[];
      }else if(value.rtc_candidate){
        const candidate={candidate:value.candidate,sdpMid:value.mid};
        if(this.pc.remoteDescription)await this.pc.addIceCandidate(candidate);
        else if(this.remoteCandidates.length<32)this.remoteCandidates.push(candidate);
      }else if(value.rtc_status==='fallback')this.fail(value.reason||'rtc_server_fallback');
    }catch(_){this.fail('rtc_signal_failed');}
  }
  send(text) {
    if(this.closed||!this.opened||this.channel.readyState!=='open'||this.channel.bufferedAmount!==0)return false;
    try{this.channel.send(text);return true;}catch(_){this.fail('rtc_send_failed');return false;}
  }
  fail(why) {
    if(this.closed)return;
    this.close();this.signal({action:'rtc_cancel'});this.onState('fallback',why);
  }
  close() {
    if(this.closed)return;this.closed=true;this.opened=false;clearTimeout(this.timer);clearTimeout(this.disconnectTimer);
    this.channel.onopen=this.channel.onclose=this.channel.onerror=this.channel.onmessage=null;
    this.pc.onconnectionstatechange=this.pc.onicecandidate=null;
    this.channel.close();this.pc.close();
  }
}
