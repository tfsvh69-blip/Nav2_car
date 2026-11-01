#include <gtest/gtest.h>
#include "carcar_mapping_console/safety.hpp"
#include "carcar_mapping_console/common.hpp"
#include "carcar_mapping_console/storage.hpp"
#include <fstream>
using namespace carcar_mapping_console;
TEST(Storage, PreservesEmergencySaveBeforeRejectingMotion){
  constexpr uintmax_t gib=1024ULL*1024*1024;
  auto state=storage_status(std::filesystem::space_info{10*gib,3*gib,2*gib-1});
  EXPECT_FALSE(state["start_allowed"]);EXPECT_TRUE(state["save_allowed"]);EXPECT_TRUE(state["warning"]);
  state=storage_status(std::filesystem::space_info{10*gib,gib,gib/4-1});
  EXPECT_FALSE(state["start_allowed"]);EXPECT_FALSE(state["save_allowed"]);
  state=storage_status(std::filesystem::space_info{10*gib,6*gib,5*gib});
  EXPECT_TRUE(state["start_allowed"]);EXPECT_TRUE(state["save_allowed"]);EXPECT_FALSE(state["warning"]);
}
TEST(Storage, NewMapDirectoryUsesExistingFilesystem){
  const auto state=storage_status(std::filesystem::path("/tmp/carcar-r27-nonexistent/path/maps"));
  EXPECT_TRUE(state["valid"]);EXPECT_GT(state["available_bytes"].get<uintmax_t>(),0U);
  EXPECT_FALSE(std::filesystem::exists("/tmp/carcar-r27-nonexistent"));
}
static Safety ready(double now = 10) {
  Safety s; s.require_video=true; s.connect("operator"); s.gateway_at=s.scan_at=s.odom_at=now;
  s.session_ready=true; s.frame("frame",now); s.challenge("token",now);
  EXPECT_TRUE(s.enable("operator","frame",now)); return s;
}
TEST(Safety, StartupZero) {Safety s; EXPECT_EQ(s.tick(10).x,0); EXPECT_FALSE(s.armed);}
TEST(Safety, StationaryReadyDoesNotRequireBrowserHeartbeats) {
  auto s=ready();s.require_video=false;
  for(int i=1;i<=250;++i){
    const double now=10+i*.02;s.gateway_at=s.scan_at=s.odom_at=now;
    EXPECT_EQ(s.tick(now).x,0);ASSERT_TRUE(s.armed)<<s.reason;
  }
  // 停车后仍只能用新挑战开始一次新的按压；旧消息不能被静止等待变新。
  s.challenge("new-press",15);ASSERT_TRUE(s.drive("operator",1,"new-press",1,0,"",15.01));
  ASSERT_TRUE(s.release("operator",2));
  s.gateway_at=s.scan_at=s.odom_at=18;EXPECT_EQ(s.tick(18).x,0);ASSERT_TRUE(s.armed);
  EXPECT_FALSE(s.drive("operator",1,"new-press",1,0,"",18.01));
  EXPECT_EQ(s.tick(18.01).x,0);
  s.challenge("second-press",18.02);ASSERT_TRUE(s.drive("operator",3,"second-press",1,0,"",18.03));
  EXPECT_GT(s.tick(18.04).x,0);
  s.gateway_at=s.scan_at=s.odom_at=19.04;EXPECT_EQ(s.tick(19.04).x,0);EXPECT_TRUE(s.armed);
  EXPECT_EQ(s.reason,"操纵授权超时");
  s.gateway_at=s.scan_at=s.odom_at=19.05;ASSERT_TRUE(s.drive("operator",4,"later",1,0,"",19.05));
  EXPECT_GT(s.tick(19.06).x,0);
}
TEST(Safety, RetriedSameConnectionCannotRevokeOrResetTheActiveOperator) {
  auto s=ready();s.require_video=false;s.change_gear("fast");s.challenge("fresh",10.01);
  ASSERT_TRUE(s.enable("operator","",10.01));
  ASSERT_TRUE(s.drive("operator",10,"fresh",1,0,"",10.02));
  // 网关在guard_session确认前可重发connect；同一ID仅确认存在，不能重新建会话。
  s.connect("operator");EXPECT_TRUE(s.armed);EXPECT_EQ(s.gear,"fast");
  EXPECT_DOUBLE_EQ(s.tick(10.03).x,.18);EXPECT_EQ(s.timing(10.03).sequence,10u);
  EXPECT_FALSE(s.drive("operator",9,"fresh",1,0,"",10.04));
  s.connect("new-operator");EXPECT_FALSE(s.armed);EXPECT_EQ(s.tick(10.05).x,0);
}
TEST(Safety, RecordedTabletLatencyDoesNotRevokeContinuousJoystick) {
  // 现场 last_stop：请求令牌已走了176.59ms，后续请求间隔47.40ms。
  // 每60ms发新令牌；同样的WiFi时延下，连续推杆仍应有命令和授权。
  auto s=ready();s.require_video=false;
  ASSERT_TRUE(s.drive("operator",1,"token",1,0,"",10.17659));
  s.gateway_at=s.scan_at=s.odom_at=10.20;s.challenge("next",10.06);
  EXPECT_GT(s.tick(10.22).x,0);ASSERT_TRUE(s.armed)<<s.reason;
  ASSERT_TRUE(s.drive("operator",2,"next",1,0,"",10.22399));
}
TEST(Safety, CompatibilityHeartbeatExpiresWithoutRevokingStationaryReady) {
  auto s=ready();s.require_video=false;
  ASSERT_TRUE(s.heartbeat("operator",1,"token",10.22));
  EXPECT_TRUE(s.armed);EXPECT_NEAR(s.timing(10.22).presence_remaining,0,1e-8);
  s.gateway_at=s.scan_at=s.odom_at=10.31;
  EXPECT_EQ(s.tick(10.31).x,0);EXPECT_TRUE(s.armed);
}
TEST(Safety, ForwardAndRotateNeverStrafe) {
  auto s=ready(); ASSERT_TRUE(s.intent("operator",1,"token","forward","frame",10.01)); EXPECT_DOUBLE_EQ(s.tick(10.02).x,.05);
  ASSERT_TRUE(s.intent("operator",2,"token","left","frame",10.03)); EXPECT_DOUBLE_EQ(s.tick(10.04).yaw,.10); EXPECT_EQ(s.tick(10.04).x,0);
}
TEST(Safety, BothGearsAndGearChangeStop) {
  auto s=ready(); ASSERT_TRUE(s.intent("operator",1,"token","forward","frame",10.01)); s.change_gear("normal");
  EXPECT_EQ(s.tick(10.02).x,0); EXPECT_FALSE(s.armed); s.challenge("fresh",10.03); ASSERT_TRUE(s.enable("operator","frame",10.03));
  ASSERT_TRUE(s.intent("operator",2,"fresh","backward","frame",10.04)); EXPECT_DOUBLE_EQ(s.tick(10.05).x,-.1);
}
TEST(Safety, FastGearRequiresReenableAndKeepsShortLease) {
  auto s=ready();s.require_video=false;
  s.change_gear("fast");EXPECT_FALSE(s.armed);EXPECT_EQ(s.tick(10.01).x,0);
  s.challenge("fast",10.02);ASSERT_TRUE(s.enable("operator","",10.02));
  ASSERT_TRUE(s.drive("operator",1,"fast",1,0,"",10.03));EXPECT_DOUBLE_EQ(s.tick(10.04).x,.18);
  ASSERT_TRUE(s.drive("operator",2,"fast",0,1,"",10.05));EXPECT_DOUBLE_EQ(s.tick(10.06).yaw,.36);
  s.gateway_at=s.scan_at=s.odom_at=11.06;EXPECT_EQ(s.tick(11.06).yaw,0);EXPECT_TRUE(s.armed);
  EXPECT_EQ(s.reason,"操纵授权超时");
}
TEST(Safety, StopReasonsDistinguishExpiredLeaseAndExpiredSensors) {
  auto expired=ready();expired.require_video=false;expired.gateway_at=expired.scan_at=expired.odom_at=12.01;
  EXPECT_TRUE(expired.drive("operator",1,"token",1,0,"",12.01));
  EXPECT_EQ(expired.reason,"按住行驶");EXPECT_NEAR(expired.timing(12.01).challenge_age,2.01,1e-9);
  auto sensor=ready();sensor.odom_at=9;
  EXPECT_FALSE(sensor.drive("operator",1,"token",1,0,"frame",10.01));EXPECT_EQ(sensor.reason,"轮式里程计过期");
  EXPECT_GT(sensor.timing(10.01).odom_age,1.0);
}
TEST(Safety, LeaseDeadlineIsIssueTimeNotReceiveTime) {
  auto s=ready(); s.require_video=false; ASSERT_TRUE(s.intent("operator",1,"token","forward","frame",10.19));
  s.gateway_at=s.scan_at=s.odom_at=11.18;EXPECT_GT(s.tick(11.18).x,0);EXPECT_TRUE(s.armed);
  s.gateway_at=s.scan_at=s.odom_at=11.20;EXPECT_EQ(s.tick(11.20).x,0); EXPECT_TRUE(s.armed);
  EXPECT_EQ(s.reason,"操纵授权超时");
  ASSERT_TRUE(s.intent("operator",2,"token","forward","frame",11.21));
  EXPECT_GT(s.tick(11.22).x,0);
}
TEST(Safety, BufferedCommandsWithIncreasingSequenceCannotExtendIssuedDeadline) {
  auto s=ready();s.require_video=false;
  // 即使序号递增，同一条旧挑战仍受原签发截止时间约束。
  s.gateway_at=s.scan_at=s.odom_at=11.50;
  ASSERT_TRUE(s.drive("operator",1,"token",1,0,"",11.50));
  EXPECT_NEAR(s.timing(11.50).lease_remaining,1.0,1e-9);
  s.gateway_at=s.scan_at=s.odom_at=12.40;
  ASSERT_TRUE(s.drive("operator",2,"token",1,0,"",12.40));
  EXPECT_NEAR(s.timing(12.40).lease_remaining,1.0,1e-9);
  EXPECT_GT(s.tick(12.40).x,0);EXPECT_TRUE(s.armed);
  s.gateway_at=s.scan_at=s.odom_at=13.41;EXPECT_EQ(s.tick(13.41).x,0);EXPECT_TRUE(s.armed);
}
TEST(Safety, DuplicateAndOutOfOrderCannotRenew) {
  auto s=ready(); ASSERT_TRUE(s.intent("operator",10,"token","forward","frame",10.01));
  EXPECT_FALSE(s.intent("operator",10,"token","forward","frame",10.15)); EXPECT_FALSE(s.intent("operator",9,"token","forward","frame",10.16));
  EXPECT_EQ(s.tick(11.01).x,0);
}
TEST(Safety, ReleaseInvalidatesQueuedCommands) {
  auto s=ready(); s.require_video=false;
  ASSERT_TRUE(s.drive("operator",1,"token",1,0,"",10.01));
  ASSERT_TRUE(s.release("operator",2));
  EXPECT_FALSE(s.drive("operator",1,"token",1,0,"",10.05));
  EXPECT_EQ(s.tick(10.06).x,0);EXPECT_TRUE(s.armed);
  ASSERT_TRUE(s.drive("operator",3,"token",1,0,"",10.07));
  EXPECT_GT(s.tick(10.08).x,0);
}
TEST(Safety, ReconnectDoesNotMove) {
  auto s=ready(); s.intent("operator",1,"token","forward","frame",10.01); s.disconnect("operator"); s.connect("new");
  EXPECT_FALSE(s.intent("operator",2,"token","forward","frame",10.02)); EXPECT_FALSE(s.armed); EXPECT_EQ(s.tick(10.03).x,0);
}
TEST(Safety, SourceVideoCannotRefreshByRepeatingTimestamp) {
  auto s=ready(); s.frame("frame",10.9); EXPECT_DOUBLE_EQ(s.video_at,10);
  s.gateway_at=s.scan_at=s.odom_at=11; EXPECT_FALSE(s.enable("operator","frame",11.01));
}
TEST(Safety, FreshSourceDoesNotHideFrozenBrowser) {
  auto s=ready(); s.frame("new",10.9); s.gateway_at=s.scan_at=s.odom_at=11;
  EXPECT_FALSE(s.enable("operator","frame",11.01)); EXPECT_EQ(s.reason,"浏览器画面过期");
}
TEST(Safety, GatewayStallStopsIndependentOfVideo) {
  auto s=ready(); s.intent("operator",1,"token","forward","frame",10.01); s.scan_at=s.odom_at=10.4; s.frame("next",10.4);
  EXPECT_EQ(s.tick(10.4).x,0); EXPECT_FALSE(s.armed); EXPECT_EQ(s.reason,"建图会话未就绪");
}
TEST(Safety, SavingPermissionImmediatelyStops) {
  auto s=ready(); s.intent("operator",1,"token","forward","frame",10.01); s.session_ready=false;
  EXPECT_EQ(s.tick(10.02).x,0); s.session_ready=true; EXPECT_FALSE(s.armed);
}
TEST(Safety, TopologyAndSensorFailuresLatchStop) {
  for(int cause=0;cause<3;++cause){auto s=ready(); s.intent("operator",1,"token","forward","frame",10.01);
    if(cause==0)s.topology_ok=false;
    if(cause==1)s.scan_at=9;
    if(cause==2)s.odom_at=9;
    EXPECT_EQ(s.tick(10.02).x,0);EXPECT_FALSE(s.armed);}
}
TEST(Safety, UnknownSessionAndDirectionRejected) {
  auto s=ready(); EXPECT_FALSE(s.intent("other",1,"token","forward","frame",10.01));
  EXPECT_FALSE(s.intent("operator",2,"token","strafe","frame",10.02)); EXPECT_FALSE(s.armed);
}
TEST(Safety, HeadlessJoystickReleaseAndRepressPreservePermissionWithSequenceBarrier) {
  auto s=ready(); s.require_video=false; s.frames.clear(); s.video_at=-1e9;
  ASSERT_TRUE(s.enable("operator","",10.01));
  ASSERT_TRUE(s.drive("operator",1,"token",.6,.8,"",10.02));
  EXPECT_NEAR(s.tick(10.03).x,.03,1e-9); EXPECT_NEAR(s.tick(10.03).yaw,.08,1e-9);
  ASSERT_TRUE(s.release("operator",2)); EXPECT_EQ(s.tick(10.04).x,0); EXPECT_TRUE(s.armed);
  EXPECT_FALSE(s.drive("operator",1,"token",1,0,"",10.05));
  EXPECT_EQ(s.tick(10.05).x,0); EXPECT_TRUE(s.armed);
  ASSERT_TRUE(s.drive("operator",3,"token",1,0,"",10.06)); EXPECT_DOUBLE_EQ(s.tick(10.07).x,.05);
  s.stop("后台"); EXPECT_FALSE(s.drive("operator",4,"token",1,0,"",10.08)); EXPECT_FALSE(s.armed);
}
TEST(Safety, HeadlessModeStillRequiresSensorsGatewayAndShortLease) {
  for(int cause=0;cause<4;++cause) {
    auto s=ready(); s.require_video=false; s.frames.clear(); s.video_at=-1e9;
    ASSERT_TRUE(s.drive("operator",1,"token",1,0,"",10.01));
    if(cause==0)s.scan_at=9;
    if(cause==1)s.odom_at=9;
    if(cause==2)s.gateway_at=9;
    if(cause==3){s.gateway_at=s.scan_at=s.odom_at=11.01;}
    EXPECT_EQ(s.tick(cause==3?11.01:10.02).x,0);
    if(cause==3){EXPECT_TRUE(s.armed);EXPECT_EQ(s.reason,"操纵授权超时");}
    else EXPECT_FALSE(s.armed);
  }
}
TEST(Safety, JoystickRejectsInvalidNumbersAndNormalizesCombinedDemand) {
  for(const double value:{1.01,-1.01,std::numeric_limits<double>::infinity(),std::numeric_limits<double>::quiet_NaN()}) {
    auto s=ready(); EXPECT_FALSE(s.drive("operator",1,"token",value,0,"frame",10.01)); EXPECT_FALSE(s.armed);
  }
  auto s=ready(); ASSERT_TRUE(s.drive("operator",1,"token",1,1,"frame",10.01));
  EXPECT_NEAR(s.tick(10.02).x,.05/std::sqrt(2),1e-9); EXPECT_NEAR(s.tick(10.02).yaw,.1/std::sqrt(2),1e-9);
}
TEST(Safety, IdleHeartbeatCompatibilityCannotCreateMotion) {
  auto s=ready(); s.require_video=false;
  s.gateway_at=s.scan_at=s.odom_at=10.25; s.challenge("new",10.25);
  ASSERT_TRUE(s.heartbeat("operator",1,"new",10.26));
  EXPECT_EQ(s.tick(10.31).x,0); EXPECT_TRUE(s.armed);
  EXPECT_FALSE(s.heartbeat("operator",1,"new",10.45));
  s.gateway_at=s.scan_at=s.odom_at=10.56;
  EXPECT_EQ(s.tick(10.56).x,0); EXPECT_TRUE(s.armed);
  s.challenge("later",10.57); EXPECT_TRUE(s.heartbeat("operator",2,"later",10.57));
  EXPECT_EQ(s.tick(10.58).x,0);
}
TEST(Safety, ExpiredHeartbeatIsObservableAndNeverExtendsPresence) {
  auto s=ready();s.require_video=false;
  ASSERT_TRUE(s.drive("operator",1,"token",1,0,"",10.01));
  ASSERT_TRUE(s.heartbeat("operator",2,"token",10.50));
  EXPECT_EQ(s.timing(10.50).request_result,"heartbeat_accepted");
  s.gateway_at=s.scan_at=s.odom_at=11.02;EXPECT_EQ(s.tick(11.02).x,0);EXPECT_TRUE(s.armed);
  EXPECT_EQ(s.reason,"操纵授权超时");
}
TEST(Coordinates, OriginRotationAndScreenY) {
  auto p=pixel_to_map(2,1,5,.5,10,20,M_PI/2);
  EXPECT_NEAR(p.first,8,1e-9);EXPECT_NEAR(p.second,21,1e-9);
}
TEST(Bundle, RequiresAllFourFilesAndRejectsTamperingTraversalAndLinks) {
  auto root=std::filesystem::temp_directory_path()/nonce();auto dir=root/"session";std::filesystem::create_directories(dir);
  json manifest{{"complete",true},{"slam_version","2.6.10"},{"base_frame","base_footprint"}};
  for(auto name:{"map.yaml","map.pgm","graph.posegraph","graph.data"}) {std::ofstream(dir/name)<<name;manifest["sha256"][name]=sha256(dir/name);}
  std::ofstream(dir/"manifest.json")<<manifest.dump(); EXPECT_NO_THROW(validate_bundle(root,"session"));
  EXPECT_THROW(validate_bundle(root,"../session"),std::exception);
  std::ofstream(dir/"graph.data")<<"损坏";EXPECT_THROW(validate_bundle(root,"session"),std::exception);
  std::filesystem::remove_all(root);
}
TEST(Bundle, IncompleteCannotResume) {
  auto root=std::filesystem::temp_directory_path()/nonce();std::filesystem::create_directories(root/"partial");
  std::ofstream(root/"partial/manifest.json")<<json{{"complete",false}}.dump();
  EXPECT_THROW(validate_bundle(root,"partial"),std::exception); std::filesystem::remove_all(root);
}
