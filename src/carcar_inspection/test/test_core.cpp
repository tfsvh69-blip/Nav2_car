// Copyright 2026 carcar maintainers. SPDX-License-Identifier: Apache-2.0
#include <gtest/gtest.h>
#include <unistd.h>
#include <atomic>
#include <fstream>
#include <thread>
#include "carcar_inspection/store.hpp"
#include "carcar_inspection/teaching.hpp"
#include "carcar_inspection/executor.hpp"
#include "carcar_inspection/upload.hpp"
#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
using namespace carcar_inspection;
namespace {
json point() {return {{"id","p1"},{"name","仪表"},{"map_id","map1"},{"map_version","v1"},
  {"pose",{{"x",1},{"y",2},{"yaw",0}}},{"target_degrees",140.5},{"board_command",94},{"verified",true}};}
json task() {return {{"id","t1"},{"name","巡检"},{"map_id","map1"},{"map_version","v1"},{"point_ids",{"p1"}},
  {"final_parking_pose",{{"x",0},{"y",0},{"yaw",0}}},{"points",json::array({point()})}};}
class Temporary {
public:
  Temporary(){char pattern[]="/tmp/carcar-inspection-core-XXXXXX";const auto * path=mkdtemp(pattern);if(!path)throw std::runtime_error("临时目录失败");directory=path;}
  ~Temporary(){std::filesystem::remove_all(directory);}
  std::filesystem::path directory;
};
Teaching prepared(){Teaching t;t.begin("operator");t.acquired("epoch",94);t.stopped(true);return t;}
}
TEST(Domain, ServoBoundsDoNotQuantizeBeyondCollisionLimit) {
  EXPECT_EQ(servo_command(160),106);
  EXPECT_EQ(servo_command(140.5),94);
  EXPECT_THROW(servo_command(160.1),std::runtime_error);
  EXPECT_THROW(servo_command(NAN),std::runtime_error);
}
TEST(Domain, OneAngleAndExplicitFinalParkingAreRequired) {auto p=point();p["views"]=json::array();EXPECT_THROW(validate_point(p),std::runtime_error);auto t=task();t.erase("final_parking_pose");EXPECT_ANY_THROW(validate_task(t));}
TEST(Domain, RepeatedPointsAndTraversalAreRejected) {
  auto t=task();t["point_ids"]={"p1","p1"};
  EXPECT_THROW(validate_task(t),std::runtime_error);
  EXPECT_THROW(valid_id("../p1"),std::runtime_error);
}
TEST(Store, RevisionConflictDoesNotOverwriteConfirmedPoint) {Temporary temp;Store s(temp.directory);auto saved=s.put("points",point(),0);EXPECT_EQ(saved["revision"],1);auto next=point();next["name"]="错误覆盖";EXPECT_THROW(s.put("points",next,0),std::runtime_error);EXPECT_EQ(s.get("points","p1")["name"],"仪表");}
TEST(Store, TaskOrderAndPhotoReferencesSurviveRestart) {Temporary temp;{Store s(temp.directory);s.put("points",point(),0);s.put("tasks",task(),0);}Store reopened(temp.directory);EXPECT_EQ(reopened.list("points").size(),1U);EXPECT_EQ(reopened.get("tasks","t1")["point_ids"][0],"p1");}
TEST(Store, RestartMarksOldExecutionInterruptedWithoutReplay) {Temporary temp;{Store s(temp.directory);s.put("runs",{{"id","run1"},{"status","RUNNING"},{"captures",{{"p1",{{"id","photo1"}}}}}},0);}Store reopened(temp.directory);const auto run=reopened.get("runs","run1");EXPECT_EQ(run["status"],"INTERRUPTED");EXPECT_EQ(run["captures"]["p1"]["id"],"photo1");}
TEST(Teaching, CannotStepBeforeStopOrSkipManyDegrees) {
  Teaching t;t.begin("operator");t.acquired("epoch",94);
  EXPECT_THROW(t.step(141.5,t.revision,false,1,1),std::runtime_error);
  t.stopped(true);
  EXPECT_THROW(t.step(145,t.revision,true,1,1),std::runtime_error);
}
TEST(Teaching, OldRevisionAndPreCommandFrameCannotConfirm) {auto t=prepared();const auto old=t.revision;t.step(140.5,t.revision,true,10,1);t.sent(true,94,"",1.1);t.image(10,true,2,.5);EXPECT_EQ(t.phase,"WAITING_FRAME");t.image(11,true,2,.5);t.viewed(11);EXPECT_THROW(t.check_confirm(old,11,true,true,2,.5),std::runtime_error);EXPECT_NO_THROW(t.check_confirm(t.revision,11,true,true,2,.5));}
TEST(Teaching, ANewStepInvalidatesThePreviousRenderedConfirmation) {auto t=prepared();t.step(140.5,t.revision,true,1,1);t.sent(true,94,"",1);t.image(2,true,2,.5);t.viewed(2);t.step(141.5,t.revision,true,2,2);EXPECT_EQ(t.displayed_frame,0U);EXPECT_THROW(t.check_confirm(t.revision,2,true,true,3,.5),std::runtime_error);}
TEST(Teaching, FailedWriteAndDisconnectedSessionCannotConfirm) {auto t=prepared();t.step(140.5,t.revision,true,1,1);t.sent(false,94,"短写",1);EXPECT_EQ(t.phase,"FAULT");t.begin("operator");t.acquired("epoch",94);t.disconnect("operator");EXPECT_EQ(t.phase,"FAULT");}
TEST(Executor, LastPhotoDoesNotCompleteBeforeFinalParking) {std::vector<std::string> operations;Executor e([&](const std::string & kind,const json & context,bool){operations.push_back((context.value("parking",false)?"parking:":"point:")+kind);return BT::NodeStatus::SUCCESS;},[]{});e.start(task());EXPECT_EQ(e.tick(),BT::NodeStatus::RUNNING);EXPECT_TRUE(e.parking);EXPECT_EQ(e.phase,"RUNNING");EXPECT_EQ(e.tick(),BT::NodeStatus::SUCCESS);EXPECT_EQ(operations.back(),"parking:stop");}
TEST(Executor, CancelHaltsInFlightActionAndCannotAdvance) {int halted=0,calls=0;Executor e([&](const std::string &,const json &,bool){++calls;return BT::NodeStatus::RUNNING;},[&]{++halted;});e.start(task());e.tick();e.cancel();EXPECT_EQ(halted,1);EXPECT_EQ(e.tick(),BT::NodeStatus::IDLE);EXPECT_EQ(calls,1);}
TEST(Executor, PauseAndResumeKeepCurrentPoint) {int halted=0;Executor e([](const std::string &,const json &,bool){return BT::NodeStatus::RUNNING;},[&]{++halted;});e.start(task());e.tick();e.pause();EXPECT_EQ(halted,1);EXPECT_EQ(e.phase,"PAUSED");e.resume();EXPECT_EQ(e.point_index,0U);EXPECT_EQ(e.phase,"RUNNING");}
TEST(Executor, NavigationFailureSkipsPointAndStillParks) {
  int parking_nav=0;Executor e([&](const std::string & kind,const json & context,bool){
    if(kind=="navigate"&&context.value("parking",false)){++parking_nav;return BT::NodeStatus::SUCCESS;}
    if(kind=="navigate")return BT::NodeStatus::FAILURE;
    return BT::NodeStatus::SUCCESS;},[]{});
  e.start(task());
  EXPECT_EQ(e.tick(),BT::NodeStatus::RUNNING);EXPECT_EQ(e.missed.size(),1U);EXPECT_TRUE(e.parking);EXPECT_EQ(e.phase,"RUNNING");
  EXPECT_EQ(e.tick(),BT::NodeStatus::SUCCESS);EXPECT_EQ(parking_nav,1);EXPECT_EQ(e.phase,"SUCCEEDED");
}
TEST(Executor, SecondApproachIsUsedWhenTheFirstCannotBeReached) {
  auto mission=task();mission["points"][0]["approaches"]=json::array({
    {{"pose",{{"x",1.0},{"y",2.0},{"yaw",0.0}}},{"target_degrees",140.5},{"board_command",94},{"verified",true}},
    {{"pose",{{"x",3.0},{"y",4.0},{"yaw",1.0}}},{"target_degrees",100.0},{"board_command",67},{"verified",true}}});
  std::vector<int> seen;Executor e([&](const std::string & kind,const json & context,bool){
    if(kind=="navigate"&&!context.value("parking",false)){seen.push_back(context.at("approach_index").get<int>());
      if(context.at("approach_index")==0)return BT::NodeStatus::FAILURE;
      EXPECT_DOUBLE_EQ(context.at("pose").at("x").get<double>(),3);EXPECT_EQ(context.at("board_command").get<int>(),67);}
    return BT::NodeStatus::SUCCESS;},[]{});
  e.start(mission);EXPECT_EQ(e.tick(),BT::NodeStatus::RUNNING);EXPECT_EQ(e.approach_index,1U);EXPECT_TRUE(e.missed.empty());
  EXPECT_EQ(e.tick(),BT::NodeStatus::RUNNING);EXPECT_TRUE(e.parking);EXPECT_EQ(seen.size(),2U);EXPECT_EQ(e.tick(),BT::NodeStatus::SUCCESS);
}
TEST(Domain, AtMostThreeApproachesAndEachKeepsItsOwnAngle) {
  auto p=point();json extra=json::array();
  for(int i=0;i<4;++i) extra.push_back({{"pose",p["pose"]},{"target_degrees",140.5},{"board_command",94},{"verified",true}});
  p["approaches"]=extra;EXPECT_THROW(normalize_point(p),std::runtime_error);
  extra.erase(extra.begin()+2,extra.end());p["approaches"]=extra;p["pose"]=extra[0]["pose"];
  const auto saved=normalize_point(p);EXPECT_EQ(saved["approaches"].size(),2U);
}
TEST(Upload, MismatchStaysPendingAndMatchingChecksumConfirmsWithoutDeletingTheFile) {
  EXPECT_EQ(interpret_upload_response("cap1","aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa",500,"{}").status,"PENDING");
  EXPECT_EQ(interpret_upload_response("cap1","aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa",200,"{\"id\":\"cap1\",\"sha256\":\"bbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbb\"}").status,"PENDING");
  const auto checksum="aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa";
  int server=socket(AF_INET,SOCK_STREAM,0);ASSERT_GE(server,0);int yes=1;setsockopt(server,SOL_SOCKET,SO_REUSEADDR,&yes,sizeof(yes));
  sockaddr_in address{};address.sin_family=AF_INET;address.sin_addr.s_addr=htonl(INADDR_LOOPBACK);address.sin_port=0;
  ASSERT_EQ(bind(server,reinterpret_cast<sockaddr*>(&address),sizeof(address)),0);ASSERT_EQ(listen(server,1),0);
  socklen_t length=sizeof(address);ASSERT_EQ(getsockname(server,reinterpret_cast<sockaddr*>(&address),&length),0);
  const auto url="http://127.0.0.1:"+std::to_string(ntohs(address.sin_port))+"/captures";
  std::atomic<int> requests{0};std::thread listener([&]{
    while(requests.load()<2){const int client=accept(server,nullptr,nullptr);if(client<0)return;std::string input(8192,'\0');
      const auto count=read(client,input.data(),input.size());if(count>0)input.resize(static_cast<size_t>(count));
      const auto turn=++requests;const auto body=turn==1?std::string("{\"id\":\"cap1\",\"sha256\":\"nope\"}"):
        std::string("{\"id\":\"cap1\",\"sha256\":\"")+checksum+"\"}";
      const auto response="HTTP/1.1 200 OK\r\nContent-Length: "+std::to_string(body.size())+"\r\nConnection: close\r\n\r\n"+body;
      const auto wrote=write(client,response.data(),response.size());(void)wrote;close(client);}});
  Temporary temp;const auto file=temp.directory/"cap1.jpg";{std::ofstream out(file,std::ios::binary);out<<"jpeg-bytes";}
  auto first=post_photo(url,"cap1",checksum,file.string());
  EXPECT_EQ(first.status,"PENDING");EXPECT_TRUE(std::filesystem::is_regular_file(file));
  auto second=post_photo(url,"cap1",checksum,file.string());
  EXPECT_EQ(second.status,"CONFIRMED");EXPECT_TRUE(std::filesystem::is_regular_file(file));
  listener.join();close(server);
  Store store(temp.directory/"queue");store.enqueue_upload("cap1",checksum,file,false);
  EXPECT_EQ(store.uploads()[0]["status"],"PENDING_CONFIGURATION");EXPECT_TRUE(store.next_upload().is_null());
  const auto queued=temp.directory/"cap2.jpg";{std::ofstream out(queued,std::ios::binary);out<<"jpeg-bytes";}
  store.enqueue_upload("cap2",checksum,queued,true);
  EXPECT_EQ(store.next_upload()["id"],"cap2");store.finish_upload("cap2","PENDING");
  EXPECT_EQ(store.next_upload()["id"],"cap2");store.finish_upload("cap2","CONFIRMED");
  EXPECT_TRUE(store.next_upload().is_null());EXPECT_TRUE(std::filesystem::is_regular_file(queued));
  EXPECT_EQ(store.uploads()[1]["status"],"CONFIRMED");
}
