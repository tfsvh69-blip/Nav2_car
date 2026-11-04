#include <gtest/gtest.h>
#include "carcar_mapping_console/inspection_runtime.hpp"
using namespace carcar_mapping_console;
TEST(InspectionRuntime, ResourceReservationPrecedesLaunchAndRejectsOldEnd) {
  char name[]="/tmp/carcar-runtime-policy-XXXXXX";std::filesystem::path root=mkdtemp(name);
  std::filesystem::create_directories(root/"maps/mapping_sessions");std::filesystem::create_directories(root/"logs");
  std::ofstream(root/"maps/map1.yaml")<<"测试地图";
  {
    InspectionRuntime runtime(root/"maps/mapping_sessions",root/"logs",(root/"data").string(),18881,false);
    InspectionRuntime::Request request;request.action="prepare";request.request_id="first";request.map_id="map1";
    InspectionRuntime::Response denied;runtime.request(request,denied,false);EXPECT_FALSE(denied.accepted);EXPECT_FALSE(runtime.occupied());
    InspectionRuntime::Response accepted;runtime.request(request,accepted,true);ASSERT_TRUE(accepted.accepted);EXPECT_TRUE(runtime.occupied());EXPECT_EQ(accepted.phase,"STARTING");
    InspectionRuntime::Response repeated;runtime.request(request,repeated,true);EXPECT_TRUE(repeated.accepted);EXPECT_EQ(accepted.generation,repeated.generation);
    request.request_id="other";InspectionRuntime::Response conflict;runtime.request(request,conflict,true);EXPECT_FALSE(conflict.accepted);
    request.action="end";request.generation="old";InspectionRuntime::Response stale;runtime.request(request,stale,true);EXPECT_FALSE(stale.accepted);EXPECT_TRUE(runtime.occupied());
    request.generation=accepted.generation;InspectionRuntime::Response stop;runtime.request(request,stop,true);EXPECT_TRUE(stop.accepted);EXPECT_EQ(stop.phase,"STOPPING");EXPECT_TRUE(runtime.occupied());
  }
  std::filesystem::remove_all(root);
}

TEST(InspectionRuntime, CompleteCandidateNeedsNoReviewButDiagnosticCannotStartVehicle) {
  const auto root=std::filesystem::temp_directory_path()/("carcar-runtime-saved-"+nonce());
  const auto directory=root/"maps/mapping_sessions/saved";std::filesystem::create_directories(directory);std::filesystem::create_directories(root/"logs");
  json manifest{{"complete",true},{"slam_version","2.6.10"},{"base_frame","base_footprint"},{"review_required",true}};
  for(const auto & name:{"map.yaml","map.pgm","graph.posegraph","graph.data"}){
    std::ofstream(directory/name)<<"fixture";manifest["sha256"][name]=sha256(directory/name);
  }
  atomic_json(directory/"manifest.json",manifest);atomic_json(directory/"publication.json",{{"status","candidate"},{"asset_sha256",manifest["sha256"]}});
  {
    // 不启动监督线程，本测试只核验车辆准备资格，不启动任何硬件进程。
    InspectionRuntime runtime(root/"maps/mapping_sessions",root/"logs",(root/"data").string(),18881,false);
    InspectionRuntime::Request request;request.action="prepare";request.request_id="candidate";request.map_id="saved";
    InspectionRuntime::Response accepted;runtime.request(request,accepted,true);EXPECT_TRUE(accepted.accepted)<<accepted.reason;
  }
  manifest["diagnostic_only"]=true;atomic_json(directory/"manifest.json",manifest);
  {
    InspectionRuntime runtime(root/"maps/mapping_sessions",root/"logs",(root/"data").string(),18881,false);
    InspectionRuntime::Request request;request.action="prepare";request.request_id="diagnostic";request.map_id="saved";
    InspectionRuntime::Response denied;runtime.request(request,denied,true);EXPECT_FALSE(denied.accepted);EXPECT_FALSE(runtime.occupied());
    EXPECT_NE(denied.reason.find("质量异常"),std::string::npos);
  }
  std::filesystem::remove_all(root);
}

TEST(InspectionRuntime, SupervisedProcessesReleaseResourcesAndRestartOnlyConsole) {
  char name[]="/tmp/carcar-runtime-process-XXXXXX";std::filesystem::path root=mkdtemp(name);
  std::filesystem::create_directories(root/"maps/mapping_sessions");std::filesystem::create_directories(root/"logs");std::filesystem::create_directories(root/"bin");
  std::ofstream(root/"maps/map1.yaml")<<"软件测试地图";
  // 子进程替身只休眠并记录自身PID，绝不调用ROS或打开设备。
  std::ofstream script(root/"bin/ros2");script<<"#!/bin/sh\ncase \"$*\" in\n*console.launch.xml*) echo $$ > \"$INSPECTION_STUB_ROOT/console.pid\";;\n*inspection_lidar.launch.xml*) echo $$ > \"$INSPECTION_STUB_ROOT/vehicle.pid\";;\n*) exit 2;;\nesac\nprintf '%s\\n' \"$@\"\nexec /bin/sleep 300\n";script.close();chmod((root/"bin/ros2").c_str(),0700);
  const std::string old_path=getenv("PATH");setenv("PATH",(root/"bin").c_str(),1);setenv("INSPECTION_STUB_ROOT",root.c_str(),1);
  auto wait=[](auto predicate){const auto until=steady()+5;while(steady()<until){if(predicate())return true;std::this_thread::sleep_for(std::chrono::milliseconds(30));}return false;};
  {
    InspectionRuntime runtime(root/"maps/mapping_sessions",root/"logs",(root/"data").string(),18881);
    ASSERT_TRUE(wait([&]{return std::filesystem::exists(root/"console.pid");}));
    InspectionRuntime::Request request;request.action="prepare";request.request_id="prepare";request.map_id="map1";
    InspectionRuntime::Response accepted;runtime.request(request,accepted,true);ASSERT_TRUE(accepted.accepted);
    ASSERT_TRUE(wait([&]{return runtime.state().phase=="ACTIVE";}));
    request.action="end";request.generation=accepted.generation;
    InspectionRuntime::Response ended;runtime.request(request,ended,true);ASSERT_TRUE(ended.accepted);
    ASSERT_TRUE(wait([&]{return !runtime.occupied();}));EXPECT_EQ(runtime.state().phase,"CONFIG_ONLY");
    request.action="prepare";request.request_id="second";InspectionRuntime::Response second;runtime.request(request,second,true);ASSERT_TRUE(second.accepted);
    ASSERT_TRUE(wait([&]{return runtime.state().phase=="ACTIVE";}));
    pid_t console_pid=-1;std::ifstream(root/"console.pid")>>console_pid;ASSERT_GT(console_pid,1);kill(console_pid,SIGTERM);
    ASSERT_TRUE(wait([&]{return !runtime.occupied();}));EXPECT_EQ(runtime.state().phase,"FAULT");
    // 退出故障后即使控制台重启，也不能自动恢复车辆。
    std::this_thread::sleep_for(std::chrono::milliseconds(300));EXPECT_FALSE(runtime.occupied());
  }
  setenv("PATH",old_path.c_str(),1);unsetenv("INSPECTION_STUB_ROOT");
  std::filesystem::remove_all(root);
}
