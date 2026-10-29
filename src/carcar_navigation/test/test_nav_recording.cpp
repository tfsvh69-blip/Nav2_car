// 真实日志节点录包回归：只在CTest隔离域216运行，不连接硬件或发布速度。
#include <chrono>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <string>
#include <thread>
#include <vector>
#include <spawn.h>
#include <signal.h>
#include <fcntl.h>
#include <sys/wait.h>
#include <unistd.h>
#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_msgs/msg/tf_message.hpp>
#include <rosbag2_cpp/reader.hpp>

using namespace std::chrono_literals;
extern char ** environ;
namespace {
struct Process {
  pid_t pid{-1};
  bool stop() {
    if (pid<=0) {return true;}
    kill(pid,SIGINT);
    for (int i=0;i<150;++i) {
      int status=0;
      if (waitpid(pid,&status,WNOHANG)==pid) {
        pid=-1;return WIFEXITED(status)&&WEXITSTATUS(status)==0;
      }
      std::this_thread::sleep_for(20ms);
    }
    kill(pid,SIGKILL);waitpid(pid,nullptr,0);pid=-1;return false;
  }
  ~Process() {stop();}
};
}
class NavRecording : public testing::TestWithParam<bool> {};
TEST_P(NavRecording, StaticTransformsAreLatchedAndPresentInClosedBag)
{
  ASSERT_NE(std::getenv("ROS_DOMAIN_ID"),nullptr);
  ASSERT_STREQ(std::getenv("ROS_DOMAIN_ID"),"216");
  ASSERT_NE(std::getenv("ROS_LOCALHOST_ONLY"),nullptr);
  ASSERT_STREQ(std::getenv("ROS_LOCALHOST_ONLY"),"1");
  int argc=0;char ** argv=nullptr;rclcpp::init(argc,argv);
  struct Shutdown {~Shutdown() {rclcpp::shutdown();}} shutdown;
  char directory[]="/tmp/carcar-recording-qos-XXXXXX";
  ASSERT_NE(mkdtemp(directory),nullptr);
  const std::filesystem::path root(directory);
  auto node=std::make_shared<rclcpp::Node>("recording_test_observer");
  geometry_msgs::msg::TransformStamped transform;transform.header.stamp=node->now();
  transform.header.frame_id="recording_test_base";transform.child_frame_id="recording_test_laser";
  transform.transform.translation.x=-.05;transform.transform.rotation.w=1;
  std::unique_ptr<tf2_ros::StaticTransformBroadcaster> broadcaster;
  auto publish_once=[&]() {
    broadcaster=std::make_unique<tf2_ros::StaticTransformBroadcaster>(node);
    broadcaster->sendTransform(transform); // 每个用例只发布一次，禁止周期重发掩盖丢失。
  };
  if(GetParam()) {publish_once();}
  Process process;
  std::vector<std::string>args={NAV_LOGGER_EXECUTABLE,"--ros-args","-p",
    "sessions_base_dir:="+root.string(),"-p","record_raw_bag:=true"};
  std::vector<char*>raw;for(auto&arg:args) {raw.push_back(arg.data());}raw.push_back(nullptr);
  posix_spawn_file_actions_t actions;posix_spawn_file_actions_init(&actions);
  posix_spawn_file_actions_addopen(&actions,STDOUT_FILENO,(root/"console.log").c_str(),O_WRONLY|O_CREAT|O_TRUNC,0600);
  posix_spawn_file_actions_adddup2(&actions,STDOUT_FILENO,STDERR_FILENO);
  const int spawned=posix_spawn(&process.pid,raw[0],&actions,nullptr,raw.data(),environ);
  posix_spawn_file_actions_destroy(&actions);ASSERT_EQ(spawned,0);
  bool found=false,latched=false;
  for(int i=0;i<100&&!found;++i) {
    for(const auto&info:node->get_subscriptions_info_by_topic("/tf_static")) {
      if(info.node_name()=="nav_bag_recorder") {
        found=true;latched=info.qos_profile().durability()==rclcpp::DurabilityPolicy::TransientLocal;
      }
    }
    if(!found) {std::this_thread::sleep_for(50ms);}
  }
  ASSERT_TRUE(found)<<root;
  EXPECT_TRUE(latched)<<"录包先于发布者启动时，/tf_static必须请求transient_local；证据目录 "<<root;
  if(!GetParam()) {publish_once();}
  std::this_thread::sleep_for(1500ms);
  EXPECT_TRUE(process.stop())<<root;
  std::filesystem::path bag;
  for(const auto&entry:std::filesystem::directory_iterator(root)) {
    if(entry.is_directory()&&std::filesystem::exists(entry.path()/"bag"/"metadata.yaml")) {bag=entry.path()/"bag";}
  }
  ASSERT_FALSE(bag.empty())<<root;
  rosbag2_cpp::Reader reader;reader.open(bag.string());bool correct_transform=false;
  while(reader.has_next()) {
    auto message=reader.read_next();
    if(message->topic_name!="/tf_static") {continue;}
    rclcpp::SerializedMessage bytes(*message->serialized_data);tf2_msgs::msg::TFMessage decoded;
    rclcpp::Serialization<tf2_msgs::msg::TFMessage>().deserialize_message(&bytes,&decoded);
    for(const auto&value:decoded.transforms) {
      if(value.header.frame_id==transform.header.frame_id&&value.child_frame_id==transform.child_frame_id&&
        value.transform.translation.x==transform.transform.translation.x) {correct_transform=true;}
    }
  }
  EXPECT_TRUE(correct_transform)<<"真实关闭录包必须包含本轮一次性静态TF；证据目录 "<<root;
}
INSTANTIATE_TEST_SUITE_P(PublisherStartupOrder,NavRecording,testing::Bool(),
  [](const testing::TestParamInfo<bool>&info) {return info.param?"PublisherBeforeRecorder":"PublisherAfterRecorder";});
