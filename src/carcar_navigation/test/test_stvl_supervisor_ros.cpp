#include <gtest/gtest.h>
#include <csignal>
#include <sys/wait.h>
#include <unistd.h>
#include <thread>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include "carcar_navigation/navigation_motion_gate.hpp"

using namespace std::chrono_literals;
TEST(StvlSupervisorRos, RequiredModeStopsAndRecoversWithoutReplayingCachedCommands) {
  rclcpp::init(0,nullptr);
  const pid_t child=fork();
  ASSERT_GE(child,0);
  if(child==0) {
    execl(SUPERVISOR_EXECUTABLE,SUPERVISOR_EXECUTABLE,"--ros-args","-p",
      "mode:=required_stvl",static_cast<char *>(nullptr));
    _exit(127);
  }
  struct ChildGuard {pid_t pid; ~ChildGuard(){kill(pid,SIGINT);waitpid(pid,nullptr,0);}} guard{child};
  auto producer=std::make_shared<rclcpp::Node>("stvl_test_inputs");
  auto gate=std::make_shared<NavigationMotionGate>(rclcpp::NodeOptions().parameter_overrides(
    std::vector<rclcpp::Parameter>{{"require_perception_health",true}}));
  auto health=producer->create_publisher<std_msgs::msg::Bool>(
    "/camera/navigation/healthy",rclcpp::QoS(1).reliable().transient_local());
  auto cloud=producer->create_publisher<sensor_msgs::msg::PointCloud2>(
    "/camera/navigation/obstacles",rclcpp::SensorDataQoS());
  auto local=producer->create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
    "/local_costmap/depth_stvl/diagnostics",1);
  auto global=producer->create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
    "/global_costmap/depth_stvl/diagnostics",1);
  auto permit=producer->create_publisher<std_msgs::msg::UInt64>("/navigation/motion_permit",1);
  auto command=producer->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel_smoothed_raw",1);
  bool allowed=false; double speed=0;size_t allowed_count=0;uint64_t sequence=0;
  auto a=producer->create_subscription<std_msgs::msg::Bool>("/navigation/perception_motion_allowed",
    rclcpp::QoS(1).reliable().transient_local(),[&](std_msgs::msg::Bool::ConstSharedPtr m){allowed=m->data;++allowed_count;});
  auto v=producer->create_subscription<geometry_msgs::msg::Twist>("/cmd_vel",1,
    [&](geometry_msgs::msg::Twist::ConstSharedPtr m){speed=m->linear.x;});
  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(producer);exec.add_node(gate);
  auto stamp=producer->now();
  auto pump=[&](double seconds,bool good,bool moving,bool frozen=false,int fault=0) {
    auto end=std::chrono::steady_clock::now()+std::chrono::duration<double>(seconds);
    auto next=std::chrono::steady_clock::now();
    while(std::chrono::steady_clock::now()<end) {
      if(std::chrono::steady_clock::now()>=next) {
        std_msgs::msg::Bool h;h.data=true;if(fault!=1) {health->publish(h);}
        if(!frozen) {stamp=producer->now();}
        sensor_msgs::msg::PointCloud2 c;c.header.stamp=fault==5 ? producer->now()-rclcpp::Duration::from_seconds(2.0) : stamp;
        c.header.frame_id="test_optical";if(fault!=2) {cloud->publish(c);}
        diagnostic_msgs::msg::DiagnosticArray d;d.header.stamp=producer->now();
        diagnostic_msgs::msg::DiagnosticStatus s;s.name="depth_stvl";
        diagnostic_msgs::msg::KeyValue current;current.key="source_current";current.value=good?"true":"false";
        diagnostic_msgs::msg::KeyValue seq;seq.key="update_seq";seq.value=std::to_string(++sequence);
        s.values={current,seq};d.status={s};
        if(fault!=3) {local->publish(d);}
        if(fault!=4) {global->publish(d);}
        if(moving) {
          std_msgs::msg::UInt64 p;p.data=123;permit->publish(p);
          geometry_msgs::msg::Twist twist;twist.linear.x=0.1;command->publish(twist);
        }
        next+=50ms;
      }
      exec.spin_some();std::this_thread::sleep_for(5ms);
    }
  };
  pump(0.5,false,true); EXPECT_GT(allowed_count,0U); EXPECT_FALSE(allowed);EXPECT_DOUBLE_EQ(speed,0);
  pump(3.5,true,true); EXPECT_TRUE(allowed);EXPECT_DOUBLE_EQ(speed,0.1);
  pump(0.2,false,true); EXPECT_FALSE(allowed);EXPECT_DOUBLE_EQ(speed,0);
  pump(3.5,true,false); EXPECT_TRUE(allowed);EXPECT_DOUBLE_EQ(speed,0);
  pump(0.2,true,true); EXPECT_DOUBLE_EQ(speed,0.1);
  pump(1.1,true,true,true); EXPECT_FALSE(allowed);EXPECT_DOUBLE_EQ(speed,0);
  pump(3.5,true,true); EXPECT_TRUE(allowed);EXPECT_DOUBLE_EQ(speed,0.1);
  // 单独中断健康心跳、点云、本地图、全局图，以及注入过期采样时间。
  for(int fault=1;fault<=5;++fault) {
    pump(1.1,true,true,false,fault);
    EXPECT_FALSE(allowed) << "fault=" << fault;
    EXPECT_DOUBLE_EQ(speed,0) << "fault=" << fault;
    pump(3.5,true,true);
    EXPECT_TRUE(allowed) << "recovery fault=" << fault;
    EXPECT_DOUBLE_EQ(speed,0.1);
  }
  kill(child,SIGTERM);
  pump(0.4,true,true); EXPECT_DOUBLE_EQ(speed,0);
  exec.remove_node(gate);exec.remove_node(producer);
  gate->stop();rclcpp::shutdown();
}
