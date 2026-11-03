#include <gtest/gtest.h>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <functional>
#include <limits>
#include <thread>
#include <vector>
#include "carcar_mapping_autonomy/backup_motion.hpp"

namespace ca=carcar_mapping_autonomy;
using Backup=nav2_msgs::action::BackUp;
using ServerHandle=rclcpp_action::ServerGoalHandle<Backup>;
using ServerHandlePtr=std::shared_ptr<ServerHandle>;
class BackupMotionTest:public testing::Test {
protected:
  static void SetUpTestSuite(){
    ASSERT_NE(std::getenv("ROS_DOMAIN_ID"),nullptr);ASSERT_STREQ(std::getenv("ROS_DOMAIN_ID"),"224");
    ASSERT_NE(std::getenv("ROS_LOCALHOST_ONLY"),nullptr);ASSERT_STREQ(std::getenv("ROS_LOCALHOST_ONLY"),"1");
    rclcpp::init(0,nullptr);
  }
  static void TearDownTestSuite(){rclcpp::shutdown();}
  rclcpp::Node::SharedPtr io,node;
  rclcpp_action::Server<Backup>::SharedPtr server;
  rclcpp::executors::SingleThreadedExecutor executor;
  std::unique_ptr<ca::BackupMotion> motion;
  std::vector<ServerHandlePtr> accepted;
  bool accept_goals=true,accept_cancel=true,finish_cancel=true;
  ca::Pose wheel;
  void SetUp()override{
    io=std::make_shared<rclcpp::Node>("backup_fixture");node=std::make_shared<rclcpp::Node>("backup_supervisor_fixture");
    server=rclcpp_action::create_server<Backup>(io,"/backup",
      [this](const rclcpp_action::GoalUUID&,std::shared_ptr<const Backup::Goal>){
        return accept_goals?rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE:rclcpp_action::GoalResponse::REJECT;},
      [this](const ServerHandlePtr){
        return accept_cancel?rclcpp_action::CancelResponse::ACCEPT:rclcpp_action::CancelResponse::REJECT;},
      [this](const ServerHandlePtr handle){accepted.push_back(handle);});
    executor.add_node(io);executor.add_node(node);motion=std::make_unique<ca::BackupMotion>(*node);
    ASSERT_TRUE(spin_until([this]{return motion->available();}));
  }
  void TearDown()override{
    if(motion){motion->cancel("测试收尾");spin_for(.08);motion.reset();spin_for(.03);}
    server.reset();accepted.clear();executor.remove_node(node);executor.remove_node(io);node.reset();io.reset();
  }
  void pump(){
    executor.spin_some();
    if(finish_cancel)for(const auto&handle:accepted)if(handle->is_canceling())
      handle->canceled(std::make_shared<Backup::Result>());
  }
  void spin_for(double seconds){
    const auto end=std::chrono::steady_clock::now()+std::chrono::duration<double>(seconds);
    while(std::chrono::steady_clock::now()<end){pump();std::this_thread::sleep_for(std::chrono::milliseconds(2));}
  }
  bool spin_until(const std::function<bool()>&predicate,double seconds=2){
    const auto end=std::chrono::steady_clock::now()+std::chrono::duration<double>(seconds);
    while(std::chrono::steady_clock::now()<end){pump();if(predicate())return true;std::this_thread::sleep_for(std::chrono::milliseconds(2));}
    return predicate();
  }
  void start(double distance=.08){
    ASSERT_TRUE(motion->start(distance,.03,10,wheel,100,"request-backup"));
    ASSERT_TRUE(spin_until([this]{return motion->status().value("phase","")=="MOVING";}));
    ASSERT_EQ(accepted.size(),1U);
  }
  void succeed_server(){accepted.back()->succeed(std::make_shared<Backup::Result>());
    ASSERT_TRUE(spin_until([this]{return motion->status().value("result_received",false);}));}
};

TEST_F(BackupMotionTest, RejectsUnsafeParametersWithoutSubmittingAction){
  const auto nan=std::numeric_limits<double>::quiet_NaN();
  for(const auto distance:{-.01,0.,.251,nan})EXPECT_FALSE(motion->start(distance,.03,10,wheel,100,"invalid"));
  for(const auto speed:{-.01,0.,.031,nan})EXPECT_FALSE(motion->start(.08,speed,10,wheel,100,"invalid"));
  for(const auto budget:{0.,31.,nan})EXPECT_FALSE(motion->start(.08,.03,budget,wheel,100,"invalid"));
  EXPECT_FALSE(motion->start(.08,.03,10,{nan,0,0},100,"invalid"));
  EXPECT_FALSE(motion->start(.08,.03,10,wheel,nan,"invalid"));
  EXPECT_FALSE(motion->start(.08,.03,10,wheel,100,""));
  spin_for(.03);EXPECT_TRUE(accepted.empty());EXPECT_TRUE(motion->terminal());
}
TEST_F(BackupMotionTest, UsesNav2BackupContractAndNeverPublishesVelocity){
  start();const auto goal=accepted.front()->get_goal();
  EXPECT_DOUBLE_EQ(goal->target.x,-.08);EXPECT_EQ(goal->target.y,0);EXPECT_EQ(goal->target.z,0);
  EXPECT_FLOAT_EQ(goal->speed,.03);EXPECT_EQ(goal->time_allowance.sec,10);
  EXPECT_TRUE(motion->active());EXPECT_FALSE(motion->terminal());
  EXPECT_EQ(node->count_publishers("/mapping/cmd_vel"),0U);
  EXPECT_EQ(node->count_publishers("/mapping/nav_raw"),0U);
}
TEST_F(BackupMotionTest, SuccessRequiresRealResultFreshWheelDistanceAndStill){
  start();motion->tick(100.5,{-.08,0,0},true,true);
  EXPECT_FALSE(motion->succeeded());EXPECT_FALSE(motion->terminal());
  succeed_server();EXPECT_EQ(motion->status()["phase"],"VERIFYING");EXPECT_FALSE(motion->terminal());
  motion->tick(100.6,{-.08,0,0},true,false);EXPECT_FALSE(motion->succeeded());
  motion->tick(100.7,{-.08,0,0},true,true);EXPECT_TRUE(motion->succeeded());
  EXPECT_TRUE(motion->terminal());EXPECT_FALSE(motion->active());EXPECT_NEAR(motion->distance_progress(),.08,1e-10);
}
TEST_F(BackupMotionTest, FalseServerSuccessWithoutWheelMotionFailsVerification){
  start();succeed_server();motion->tick(100.2,wheel,true,true);EXPECT_FALSE(motion->succeeded());
  motion->tick(102.1,wheel,true,true);EXPECT_TRUE(motion->failed());EXPECT_TRUE(motion->terminal());
  EXPECT_NE(motion->status().value("reason","").find("轮式距离"),std::string::npos);
}
TEST_F(BackupMotionTest, CancellationAfterSuccessKeepsTrackingUntilWheelStill){
  start();motion->tick(100.2,{-.020,0,0},true,false);succeed_server();
  motion->tick(100.3,{-.025,0,0},true,false);
  // 真实成功结果先到，但轮式尚未停稳；后续取消不能漏记制动滑行。
  motion->cancel("成功核验期间安全条件变化");
  EXPECT_TRUE(motion->terminal());EXPECT_TRUE(motion->failed());EXPECT_FALSE(motion->succeeded());
  motion->tick(100.4,{-.030,0,0},true,false);
  EXPECT_NEAR(motion->distance_progress(),.030,1e-10);
  motion->tick(100.7,{-.035,0,0},true,true);
  EXPECT_NEAR(motion->distance_progress(),.035,1e-10);
  // 已核对停稳后封存本轮距离，后续无关轮式位移不计入这次后退。
  motion->tick(101.0,{-.050,0,0},true,true);
  EXPECT_NEAR(motion->distance_progress(),.035,1e-10);
}
TEST_F(BackupMotionTest, VerificationTimeoutKeepsTrackingBrakingUntilWheelStill){
  start();motion->tick(100.1,{-.080,0,0},true,false);succeed_server();
  // 已达到距离而未停稳，核验超时只能宣告失败，不能提前冻结实际行程。
  motion->tick(102.2,{-.082,0,0},true,false);
  EXPECT_TRUE(motion->terminal());EXPECT_TRUE(motion->failed());EXPECT_FALSE(motion->succeeded());
  motion->tick(102.4,{-.088,0,0},true,false);
  EXPECT_NEAR(motion->distance_progress(),.088,1e-10);
  motion->tick(102.6,{-.090,0,0},true,true);
  EXPECT_NEAR(motion->distance_progress(),.090,1e-10);
  motion->tick(102.8,{-.120,0,0},true,true);
  EXPECT_NEAR(motion->distance_progress(),.090,1e-10);
}
TEST_F(BackupMotionTest, ProtocolTerminalCannotBeReplacedUntilWheelStill){
  start();motion->tick(100.1,{-.020,0,0},true,false);motion->cancel("取消后等待制动完成");
  ASSERT_TRUE(spin_until([this]{return motion->terminal();}));
  // 真实取消结果只结束协议，上一轮仍在滑行时不能被新请求覆盖。
  ASSERT_FALSE(motion->start(.01,.02,2,{-.020,0,0},100.2,"while-coasting"));
  motion->tick(100.3,{-.030,0,0},true,false);
  motion->tick(100.6,{-.040,0,0},true,true);
  EXPECT_NEAR(motion->distance_progress(),.040,1e-10);
  EXPECT_TRUE(motion->start(.01,.02,2,{-.040,0,0},100.7,"after-still"));
}
TEST_F(BackupMotionTest, StaleWheelFeedbackCancelsInsteadOfBlindRetreat){
  start();motion->tick(100.1,wheel,false,true);EXPECT_TRUE(motion->failed());
  ASSERT_TRUE(spin_until([this]{return motion->terminal();}));EXPECT_FALSE(motion->succeeded());
  EXPECT_EQ(motion->status()["result"],"CANCELED");
}
TEST_F(BackupMotionTest, ForwardMotionIsNotCountedAsBackupProgress){
  start();motion->tick(100.1,{.006,0,0},true,false);EXPECT_TRUE(motion->failed());
  EXPECT_LT(motion->distance_progress(),0);ASSERT_TRUE(spin_until([this]{return motion->terminal();}));
}
TEST_F(BackupMotionTest, SideDriftStopsStraightRetreat){
  start();motion->tick(100.1,{-.01,.011,0},true,false);EXPECT_TRUE(motion->failed());
  EXPECT_NE(motion->status().value("reason","").find("横向"),std::string::npos);
}
TEST_F(BackupMotionTest, YawDriftStopsStraightRetreat){
  start();motion->tick(100.1,{-.01,0,.051},true,false);EXPECT_TRUE(motion->failed());
  EXPECT_NE(motion->status().value("reason","").find("朝向"),std::string::npos);
}
TEST_F(BackupMotionTest, OvershootStopsRatherThanClaimingExtraProgressIsSuccess){
  start();motion->tick(100.1,{-.096,0,0},true,false);EXPECT_TRUE(motion->failed());EXPECT_FALSE(motion->succeeded());
}
TEST_F(BackupMotionTest, NoProgressUsesSteadyTwoSecondBudget){
  start();motion->tick(101.9,wheel,true,true);EXPECT_FALSE(motion->failed());
  motion->tick(102.01,wheel,true,true);EXPECT_TRUE(motion->failed());
  EXPECT_NE(motion->status().value("reason","").find("没有有效后退进展"),std::string::npos);
}
TEST_F(BackupMotionTest, SmallRealProgressRefreshesStallBudget){
  start();motion->tick(101.5,{-.002,0,0},true,false);
  motion->tick(103,{-.004,0,0},true,false);EXPECT_FALSE(motion->failed());
}
TEST_F(BackupMotionTest, CancellationBeforeLateAcceptanceCancelsTheAcceptedGoal){
  ASSERT_TRUE(motion->start(.08,.03,10,wheel,100,"cancel-before-accept"));
  motion->cancel("操作者暂停");EXPECT_FALSE(motion->terminal());EXPECT_TRUE(motion->failed());
  ASSERT_TRUE(spin_until([this]{return motion->terminal();}));
  ASSERT_EQ(accepted.size(),1U);EXPECT_EQ(motion->status()["result"],"CANCELED");EXPECT_FALSE(motion->succeeded());
}
TEST_F(BackupMotionTest, CancellationTimeoutNeverFabricatesTerminalState){
  finish_cancel=false;start();motion->cancel("操作者暂停");spin_for(.08);
  motion->tick(105.1,wheel,true,true);EXPECT_TRUE(motion->failed());EXPECT_FALSE(motion->terminal());
  EXPECT_EQ(motion->status()["phase"],"FAULT");EXPECT_TRUE(motion->status().value("fault",false));
  EXPECT_FALSE(motion->start(.01,.02,2,wheel,106,"second"));
  ASSERT_TRUE(accepted.front()->is_canceling());accepted.front()->canceled(std::make_shared<Backup::Result>());
  ASSERT_TRUE(spin_until([this]{return motion->terminal();}));
  EXPECT_TRUE(motion->failed());EXPECT_EQ(motion->status()["phase"],"FAULT");EXPECT_FALSE(motion->succeeded());
}
TEST_F(BackupMotionTest, LateAcceptanceAfterTimeoutIsCanceledAndNotReused){
  ASSERT_TRUE(motion->start(.08,.03,10,wheel,100,"late-goal"));
  motion->tick(102.1,wheel,true,true);EXPECT_TRUE(motion->failed());EXPECT_TRUE(motion->status().value("goal_pending",false));
  motion->tick(107.2,wheel,true,true);EXPECT_EQ(motion->status()["phase"],"FAULT");EXPECT_FALSE(motion->terminal());
  ASSERT_TRUE(spin_until([this]{return motion->terminal();}));EXPECT_EQ(motion->status()["result"],"CANCELED");
  EXPECT_EQ(motion->status()["phase"],"FAULT");
}
TEST_F(BackupMotionTest, RejectingCancelIsNotARealMotionTerminal){
  accept_cancel=false;start();motion->cancel("取消后退");spin_for(.08);
  EXPECT_FALSE(motion->terminal());EXPECT_TRUE(motion->failed());
  accepted.front()->abort(std::make_shared<Backup::Result>());
  ASSERT_TRUE(spin_until([this]{return motion->terminal();}));EXPECT_FALSE(motion->succeeded());
}
TEST_F(BackupMotionTest, LateSuccessfulResultCannotUndoOperatorCancellation){
  accept_cancel=false;start();motion->cancel("操作者终止");spin_for(.08);succeed_server();
  motion->tick(100.1,{-.08,0,0},true,true);EXPECT_TRUE(motion->failed());
  EXPECT_TRUE(motion->terminal());EXPECT_FALSE(motion->succeeded());
}
TEST_F(BackupMotionTest, RejectedGoalIsTerminalWithoutPretendingItWasExecuted){
  accept_goals=false;ASSERT_TRUE(motion->start(.08,.03,10,wheel,100,"reject"));
  ASSERT_TRUE(spin_until([this]{return motion->terminal();}));EXPECT_TRUE(motion->failed());
  EXPECT_FALSE(motion->status().value("result_received",true));EXPECT_TRUE(accepted.empty());
}
TEST_F(BackupMotionTest, DuplicateRequestAndRequestDuringVerificationAreRejected){
  start();EXPECT_FALSE(motion->start(.01,.02,2,wheel,100.1,"duplicate"));succeed_server();
  EXPECT_FALSE(motion->start(.01,.02,2,wheel,100.2,"while-verifying"));
  motion->tick(100.3,{-.08,0,0},true,true);EXPECT_TRUE(motion->succeeded());
  EXPECT_TRUE(motion->start(.01,.02,2,{-.08,0,0},100.4,"next"));
}
TEST_F(BackupMotionTest, MillimeterScaleDistanceHasNoLargeFixedMinimum){
  start(.001);succeed_server();motion->tick(100.1,{-.001,0,0},true,true);
  EXPECT_TRUE(motion->succeeded());EXPECT_TRUE(motion->terminal());
}
TEST_F(BackupMotionTest, WheelProjectionWorksAtArbitraryInitialYaw){
  wheel={1,2,M_PI/2};start();motion->tick(100.3,{1,1.92,M_PI/2},true,false);succeed_server();
  motion->tick(100.4,{1,1.92,M_PI/2},true,true);EXPECT_TRUE(motion->succeeded());
  EXPECT_NEAR(motion->distance_progress(),.08,1e-10);
}
TEST_F(BackupMotionTest, InvalidClockLocksUntilRealActionResult){
  start();motion->tick(99,wheel,true,true);EXPECT_TRUE(motion->failed());
  EXPECT_EQ(motion->status()["phase"],"FAULT");
  ASSERT_TRUE(spin_until([this]{return motion->terminal();}));EXPECT_FALSE(motion->succeeded());
}
TEST_F(BackupMotionTest, TimeBudgetIsIndependentOfServerRosClock){
  ASSERT_TRUE(motion->start(.08,.03,.5,wheel,100,"time-budget"));
  ASSERT_TRUE(spin_until([this]{return !accepted.empty();}));
  motion->tick(100.6,{-.005,0,0},true,false);EXPECT_TRUE(motion->failed());
  EXPECT_NE(motion->status().value("reason","").find("时间预算"),std::string::npos);
}
TEST_F(BackupMotionTest, DestructionWithPendingAcceptanceStillCancelsItsLateGoal){
  ASSERT_TRUE(motion->start(.08,.03,10,wheel,100,"destroy-pending"));motion.reset();
  ASSERT_TRUE(spin_until([this]{return accepted.size()==1&&accepted.front()->is_canceling();},.5)||
    (accepted.size()==1&&!accepted.front()->is_active()));
  spin_for(.05);ASSERT_EQ(accepted.size(),1U);EXPECT_FALSE(accepted.front()->is_active());
}
