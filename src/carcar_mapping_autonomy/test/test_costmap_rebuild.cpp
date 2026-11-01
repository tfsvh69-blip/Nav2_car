#include <gtest/gtest.h>
#include <chrono>
#include <memory>
#include <thread>
#include "carcar_mapping_autonomy/costmap_rebuild.hpp"

namespace ca=carcar_mapping_autonomy;
namespace {
double seconds(){return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();}
class CostmapRebuildTest:public testing::Test {
protected:
  static void SetUpTestSuite(){int argc=0;rclcpp::init(argc,nullptr);}
  static void TearDownTestSuite(){rclcpp::shutdown();}
  void SetUp()override{
    n=std::make_shared<rclcpp::Node>("costmap_rebuild_fixture");
    const auto qos=rclcpp::QoS(1).reliable().transient_local();
    global=n->create_publisher<ca::CostmapRebuild::Costmap>("/global_costmap/costmap_raw",qos);
    local=n->create_publisher<ca::CostmapRebuild::Costmap>("/local_costmap/costmap_raw",qos);
    scans=n->create_publisher<sensor_msgs::msg::LaserScan>("/scan/filtered",rclcpp::SensorDataQoS());
    helper=std::make_unique<ca::CostmapRebuild>(*n);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
  void TearDown()override{helper.reset();global_service.reset();local_service.reset();scans.reset();global.reset();local.reset();n.reset();}
  void services(bool both=true){
    global_service=n->create_service<ca::CostmapRebuild::Clear>("/global_costmap/clear_entirely_global_costmap",
      [this](ca::CostmapRebuild::Clear::Request::SharedPtr,ca::CostmapRebuild::Clear::Response::SharedPtr){++global_clears;});
    if(both)local_service=n->create_service<ca::CostmapRebuild::Clear>("/local_costmap/clear_entirely_local_costmap",
      [this](ca::CostmapRebuild::Clear::Request::SharedPtr,ca::CostmapRebuild::Clear::Response::SharedPtr){++local_clears;});
  }
  void spin(){rclcpp::spin_some(n);helper->tick(seconds());}
  template<class Predicate>bool until(Predicate done,double timeout){
    const auto end=seconds()+timeout;
    while(seconds()<end){spin();if(done())return true;std::this_thread::sleep_for(std::chrono::milliseconds(5));}
    return done();
  }
  bool begin(){
    services();if(!helper->begin(seconds()))return false;
    return until([this]{return helper->state()["phase"]=="REBUILDING";},2);
  }
  sensor_msgs::msg::LaserScan scan(const rclcpp::Time&stamp){
    sensor_msgs::msg::LaserScan m;m.header.stamp=stamp;m.header.frame_id="laser_frame";
    m.angle_min=-1;m.angle_increment=.01;m.range_min=.05;m.range_max=8;m.ranges.assign(200,2);return m;
  }
  ca::CostmapRebuild::Costmap map(const rclcpp::Time&stamp,const char*frame){
    ca::CostmapRebuild::Costmap m;m.header.stamp=stamp;m.header.frame_id=frame;
    m.metadata.layer="master";m.metadata.resolution=.05;m.metadata.size_x=20;m.metadata.size_y=20;
    m.metadata.origin.orientation.w=1;m.data.assign(400,0);return m;
  }
  void observe(bool feed_global=true,bool feed_local=true,bool feed_scan=true){
    const auto stamp=n->now();if(feed_scan)scans->publish(scan(stamp));
    if(feed_global)global->publish(map(stamp,"map"));
    if(feed_local)local->publish(map(stamp,"odom"));
    spin();std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }
  bool rebuild(){
    if(!begin())return false;
    const auto end=seconds()+2;
    while(seconds()<end){observe();if(helper->ready())return true;}
    return helper->ready();
  }
  rclcpp::Node::SharedPtr n;
  std::unique_ptr<ca::CostmapRebuild>helper;
  rclcpp::Publisher<ca::CostmapRebuild::Costmap>::SharedPtr global,local;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scans;
  rclcpp::Service<ca::CostmapRebuild::Clear>::SharedPtr global_service,local_service;
  int global_clears=0,local_clears=0;
};
TEST_F(CostmapRebuildTest,ConstructionDoesNotClearAndMissingServiceFailsBoundedly){
  services(false);spin();EXPECT_EQ(global_clears,0);EXPECT_EQ(local_clears,0);
  const auto started=seconds();ASSERT_TRUE(helper->begin(started));spin();helper->tick(started+5.01);
  EXPECT_TRUE(helper->failed());EXPECT_FALSE(helper->ready());EXPECT_EQ(global_clears,0);
  EXPECT_NE(helper->state()["error"].get<std::string>().find("服务"),std::string::npos);
}
TEST_F(CostmapRebuildTest,BothClearsNewScansAndRawMapsAreRequired){
  ASSERT_TRUE(begin());EXPECT_EQ(global_clears,1);EXPECT_EQ(local_clears,1);
  EXPECT_FALSE(helper->begin(seconds()))<<"重复begin不能再次清理";
  const auto end=seconds()+1.15;while(seconds()<end)observe(true,false);
  EXPECT_FALSE(helper->ready());EXPECT_FALSE(helper->state()["local"]["qualified"]);
  ASSERT_TRUE(until([this]{observe();return helper->ready();},.5));
  EXPECT_EQ(global_clears,1);EXPECT_EQ(local_clears,1);
  EXPECT_FALSE(helper->state()["global"]["update_time_available"])<<"Humble raw的0字段不伪装有效";
}
TEST_F(CostmapRebuildTest,FixedThirdScanThresholdAllowsSlowCostmapsWithContinuousScans){
  ASSERT_TRUE(begin());
  for(int i=0;i<5;++i)observe();
  const auto fixed=helper->state()["minimum_required_stamp_ns"].get<int64_t>();ASSERT_GT(fixed,0);
  const auto end=seconds()+1.2;
  while(seconds()<end){observe(false,false);if(int(seconds()*10)%3==0)observe();}
  EXPECT_EQ(helper->state()["minimum_required_stamp_ns"].get<int64_t>(),fixed);
  ASSERT_TRUE(until([this]{observe();return helper->ready();},.5));
  EXPECT_GT(helper->state()["latest_scan_stamp_ns"].get<int64_t>(),fixed);
}
TEST_F(CostmapRebuildTest,OldAndMalformedMessagesCannotProveRebuild){
  ASSERT_TRUE(begin());const auto stale=n->now()-rclcpp::Duration::from_seconds(2);
  for(int i=0;i<5;++i){scans->publish(scan(stale));global->publish(map(stale,"map"));local->publish(map(stale,"odom"));spin();}
  EXPECT_EQ(helper->state()["scan_count"],0);EXPECT_FALSE(helper->ready());
  auto invalid=scan(n->now());invalid.ranges.assign(200,std::numeric_limits<float>::quiet_NaN());scans->publish(invalid);
  auto wrong=map(n->now(),"map");wrong.data.pop_back();global->publish(wrong);spin();
  EXPECT_EQ(helper->state()["scan_count"],0);EXPECT_EQ(helper->global_costmap(),nullptr);
  const auto end=seconds()+1.15;while(seconds()<end)observe();ASSERT_TRUE(helper->ready());
  const auto good_stamp=helper->global_costmap()->header.stamp;
  global->publish(map(stale,"map"));spin();EXPECT_EQ(helper->global_costmap()->header.stamp,good_stamp);
}
TEST_F(CostmapRebuildTest,ReadyExpiresOnScanSilenceAndRecoversWithoutSecondClear){
  ASSERT_TRUE(rebuild());const auto end=seconds()+.65;while(seconds()<end)observe(true,true,false);
  EXPECT_FALSE(helper->ready());EXPECT_EQ(helper->state()["phase"],"READY");
  ASSERT_TRUE(until([this]{observe();return helper->ready();},.5));
  EXPECT_EQ(global_clears,1);EXPECT_EQ(local_clears,1);
}
TEST_F(CostmapRebuildTest,ReadyExpiresOnOneCostmapSilenceAndRecoversWithoutSecondClear){
  ASSERT_TRUE(rebuild());const auto end=seconds()+1.15;while(seconds()<end)observe(true,false,true);
  EXPECT_FALSE(helper->ready());EXPECT_FALSE(helper->state()["local"]["qualified"]);
  ASSERT_TRUE(until([this]{observe();return helper->ready();},.5));
  EXPECT_EQ(global_clears,1);EXPECT_EQ(local_clears,1);
}
TEST_F(CostmapRebuildTest,ExplicitOldUpdateTimestampIsNotAcceptedAsNewUpdate){
  ASSERT_TRUE(begin());const auto old=n->now()-rclcpp::Duration::from_seconds(2);
  const auto end=seconds()+1.15;
  while(seconds()<end){const auto stamp=n->now();scans->publish(scan(stamp));auto m=map(stamp,"map");m.metadata.update_time=old;global->publish(m);local->publish(map(stamp,"odom"));spin();std::this_thread::sleep_for(std::chrono::milliseconds(20));}
  EXPECT_FALSE(helper->ready());EXPECT_TRUE(helper->state()["global"]["update_time_available"]);
  ASSERT_TRUE(until([this]{observe();return helper->ready();},.5));
}
TEST_F(CostmapRebuildTest,ExpiredTransactionCannotBeRevivedByLateData){
  ASSERT_TRUE(begin());const auto start=helper->state()["started_at"].get<double>();helper->tick(start+5.1);
  EXPECT_TRUE(helper->failed());for(int i=0;i<5;++i)observe();EXPECT_FALSE(helper->ready());
  EXPECT_EQ(helper->state()["phase"],"FAILED");
}
TEST_F(CostmapRebuildTest,MissingOneClearReplyTimesOutAndLateReplyCannotReviveTransaction){
  services(false);std::shared_ptr<rmw_request_id_t> deferred;
  local_service=n->create_service<ca::CostmapRebuild::Clear>("/local_costmap/clear_entirely_local_costmap",
    [this,&deferred](std::shared_ptr<rmw_request_id_t> header,ca::CostmapRebuild::Clear::Request::SharedPtr){
      ++local_clears;deferred=header;
    });
  const auto began=seconds();ASSERT_TRUE(helper->begin(began));
  ASSERT_TRUE(until([&]{return deferred&&helper->state()["global_clear_done"].get<bool>();},2));
  EXPECT_EQ(helper->state()["phase"],"CLEARING");EXPECT_FALSE(helper->ready());
  helper->tick(began+5.01);ASSERT_TRUE(helper->failed());
  ca::CostmapRebuild::Clear::Response response;local_service->send_response(*deferred,response);
  for(int i=0;i<5;++i)observe();
  EXPECT_FALSE(helper->ready());EXPECT_EQ(helper->state()["phase"],"FAILED");
  EXPECT_EQ(global_clears,1);EXPECT_EQ(local_clears,1);
}
}
