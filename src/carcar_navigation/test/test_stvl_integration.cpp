#include <gtest/gtest.h>
#include <chrono>
#include <functional>
#include <cmath>
#include <thread>
#include <nav2_costmap_2d/layered_costmap.hpp>
#include <nav2_costmap_2d/costmap_layer.hpp>
#include <nav2_util/lifecycle_node.hpp>
#include <pluginlib/class_loader.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/create_timer_ros.h>
#include <tf2_ros/transform_listener.h>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>

using namespace std::chrono_literals;
namespace {
struct Point {float x,y,z;};
class StvlIntegration : public testing::TestWithParam<double> {
protected:
  static void SetUpTestSuite() {rclcpp::init(0,nullptr);}
  static void TearDownTestSuite() {rclcpp::shutdown();}
  void SetUp() override {
    std::vector<rclcpp::Parameter> params{
      {"depth_stvl.enabled",true},{"depth_stvl.voxel_decay",GetParam()},
      {"depth_stvl.decay_model",0},{"depth_stvl.voxel_size",0.02},
      {"depth_stvl.mark_threshold",1},{"depth_stvl.track_unknown_space",false},
      {"depth_stvl.combination_method",1},{"depth_stvl.publish_voxel_map",true},
      {"depth_stvl.update_footprint_enabled",true},
      {"depth_stvl.observation_sources","near regular"},
      {"laser.observation_sources","scan"},{"laser.scan.topic","/stvl_integration/scan"},
      {"laser.scan.data_type","LaserScan"},{"laser.scan.marking",true},
      {"laser.scan.clearing",false},{"static.map_topic","/stvl_integration/map"},
      {"static.map_subscribe_transient_local",true},
      {"inflation.inflation_radius",0.2},{"inflation.cost_scaling_factor",5.0},
      {"robot_base_frame","map"}, {"transform_tolerance",0.1}};
    for (const std::string source : {"near","regular"}) {
      const auto prefix="depth_stvl."+source+".";
      params.emplace_back(prefix+"topic","/stvl_integration/points");
      params.emplace_back(prefix+"data_type","PointCloud2");
      params.emplace_back(prefix+"marking",true);
      params.emplace_back(prefix+"clearing",false);
      params.emplace_back(prefix+"filter","passthrough");
      params.emplace_back(prefix+"min_obstacle_height",source=="near" ? 0.015 : 0.05);
      params.emplace_back(prefix+"max_obstacle_height",0.26);
      params.emplace_back(prefix+"obstacle_range",source=="near" ? 0.85 : 2.0);
      params.emplace_back(prefix+"observation_persistence",0.0);
      params.emplace_back(prefix+"clear_after_reading",true);
      params.emplace_back(prefix+"expected_update_rate",0.5);
    }
    loader_=std::make_unique<pluginlib::ClassLoader<nav2_costmap_2d::Layer>>(
      "nav2_costmap_2d","nav2_costmap_2d::Layer");
    node_=std::make_shared<nav2_util::LifecycleNode>("costmap","/stvl_integration",
      rclcpp::NodeOptions().parameter_overrides(params));
    // Costmap2DROS normally declares these global parameters before plugins.
    node_->declare_parameter("track_unknown_space",false);
    node_->declare_parameter("use_maximum",false);
    node_->declare_parameter("lethal_cost_threshold",100);
    node_->declare_parameter("unknown_cost_value",255);
    node_->declare_parameter("trinary_costmap",true);
    node_->declare_parameter("transform_tolerance",0.1);
    node_->declare_parameter("robot_base_frame","map");
    buffer_=std::make_unique<tf2_ros::Buffer>(node_->get_clock());
    listener_=std::make_unique<tf2_ros::TransformListener>(*buffer_);
    group_=node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    buffer_->setCreateTimerInterface(std::make_shared<tf2_ros::CreateTimerROS>(
      node_->get_node_base_interface(),node_->get_node_timers_interface(),group_));
    map_=std::make_unique<nav2_costmap_2d::LayeredCostmap>("map",false,false);
    map_->resizeMap(100,100,0.05,-2.5,-2.5);
    const std::vector<std::pair<std::string,std::string>> plugins{
      {"static","nav2_costmap_2d::StaticLayer"}, {"laser","nav2_costmap_2d::ObstacleLayer"},
      {"depth_stvl","carcar_navigation::MonitoredStvlLayer"},
      {"inflation","nav2_costmap_2d::InflationLayer"}};
    for (const auto & spec : plugins) {
      auto layer=loader_->createSharedInstance(spec.second);
      map_->addPlugin(layer); layer->initialize(map_.get(),spec.first,buffer_.get(),node_,group_);
      layer->activate();
      if (spec.first=="depth_stvl") {depth_=std::dynamic_pointer_cast<nav2_costmap_2d::CostmapLayer>(layer);}
    }
    std::vector<geometry_msgs::msg::Point> footprint(4);
    footprint[0].x=-0.1;footprint[0].y=-0.1;footprint[1].x=-0.1;footprint[1].y=0.1;
    footprint[2].x=0.1;footprint[2].y=0.1;footprint[3].x=0.1;footprint[3].y=-0.1;
    map_->setFootprint(footprint);
    pub_=node_->create_publisher<sensor_msgs::msg::PointCloud2>("points",rclcpp::SensorDataQoS());
    laser_pub_=node_->create_publisher<sensor_msgs::msg::LaserScan>("scan",rclcpp::SensorDataQoS());
    map_pub_=node_->create_publisher<nav_msgs::msg::OccupancyGrid>("map",rclcpp::QoS(1).transient_local());
    voxels_=node_->create_subscription<sensor_msgs::msg::PointCloud2>("voxel_grid",rclcpp::QoS(1),
      [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr msg){voxel_count_=msg->width*msg->height;});
    diag_=node_->create_subscription<diagnostic_msgs::msg::DiagnosticArray>("depth_stvl/diagnostics",1,
      [this](diagnostic_msgs::msg::DiagnosticArray::ConstSharedPtr msg){
        for(const auto & s:msg->status) {for(const auto & v:s.values) {
          if(v.key=="source_current") {source_current_=v.value=="true";}
        }}
      });
    executor_.add_node(node_->get_node_base_interface());
    in_executor_=true;
    publish_static(false); pump(0.6);
  }
  void TearDown() override {
    if (in_executor_) {executor_.remove_node(node_->get_node_base_interface());}
    for (auto & plugin:*map_->getPlugins()) {plugin->deactivate();}
    depth_.reset(); map_.reset(); voxels_.reset();diag_.reset();
    pub_.reset();laser_pub_.reset();map_pub_.reset();group_.reset();listener_.reset();buffer_.reset();node_.reset();
    loader_.reset();
  }
  void publish_static(bool occupied) {
    nav_msgs::msg::OccupancyGrid m;m.header.frame_id="map";m.header.stamp=node_->now();
    m.info.width=100;m.info.height=100;m.info.resolution=0.05;
    m.info.origin.position.x=-2.5;m.info.origin.position.y=-2.5;m.info.origin.orientation.w=1;
    m.data.resize(10000,0);
    if(occupied) {m.data[55*100+62]=100;} // (0.61,0.26)
    map_pub_->publish(m);
  }
  void publish(const std::vector<Point> & points, const std::string & frame="map") {
    sensor_msgs::msg::PointCloud2 c;c.header.frame_id=frame;c.header.stamp=node_->now();
    sensor_msgs::PointCloud2Modifier mod(c);mod.setPointCloud2FieldsByString(1,"xyz");mod.resize(points.size());
    sensor_msgs::PointCloud2Iterator<float> x(c,"x"),y(c,"y"),z(c,"z");
    for(const auto & p:points) {*x=p.x;*y=p.y;*z=p.z;++x;++y;++z;}
    pub_->publish(c);
  }
  void publish_laser() {
    sensor_msgs::msg::LaserScan scan;scan.header.frame_id="map";scan.header.stamp=node_->now();
    scan.angle_min=std::atan2(0.01,0.61);scan.angle_max=scan.angle_min+0.001F;
    scan.angle_increment=0.001;scan.range_min=0.05;scan.range_max=2;
    scan.ranges={static_cast<float>(std::hypot(0.61,0.01)),
      static_cast<float>(std::hypot(0.61,0.01))};
    laser_pub_->publish(scan);
  }
  void pump(double seconds,const std::function<void()> & feed={}) {
    const auto end=std::chrono::steady_clock::now()+std::chrono::duration<double>(seconds);
    auto next=std::chrono::steady_clock::now();
    while(std::chrono::steady_clock::now()<end) {
      if(std::chrono::steady_clock::now()>=next) {
        if(feed) {feed();}
        executor_.spin_some(); map_->updateMap(0,0,0); next+=200ms;
      }
      executor_.spin_some();std::this_thread::sleep_for(10ms);
    }
  }
  unsigned char cost(double x,double y,nav2_costmap_2d::Costmap2D * grid=nullptr) {
    if(!grid) {grid=map_->getCostmap();} unsigned int mx=0,my=0;
    EXPECT_TRUE(grid->worldToMap(x,y,mx,my));return grid->getCost(mx,my);
  }
  std::unique_ptr<pluginlib::ClassLoader<nav2_costmap_2d::Layer>> loader_;
  std::shared_ptr<nav2_util::LifecycleNode> node_;
  std::unique_ptr<tf2_ros::Buffer> buffer_;
  std::unique_ptr<tf2_ros::TransformListener> listener_;
  rclcpp::CallbackGroup::SharedPtr group_;
  std::unique_ptr<nav2_costmap_2d::LayeredCostmap> map_;
  std::shared_ptr<nav2_costmap_2d::CostmapLayer> depth_;
  rclcpp::executors::SingleThreadedExecutor executor_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr laser_pub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr voxels_;
  rclcpp::Subscription<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diag_;
  size_t voxel_count_=0;
  bool source_current_=false;
  bool in_executor_=false;
};
TEST_P(StvlIntegration, EmptyMarkingCloudExpiresVoxelLayerMasterAndInflation) {
  pump(1,[this]{publish({{0.61F,0.41F,0.10F}});});
  ASSERT_EQ(cost(0.61,0.41),nav2_costmap_2d::LETHAL_OBSTACLE);
  ASSERT_GT(cost(0.71,0.41),nav2_costmap_2d::FREE_SPACE);
  ASSERT_GT(voxel_count_,0U);
  pump(GetParam()+0.8,[this]{publish({});});
  EXPECT_EQ(voxel_count_,0U);
  EXPECT_EQ(cost(0.61,0.41,depth_.get()),nav2_costmap_2d::FREE_SPACE);
  EXPECT_EQ(cost(0.61,0.41),nav2_costmap_2d::FREE_SPACE);
  EXPECT_EQ(cost(0.71,0.41),nav2_costmap_2d::FREE_SPACE);
  EXPECT_TRUE(source_current_); // 零障碍的有效云依然正常。
}
TEST_P(StvlIntegration, TwoCentimeterMasterGridAlsoExpires) {
  nav_msgs::msg::OccupancyGrid m;m.header.frame_id="map";m.header.stamp=node_->now();
  m.info.width=200;m.info.height=200;m.info.resolution=0.02;
  m.info.origin.position.x=-2;m.info.origin.position.y=-2;m.info.origin.orientation.w=1;
  m.data.resize(40000,0);map_pub_->publish(m);pump(0.4);
  EXPECT_NEAR(map_->getCostmap()->getResolution(),0.02,1e-8);
  pump(0.8,[this]{publish({{0.61F,0.41F,0.10F}});});
  ASSERT_EQ(cost(0.61,0.41),nav2_costmap_2d::LETHAL_OBSTACLE);
  ASSERT_GT(cost(0.71,0.41),nav2_costmap_2d::FREE_SPACE);
  pump(GetParam()+0.8,[this]{publish({});});
  EXPECT_EQ(voxel_count_,0U);
  EXPECT_EQ(cost(0.61,0.41),nav2_costmap_2d::FREE_SPACE);
  EXPECT_EQ(cost(0.71,0.41),nav2_costmap_2d::FREE_SPACE);
}
TEST_P(StvlIntegration, PartialVoxelExpiryKeepsOtherVoxelInSameMasterCell) {
  pump(0.8,[this]{publish({{1.011F,0.411F,0.10F}});});
  pump(GetParam()+0.8,[this]{publish({{1.031F,0.431F,0.14F}});});
  EXPECT_EQ(cost(1.011,0.411),nav2_costmap_2d::LETHAL_OBSTACLE);
  EXPECT_EQ(voxel_count_,1U);
}
TEST_P(StvlIntegration, ExpiringDepthPreservesRealStaticAndLaserLayers) {
  publish_static(true);
  pump(0.6,[this]{publish_laser();});
  ASSERT_EQ(cost(0.61,0.01),nav2_costmap_2d::LETHAL_OBSTACLE);
  pump(1,[this]{publish({{0.61F,0.26F,0.10F},{0.61F,0.01F,0.10F}});publish_laser();});
  ASSERT_EQ(cost(0.61,0.26),nav2_costmap_2d::LETHAL_OBSTACLE);
  ASSERT_EQ(cost(0.61,0.01),nav2_costmap_2d::LETHAL_OBSTACLE);
  pump(GetParam()+0.8,[this]{publish({});publish_laser();});
  EXPECT_EQ(voxel_count_,0U);
  EXPECT_EQ(cost(0.61,0.26),nav2_costmap_2d::LETHAL_OBSTACLE);
  EXPECT_EQ(cost(0.61,0.01),nav2_costmap_2d::LETHAL_OBSTACLE);
}
TEST_P(StvlIntegration, RuntimeParametersAreRejectedAndInputTimeoutIsObservable) {
  EXPECT_FALSE(node_->set_parameter(rclcpp::Parameter("depth_stvl.enabled",false)).successful);
  EXPECT_FALSE(node_->set_parameter(rclcpp::Parameter("depth_stvl.voxel_decay",0.1)).successful);
  pump(0.8,[this]{publish({{0.61F,0.41F,0.10F}});});
  EXPECT_TRUE(source_current_);
  EXPECT_EQ(cost(0.61,0.41),nav2_costmap_2d::LETHAL_OBSTACLE);
  pump(0.9);
  EXPECT_FALSE(source_current_);
  pump(0.8,[this]{publish({});});
  EXPECT_TRUE(source_current_);
  pump(0.9,[this]{publish({{0.61F,0.41F,0.10F}},"missing_camera_frame");});
  EXPECT_FALSE(source_current_);
}
INSTANTIATE_TEST_SUITE_P(LocalAndGlobal,StvlIntegration,testing::Values(2.0,5.0));
// 静止快速遗忘候选：复用真实插件的三维、二维、膨胀及分层检查。
INSTANTIATE_TEST_SUITE_P(FastStaticCandidate,StvlIntegration,testing::Values(1.0,1.5));
}  // namespace
