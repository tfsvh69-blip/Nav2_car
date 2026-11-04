#include <gtest/gtest.h>
#include <thread>
#include <nav2_costmap_2d/inflation_layer.hpp>
#include <nav2_costmap_2d/costmap_filters/keepout_filter.hpp>
#include <nav2_msgs/msg/costmap_filter_info.hpp>
#include "carcar_mapping_autonomy/display_costmap_layer.hpp"
namespace ca=carcar_mapping_autonomy;
namespace {
// 输入夹具：真实膨胀层围绕障碍计算，真实KeepoutFilter覆盖另一块自由区域。
class Seed : public nav2_costmap_2d::Layer {
public:
 void onInitialize() override {enabled_=true;current_=true;}
 void updateBounds(double,double,double,double*a,double*b,double*c,double*d)override{*a=0;*b=0;*c=3;*d=3;}
 void updateCosts(nav2_costmap_2d::Costmap2D&m,int,int,int,int)override{m.setCost(20,20,254);}
 void reset()override{} bool isClearable()override{return false;}
};
TEST(DisplayCostmap, RealInflationAndKeepoutRemainByteIdentical) {
 int argc=0;rclcpp::init(argc,nullptr);
 auto observer=std::make_shared<rclcpp::Node>("display_observer");
 auto info=observer->create_publisher<nav2_msgs::msg::CostmapFilterInfo>("/display_test/filter_info",rclcpp::QoS(1).transient_local());
 auto mask=observer->create_publisher<nav_msgs::msg::OccupancyGrid>("/display_test/mask",rclcpp::QoS(1).transient_local());
 nav_msgs::msg::OccupancyGrid::SharedPtr seen;
 auto sub=observer->create_subscription<nav_msgs::msg::OccupancyGrid>("/global_costmap/costmap_before_filters",rclcpp::QoS(1).transient_local(),[&](nav_msgs::msg::OccupancyGrid::SharedPtr m){seen=m;});
 rclcpp::executors::SingleThreadedExecutor exec;exec.add_node(observer);
 std::vector<std::shared_ptr<nav2_util::LifecycleNode>> nodes;
 std::vector<std::unique_ptr<tf2_ros::Buffer>> buffers;
 std::vector<std::unique_ptr<nav2_costmap_2d::LayeredCostmap>> maps;
 for(int i=0;i<2;++i){
  auto node=std::make_shared<nav2_util::LifecycleNode>("costmap",i?"/global_costmap":"/baseline");
  node->declare_parameter("keepout.filter_info_topic",std::string("/display_test/filter_info"));
  node->declare_parameter("inflation.inflation_radius",.6);
  node->declare_parameter("inflation.cost_scaling_factor",3.);
  auto map=std::make_unique<nav2_costmap_2d::LayeredCostmap>("map",false,false);map->resizeMap(60,60,.05,0,0);
  auto tf=std::make_unique<tf2_ros::Buffer>(node->get_clock());
  auto group=node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  auto seed=std::make_shared<Seed>();map->addPlugin(seed);seed->initialize(map.get(),"seed",tf.get(),node,group);
  auto inflation=std::make_shared<nav2_costmap_2d::InflationLayer>();map->addPlugin(inflation);inflation->initialize(map.get(),"inflation",tf.get(),node,group);
  if(i){auto display=std::make_shared<ca::DisplayCostmapLayer>();map->addPlugin(display);display->initialize(map.get(),"display",tf.get(),node,group);}
  auto filter=std::make_shared<nav2_costmap_2d::KeepoutFilter>();map->addFilter(filter);filter->initialize(map.get(),"keepout",tf.get(),node,group);filter->activate();
  std::vector<geometry_msgs::msg::Point>footprint(4);footprint[0].x=footprint[1].x=-.14;footprint[2].x=footprint[3].x=.14;footprint[0].y=footprint[3].y=-.13;footprint[1].y=footprint[2].y=.13;map->setFootprint(footprint);
  exec.add_node(node->get_node_base_interface());nodes.push_back(node);buffers.push_back(std::move(tf));maps.push_back(std::move(map));
 }
 auto spin=[&]{for(int n=0;n<50;++n){exec.spin_some();std::this_thread::sleep_for(std::chrono::milliseconds(10));}};
 nav2_msgs::msg::CostmapFilterInfo f;f.type=0;f.base=0;f.multiplier=1;f.filter_mask_topic="/display_test/mask";info->publish(f);
 nav_msgs::msg::OccupancyGrid g;g.header.frame_id="map";g.info.width=g.info.height=60;g.info.resolution=.05;g.info.origin.orientation.w=1;g.data.assign(3600,0);
 // 禁入同时覆盖自由格和软膨胀格，显示出口必须保留原来不同的数值。
 g.data[45*60+45]=100;g.data[20*60+26]=100;mask->publish(g);spin();
 for(int cycle=0;cycle<3;++cycle){
  for(auto&map:maps)map->updateMap(1,1,0);spin();
  auto*a=maps[0]->getCostmap();auto*b=maps[1]->getCostmap();
  EXPECT_TRUE(std::equal(a->getCharMap(),a->getCharMap()+3600,b->getCharMap()));
  EXPECT_EQ(b->getCost(45,45),254);EXPECT_EQ(b->getCost(26,20),254);
  ASSERT_TRUE(seen);ASSERT_EQ(seen->data.size(),3600u);
  EXPECT_EQ(seen->data[45*60+45],0);EXPECT_GT(seen->data[20*60+26],0);EXPECT_LT(seen->data[20*60+26],99);
  EXPECT_EQ(seen->data[20*60+20],100);EXPECT_EQ(seen->header.frame_id,"map");
 }
 // 改变真实膨胀半径后显示跟随变化，两个最终导航栅格仍完全相同。
 const auto before=seen->data;
 for(auto&node:nodes)node->set_parameter(rclcpp::Parameter("inflation.inflation_radius",.9));
 for(auto&map:maps)map->updateMap(1,1,0);spin();
 EXPECT_NE(before,seen->data);
 EXPECT_TRUE(std::equal(maps[0]->getCostmap()->getCharMap(),maps[0]->getCostmap()->getCharMap()+3600,maps[1]->getCostmap()->getCharMap()));
 exec.cancel();maps.clear();buffers.clear();nodes.clear();rclcpp::shutdown();
}
}
