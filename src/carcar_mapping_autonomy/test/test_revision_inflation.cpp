#include <gtest/gtest.h>
#include <thread>
#include <tf2_ros/buffer.h>
#include <nav2_util/lifecycle_node.hpp>
#include "carcar_mapping_autonomy/parameter_revision_fence.hpp"
#include "carcar_mapping_autonomy/revision_inflation_layer.hpp"
#include "carcar_mapping_autonomy/tuning_transaction.hpp"
namespace ca=carcar_mapping_autonomy;
namespace {
class RevisionInflation:public testing::Test {
protected:
  static void SetUpTestSuite(){int argc=0;rclcpp::init(argc,nullptr);}
  static void TearDownTestSuite(){rclcpp::shutdown();}
  void SetUp()override{
    node=std::make_shared<nav2_util::LifecycleNode>("local_costmap","/local_costmap");
    observer=std::make_shared<rclcpp::Node>("inflation_ack_observer");
    subscription=observer->create_subscription<std_msgs::msg::String>("/mapping/tuning/inflation_ack",rclcpp::QoS(20).reliable().transient_local(),
      [this](const std_msgs::msg::String&m){messages.push_back(nlohmann::json::parse(m.data));});
    map=std::make_unique<nav2_costmap_2d::LayeredCostmap>("map",false,false);map->resizeMap(40,40,.05,0,0);
    tf=std::make_unique<tf2_ros::Buffer>(node->get_clock());layer=std::make_shared<ca::RevisionInflationLayer>();map->addPlugin(layer);
    layer->initialize(map.get(),"inflation_layer",tf.get(),node,node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive));
    std::vector<geometry_msgs::msg::Point>footprint(4);footprint[0].x=footprint[1].x=-.14;footprint[2].x=footprint[3].x=.14;
    footprint[0].y=footprint[3].y=-.13;footprint[1].y=footprint[2].y=.13;map->setFootprint(footprint);
    ASSERT_TRUE(change(1,.3,5.));cycle();spin();messages.clear();
  }
  void TearDown()override{layer.reset();map.reset();tf.reset();subscription.reset();observer.reset();node.reset();}
  bool change(int64_t revision,double radius,double factor){return node->set_parameters_atomically({
    rclcpp::Parameter("inflation_layer.inflation_radius",radius),rclcpp::Parameter("inflation_layer.cost_scaling_factor",factor),
    rclcpp::Parameter("inflation_layer.tuning_revision",revision)}).successful;}
  void bounds(){double minx=0,miny=0,maxx=2,maxy=2;layer->updateBounds(1,1,0,&minx,&miny,&maxx,&maxy);}
  void costs(){auto*m=map->getCostmap();m->resetMap(0,0,40,40);m->setCost(10,10,254);layer->updateCosts(*m,0,0,40,40);}
  void cycle(){bounds();costs();}
  void spin(){for(int i=0;i<30;++i){rclcpp::spin_some(observer);std::this_thread::sleep_for(std::chrono::milliseconds(5));}}
  std::shared_ptr<nav2_util::LifecycleNode>node;rclcpp::Node::SharedPtr observer;
  std::unique_ptr<nav2_costmap_2d::LayeredCostmap>map;std::unique_ptr<tf2_ros::Buffer>tf;std::shared_ptr<ca::RevisionInflationLayer>layer;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription;std::vector<nlohmann::json>messages;
};
TEST_F(RevisionInflation,ReadbackAndBoundsAloneCannotAcknowledgeActualInflation){
  ASSERT_TRUE(change(2,.5,3.));EXPECT_DOUBLE_EQ(node->get_parameter("inflation_layer.cost_scaling_factor").as_double(),3.);
  spin();EXPECT_TRUE(messages.empty());bounds();spin();EXPECT_TRUE(messages.empty());costs();spin();
  ASSERT_EQ(messages.size(),1U);EXPECT_TRUE(ca::tuning_ack_matches(messages.back(),"/local_costmap/local_costmap",2,.5,3.));
  const auto expected=static_cast<unsigned char>(252*std::exp(-3.*(.25-.13)));
  EXPECT_EQ(map->getCostmap()->getCost(15,10),expected);
}
TEST_F(RevisionInflation,ChangingCoefficientsBetweenBoundsAndCostsCannotPublishFalseCompletion){
  ASSERT_TRUE(change(2,.5,3.));bounds();ASSERT_TRUE(change(3,.4,7.));costs();spin();EXPECT_TRUE(messages.empty());
  cycle();spin();ASSERT_EQ(messages.size(),1U);EXPECT_TRUE(ca::tuning_ack_matches(messages.back(),"/local_costmap/local_costmap",3,.4,7.));
}
TEST_F(RevisionInflation,StaleRequestsAreRejectedBeforeUpstreamInternalMutation){
  ASSERT_TRUE(change(3,.4,7.));EXPECT_FALSE(change(2,.5,1.));cycle();spin();
  EXPECT_DOUBLE_EQ(node->get_parameter("inflation_layer.cost_scaling_factor").as_double(),7.);
  ASSERT_EQ(messages.size(),1U);EXPECT_EQ(messages.back().at("factor"),7.);
}
TEST_F(RevisionInflation,ControllerFenceRejectsLateRevisionBeforePrimaryCallback){
  auto controller=std::make_shared<rclcpp_lifecycle::LifecycleNode>("revision_controller");
  controller->declare_parameter("FollowPath.inflation_cost_scaling_factor",5.);double effective=5.;
  auto native=controller->add_on_set_parameters_callback([&effective](const std::vector<rclcpp::Parameter>&parameters){
    for(const auto&p:parameters)if(p.get_name()=="FollowPath.inflation_cost_scaling_factor")effective=p.as_double();
    rcl_interfaces::msg::SetParametersResult result;result.successful=true;return result;
  });
  ca::ParameterRevisionFence fence;fence.configure(controller,"FollowPath");fence.activate();
  EXPECT_TRUE(controller->set_parameters_atomically({rclcpp::Parameter("FollowPath.inflation_cost_scaling_factor",3.),rclcpp::Parameter("FollowPath.tuning_revision",8)}).successful);
  EXPECT_DOUBLE_EQ(effective,3.);
  EXPECT_FALSE(controller->set_parameters_atomically({rclcpp::Parameter("FollowPath.inflation_cost_scaling_factor",9.),rclcpp::Parameter("FollowPath.tuning_revision",7)}).successful);
  EXPECT_DOUBLE_EQ(effective,3.);fence.deactivate();
}
}
