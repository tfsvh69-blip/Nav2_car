// 使用真实ranker节点与只读假规划服务，验证同一安全路径上的前缀替代；不发布速度。
#define FRONTIER_RANKER_NO_MAIN
#include "../src/frontier_ranker.cpp"
#include <gtest/gtest.h>
#include <tf2_ros/transform_broadcaster.h>
#include <thread>
TEST(FrontierRankerContract, BlockedBestPrefixSelectsNearbyAlternativeBeforeClientFiltering){
 int argc=0;rclcpp::init(argc,nullptr);
 struct Shutdown {~Shutdown(){if(rclcpp::ok())rclcpp::shutdown();}} shutdown_guard;
 auto fixture=std::make_shared<rclcpp::Node>("frontier_ranker_contract_fixture");
 auto ranker=std::make_shared<FrontierRanker>();
 rclcpp::executors::SingleThreadedExecutor executor;executor.add_node(fixture);executor.add_node(ranker);
 tf2_ros::TransformBroadcaster transforms(*fixture);
 const auto qos=rclcpp::QoS(1).reliable().transient_local();
 auto maps=fixture->create_publisher<nav_msgs::msg::OccupancyGrid>("/map",qos);
 auto states=fixture->create_publisher<std_msgs::msg::String>("/mapping/autonomy/state",qos);
 auto requests=fixture->create_publisher<std_msgs::msg::String>("/mapping/exploration/rank_request",10);
 ca::Json response=ca::Json::object();
 auto results=fixture->create_subscription<std_msgs::msg::String>("/mapping/exploration/rank_result",10,
   [&](const std_msgs::msg::String&m){response=ca::Json::parse(m.data);});
 auto planner=rclcpp_action::create_server<Plan>(fixture,"/compute_path_to_pose",
   [](const auto&,auto){return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;},
   [](auto){return rclcpp_action::CancelResponse::ACCEPT;},[&](auto handle){
     auto result=std::make_shared<Plan::Result>();const auto goal=handle->get_goal();
     result->path.header.frame_id="map";result->path.header.stamp=fixture->now();
     const auto&a=goal->start.pose.position;const auto&b=goal->goal.pose.position;
     const int steps=std::max(1,int(std::ceil(std::hypot(b.x-a.x,b.y-a.y)/.05)));
     for(int i=0;i<=steps;++i){geometry_msgs::msg::PoseStamped p;p.header=result->path.header;
       p.pose.position.x=a.x+(b.x-a.x)*double(i)/steps;p.pose.position.y=a.y+(b.y-a.y)*double(i)/steps;
       p.pose.orientation.w=1;result->path.poses.push_back(p);}
     handle->succeed(result);
   });
 auto publish=fixture->create_wall_timer(std::chrono::milliseconds(50),[&]{
   geometry_msgs::msg::TransformStamped tf;tf.header.frame_id="map";tf.child_frame_id="base_footprint";
   tf.header.stamp=fixture->now();tf.transform.rotation.w=1;transforms.sendTransform(tf);
   nav_msgs::msg::OccupancyGrid map;map.header=tf.header;map.info.resolution=.05;map.info.width=map.info.height=160;
   map.info.origin.position.x=map.info.origin.position.y=-4;map.info.origin.orientation.w=1;map.data.assign(25600,0);
   for(int y=0;y<160;++y)for(int x=120;x<160;++x)map.data[y*160+x]=-1;maps->publish(map);
   std_msgs::msg::String state;state.data=ca::Json{{"task_id","contract"},{"phase","EXPLORING"},{"healthy",true},
     {"generation",1},{"navigation_generation",1},{"anchored",true},{"origin",{0,0,0}},{"radius_m",20.}}.dump();
   states->publish(state);
 });
 auto until=[&](const std::function<bool()>&condition,double seconds){
   const auto end=std::chrono::steady_clock::now()+std::chrono::duration<double>(seconds);
   while(std::chrono::steady_clock::now()<end){executor.spin_some();if(condition())return true;std::this_thread::sleep_for(std::chrono::milliseconds(5));}
   return condition();
 };
 until([]{return false;},1.5);
 ca::Json request={{"task_id","contract"},{"request_id","unblocked"},{"generation",1},{"navigation_generation",1},
   {"candidates",ca::Json::array({{2.025,0.}})},{"visits",ca::Json::array()},{"blocked_observations",ca::Json::array()}};
 std_msgs::msg::String message;message.data=request.dump();requests->publish(message);
 ASSERT_TRUE(until([&]{return response.value("request_id",std::string())=="unblocked";},6));
 ASSERT_FALSE(response.at("candidates").empty())<<response.dump();
 const auto first=response["candidates"][0];const auto center=first["observation"];
 request["request_id"]="blocked-prefix";
 request["blocked_observations"]=ca::Json::array({{{"center",center},{"radius",.30},{"until",fixture->now().seconds()+60.},{"kind","visited_observation"}}});
 response=ca::Json::object();message.data=request.dump();requests->publish(message);
 ASSERT_TRUE(until([&]{return response.value("request_id",std::string())=="blocked-prefix";},6));
 ASSERT_FALSE(response.at("candidates").empty())<<response.dump();
 const auto alternative=response["candidates"][0];
 EXPECT_EQ(alternative["frontier"],first["frontier"]);
 const auto chosen=alternative["observation"];
 EXPECT_GT(std::hypot(chosen[0].get<double>()-center[0].get<double>(),chosen[1].get<double>()-center[1].get<double>()),.30);
 EXPECT_LT(alternative["path_m"].get<double>(),alternative["full_path_m"].get<double>()-.10);
 EXPECT_TRUE(alternative["path_prefix"].get<bool>());
 executor.remove_node(ranker);executor.remove_node(fixture);
 publish.reset();planner.reset();results.reset();requests.reset();states.reset();maps.reset();ranker.reset();
 rclcpp::shutdown();
}
