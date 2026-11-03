#include <gtest/gtest.h>
#include <rclcpp/serialization.hpp>
#include <rclcpp/serialized_message.hpp>
#include <cstring>
#include <random>
#include "carcar_mapping_autonomy/task_walls.hpp"
#include "carcar_mapping_autonomy/frontier_gain.hpp"
#include "carcar_mapping_autonomy/subcell_departure.hpp"
#include "carcar_mapping_autonomy/recovery_wall_review.hpp"
namespace ca=carcar_mapping_autonomy;
namespace {
nav_msgs::msg::OccupancyGrid grid(unsigned width=12,unsigned height=12,double resolution=.1){
  nav_msgs::msg::OccupancyGrid map;map.header.frame_id="map";map.header.stamp.sec=123;
  map.info.map_load_time.sec=120;map.info.width=width;map.info.height=height;
  map.info.resolution=resolution;map.info.origin.orientation.w=1;map.data.assign(width*height,0);return map;
}
ca::Json wall(double x1,double y1,double x2,double y2){
  return ca::Json::array({{{"id","wall-a"},{"points",{{x1,y1},{x2,y2}}}}});
}
ca::Json replace(const ca::Json&walls,uint64_t revision=0,uint64_t generation=3,std::string id="replace-1"){
  return {{"action","replace"},{"task_id","task-a"},{"request_id",id},
    {"navigation_generation",generation},{"expected_revision",revision},{"walls",walls}};
}
ca::TaskScene ready_scene(){ca::TaskScene scene("task-a");scene.generation(3);
  scene.map(std::make_shared<nav_msgs::msg::OccupancyGrid>(grid()));return scene;}
// 独立逐格线段裁剪作为几何判据，不复用DDA的分支或步进规则。
bool square_intersection(const ca::Pose&a,const ca::Pose&b,int x,int y){
  long double low=0,high=1;
  auto clip=[&](long double position,long double delta,long double minimum,long double maximum){
    if(delta==0)return position>=minimum&&position<=maximum;
    auto t0=(minimum-position)/delta,t1=(maximum-position)/delta;
    if(t0>t1)std::swap(t0,t1);
    low=std::max(low,t0);high=std::min(high,t1);return low<=high;
  };
  return clip(a.x,b.x-a.x,x,x+1)&&clip(a.y,b.y-a.y,y,y+1)&&low<=1&&high>=0;
}
}
TEST(TaskWalls,EmptyWallsPreserveCompleteSerializedOriginalAndOnlyMaskIsBlank){
  auto map=grid();map.info.origin.position.x=-2.3;map.info.origin.position.y=5.7;
  map.data[2]=-1;map.data[20]=17;map.data[35]=100;
  const auto result=ca::rasterize_task_walls(map,{});
  rclcpp::Serialization<nav_msgs::msg::OccupancyGrid>serializer;
  rclcpp::SerializedMessage before,after;serializer.serialize_message(&map,&before);
  serializer.serialize_message(&result.navigation,&after);
  ASSERT_EQ(before.size(),after.size());
  EXPECT_EQ(std::memcmp(before.get_rcl_serialized_message().buffer,after.get_rcl_serialized_message().buffer,before.size()),0);
  EXPECT_TRUE(std::all_of(result.mask.data.begin(),result.mask.data.end(),[](int value){return value==0;}));
}
TEST(TaskWalls,SupercoverMatchesClosedSquareOracleAcrossCornersGridLinesAndReversal){
  const auto map=grid(6,6,1);std::mt19937 random(3907);
  for(unsigned trial=0;trial<1200;++trial){
    const ca::Pose a{double(random()%25)/4,double(random()%25)/4,0};
    const ca::Pose b{double(random()%25)/4,double(random()%25)/4,0};
    if(a.x==b.x&&a.y==b.y)continue;
    const ca::TaskWallSet walls{{"oracle",{a,b}}};const auto result=ca::rasterize_task_walls(map,walls);
    const auto reverse=ca::rasterize_task_walls(map,{{"reverse",{b,a}}});EXPECT_EQ(result.mask.data,reverse.mask.data);
    for(int y=0;y<6;++y)for(int x=0;x<6;++x)
      ASSERT_EQ(result.mask.data[y*6+x]==100,square_intersection(a,b,x,y))
        <<"trial="<<trial<<" cell="<<x<<","<<y<<" start="<<a.x<<","<<a.y<<" end="<<b.x<<","<<b.y;
  }
}
TEST(TaskWalls,RotatedGridAndExpandedOriginKeepMeterCoordinatesAndUnknownIsNeverFreed){
  auto map=grid(8,8,1);map.info.origin.position.x=-2;map.info.origin.position.y=-1;
  map.data[4*8+3]=-1;map.data[1]=100;
  const auto walls=ca::parse_task_walls(wall(1.5,0.2,1.5,5.8));
  const auto before=ca::rasterize_task_walls(map,walls);
  EXPECT_EQ(before.navigation.data[4*8+3],100);EXPECT_EQ(before.navigation.data[1],100);
  auto expanded=grid(12,12,1);expanded.info.origin.position.x=-4;expanded.info.origin.position.y=-3;
  const auto after=ca::rasterize_task_walls(expanded,walls);
  EXPECT_EQ(after.mask.data[5*12+5],100);EXPECT_EQ(after.mask.data[5*12+3],0);
  auto rotated=grid(8,8,1);const double angle=.37;rotated.info.origin.orientation.z=std::sin(angle/2);
  rotated.info.origin.orientation.w=std::cos(angle/2);
  auto world=[&](double x,double y){return ca::Pose{std::cos(angle)*x-std::sin(angle)*y,std::sin(angle)*x+std::cos(angle)*y,0};};
  const auto raster=ca::rasterize_task_walls(rotated,{{"rotated",{world(2.25,.5),world(2.25,6.5)}}});
  for(int y=0;y<7;++y)EXPECT_EQ(raster.mask.data[y*8+2],100);
}
TEST(TaskWalls,IndependentWallCutsReachabilityVisibilityAndUnknownGainWithoutEditingRawMap){
  auto map=grid(80,80,.05);
  for(int y=0;y<80;++y)for(int x=50;x<80;++x)map.data[y*80+x]=-1;
  const auto original=map.data;const ca::Pose start{1.7,2,0},end{2.25,2,0};
  ASSERT_TRUE(ca::free_connector(map,start,end));
  ASSERT_TRUE(ca::observation_visible(map,start,{2.51,2,0}));
  const auto rendered=ca::rasterize_task_walls(map,ca::parse_task_walls(wall(2,0,2,4)));
  EXPECT_FALSE(ca::free_connector(rendered.navigation,start,end));
  EXPECT_FALSE(ca::observation_visible(rendered.navigation,start,{2.51,2,0}));
  const auto raw_gain=ca::visible_unknown_cells(map,start),limited_gain=ca::visible_unknown_cells(rendered.navigation,start);
  const auto right=[](uint64_t key){return int32_t(uint32_t(key>>32))>=25;};
  EXPECT_TRUE(std::any_of(raw_gain.begin(),raw_gain.end(),right));
  EXPECT_FALSE(std::any_of(limited_gain.begin(),limited_gain.end(),right));EXPECT_EQ(map.data,original);
}
TEST(TaskWalls,ContinuousSegmentGuardRejectsCrossingTangencyAndUnsafeStationaryFootprint){
  const auto walls=wall(1,-1,1,1);
  EXPECT_FALSE(ca::walls_segment_safe(walls,{0,0,0},{2,0,0}));
  EXPECT_FALSE(ca::walls_segment_safe(walls,{.78,0,0},{.78,.2,0},.22));
  EXPECT_FALSE(ca::walls_segment_safe(walls,{1,0,0},{1,0,0}));
  EXPECT_FALSE(ca::walls_segment_safe(walls,{0,1.22,0},{2,1.22,0},.22));
  EXPECT_TRUE(ca::walls_segment_safe(walls,{0,1.23,0},{2,1.23,0},.22));
  EXPECT_TRUE(ca::walls_segment_safe(ca::Json::array(),{0,0,0},{2,0,0}));
  EXPECT_FALSE(ca::walls_segment_safe(ca::Json::array(),{NAN,0,0},{2,0,0}));
  EXPECT_FALSE(ca::walls_segment_safe(ca::Json::object(),{0,0,0},{2,0,0}));
}
TEST(TaskWalls,IdentityGenerationRevisionAndInvalidGeometryNeverOverwriteAcceptedWalls){
  auto scene=ready_scene();ASSERT_TRUE(scene.ready());const auto valid=replace(wall(.6,.1,.6,1.1));
  ASSERT_TRUE(scene.request(valid));const auto applied=scene.output()->navigation;const auto sequence=scene.output_sequence();
  EXPECT_TRUE(scene.request(valid));EXPECT_EQ(scene.state()["revision"],1);EXPECT_EQ(scene.output_sequence(),sequence);
  std::vector<ca::Json>invalid;
  auto wrong=replace(ca::Json::array(),1,3,"wrong-task");wrong["task_id"]="other-task";invalid.push_back(wrong);
  invalid.push_back(replace(ca::Json::array(),1,2,"old-generation"));
  invalid.push_back(replace(ca::Json::array(),0,3,"old-revision"));
  invalid.push_back(replace(wall(-.1,.1,.4,.4),1,3,"outside"));
  invalid.push_back(replace(wall(.1,.1,1.4,.4),1,3,"outside-other"));
  wrong=replace(wall(.1,.1,.4,.4),1,3,"bad-number");wrong["walls"][0]["points"][0][0]="NaN";invalid.push_back(wrong);
  wrong=replace(wall(.1,.1,.4,.4),1,3,"negative-revision");wrong["expected_revision"]=-1;invalid.push_back(wrong);
  wrong=valid;wrong["walls"]=ca::Json::array();invalid.push_back(wrong);
  for(const auto&request:invalid){EXPECT_FALSE(scene.request(request))<<request.dump();EXPECT_TRUE(scene.ready());
    EXPECT_EQ(scene.state()["revision"],1);EXPECT_EQ(scene.state()["request_result"],"rejected");EXPECT_EQ(scene.output()->navigation,applied);}
}
TEST(TaskWalls,BoundedInputAndNonfiniteCoordinatesAreRejectedBeforeRasterization){
  auto many=ca::Json::array();for(int i=0;i<33;++i)many.push_back({{"id","wall-"+std::to_string(i)},{"points",{{.1,.1},{.2,.2}}}});
  EXPECT_THROW(ca::parse_task_walls(many),std::invalid_argument);
  auto duplicate=wall(.1,.1,.4,.4);duplicate.push_back(duplicate[0]);EXPECT_THROW(ca::parse_task_walls(duplicate),std::invalid_argument);
  auto bad=wall(.1,.1,.4,.4);bad[0]["points"][1][0]=INFINITY;EXPECT_THROW(ca::parse_task_walls(bad),std::invalid_argument);
  bad=wall(.1,.1,.1,.1);EXPECT_THROW(ca::parse_task_walls(bad),std::invalid_argument);
  auto map=grid();map.data[0]=-2;EXPECT_THROW(ca::rasterize_task_walls(map,{}),std::invalid_argument);
}
TEST(TaskWalls,EpochChangeKeepsOutputFrozenUntilExactRevisionConfirmationOnLatestMap){
  auto scene=ready_scene();ASSERT_TRUE(scene.request(replace(wall(.6,.1,.6,1.1))));
  const auto output=scene.output();const auto sequence=scene.output_sequence();scene.generation(4);
  EXPECT_FALSE(scene.ready());EXPECT_EQ(scene.state()["phase"],"REVIEW_REQUIRED");
  EXPECT_FALSE(scene.state().contains("navigation_fingerprint"));
  EXPECT_FALSE(scene.request({{"action","confirm"},{"task_id","task-a"},{"request_id","confirm-before-map"},
    {"navigation_generation",4},{"revision",1}}));
  EXPECT_EQ(scene.output_sequence(),sequence);
  auto expanded=grid(24,24,.1);expanded.info.origin.position.x=-.4;expanded.info.origin.position.y=-.4;
  scene.map(std::make_shared<nav_msgs::msg::OccupancyGrid>(expanded));EXPECT_EQ(scene.output(),output);
  auto confirm=ca::Json{{"action","confirm"},{"task_id","task-a"},{"request_id","confirm-1"},{"navigation_generation",4},{"revision",0}};
  EXPECT_FALSE(scene.request(confirm));EXPECT_EQ(scene.output_sequence(),sequence);
  confirm["request_id"]="confirm-old-generation";confirm["navigation_generation"]=3;confirm["revision"]=1;
  EXPECT_FALSE(scene.request(confirm));EXPECT_EQ(scene.state()["phase"],"REVIEW_REQUIRED");
  confirm["request_id"]="confirm-2";confirm["navigation_generation"]=4;ASSERT_TRUE(scene.request(confirm));
  EXPECT_TRUE(scene.ready());EXPECT_EQ(scene.state()["revision"],1);EXPECT_EQ(scene.state()["applied_navigation_generation"],4);
  EXPECT_EQ(scene.state()["navigation_fingerprint"],std::to_string(ca::map_content_fingerprint(scene.output()->navigation)));
  EXPECT_EQ(scene.output()->navigation.info,expanded.info);EXPECT_GT(scene.output_sequence(),sequence);
}
TEST(TaskWalls,ClearDuringReviewThenFreshTaskCannotInheritPreviousWalls){
  auto scene=ready_scene();ASSERT_TRUE(scene.request(replace(wall(.6,.1,.6,1.1))));scene.generation(4);
  ASSERT_TRUE(scene.request(replace(ca::Json::array(),1,4,"clear")));EXPECT_TRUE(scene.ready());
  EXPECT_TRUE(scene.state()["walls"].empty());EXPECT_EQ(scene.output()->navigation,grid());
  scene.generation(5);EXPECT_TRUE(scene.ready());
  ca::TaskScene next("different-task");next.generation(7);next.map(std::make_shared<nav_msgs::msg::OccupancyGrid>(grid()));
  EXPECT_TRUE(next.ready());EXPECT_EQ(next.state()["revision"],0);EXPECT_TRUE(next.state()["walls"].empty());
  EXPECT_EQ(next.output()->navigation,grid());
}
TEST(TaskWalls,StaleInvalidOrShrunkRawMapCannotReleaseOldConstraints){
  auto scene=ready_scene();ASSERT_TRUE(scene.request(replace(wall(.6,.1,.6,1.1))));
  scene.stale();EXPECT_FALSE(scene.ready());EXPECT_EQ(scene.state()["phase"],"FAILED");
  scene.map(std::make_shared<nav_msgs::msg::OccupancyGrid>(grid()));EXPECT_TRUE(scene.ready());
  const auto output=scene.output();scene.map(std::make_shared<nav_msgs::msg::OccupancyGrid>(grid(4,4,.1)));
  EXPECT_FALSE(scene.ready());EXPECT_EQ(scene.state()["phase"],"REVIEW_REQUIRED");EXPECT_EQ(scene.output(),output);
}
TEST(TaskWalls,ManualGuardIncludesMeasuredForwardBrakingWhenRequestedDirectionReverses){
  geometry_msgs::msg::Twist reverse,measured;reverse.linear.x=-.05;
  const auto walls=wall(.26,-1,.26,1);
  EXPECT_TRUE(ca::walls_motion_safe(walls,{0,0,0},reverse,measured));
  measured.linear.x=.10;EXPECT_FALSE(ca::walls_motion_safe(walls,{0,0,0},reverse,measured));
  measured.linear.x=NAN;EXPECT_FALSE(ca::walls_motion_safe(walls,{0,0,0},reverse,measured));
}
TEST(TaskWalls,HumanReviewBudgetIsSeparateBoundedAndCannotBeResetByToggling){
  ca::RecoveryWallReviewWait wait;wait.update(10,true);wait.update(110,false);
  EXPECT_DOUBLE_EQ(wait.elapsed(200),100);wait.update(220,true);wait.update(400,false);
  EXPECT_DOUBLE_EQ(wait.elapsed(500),280);EXPECT_FALSE(wait.expired(500));
  wait.update(510,true);EXPECT_TRUE(wait.expired(531));wait.update(531,false);EXPECT_TRUE(wait.expired(600));
  wait.reset();EXPECT_DOUBLE_EQ(wait.elapsed(900),0);EXPECT_FALSE(wait.expired(900));
  ca::Json state{{"task_id","a"},{"generation",4},{"phase","RECOVERING"},{"still",true},
    {"navigation_terminal",true},{"output",{0.,0.}},{"walls",{{"review_allowed",true},
    {"phase","REVIEW_REQUIRED"},{"walls",wall(.5,0,.5,1)}}}};
  EXPECT_TRUE(ca::wall_review_authorized(state,"a",4));EXPECT_FALSE(ca::wall_review_authorized(state,"a",5));
  EXPECT_FALSE(ca::wall_review_authorized(state,"different",4));
  state["output"][0]=.01;EXPECT_FALSE(ca::wall_review_authorized(state,"a",4));
  state["output"][0]=0.;state["walls"]["review_allowed"]=false;EXPECT_FALSE(ca::wall_review_authorized(state,"a",4));
}
