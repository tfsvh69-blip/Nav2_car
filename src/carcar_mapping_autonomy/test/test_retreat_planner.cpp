#include <fstream>
#include <gtest/gtest.h>
#include "carcar_mapping_autonomy/retreat_planner.hpp"

namespace ca=carcar_mapping_autonomy;
namespace {
constexpr double pi=3.14159265358979323846;
nav_msgs::msg::OccupancyGrid open_map(){
  nav_msgs::msg::OccupancyGrid m;m.header.frame_id="map";m.info.width=m.info.height=100;
  m.info.resolution=.05;m.info.origin.position.x=m.info.origin.position.y=-2.51;
  m.info.origin.orientation.w=1;m.data.assign(10000,0);return m;
}
sensor_msgs::msg::LaserScan open_scan(){
  sensor_msgs::msg::LaserScan s;s.header.frame_id="laser_frame";s.angle_min=-pi;
  s.angle_increment=pi/360;s.angle_max=s.angle_min+719*s.angle_increment;
  s.range_min=.05;s.range_max=12;s.ranges.assign(720,2.);return s;
}
void point(sensor_msgs::msg::LaserScan&s,double x,double y,const ca::Pose&laser={}){
  const double dx=x-laser.x,dy=y-laser.y;
  const double angle=std::remainder(std::atan2(dy,dx)-laser.yaw-s.angle_min,2*pi);
  const double positive=angle<0?angle+2*pi:angle;
  const int index=std::llround(positive/std::abs(s.angle_increment));
  ASSERT_GE(index,0);ASSERT_LT(index,int(s.ranges.size()));s.ranges[index]=std::hypot(dx,dy);
}
sensor_msgs::msg::LaserScan table_scan(const ca::Pose&laser={}){
  auto s=open_scan();point(s,0,-.24,laser);return s;
}
void set_cell(nav_msgs::msg::OccupancyGrid&m,double x,double y,int8_t value){
  const int ix=std::floor((x-m.info.origin.position.x)/m.info.resolution);
  const int iy=std::floor((y-m.info.origin.position.y)/m.info.resolution);
  ASSERT_GE(ix,0);ASSERT_GE(iy,0);ASSERT_LT(ix,int(m.info.width));ASSERT_LT(iy,int(m.info.height));
  m.data[iy*m.info.width+ix]=value;
}
struct Fixture {
  nav_msgs::msg::OccupancyGrid map;sensor_msgs::msg::LaserScan scan;ca::Pose pose,laser;
};
Fixture recorded(){
#ifdef RETREAT_FIXTURE
  std::ifstream f(RETREAT_FIXTURE);
#else
  std::ifstream f(std::string(__FILE__).substr(0,std::string(__FILE__).find_last_of('/'))+"/data/retreat_table_leg_20261101.json");
#endif
  if(!f)throw std::runtime_error("桌腿原始夹具缺失");
  ca::Json j;f>>j;Fixture result;
  const auto&m=j["map"];result.map.header.frame_id="map";result.map.info.width=m["width"];
  result.map.info.height=m["height"];result.map.info.resolution=m["resolution"];
  result.map.info.origin.position.x=m["origin"][0];result.map.info.origin.position.y=m["origin"][1];
  result.map.info.origin.orientation.w=1;result.map.data=m["data"].get<std::vector<int8_t>>();
  const auto&s=j["scan"];result.scan.header.frame_id=s["frame"];result.scan.angle_min=s["angle_min"];
  result.scan.angle_max=s["angle_max"];result.scan.angle_increment=s["angle_increment"];
  result.scan.range_min=s["range_min"];result.scan.range_max=s["range_max"];
  for(const auto&r:s["ranges"])result.scan.ranges.push_back(r.is_null()?std::numeric_limits<float>::quiet_NaN():r.get<float>());
  result.pose={j["pose_map"][0],j["pose_map"][1],j["pose_map"][2]};
  result.laser={j["laser_in_base"][0],j["laser_in_base"][1],j["laser_in_base"][2]};return result;
}
}

TEST(RetreatPlanner, RecordedTableLegReproducesCircularGridBlockWithoutRectangleCollision){
  const auto f=recorded();ca::retreat_detail::Observation observation;std::string reason;ca::RetreatOptions o;
  ASSERT_TRUE(ca::retreat_detail::observe(f.scan,f.laser,o,observation,reason));
  const auto dynamic=ca::retreat_detail::endpoint_grid(f.map,{observation},f.pose);
  ASSERT_TRUE(ca::free_footprint(f.map,f.pose));
  EXPECT_FALSE(ca::free_footprint(dynamic,f.pose,o.dynamic_radius));
  // 原始现场反例必须保留：圆与整格相交，但实际矩形加2厘米余量无回波侵入。
  const ca::retreat_detail::Rectangle rectangle{-o.rear-o.margin,o.front+o.margin,-o.half_width-o.margin,o.half_width+o.margin};
  for(const auto&p:observation.endpoints)EXPECT_FALSE(ca::retreat_detail::inside(p,rectangle));
  EXPECT_TRUE(ca::retreat_safe(f.map,f.scan,f.pose,f.laser,0,0,o,&reason))<<reason;
}

TEST(RetreatPlanner, RecordedRearMissingRaysCannotBeInventedAsFreeSpace){
  const auto f=recorded();const auto plan=ca::plan_retreat(f.map,f.scan,f.pose,f.laser);
  EXPECT_FALSE(plan.allowed);EXPECT_TRUE(plan.needs_retreat);
  EXPECT_NE(plan.reason.find("激光"),std::string::npos)<<plan.reason;
}

TEST(RetreatPlanner, RecordedGeometryWithExplicitSoftwareRearReferenceCanRetreat){
  auto f=recorded();
  // 仅软件对照：补入假定的2米有限参考回波；绝不把此变体当作现场实测或替换原夹具。
  for(auto&range:f.scan.ranges)if(!std::isfinite(range))range=2.f;
  const auto plan=ca::plan_retreat(f.map,f.scan,f.pose,f.laser);
  ASSERT_TRUE(plan.allowed)<<plan.reason;EXPECT_TRUE(plan.needs_retreat);
  EXPECT_GT(plan.distance,0);EXPECT_LE(plan.distance,.25);
  EXPECT_TRUE(ca::retreat_safe(f.map,f.scan,f.pose,f.laser,plan.distance,plan.speed,{}));
}

TEST(RetreatPlanner, SideTableLegCanUseShortStraightRetreatWhenRearIsObserved){
  const auto map=open_map(),original=map;const auto scan=table_scan();const auto plan=ca::plan_retreat(map,scan,{},{});
  ASSERT_TRUE(plan.allowed)<<plan.reason;EXPECT_TRUE(plan.needs_retreat);EXPECT_TRUE(plan.clears_dynamic_envelope);
  EXPECT_GT(plan.distance,0);EXPECT_LT(plan.distance,.05);EXPECT_GT(plan.speed,0);EXPECT_LE(plan.speed,.03);
  EXPECT_GE(plan.clearance,.02);EXPECT_EQ(map,original);
  EXPECT_TRUE(ca::retreat_safe(map,scan,{},{},plan.distance,plan.speed,{}));
}

TEST(RetreatPlanner, TightRearSpaceReturnsAdaptiveShortRetreatRatherThanFixedMinimum){
  auto scan=table_scan();point(scan,-.17,0);const auto plan=ca::plan_retreat(open_map(),scan,{},{});
  ASSERT_TRUE(plan.allowed)<<plan.reason;EXPECT_TRUE(plan.needs_retreat);EXPECT_FALSE(plan.clears_dynamic_envelope);
  EXPECT_GT(plan.distance,.001);EXPECT_LT(plan.distance,.01);EXPECT_GT(plan.speed,0);EXPECT_LT(plan.speed,.03);
}

TEST(RetreatPlanner, RearObstacleInsideCurrentMarginPreventsRetreat){
  auto scan=table_scan();point(scan,-.155,0);const auto plan=ca::plan_retreat(open_map(),scan,{},{});
  EXPECT_FALSE(plan.allowed);EXPECT_EQ(plan.distance,0);EXPECT_EQ(plan.speed,0);
}

TEST(RetreatPlanner, BrakingTailIsCheckedBeyondRequestedEndpoint){
  auto scan=table_scan();point(scan,-.263,0);std::string reason;ca::RetreatOptions o;
  EXPECT_TRUE(ca::retreat_safe(open_map(),scan,{},{},.10,0,o,&reason))<<reason;
  EXPECT_FALSE(ca::retreat_safe(open_map(),scan,{},{},.10,.03,o,&reason));
  EXPECT_FALSE(reason.empty());
}

TEST(RetreatPlanner, RearNanInfinityAndBelowMinimumEachRefuseMotion){
  for(const float bad:{std::numeric_limits<float>::quiet_NaN(),std::numeric_limits<float>::infinity(),.01f}){
    auto scan=table_scan();scan.ranges[0]=bad;std::string reason;
    EXPECT_FALSE(ca::retreat_safe(open_map(),scan,{},{},.01,.01,{},&reason));
    EXPECT_FALSE(reason.empty());
  }
}

TEST(RetreatPlanner, UnobservedRearFieldOfViewRefusesMotion){
  auto scan=table_scan();scan.angle_min=-pi/2;scan.angle_increment=pi/360;
  scan.ranges.resize(361);scan.angle_max=scan.angle_min+360*scan.angle_increment;
  EXPECT_FALSE(ca::retreat_safe(open_map(),scan,{},{},.01,.01,{}));
}

TEST(RetreatPlanner, InvalidRayOutsideRearNewRegionDoesNotDemandSeeingThroughBody){
  auto scan=table_scan();scan.ranges[360]=std::numeric_limits<float>::quiet_NaN();
  EXPECT_TRUE(ca::retreat_safe(open_map(),scan,{},{},.03,.01,{}));
}

TEST(RetreatPlanner, SparseRearRaysCannotCoverDeclaredMargin){
  auto scan=table_scan();scan.angle_increment=.08;scan.ranges.assign(79,2.0);
  scan.angle_max=scan.angle_min+78*scan.angle_increment;
  EXPECT_FALSE(ca::retreat_safe(open_map(),scan,{},{},.20,.01,{}));
}

TEST(RetreatPlanner, StaticUnknownOrOccupiedAlongRetreatRemainsForbidden){
  for(int8_t value:{int8_t(-1),int8_t(100)}){
    auto map=open_map();set_cell(map,-.38,0,value);const auto before=map.data;
    ASSERT_TRUE(ca::free_footprint(map,{}));
    EXPECT_FALSE(ca::retreat_safe(map,table_scan(),{},{},.20,.01,{}));EXPECT_EQ(map.data,before);
  }
}

TEST(RetreatPlanner, AlreadyUnsafeStaticFootprintCannotUseRawRayException){
  auto map=open_map();set_cell(map,0,0,-1);
  EXPECT_FALSE(ca::retreat_safe(map,table_scan(),{},{},0,0,{}));
  EXPECT_FALSE(ca::plan_retreat(map,table_scan(),{},{}).allowed);
}

TEST(RetreatPlanner, RotatingMapAndPoseTogetherPreservesGeometry){
  auto map=open_map();const double a=.73,c=std::cos(a),s=std::sin(a);
  const double x=map.info.origin.position.x,y=map.info.origin.position.y;
  map.info.origin.position.x=c*x-s*y;map.info.origin.position.y=s*x+c*y;
  map.info.origin.orientation.z=std::sin(a/2);map.info.origin.orientation.w=std::cos(a/2);
  const auto plan=ca::plan_retreat(map,table_scan(),{0,0,a},{});
  ASSERT_TRUE(plan.allowed)<<plan.reason;EXPECT_TRUE(plan.clears_dynamic_envelope);
  EXPECT_TRUE(ca::retreat_safe(map,table_scan(),{0,0,a},{},plan.distance,plan.speed,{}));
}

TEST(RetreatPlanner, OffsetLaserUsesActualOriginForRearCoverageAndPoints){
  const ca::Pose laser{-.05,.015,.15};const auto scan=table_scan(laser);
  const auto plan=ca::plan_retreat(open_map(),scan,{},laser);
  ASSERT_TRUE(plan.allowed)<<plan.reason;EXPECT_TRUE(plan.needs_retreat);EXPECT_TRUE(plan.clears_dynamic_envelope);
  EXPECT_TRUE(ca::retreat_safe(open_map(),scan,{},laser,plan.distance,plan.speed,{}));
}

TEST(RetreatPlanner, LimitedDistanceReturnsShortSafeActionIfFullReleaseIsUnavailable){
  const ca::Pose laser{-.05,0,0};auto scan=open_scan();point(scan,-.05,-.24,laser);
  ca::RetreatOptions o;o.max_distance=.025;const auto plan=ca::plan_retreat(open_map(),scan,{},laser,o);
  ASSERT_TRUE(plan.allowed)<<plan.reason;EXPECT_TRUE(plan.needs_retreat);EXPECT_FALSE(plan.clears_dynamic_envelope);
  EXPECT_GT(plan.distance,0);EXPECT_LT(plan.distance,o.max_distance);
}

TEST(RetreatPlanner, ClearCurrentRawEnvelopeDoesNotInventRetreatForHistoricalCells){
  const auto plan=ca::plan_retreat(open_map(),open_scan(),{},{});
  EXPECT_FALSE(plan.allowed);EXPECT_FALSE(plan.needs_retreat);EXPECT_TRUE(plan.clears_dynamic_envelope);
  EXPECT_EQ(plan.distance,0);EXPECT_EQ(plan.speed,0);
}

TEST(RetreatPlanner, NegativeOrNonfiniteCommandAndBudgetAreRejected){
  const auto map=open_map();const auto scan=table_scan();ca::RetreatOptions o;
  EXPECT_FALSE(ca::retreat_safe(map,scan,{},{},-.01,.01,o));
  EXPECT_FALSE(ca::retreat_safe(map,scan,{},{},.01,-.01,o));
  EXPECT_FALSE(ca::retreat_safe(map,scan,{},{},.25,.03,o));
  for(double value:{-1.,std::numeric_limits<double>::infinity(),std::numeric_limits<double>::quiet_NaN()}){
    o.max_distance=value;EXPECT_FALSE(ca::plan_retreat(map,scan,{},{},o).allowed);
  }
}

TEST(RetreatPlanner, InvalidFootprintTransformAndScanMetadataAreRejected){
  ca::RetreatOptions o;o.margin=0;EXPECT_FALSE(ca::plan_retreat(open_map(),table_scan(),{},{},o).allowed);
  o={};o.braking_deceleration=0;EXPECT_FALSE(ca::plan_retreat(open_map(),table_scan(),{},{},o).allowed);
  auto scan=table_scan();scan.angle_max=0;EXPECT_FALSE(ca::retreat_safe(open_map(),scan,{},{},.01,.01,{}));
  EXPECT_FALSE(ca::retreat_safe(open_map(),table_scan(),{},{.5,0,0},.01,.01,{}));
}

TEST(RetreatPlanner, CurrentFootprintCheckDoesNotAuthorizeForwardOrRotation){
  const auto plan=ca::plan_retreat(open_map(),table_scan(),{},{});
  ASSERT_TRUE(plan.allowed);const auto end=ca::retreat_detail::end_pose({1,2,.6},plan.distance);
  EXPECT_LT(end.x,1);EXPECT_LT(end.y,2);EXPECT_DOUBLE_EQ(end.yaw,.6);
  // 本模块只返回后退正幅值；不存在前进、横移或转向指令字段。
  EXPECT_GT(plan.distance,0);EXPECT_GT(plan.speed,0);
}

TEST(RetreatPlanner, ComplementaryRealRaysFromTwoFramesCanJointlyCoverRear){
  auto a=std::make_shared<sensor_msgs::msg::LaserScan>(table_scan());
  auto b=std::make_shared<sensor_msgs::msg::LaserScan>(table_scan());
  a->ranges[10]=std::numeric_limits<float>::quiet_NaN();
  b->ranges[20]=std::numeric_limits<float>::quiet_NaN();
  EXPECT_FALSE(ca::retreat_safe(open_map(),*a,{},{},.03,.01,{}));
  EXPECT_FALSE(ca::retreat_safe(open_map(),*b,{},{},.03,.01,{}));
  const std::vector<ca::RetreatScan> frames{{a,{}},{b,{}}};
  EXPECT_TRUE(ca::retreat_safe(open_map(),frames,{},.03,.01,{}));
  const auto plan=ca::plan_retreat(open_map(),frames,{});
  ASSERT_TRUE(plan.allowed)<<plan.reason;EXPECT_TRUE(plan.clears_dynamic_envelope);
}

TEST(RetreatPlanner, CommonMissingRearViewAcrossFramesRemainsForbidden){
  auto a=std::make_shared<sensor_msgs::msg::LaserScan>(table_scan());
  auto b=std::make_shared<sensor_msgs::msg::LaserScan>(table_scan());
  a->ranges[10]=b->ranges[10]=std::numeric_limits<float>::quiet_NaN();
  const std::vector<ca::RetreatScan> frames{{a,{}},{b,{}}};
  EXPECT_FALSE(ca::retreat_safe(open_map(),frames,{},.03,.01,{}));
}

TEST(RetreatPlanner, AnyHistoricalObstacleInsideSweepPreventsUnionFromHidingIt){
  auto current=std::make_shared<sensor_msgs::msg::LaserScan>(table_scan());
  auto historical=std::make_shared<sensor_msgs::msg::LaserScan>(table_scan());point(*historical,-.18,0);
  const std::vector<ca::RetreatScan> frames{{current,{}},{historical,{}}};
  EXPECT_FALSE(ca::retreat_safe(open_map(),frames,{},.04,.01,{}));
}

TEST(RetreatPlanner, UnionUsesEachFrameTransformedLaserOrigin){
  const ca::Pose shifted{-.04,.01,.12};
  auto current=std::make_shared<sensor_msgs::msg::LaserScan>(table_scan());
  auto historical=std::make_shared<sensor_msgs::msg::LaserScan>(table_scan(shifted));
  current->ranges[10]=std::numeric_limits<float>::quiet_NaN();
  const std::vector<ca::RetreatScan> frames{{current,{}},{historical,shifted}};
  EXPECT_TRUE(ca::retreat_safe(open_map(),frames,{},.03,.01,{}));
  const auto plan=ca::plan_retreat(open_map(),frames,{});EXPECT_TRUE(plan.allowed)<<plan.reason;
}

TEST(RetreatPlanner, EmptyMissingOrUnboundedFrameSetsAreRejected){
  const auto scan=std::make_shared<sensor_msgs::msg::LaserScan>(table_scan());
  EXPECT_FALSE(ca::retreat_safe(open_map(),std::vector<ca::RetreatScan>{},{},.01,.01,{}));
  EXPECT_FALSE(ca::retreat_safe(open_map(),std::vector<ca::RetreatScan>{{nullptr,{}}},{},.01,.01,{}));
  EXPECT_FALSE(ca::retreat_safe(open_map(),std::vector<ca::RetreatScan>(9,{scan,{}}),{},.01,.01,{}));
}

TEST(RetreatPlanner, AllInvalidReturnsCannotPassEvenCurrentFootprintInspection){
  auto scan=open_scan();scan.ranges.assign(scan.ranges.size(),std::numeric_limits<float>::quiet_NaN());
  EXPECT_FALSE(ca::retreat_safe(open_map(),scan,{},{},0,0,{}));
  const auto plan=ca::plan_retreat(open_map(),scan,{},{});
  EXPECT_FALSE(plan.allowed);EXPECT_FALSE(plan.clears_dynamic_envelope);
}

TEST(RetreatPlanner, EveryOptionAndPoseCoordinateRequiresFiniteInput){
  double ca::RetreatOptions::*fields[]={&ca::RetreatOptions::max_distance,&ca::RetreatOptions::max_speed,
    &ca::RetreatOptions::braking_deceleration,&ca::RetreatOptions::response_seconds,&ca::RetreatOptions::margin,
    &ca::RetreatOptions::front,&ca::RetreatOptions::rear,&ca::RetreatOptions::half_width,&ca::RetreatOptions::dynamic_radius};
  for(const auto field:fields){
    ca::RetreatOptions o;o.*field=std::numeric_limits<double>::quiet_NaN();
    EXPECT_FALSE(ca::retreat_safe(open_map(),table_scan(),{},{},.01,.01,o));
  }
  const double nan=std::numeric_limits<double>::quiet_NaN();
  for(const ca::Pose p:std::vector<ca::Pose>{{nan,0,0},{0,nan,0},{0,0,nan}}){
    EXPECT_FALSE(ca::retreat_safe(open_map(),table_scan(),p,{},.01,.01,{}));
    EXPECT_FALSE(ca::retreat_safe(open_map(),table_scan(),{},p,.01,.01,{}));
  }
}
