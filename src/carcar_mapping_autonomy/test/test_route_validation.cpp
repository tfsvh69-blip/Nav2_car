#include <gtest/gtest.h>
#include "carcar_mapping_autonomy/route_validation.hpp"
#include "carcar_mapping_autonomy/planning_departure.hpp"
using namespace carcar_mapping_autonomy;
static RouteValidator::Job route(){
 RouteValidator::Job j;auto m=std::make_shared<nav_msgs::msg::OccupancyGrid>();m->header.frame_id="map";
 m->info.width=m->info.height=100;m->info.resolution=.05;m->info.origin.position.x=m->info.origin.position.y=-2.5;m->info.origin.orientation.w=1;m->data.assign(10000,0);j.map=m;
 auto p=std::make_shared<nav_msgs::msg::Path>();p->header.frame_id="map";
 geometry_msgs::msg::PoseStamped a,b;a.pose.orientation.w=b.pose.orientation.w=1;b.pose.position.x=1;p->poses={a,b};j.path=p;j.target={1,0,0};return j;
}
TEST(RouteValidation, SamplesBetweenSparseWaypoints){auto j=route();auto m=std::make_shared<nav_msgs::msg::OccupancyGrid>(*j.map);m->data[50*100+60]=-1;j.map=m;EXPECT_FALSE(RouteValidator::check(j));}
TEST(RouteValidation, TraversedObstacleDoesNotInvalidateRemainingPath){auto j=route();auto m=std::make_shared<nav_msgs::msg::OccupancyGrid>(*j.map);m->data[50*100+50]=100;j.map=m;
 auto p=std::make_shared<nav_msgs::msg::Path>(*j.path);auto mid=p->poses[0];mid.pose.position.x=.7;p->poses.insert(p->poses.begin()+1,mid);j.path=p;j.current={.7,0,0};j.remaining=true;EXPECT_TRUE(RouteValidator::check(j));}
TEST(RouteValidation, InvalidEndpointAndTinyResolutionFailClosed){auto j=route();j.target={2,0,0};EXPECT_FALSE(RouteValidator::check(j));
 auto m=std::make_shared<nav_msgs::msg::OccupancyGrid>(*j.map);m->info.resolution=1e-15;j.map=m;EXPECT_FALSE(RouteValidator::check(j));}
TEST(RouteValidation, LargeUnchangedMapAndGrowthPreserveWorldRoute){auto j=route();auto m=std::make_shared<nav_msgs::msg::OccupancyGrid>(*j.map);m->info.width=1000;m->info.height=1000;m->data.assign(1000000,0);j.map=m;
 EXPECT_TRUE(RouteValidator::check(j));auto shifted=*m;shifted.info.origin.position.x-=.05;EXPECT_FALSE(same_grid(*m,shifted));j.map=std::make_shared<nav_msgs::msg::OccupancyGrid>(shifted);EXPECT_TRUE(RouteValidator::check(j));}
TEST(RouteValidation, WorkerReturnsOperationAndMapVersion){RouteValidator worker;auto j=route();j.operation=7;j.attempt=3;j.revision=19;worker.submit(j);
 std::optional<RouteValidator::Result> result;for(int i=0;i<100&&!result;++i){std::this_thread::sleep_for(std::chrono::milliseconds(2));result=worker.take();}
 ASSERT_TRUE(result);EXPECT_TRUE(result->safe);EXPECT_EQ(result->job.operation,7U);EXPECT_EQ(result->job.attempt,3U);EXPECT_EQ(result->job.revision,19U);}
TEST(PlanningMask, PreservesUnknownAndObstacleWithoutEditingMap){auto j=route();auto map=*j.map;map.data[50*100+60]=-1;map.data[60*100+60]=100;
 auto original=map.data;auto mask=planning_keepout_mask(map);ASSERT_TRUE(mask);
 EXPECT_EQ(map.data,original);EXPECT_EQ(mask->data[50*100+60],100);EXPECT_EQ(mask->data[50*100+57],100);
 EXPECT_EQ(mask->data[60*100+60],100);EXPECT_EQ(mask->data[50*100+50],0);
 map.info.origin.orientation.z=std::sin(.1);map.info.origin.orientation.w=std::cos(.1);EXPECT_FALSE(planning_keepout_mask(map));}

TEST(PlanningMask, GridDiagonalCannotCutAcrossUnknownFootprint){auto j=route();auto m=*j.map;m.data[46*100+66]=-1;
 auto mask=planning_keepout_mask(m);ASSERT_TRUE(mask);
 // 两端圆包络安全，弦中点确实与未知方块相交；检查真实几何，避免依赖旧格心量化误差。
 const Pose a{.550,-.175,0},b{.825,.100,0},between{.6875,-.0375,0};
 EXPECT_TRUE(free_footprint(m,a));EXPECT_TRUE(free_footprint(m,b));EXPECT_FALSE(free_footprint(m,between));
 EXPECT_FALSE(planning_footprint(m,a));
 int x=std::floor((a.x-m.info.origin.position.x)/m.info.resolution),y=std::floor((a.y-m.info.origin.position.y)/m.info.resolution);
 EXPECT_EQ(mask->data[y*m.info.width+x],100);
}

TEST(PlanningMask, DepartureOnlyReleasesExtraMarginAndRetainsRealExclusions){
 auto j=route();auto m=*j.map;for(int x=0;x<100;++x)m.data[62*100+x]=100;
 auto mask=planning_keepout_mask(m);ASSERT_TRUE(mask);const Pose current{0,.34,0};
 ASSERT_TRUE(free_footprint(m,current));ASSERT_FALSE(planning_footprint(m,current));
 ASSERT_TRUE(allow_planning_departure(m,current,*mask));
 int cx=50,cy=56;EXPECT_EQ(mask->data[cy*100+cx],0);
 for(int y=0;y<100;++y)for(int x=0;x<100;++x){if(mask->data[y*100+x]==0){EXPECT_TRUE(free_footprint(m,cell_world(m,x,y)));}}
 EXPECT_EQ(mask->data[62*100+cx],100);EXPECT_EQ(mask->data[59*100+cx],100);
 m.data[cy*100+cx]=-1;auto blocked=planning_keepout_mask(m);ASSERT_TRUE(blocked);
 auto original=blocked->data;EXPECT_FALSE(allow_planning_departure(m,current,*blocked));EXPECT_EQ(original,blocked->data);
}

TEST(FootprintGeometry, SubCellPositionDoesNotRejectClearCorner){
 auto m=*route().map;m.info.origin.position.x=m.info.origin.position.y=0;
 m.data[27*100+26]=100;
 // NAV-018实车：格心近似判为相撞，但0.22m圆到方块的最近距离约0.226m。
 EXPECT_TRUE(free_footprint(m,{1.52342435,1.54509796,0}));
 // 同一栅格内向障碍移动后必须拒绝，不能把整格位置视为等价。
 EXPECT_FALSE(free_footprint(m,{1.501,1.501,0}));
}

TEST(FootprintGeometry, SubCellPositionCannotHideDiagonalCollision){
 auto m=*route().map;m.info.origin.position.x=m.info.origin.position.y=0;
 m.data[32*100+35]=-1;
 // 格心距离sqrt(29)*0.05使旧算法跳过；真实车位到未知方块却只有约0.207m。
 EXPECT_FALSE(free_footprint(m,{1.549,1.549,0}));
 EXPECT_TRUE(free_footprint(m,{1.501,1.501,0}));
}

TEST(FootprintGeometry, TangencyOutsideAndRotationKeepWholeEnvelope){
 auto m=*route().map;m.info.origin.position.x=m.info.origin.position.y=0;
 m.data[30*100+40]=-1;const double cell=m.info.resolution;
 const Pose touching{40*cell-.22,30.5*cell,0},clear{touching.x-.001,touching.y,0};
 EXPECT_FALSE(free_footprint(m,touching));EXPECT_TRUE(free_footprint(m,clear));
 EXPECT_FALSE(free_footprint(m,{.1,1,0}));EXPECT_FALSE(free_footprint(m,clear,-.001));
 EXPECT_FALSE(free_footprint(m,clear,std::numeric_limits<double>::quiet_NaN()));
 const double angle=.37;m.info.origin.position.x=-.8;m.info.origin.position.y=.4;
 m.info.origin.orientation.z=std::sin(angle/2);m.info.origin.orientation.w=std::cos(angle/2);
 auto world=[&](const Pose&p){return Pose{-.8+std::cos(angle)*p.x-std::sin(angle)*p.y,.4+std::sin(angle)*p.x+std::cos(angle)*p.y,0};};
 EXPECT_FALSE(free_footprint(m,world(touching)));EXPECT_TRUE(free_footprint(m,world(clear)));
}

TEST(FootprintGeometry, UnsafeGridCenterCannotBeReportedAsDeparture){
 auto m=*route().map;m.info.origin.position.x=m.info.origin.position.y=0;m.data[27*100+26]=100;
 const Pose current{1.52342435,1.54509796,0};ASSERT_TRUE(free_footprint(m,current));
 ASSERT_FALSE(free_footprint(m,cell_world(m,30,30)));auto mask=planning_keepout_mask(m);ASSERT_TRUE(mask);
 const auto original=mask->data;EXPECT_FALSE(allow_planning_departure(m,current,*mask));EXPECT_EQ(mask->data,original);
}

TEST(FootprintGeometry, ExactTangencyIsSymmetricAtNegativeCellEdges){
 auto m=*route().map;m.info.origin.position.x=m.info.origin.position.y=0;m.info.resolution=.125;
 // 二进制精确数值，排除浮点取整巧合：圆与四个方向的方格恰好相切。
 for(const auto&cell:std::vector<std::pair<int,int>>{{37,40},{42,40},{40,37},{40,42}}){
  m.data.assign(10000,0);m.data[cell.second*100+cell.first]=-1;
  EXPECT_FALSE(free_footprint(m,{5,5,0},.25))<<cell.first<<","<<cell.second;
 }
 m.data.assign(10000,0);EXPECT_FALSE(free_footprint(m,{.25,5,0},.25));
 EXPECT_FALSE(free_footprint(m,{5,.25,0},.25));EXPECT_TRUE(free_footprint(m,{.251,5,0},.25));
}

TEST(ObservationVisibility, WallCannotBeProjectedThroughButOpenDoorIsVisible){
 auto m=*route().map;Task task;for(int y=0;y<100;++y){m.data[y*100+63]=100;for(int x=64;x<100;++x)m.data[y*100+x]=-1;}
 const Pose frontier{.725,0,0};EXPECT_FALSE(safe_observation(m,frontier,task));
 for(int y=40;y<=60;++y)m.data[y*100+63]=0;
 auto goal=safe_observation(m,frontier,task);ASSERT_TRUE(goal);EXPECT_TRUE(observation_visible(m,*goal,frontier));
}

TEST(ObservationVisibility, UnknownEndpointCroppedEdgeAndRotatedCorner){
 auto m=*route().map;auto from=cell_world(m,50,50),to=cell_world(m,55,55);m.data[51*100+52]=100;
 EXPECT_FALSE(observation_visible(m,from,to));m.data[51*100+52]=0;m.data[55*100+55]=-1;
 EXPECT_TRUE(observation_visible(m,from,to));m.data[53*100+53]=-1;EXPECT_FALSE(observation_visible(m,from,to));
 m.data[53*100+53]=0;m.data[51*100+52]=100;m.info.origin.orientation.z=std::sin(.37/2);m.info.origin.orientation.w=std::cos(.37/2);
 EXPECT_FALSE(observation_visible(m,cell_world(m,50,50),cell_world(m,55,55)));
 m=*route().map;EXPECT_TRUE(observation_visible(m,cell_world(m,6,50),cell_world(m,-1,50)));
 m.data[50*100+2]=100;EXPECT_FALSE(observation_visible(m,cell_world(m,6,50),cell_world(m,-1,50)));
 EXPECT_FALSE(safe_observation(m,{1e100,0,0},Task{}));
}
