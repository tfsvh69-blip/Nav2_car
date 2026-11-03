#include <gtest/gtest.h>
#include "carcar_mapping_autonomy/recovery_initializer.hpp"
using namespace carcar_mapping_autonomy;
namespace {
nav_msgs::msg::OccupancyGrid reference(){
  nav_msgs::msg::OccupancyGrid m;m.header.frame_id="map";m.info.resolution=.05;
  m.info.width=m.info.height=100;m.info.origin.position.x=m.info.origin.position.y=-2.5;m.info.origin.orientation.w=1;
  m.data.assign(10000,0);
  for(unsigned y=0;y<100;++y)for(unsigned x=0;x<100;++x){
    const double wx=-2.5+(x+.5)*.05,wy=-2.5+(y+.5)*.05;
    if(std::abs(wx)>2.3||std::abs(wy)>2.0||(wx>.8&&wx<1.3&&wy>.3&&wy<.8)||
      (wx>-.9&&wx<-.5&&wy>1.0&&wy<1.4))m.data[y*100+x]=100;
  }
  return m;
}
sensor_msgs::msg::LaserScan scan(const nav_msgs::msg::OccupancyGrid&m,const Pose&p){
  sensor_msgs::msg::LaserScan s;s.angle_min=-M_PI;s.angle_increment=2*M_PI/180;s.angle_max=M_PI;
  s.range_min=.05;s.range_max=8;s.ranges.assign(181,8);
  for(size_t i=0;i<s.ranges.size();++i){const double angle=p.yaw+s.angle_min+i*s.angle_increment;
    for(double r=.05;r<7.95;r+=.005){const int x=std::floor((p.x+r*std::cos(angle)+2.5)/.05),y=std::floor((p.y+r*std::sin(angle)+2.5)/.05);
      if(x<0||y<0||x>=100||y>=100||m.data[y*100+x]>=65){s.ranges[i]=r;break;}
    }
  }
  return s;
}
}
TEST(RecoveryInitializer, ExistingKartoMatcherCorrectsTranslationAndYawBeforeAmcl){
  const auto m=reference();const Pose truth{.30,-.10,.20};const auto s=scan(m,truth);
  const auto result=recovery_initial_pose(m,s,{0,0,0},{});
  EXPECT_NEAR(result.x,truth.x,.08);EXPECT_NEAR(result.y,truth.y,.08);EXPECT_NEAR(result.yaw,truth.yaw,.05);
}
TEST(RecoveryInitializer, MissingRangesCannotProduceArtificialBoundaryAndRegistrationIsReleased){
  const auto m=reference();auto s=scan(m,{});std::fill(s.ranges.begin(),s.ranges.end(),std::numeric_limits<float>::quiet_NaN());
  EXPECT_THROW(recovery_initial_pose(m,s,{},{}),std::runtime_error);
  EXPECT_NO_THROW(recovery_initial_pose(m,scan(m,{}),{},{}));
}
