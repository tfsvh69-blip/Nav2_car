// 候选约束的几何回归：新开放格必须仍满足基础圆，未知／图外不放宽。
#include <iostream>
#include <gtest/gtest.h>
#include <random>
#include <stdexcept>
#include "carcar_mapping_autonomy/map_geometry.hpp"
namespace ca=carcar_mapping_autonomy;
void require(bool ok,const char*why){if(!ok)throw std::runtime_error(why);}
TEST(PlanningMargin, KnownCorridorsAndUnknownBoundaryInvariants){
 for(int outside:{-1,100})for(unsigned width:{9,10,11,12,13}){
  nav_msgs::msg::OccupancyGrid m;m.header.frame_id="map";m.info.width=60;m.info.height=width+40;m.info.resolution=.05;m.info.origin.orientation.w=1;
  m.data.assign(m.info.width*m.info.height,outside);
  for(unsigned y=20;y<20+width;++y)for(unsigned x=0;x<60;++x)m.data[y*60+x]=0;
  auto mask=ca::planning_keepout_mask(m);require(bool(mask),"掩膜缺失");int count=0,oldcount=0;
  for(unsigned y=20;y<20+width;++y){auto p=ca::cell_world(m,30,y);count+=mask->data[y*60+30]==0;oldcount+=ca::free_footprint(m,p,.22+std::sqrt(2.)*m.info.resolution);}
  if(outside==-1)require(count==oldcount,"未知边界被放宽");
  if(outside==100&&width==11)require(count==1&&oldcount==0,"55厘米已知直道未开放");
  if(width<=10)require(count==0,"50厘米及以下不应误报可用");
  std::cout<<"outside="<<outside<<",width="<<width*.05<<",old="<<oldcount<<",new="<<count<<'\n';
 }
 std::mt19937 random(17);size_t newly_open=0;
 for(int round=0;round<50;++round){
  nav_msgs::msg::OccupancyGrid m;m.header.frame_id="map";m.info.width=m.info.height=40;m.info.resolution=.05;m.info.origin.orientation.w=1;m.data.assign(1600,0);
  for(auto&v:m.data){auto n=random()%200;v=n==0?-1:n<3?100:0;}
  auto mask=ca::planning_keepout_mask(m);
  for(int y=0;y<40;++y)for(int x=0;x<40;++x){auto p=ca::cell_world(m,x,y);const bool open=mask->data[y*40+x]==0;
   const bool old=ca::free_footprint(m,p,.22+std::sqrt(2.)*m.info.resolution);
   require(!old||open,"候选反而关闭旧可用格");
   if(open){require(ca::free_footprint(m,p),"新开放格违反22厘米基础检查");ca::FootprintEvidence evidence;ca::free_footprint(m,p,.22+std::sqrt(2.)*m.info.resolution,&evidence);require(!evidence.invalid&&!evidence.outside&&evidence.unknown==0,"新开放格违反未知／图外保护");}
   newly_open+=open&&!old;
  }
 }
 std::cout<<"80000格固定种子几何对照通过，新增可用格="<<newly_open<<'\n';
}
