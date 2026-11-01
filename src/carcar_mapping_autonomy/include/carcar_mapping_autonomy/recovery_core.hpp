#pragma once
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <queue>
#include <string>
#include <vector>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include "carcar_mapping_autonomy/task.hpp"

namespace carcar_mapping_autonomy {
// 仅用重力方向的相对变化作受扰观察，不产生姿态或融合里程计。
// 行驶中的比力方向会随加减速变化，不能单独当成车身倾斜。
// 轮式已停稳后的持续倾斜，或明显冲击、角速度，才进入受扰验证。
class DisturbanceMonitor {
public:
  bool ready=false,disturbed=false,stable=false;
  double tilt=0,norm=0,gyro=0,stable_since=0,last_at=-1e9;
  std::string reason;
  void update(const std::array<double,3>&a,const std::array<double,3>&g,bool still,double now) {
    if(now-last_at>.5){stable_since=0;if(!ready){count_=0;sum_={};}}
    last_at=now;norm=length(a);gyro=length(g);
    if(!std::isfinite(norm)||!std::isfinite(gyro)||norm<1e-3){
      stable=false;stable_since=0;reason="IMU观测无效";if(!ready){count_=0;sum_={};}return;
    }
    const bool quiet=still&&gyro<.12&&norm>8.0&&norm<11.6;
    if(!ready){
      if(!quiet){
        count_=0;sum_={};stable_since=0;stable=false;
        reason=!still?"等待轮式停稳后采集IMU重力基线":(gyro>=.12?"IMU角速度尚未满足静稳门槛0.12rad/s":"IMU加速度模长不在8.0～11.6m/s²，需核对单位／数据质量");
        return;
      }
      if(!count_)baseline_at_=now;
      for(size_t i=0;i<3;++i)sum_[i]+=a[i];
      ++count_;
      reason="采集停稳IMU重力基线：需要连续2秒且至少20个有效样本";
      if(now-baseline_at_>=2&&count_>=20){
        const double s=length(sum_);for(size_t i=0;i<3;++i)baseline_[i]=sum_[i]/s;
        ready=true;stable_since=now;stable=true;reason.clear();
      }
      return;
    }
    double dot=0;for(size_t i=0;i<3;++i)dot+=a[i]/norm*baseline_[i];
    tilt=std::acos(std::clamp(dot,-1.0,1.0));
    // 先验候选阈值；不是本车倾角标定结论。无需四元数或轴向猜测。
    // 起步瞬间比力会先于轮式“未停稳”变化；倾斜需在停稳状态下持续，避免一次采样就闭门。
    if(still&&tilt>.14){if(lean_since_<=0)lean_since_=now;}else lean_since_=0;
    // 0.10m/s短测中正常加减速曾出现单帧14m/s²；冲击也要连续，避免一次振动闭门。
    const bool shock_now=norm<5||norm>16||gyro>1.5;
    if(shock_now){if(shock_since_<=0)shock_since_=now;}else shock_since_=0;
    const bool leaned=lean_since_>0&&now-lean_since_>=.75;
    const bool shock=shock_since_>0&&now-shock_since_>=.25;
    if(leaned||shock){
      disturbed=true;reason=leaned?"IMU重力方向变化超过8度候选阈值":"IMU冲击／角速度异常";
    }
    stable=quiet&&tilt<.07;
    if(!disturbed)reason=stable?"":"IMU观察到运动或重力方向尚未回到稳定范围，等待停稳";
    if(!stable)stable_since=0;else if(stable_since==0)stable_since=now;
  }
  bool settled(double now,double hold=2)const{return ready&&stable&&now-last_at<.5&&stable_since>0&&now-stable_since>=hold;}
  void acknowledge(){disturbed=false;reason.clear();}
private:
  static double length(const std::array<double,3>&v){return std::hypot(v[0],std::hypot(v[1],v[2]));}
  size_t count_=0;double baseline_at_=0,lean_since_=0,shock_since_=0;std::array<double,3>baseline_{},sum_{};
};
struct ReferenceScore {
  double mean=.4,coverage=0,hit_ratio=0,free_conflict=1,cost=1;
  size_t points=0;
  bool acceptable()const{return points>=50&&coverage>=.80&&hit_ratio>=.70&&mean<=.10&&free_conflict<=.08;}
};
// AMCL候选的独立几何检查：只读检查点，未知区不提供匹配支持。
// 距离场为8邻域度量，误差上界由栅格分辨率限制；绝不向生产图试错。
class ReferenceMatcher {
public:
  struct Ray{double x,y,ox,oy;};
  explicit ReferenceMatcher(const nav_msgs::msg::OccupancyGrid&m):map_(m){
    const auto n=static_cast<size_t>(m.info.width)*m.info.height;
    if(n==0||n>12000000||m.data.size()!=n||!std::isfinite(m.info.resolution)||m.info.resolution<=0)
      throw std::runtime_error("检查点栅格无效");
    if(std::abs(m.info.origin.orientation.x)>1e-6||std::abs(m.info.origin.orientation.y)>1e-6||
      std::abs(m.info.origin.orientation.z)>1e-6||std::abs(m.info.origin.orientation.w-1)>1e-6)
      throw std::runtime_error("检查点地图原点旋转不受支持");
    distance_.assign(n,.5f);
    using Entry=std::pair<float,size_t>;std::priority_queue<Entry,std::vector<Entry>,std::greater<Entry>> q;
    for(size_t i=0;i<n;++i)if(m.data[i]>=65){distance_[i]=0;q.push({0,i});}
    while(!q.empty()){
      auto [d,i]=q.top();q.pop();if(d!=distance_[i]||d>.45)continue;
      const int x=i%m.info.width,y=i/m.info.width;
      for(int dy=-1;dy<=1;++dy)for(int dx=-1;dx<=1;++dx){
        if(!dx&&!dy)continue;
        const int nx=x+dx,ny=y+dy;
        if(nx<0||ny<0||nx>=static_cast<int>(m.info.width)||ny>=static_cast<int>(m.info.height))continue;
        const size_t ni=static_cast<size_t>(ny)*m.info.width+nx;
        const float next=d+m.info.resolution*((dx&&dy)?1.41421356f:1.f);
        if(next<distance_[ni]){distance_[ni]=next;q.push({next,ni});}
      }
    }
  }
  ReferenceScore score(const Pose&p,const std::vector<Ray>&rays)const{
    ReferenceScore s;s.points=rays.size();if(rays.empty())return s;
    size_t known=0,hits=0,free_samples=0,conflicts=0;double sum=0;
    const double c=std::cos(p.yaw),t=std::sin(p.yaw);
    for(const auto&r:rays){
      const double wx=p.x+c*r.x-t*r.y,wy=p.y+t*r.x+c*r.y;
      const auto i=index(wx,wy);const double endpoint_distance=supported_endpoint_distance(wx,wy,i);
      if(std::isfinite(endpoint_distance)){++known;const double d=std::min(.4,endpoint_distance);sum+=d;if(d<=.12)++hits;}
      else sum+=.4;
      // 不在激光击中点附近检查自由空间，避免将栅格化误差当穿墙。
      for(double f:{.2,.4,.6,.8}){
        const double x=r.ox+(r.x-r.ox)*f,y=r.oy+(r.y-r.oy)*f;
        const auto j=index(p.x+c*x-t*y,p.y+t*x+c*y);
        if(j>=0&&map_.data[j]>=0){++free_samples;if(map_.data[j]>=65)++conflicts;}
      }
    }
    s.mean=sum/rays.size();s.coverage=double(known)/rays.size();s.hit_ratio=double(hits)/rays.size();
    s.free_conflict=free_samples?double(conflicts)/free_samples:1;
    s.cost=s.mean+.4*(1-s.coverage)+.5*s.free_conflict;return s;
  }
  struct Verification{ReferenceScore score;double ambiguity_margin=0,global_margin=0;bool unique=false;size_t global_evaluations=0;};
  Verification verify(const Pose&p,const std::vector<Ray>&rays)const{
    Verification v;v.score=score(p,rays);
    if(!v.score.acceptable())return v;
    double alternative=std::numeric_limits<double>::infinity();
    // 明确检查与候选显著不同的附近位置／朝向；重复走廊和镜像可拒绝。
    for(int iy=-5;iy<=5;++iy)for(int ix=-5;ix<=5;++ix)for(int ia=-4;ia<=4;++ia){
      const double dx=ix*.12,dy=iy*.12,da=ia*.15;
      if(std::hypot(dx,dy)<.30&&std::abs(da)<.30)continue;
      Pose q{p.x+dx,p.y+dy,p.yaw+da};alternative=std::min(alternative,score(q,rays).cost);
    }
    v.ambiguity_margin=alternative-v.score.cost;
    // 固定地图全范围的离散竞争者检查，再细化16个最佳竞争区域。
    // 工作量有界；这是采样歧义检查，不声称数学上的全局唯一性。
    std::vector<Ray> sparse;const size_t stride=std::max<size_t>(1,rays.size()/36);
    for(size_t i=0;i<rays.size();i+=stride)sparse.push_back(rays[i]);
    const double width=map_.info.width*map_.info.resolution,height=map_.info.height*map_.info.resolution;
    const double step=std::max(.30,std::sqrt(width*height/2500.0));
    struct Alternative{double cost;Pose pose;};std::vector<Alternative>best;
    for(double y=map_.info.origin.position.y+step*.5;y<map_.info.origin.position.y+height;y+=step)
      for(double x=map_.info.origin.position.x+step*.5;x<map_.info.origin.position.x+width;x+=step){
        const auto cell=index(x,y);if(cell<0||map_.data[cell]<0||map_.data[cell]>=65)continue;
        for(int ia=0;ia<24;++ia){
          Pose q{x,y,ia*M_PI/12};
          if(std::hypot(q.x-p.x,q.y-p.y)<.6&&std::abs(std::remainder(q.yaw-p.yaw,2*M_PI))<.6)continue;
          const double cost=score(q,sparse).cost;++v.global_evaluations;
          if(best.size()<16||cost<best.back().cost){
            best.push_back({cost,q});std::sort(best.begin(),best.end(),[](const auto&a,const auto&b){return a.cost<b.cost;});if(best.size()>16)best.pop_back();
          }
        }
      }
    double global=std::numeric_limits<double>::infinity();
    for(const auto&candidate:best)for(int iy=-2;iy<=2;++iy)for(int ix=-2;ix<=2;++ix)for(int ia=-2;ia<=2;++ia){
      const Pose q{candidate.pose.x+ix*step/4,candidate.pose.y+iy*step/4,candidate.pose.yaw+ia*M_PI/48};
      if(std::hypot(q.x-p.x,q.y-p.y)<.30&&std::abs(std::remainder(q.yaw-p.yaw,2*M_PI))<.30)continue;
      global=std::min(global,score(q,rays).cost);
    }
    v.global_margin=global-v.score.cost;v.unique=v.global_evaluations>=24&&v.ambiguity_margin>=.025&&v.global_margin>=.025;
    return v;
  }
private:
  double supported_endpoint_distance(double x,double y,int64_t i)const{
    if(i>=0&&map_.data[i]>=0)return distance_[i];
    // 墙线位于格内而非格心。亚格位姿误差可能让回波落到相邻未知格或裁剪边界外，
    // 只允许一个栅格对角线内、确有占用支持的端点；真正未覆盖区仍不能贡献匹配。
    const double resolution=map_.info.resolution,tolerance=std::sqrt(2.0)*resolution;
    if(i>=0&&distance_[i]>2*resolution)return std::numeric_limits<double>::infinity();
    if(!std::isfinite(x)||!std::isfinite(y))return std::numeric_limits<double>::infinity();
    const double x0=map_.info.origin.position.x,y0=map_.info.origin.position.y;
    const double x1=x0+map_.info.width*resolution,y1=y0+map_.info.height*resolution;
    if(x<x0-resolution||x>x1+resolution||y<y0-resolution||y>y1+resolution)return std::numeric_limits<double>::infinity();
    const int ix=int(std::floor((x-x0)/resolution)),iy=int(std::floor((y-y0)/resolution));
    double best=std::numeric_limits<double>::infinity();
    for(int cy=std::max(0,iy-2);cy<=std::min(int(map_.info.height)-1,iy+2);++cy)
      for(int cx=std::max(0,ix-2);cx<=std::min(int(map_.info.width)-1,ix+2);++cx){
        if(map_.data[size_t(cy)*map_.info.width+cx]<65)continue;
        best=std::min(best,std::hypot(x-(x0+(cx+.5)*resolution),y-(y0+(cy+.5)*resolution)));
      }
    return best<=tolerance?best:std::numeric_limits<double>::infinity();
  }
  int64_t index(double x,double y)const{
    if(!std::isfinite(x)||!std::isfinite(y))return -1;
    const int ix=std::floor((x-map_.info.origin.position.x)/map_.info.resolution),
      iy=std::floor((y-map_.info.origin.position.y)/map_.info.resolution);
    if(ix<0||iy<0||ix>=static_cast<int>(map_.info.width)||iy>=static_cast<int>(map_.info.height))return -1;
    return static_cast<int64_t>(iy)*map_.info.width+ix;
  }
  nav_msgs::msg::OccupancyGrid map_;std::vector<float>distance_;
};
class MultiFrameVerification {
public:
  size_t count=0;int64_t last_stamp=0;double first_time=0;
  bool observe(const Pose&p,int64_t stamp,double now,bool quality,size_t required=6,double hold=1.0){
    if(stamp<=last_stamp)return false;
    last_stamp=stamp;
    if(!quality){count=0;return false;}
    if(count&&(std::hypot(p.x-last_.x,p.y-last_.y)>.08||std::abs(std::remainder(p.yaw-last_.yaw,2*M_PI))>.08))count=0;
    if(count==0)first_time=now;
    last_=p;++count;
    return count>=required&&now-first_time>=hold;
  }
  void reset(){*this={};}
private:Pose last_;
};
} // namespace carcar_mapping_autonomy
