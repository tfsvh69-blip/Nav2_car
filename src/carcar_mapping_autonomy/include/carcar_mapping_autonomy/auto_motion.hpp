#pragma once
#include <chrono>
#include <geometry_msgs/msg/twist.hpp>
#include "carcar_mapping_autonomy/subcell_departure.hpp"

namespace carcar_mapping_autonomy {
struct AutoMotionCheck {
  geometry_msgs::msg::Twist velocity;
  double scale=0,compute_ms=0;
  std::string reason;
};
// 采用Nav2 Collision Monitor的前向预测思路，在现有监督器中审查最终速度。
// 使用原图未知拒绝和连续圆包络，同时检查独立激光端点；不是新增速度发布入口。
inline AutoMotionCheck auto_motion_check(const nav_msgs::msg::OccupancyGrid&map,const Task&task,
    const Pose&start,const std::vector<Pose>&laser_points,const geometry_msgs::msg::Twist&requested,
    const geometry_msgs::msg::Twist&measured) {
  AutoMotionCheck result;const auto began=std::chrono::steady_clock::now();
  auto finish=[&](){result.compute_ms=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-began).count();return result;};
  if(!grid_valid(map)||!std::isfinite(start.x)||!std::isfinite(start.y)||!std::isfinite(start.yaw)||laser_points.size()>4096||
      !std::isfinite(requested.linear.x)||!std::isfinite(requested.angular.z)||!std::isfinite(measured.linear.x)||!std::isfinite(measured.angular.z)||
      !std::isfinite(requested.linear.y)||std::abs(requested.linear.y)>1e-9||!std::isfinite(measured.linear.y)||
      requested.linear.x<0||requested.linear.x>.18||std::abs(requested.angular.z)>.36||
      std::abs(measured.linear.x)>.25||std::abs(measured.angular.z)>.6||std::abs(measured.linear.y)>.02){
    result.reason="最终运动审查输入无效或超出已验证的前进包络";return finish();
  }
  for(const auto&p:laser_points)if(!std::isfinite(p.x)||!std::isfinite(p.y)){
    result.reason="最终运动审查含无效激光端点";return finish();
  }
  // 延迟0.30秒后按现有0.20m/s²、0.80rad/s²向候选速度变化；预测1秒后制动至停稳。
  // 这是软件验证模型，不把配置加速度当作实测制动能力；上线前仍须受看护核验。
  constexpr double dt=.05,reaction=.30,drive=1.,accel=.20,angular_accel=.80;
  bool budget=false;std::string rejected;
  auto safe=[&](double scale){
    Pose pose=start;double v=measured.linear.x,w=measured.angular.z;
    for(int step=0;step<50;++step){
      if(step%10==0&&std::chrono::steady_clock::now()-began>std::chrono::milliseconds(8)){
        budget=true;rejected="最终扫掠检查超过8毫秒预算";return false;
      }
      const double t=step*dt;
      if(t+1e-9>=reaction){
        const double target_v=t<drive?requested.linear.x*scale:0;
        const double target_w=t<drive?requested.angular.z*scale:0;
        v+=std::clamp(target_v-v,-accel*dt,accel*dt);
        w+=std::clamp(target_w-w,-angular_accel*dt,angular_accel*dt);
      }
      Pose next=pose;const double angle=w*dt;
      if(std::abs(w)<1e-9){next.x+=v*dt*std::cos(pose.yaw);next.y+=v*dt*std::sin(pose.yaw);}
      else{next.x+=v/w*(std::sin(pose.yaw+angle)-std::sin(pose.yaw));next.y+=v/w*(std::cos(pose.yaw)-std::cos(pose.yaw+angle));}
      next.yaw=std::remainder(pose.yaw+angle,2*M_PI);
      // 弧线与弦的偏差加入圆半径，逐段连续检查，不漏过格角或单个回波。
      const double sagitta=std::abs(w)<1e-9?0:std::abs(v/w)*(1-std::cos(std::abs(angle)/2));
      const double radius=.22+sagitta;
      if(!task.inside(next)||!free_connector(map,pose,next,radius)){
        rejected="预测执行与制动范围触及原图障碍、未知区域或任务边界";return false;
      }
      const double dx=next.x-pose.x,dy=next.y-pose.y,length2=dx*dx+dy*dy;
      for(const auto&p:laser_points){
        const double u=length2>1e-15?std::clamp(((p.x-pose.x)*dx+(p.y-pose.y)*dy)/length2,0.,1.):0;
        if(std::hypot(p.x-pose.x-u*dx,p.y-pose.y-u*dy)<=radius){
          rejected="实时激光进入预测执行与制动范围";return false;
        }
      }
      pose=next;
      if(t>=drive&&std::abs(v)+std::abs(w)<1e-9)return true;
    }
    rejected="预测时间内未能证明停稳";return false;
  };
  if(safe(1)){result.velocity=requested;result.scale=1;return finish();}
  if(budget){result.reason=rejected;return finish();}
  const auto obstruction=rejected;
  if(!safe(0)){result.reason=rejected+"；保持零输出等待净空恢复";return finish();}
  double lower=0,upper=1;
  for(int i=0;i<7;++i){const double middle=(lower+upper)/2;if(safe(middle))lower=middle;else upper=middle;if(budget)break;}
  if(budget){result.reason=rejected;return finish();}
  result.scale=lower;result.velocity.linear.x=requested.linear.x*lower;result.velocity.angular.z=requested.angular.z*lower;
  result.reason=obstruction+"；按已验证扫掠缩减速度并持续观测";return finish();
}
}  // namespace carcar_mapping_autonomy
