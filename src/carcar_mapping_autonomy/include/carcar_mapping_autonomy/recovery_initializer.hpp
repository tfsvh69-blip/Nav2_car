#pragma once
#include <memory>
#include <limits>
#include <tbb/global_control.h>
#include <karto_sdk/Mapper.h>
#include <sensor_msgs/msg/laser_scan.hpp>
#include "carcar_mapping_autonomy/recovery_core.hpp"

namespace carcar_mapping_autonomy {
// 复用SLAM Toolbox 2.6.10 Karto的相关匹配器，只构造只读参考栅格。
// 公有CorrelationGrid/CorrelateScan接口负责搜索；不加载或改写生产位姿图。
// 匹配响应仅用于AMCL初始候选，仍须通过独立几何、多解和多帧验证。
inline Pose recovery_initial_pose(const nav_msgs::msg::OccupancyGrid & map,
  const sensor_msgs::msg::LaserScan & input,const Pose & prior,const Pose & laser_offset){
  if(input.ranges.empty()||input.angle_increment<=0)throw std::runtime_error("重定位初值缺少有效激光");
  const tbb::global_control workers(tbb::global_control::max_allowed_parallelism,2);
  const double range=std::min(8.0,double(input.range_max));
  karto::Mapper mapper;
  std::unique_ptr<karto::ScanMatcher> matcher(karto::ScanMatcher::Create(&mapper,1.5,.025,.05,range));
  if(!matcher)throw std::runtime_error("Karto旁路匹配器初始化失败");
  auto*grid=matcher->GetCorrelationGrid();const auto roi=grid->GetROI();
  const karto::Pose2 sensor_center= karto::Transform(karto::Pose2(prior.x,prior.y,prior.yaw)).TransformPose(
    karto::Pose2(laser_offset.x,laser_offset.y,laser_offset.yaw));
  grid->GetCoordinateConverter()->SetOffset(karto::Vector2<double>(
    sensor_center.GetX()-.5*(roi.GetWidth()-1)*grid->GetResolution(),
    sensor_center.GetY()-.5*(roi.GetHeight()-1)*grid->GetResolution()));
  grid->Clear();
  for(unsigned y=0;y<map.info.height;++y)for(unsigned x=0;x<map.info.width;++x){
    if(map.data[size_t(y)*map.info.width+x]<65)continue;
    const auto p=grid->WorldToGrid(karto::Vector2<double>(map.info.origin.position.x+(x+.5)*map.info.resolution,
      map.info.origin.position.y+(y+.5)*map.info.resolution));
    if(p.GetX()<roi.GetX()||p.GetY()<roi.GetY()||p.GetX()>=roi.GetX()+roi.GetWidth()||p.GetY()>=roi.GetY()+roi.GetHeight())continue;
    *grid->GetDataPointer(p)=karto::GridStates_Occupied;grid->SmearPoint(p);
  }
  const karto::Name name("recovery_reference_laser");
  std::unique_ptr<karto::LaserRangeFinder>laser(karto::LaserRangeFinder::CreateLaserRangeFinder(karto::LaserRangeFinder_Custom,name));
  laser->SetMinimumRange(std::max(.15,double(input.range_min)));laser->SetMaximumRange(input.range_max);
  laser->SetMinimumAngle(input.angle_min);laser->SetMaximumAngle(input.angle_min+(input.ranges.size()-1)*input.angle_increment);
  laser->SetAngularResolution(input.angle_increment);laser->SetRangeThreshold(range);
  laser->SetOffsetPose(karto::Pose2(laser_offset.x,laser_offset.y,laser_offset.yaw));
  karto::SensorManager::GetInstance()->RegisterSensor(laser.get());
  struct Registration{ karto::LaserRangeFinder*laser;~Registration(){karto::SensorManager::GetInstance()->UnregisterSensor(laser);} } registration{laser.get()};
  karto::RangeReadingsVector readings;readings.reserve(input.ranges.size());
  // Karto的范围比较包含上界；缺测必须保留NaN，不能伪造成阈值处的墙。
  for(const auto value:input.ranges)readings.push_back(std::isfinite(value)&&value>=laser->GetMinimumRange()&&value<range-.05?
    value:std::numeric_limits<double>::quiet_NaN());
  karto::LocalizedRangeScan scan(name,readings);scan.SetOdometricPose(karto::Pose2(prior.x,prior.y,prior.yaw));
  scan.SetCorrectedPose(karto::Pose2(prior.x,prior.y,prior.yaw));
  karto::Pose2 mean;karto::Matrix3 covariance;covariance.SetToIdentity();
  try{
    const double response=matcher->CorrelateScan(&scan,sensor_center,karto::Vector2<double>(.75,.75),
      karto::Vector2<double>(.05,.05),.60,.05,false,mean,covariance,false);
    if(response<.50)throw std::runtime_error("Karto旁路粗匹配响应不足，不能建立AMCL初值");
    matcher->CorrelateScan(&scan,mean,karto::Vector2<double>(.05,.05),
      karto::Vector2<double>(.025,.025),.05,.01,false,mean,covariance,true);
    scan.SetSensorPose(mean);const auto p=scan.GetCorrectedPose();
    Pose output{p.GetX(),p.GetY(),p.GetHeading()};
    if(std::hypot(output.x-prior.x,output.y-prior.y)>.75||std::abs(std::remainder(output.yaw-prior.yaw,2*M_PI))>.60)
      throw std::runtime_error("Karto旁路候选超出有限恢复范围");
    return output;
  }catch(...){throw;}
}
}
