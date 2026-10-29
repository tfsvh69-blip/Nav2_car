#pragma once
#include <memory>
#include <cmath>
#include <limits>
#include <karto_sdk/Karto.h>
namespace carcar_mapping_autonomy {
// Karto把最大端点映射到Round(extent/resolution)，网格必须包含该索引。
// 只修复包围盒的闭区间大小；射线计数、占用阈值、坐标原点及SDK实现不变。
class InclusiveOccupancyGrid:public karto::OccupancyGrid {
 using karto::OccupancyGrid::OccupancyGrid;
public:
 static std::unique_ptr<karto::OccupancyGrid> create(const karto::LocalizedRangeScanVector&scans,
   double resolution,unsigned min_pass,double occupancy){
  if(scans.empty()||!std::isfinite(resolution)||resolution<=0)return {};
  karto::BoundingBox2 bounds;bool any=false;for(const auto*scan:scans)if(scan){bounds.Add(scan->GetBoundingBox());any=true;}
  if(!any)return {};
  const auto size=bounds.GetSize();const double width=std::floor(size.GetWidth()/resolution+.5)+1;
  const double height=std::floor(size.GetHeight()/resolution+.5)+1;
  if(!std::isfinite(width)||!std::isfinite(height)||width<1||height<1||
    width>std::numeric_limits<kt_int32s>::max()||height>std::numeric_limits<kt_int32s>::max()||width*height>100000000)return {};
  auto grid=std::unique_ptr<InclusiveOccupancyGrid>(new InclusiveOccupancyGrid(
    static_cast<kt_int32s>(width),static_cast<kt_int32s>(height),bounds.GetMinimum(),resolution));
  grid->SetMinPassThrough(min_pass);grid->SetOccupancyThreshold(occupancy);grid->CreateFromScans(scans);return grid;
 }
};
inline std::unique_ptr<karto::OccupancyGrid> rasterize_scans(
 const karto::LocalizedRangeScanVector& scans,double resolution,unsigned min_pass,double occupancy){
 return InclusiveOccupancyGrid::create(scans,resolution,min_pass,occupancy);
}
}
