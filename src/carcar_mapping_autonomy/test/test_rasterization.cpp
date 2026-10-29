#include <gtest/gtest.h>
#include "carcar_mapping_autonomy/rasterization.hpp"
using namespace carcar_mapping_autonomy;
static std::unique_ptr<karto::OccupancyGrid> cardinal_grid(bool endpoints){
 std::unique_ptr<karto::LaserRangeFinder> laser(karto::LaserRangeFinder::CreateLaserRangeFinder(karto::LaserRangeFinder_Custom,karto::Name("raster-boundary-laser")));
 laser->SetMinimumRange(.05);laser->SetMaximumRange(12.);laser->SetRangeThreshold(endpoints?8.:2.);
 laser->SetMinimumAngle(-M_PI);laser->SetMaximumAngle(M_PI);laser->SetAngularResolution(M_PI/2.);
 laser->SetOffsetPose(karto::Pose2(0,0,0));karto::SensorManager::GetInstance()->RegisterSensor(laser.get());
 std::vector<std::unique_ptr<karto::LocalizedRangeScan>> storage;karto::LocalizedRangeScanVector scans;
 for(int i=0;i<4;++i){auto scan=std::make_unique<karto::LocalizedRangeScan>(laser->GetName(),std::vector<double>(5,2.));scan->SetCorrectedPose(karto::Pose2(0,0,0));scans.push_back(scan.get());storage.push_back(std::move(scan));}
 auto grid=rasterize_scans(scans,.05,2,.1);karto::SensorManager::GetInstance()->UnregisterSensor(laser.get());return grid;
}
TEST(Rasterization, MaximumMeasuredEndpointsRemainInsideGridAndOccupied){
 auto grid=cardinal_grid(true);ASSERT_TRUE(grid);
 for(const auto&point:std::vector<karto::Vector2<kt_double>>{{-2,0},{0,-2},{2,0},{0,2}}){
  const auto cell=grid->WorldToGrid(point);const bool valid=grid->IsValidGridIndex(cell);EXPECT_TRUE(valid)<<point.GetX()<<","<<point.GetY();
  if(valid){EXPECT_EQ(*grid->GetDataPointer(cell),karto::GridStates_Occupied)<<point.GetX()<<","<<point.GetY();}
 }
}
TEST(Rasterization, RangeLimitDoesNotInventOccupiedBoundary){
 auto grid=cardinal_grid(false);ASSERT_TRUE(grid);
 for(const auto&point:std::vector<karto::Vector2<kt_double>>{{-2,0},{0,-2},{2,0},{0,2}}){
  const auto cell=grid->WorldToGrid(point);ASSERT_TRUE(grid->IsValidGridIndex(cell));
  EXPECT_NE(*grid->GetDataPointer(cell),karto::GridStates_Occupied);
 }
}
