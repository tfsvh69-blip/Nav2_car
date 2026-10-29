#include "slam_toolbox/slam_mapper.hpp"
#include "carcar_mapping_autonomy/rasterization.hpp"
// 仅mapping_slam可执行文件内的2.6.10兼容入口，不替换系统库或普通SLAM入口。
// 父类地图发布/保存仍调用官方SMapper接口；使用原图优化后的扫描及原计数阈值。
namespace mapper_utils {
karto::OccupancyGrid* SMapper::getOccupancyGrid(const double&resolution){
 return carcar_mapping_autonomy::rasterize_scans(mapper_->GetAllProcessedScans(),resolution,
   static_cast<unsigned>(mapper_->getParamMinPassThrough()),mapper_->getParamOccupancyThreshold()).release();
}
}
