#pragma once
#include <karto_sdk/Mapper.h>
namespace carcar_mapping_autonomy {
// 冻结后的单帧核验允许原有局部匹配，禁止借此再次优化整张位姿图。
// 作用域退出（包括异常）恢复原开关；正式建图仍执行正常回环及LoopGuard复核。
class FrozenScanValidation {
public:
  FrozenScanValidation(karto::Mapper*mapper,bool frozen):mapper_(frozen?mapper:nullptr){
    if(mapper_){saved_=mapper_->getParamDoLoopClosing();mapper_->setParamDoLoopClosing(false);}
  }
  ~FrozenScanValidation(){if(mapper_)mapper_->setParamDoLoopClosing(saved_);}
  FrozenScanValidation(const FrozenScanValidation&)=delete;
  FrozenScanValidation&operator=(const FrozenScanValidation&)=delete;
private:
  karto::Mapper*mapper_=nullptr;bool saved_=false;
};
}
