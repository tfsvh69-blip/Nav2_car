#include <gtest/gtest.h>
#include <limits>
#include <vector>
#include "carcar_mapping_autonomy/tuning_transaction.hpp"
namespace ca=carcar_mapping_autonomy;
TEST(TuningValues,RejectsMissingUnknownNonfiniteAndOutOfRangeFields){
  const ca::TuningValues v;auto j=v.json();EXPECT_NO_THROW(ca::TuningValues::parse(j));
  j.erase("local_cost_scaling_factor");EXPECT_THROW(ca::TuningValues::parse(j),std::runtime_error);
  j=v.json();j["extra"]=1;EXPECT_THROW(ca::TuningValues::parse(j),std::runtime_error);
  for(const auto&entry:std::vector<std::pair<std::string,double>>{{"global_inflation_radius",.219},{"global_inflation_radius",1.501},
      {"local_inflation_radius",1.01},{"local_cost_scaling_factor",.49},{"global_cost_scaling_factor",15.01},
      {"local_inflation_radius",std::numeric_limits<double>::infinity()}}){j=v.json();j[entry.first]=entry.second;EXPECT_THROW(ca::TuningValues::parse(j),std::runtime_error);}
}
TEST(TuningValues,ConservativeGuardDoesNotUnderstateEitherInterpretationCoefficient){
  EXPECT_DOUBLE_EQ(ca::tuning_guard_factor(5,3),5);EXPECT_DOUBLE_EQ(ca::tuning_guard_factor(5,8),8);
  EXPECT_DOUBLE_EQ(ca::tuning_cost_distance(.3,.22),.22);EXPECT_DOUBLE_EQ(ca::tuning_cost_distance(.3,.6),.3);
}
TEST(TuningAck,RequiresExactNodeRevisionCoefficientsAndCompletedFullCycle){
  nlohmann::json j={{"node","/local_costmap/local_costmap"},{"layer","inflation_layer"},{"revision",12},{"radius",.3},{"factor",5.},{"completed",true},{"full_rebuild",true}};
  EXPECT_TRUE(ca::tuning_ack_matches(j,"/local_costmap/local_costmap",12,.3,5));
  EXPECT_FALSE(ca::tuning_ack_matches(j,"/local_costmap/local_costmap",13,.3,5));
  EXPECT_FALSE(ca::tuning_ack_matches(j,"/global_costmap/global_costmap",12,.3,5));
  EXPECT_FALSE(ca::tuning_ack_matches(j,"/local_costmap/local_costmap",12,.3,3));
  j["full_rebuild"]=false;EXPECT_FALSE(ca::tuning_ack_matches(j,"/local_costmap/local_costmap",12,.3,5));
}
