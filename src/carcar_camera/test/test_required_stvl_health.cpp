#include <gtest/gtest.h>
#include "carcar_camera/required_stvl_health.hpp"

namespace {
void feed(carcar_camera::RequiredStvlHealth & h,int i,bool local=true,bool global=true) {
  const double t=static_cast<double>(100+i*10)/100.0;
  h.camera(true,t); h.cloud(1000000000LL+i*100000000LL,0.01,t);
  h.layer(0,local,i+1,t); h.layer(1,global,i+1,t);
}
}
TEST(RequiredStvlHealth, StartupAndThreeSecondRecovery) {
  carcar_camera::RequiredStvlHealth h;
  EXPECT_FALSE(h.allowed(0));
  for (int i=0;i<30;++i) {feed(h,i); EXPECT_FALSE(h.allowed((100+i*10)/100.0));}
  feed(h,31); EXPECT_TRUE(h.allowed(4.1)) << h.reason();
  h.camera(false,4.2); EXPECT_FALSE(h.allowed(4.2));
  for (int i=33;i<63;++i) {feed(h,i); EXPECT_FALSE(h.allowed((100+i*10)/100.0));}
  feed(h,64); EXPECT_TRUE(h.allowed(7.4));
}
TEST(RequiredStvlHealth, OldCloudCannotRenewAndLayerRestartCloses) {
  carcar_camera::RequiredStvlHealth h;
  for (int i=0;i<=31;++i) {feed(h,i); h.allowed((100+i*10)/100.0);}
  ASSERT_TRUE(h.allowed(4.1)) << h.reason();
  h.camera(true,4.7); h.cloud(4100000000LL,0.01,4.7);
  h.layer(0,true,99,4.7); h.layer(1,true,99,4.7);
  EXPECT_FALSE(h.allowed(4.7));
  feed(h,38); h.allowed(4.8);
  h.layer(0,true,1,4.81); EXPECT_FALSE(h.allowed(4.81));
}
TEST(RequiredStvlHealth, CurrentSourceAndAdvancingUpdateAreBothRequired) {
  carcar_camera::RequiredStvlHealth h;
  for (int i=0;i<=31;++i) {feed(h,i); h.allowed((100+i*10)/100.0);}
  h.layer(0,false,33,4.2); EXPECT_FALSE(h.allowed(4.2));
  for (int i=33;i<=64;++i) {feed(h,i); h.allowed((100+i*10)/100.0);}
  ASSERT_TRUE(h.allowed(7.4));
  h.camera(true,8.2); h.cloud(8200000000LL,0.01,8.2);
  h.layer(0,true,65,8.2); h.layer(1,true,65,8.2);
  EXPECT_FALSE(h.allowed(8.2)); // 相同序号不能伪造地图心跳。
}
TEST(RequiredStvlHealth, FutureCloudAndMissingSecondMapHoldStop) {
  carcar_camera::RequiredStvlHealth h;
  h.camera(true,1); h.cloud(1000000000LL,-0.01,1);
  EXPECT_FALSE(h.allowed(1));
  for (int i=1;i<40;++i) {
    h.camera(true,1+i*0.1); h.cloud(1000000000LL+i*100000000LL,0,1+i*0.1);
    h.layer(0,true,i,1+i*0.1); EXPECT_FALSE(h.allowed(1+i*0.1));
  }
}
