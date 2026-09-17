// Copyright 2026 carcar maintainers
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <gtest/gtest.h>
#include "carcar_navigation/navigation_indicator_logic.hpp"

using namespace carcar_navigation;

namespace {
IndicatorStatusInput stage_input(const std::string & stage, bool still = false)
{
  IndicatorStatusInput input;
  input.stage_code = stage;
  input.cause_code = "NONE";
  input.is_still = still;
  input.status_age_s = 0.0;
  return input;
}

IndicatorStatusInput cause_input(const std::string & cause)
{
  IndicatorStatusInput input;
  input.stage_code = "FOLLOWING";
  input.cause_code = cause;
  input.is_still = cause != "NONE" && cause != "GOAL_REACHED" && cause != "USER_CANCELLED";
  input.status_age_s = 0.0;
  if (cause == "GOAL_REACHED") {
    input.stage_code = "SUCCEEDED";
    input.is_still = true;
  } else if (cause == "USER_CANCELLED") {
    input.stage_code = "CANCELED";
    input.is_still = true;
  }
  return input;
}
}  // namespace

TEST(NavigationIndicatorLogicTest, ApplyFlashPatterns)
{
  const ColorRGB cyan{0, 85, 85};
  const ColorRGB off{0, 0, 0};

  EXPECT_EQ(apply_flash(FlashMode::SOLID, 0.0, cyan), cyan);
  EXPECT_EQ(apply_flash(FlashMode::SOLID, 0.7, cyan), cyan);

  EXPECT_EQ(apply_flash(FlashMode::SLOW_FLASH, 0.2, cyan), cyan);
  EXPECT_EQ(apply_flash(FlashMode::SLOW_FLASH, 0.7, cyan), off);
  EXPECT_EQ(apply_flash(FlashMode::SLOW_FLASH, 1.2, cyan), cyan);

  EXPECT_EQ(apply_flash(FlashMode::FAST_FLASH, 0.1, cyan), cyan);
  EXPECT_EQ(apply_flash(FlashMode::FAST_FLASH, 0.35, cyan), off);
  EXPECT_EQ(apply_flash(FlashMode::FAST_FLASH, 0.6, cyan), cyan);
}

TEST(NavigationIndicatorLogicTest, StageVisualMapping)
{
  const uint8_t b = kDefaultIndicatorBrightness;
  const ColorRGB cyan{0, b, b};
  const ColorRGB red{b, 0, 0};

  auto vis = [&](const std::string & stage) {
    return operator_visual(classify_operator_light(stage_input(stage), 2.0), b);
  };

  auto v_idle = vis("IDLE");
  EXPECT_EQ(v_idle.color, cyan);
  EXPECT_EQ(v_idle.mode, FlashMode::SOLID);

  auto v_loc_wait = vis("LOC_WAIT");
  EXPECT_EQ(v_loc_wait.color, cyan);
  EXPECT_EQ(v_loc_wait.mode, FlashMode::SLOW_FLASH);

  auto v_follow = vis("FOLLOWING");
  EXPECT_EQ(v_follow.color, cyan);
  EXPECT_EQ(v_follow.mode, FlashMode::SOLID);

  auto v_plan = vis("PLANNING");
  EXPECT_EQ(v_plan.color, cyan);
  EXPECT_EQ(v_plan.mode, FlashMode::SOLID);

  auto v_replan = vis("REPLANNING");
  EXPECT_EQ(v_replan.color, cyan);
  EXPECT_EQ(v_replan.mode, FlashMode::SOLID);

  auto v_spin = vis("SPINNING");
  EXPECT_EQ(v_spin.color, cyan);
  EXPECT_EQ(v_spin.mode, FlashMode::SLOW_FLASH);

  auto v_backing = vis("BACKING");
  EXPECT_EQ(v_backing.color, cyan);
  EXPECT_EQ(v_backing.mode, FlashMode::SLOW_FLASH);

  auto v_park = vis("PARK_WAIT");
  EXPECT_EQ(v_park.color, cyan);
  EXPECT_EQ(v_park.mode, FlashMode::SLOW_FLASH);

  auto v_failed = vis("FAILED");
  EXPECT_EQ(v_failed.color, red);
  EXPECT_EQ(v_failed.mode, FlashMode::SOLID);

  EXPECT_EQ(cyan, (ColorRGB{0, 85, 85}));
}

TEST(NavigationIndicatorLogicTest, CauseVisualMapping)
{
  const uint8_t b = kDefaultIndicatorBrightness;
  const ColorRGB cyan{0, b, b};
  const ColorRGB red{b, 0, 0};

  auto vis = [&](const std::string & cause) {
    return operator_visual(classify_operator_light(cause_input(cause), 2.0), b);
  };

  auto v_tf = vis("TF_UNAVAILABLE");
  EXPECT_EQ(v_tf.color, cyan);
  EXPECT_EQ(v_tf.mode, FlashMode::SLOW_FLASH);

  auto v_scan = vis("SCAN_OBSTACLE");
  EXPECT_EQ(v_scan.color, cyan);
  EXPECT_EQ(v_scan.mode, FlashMode::SLOW_FLASH);

  auto v_costmap = vis("COSTMAP_OBSTACLE");
  EXPECT_EQ(v_costmap.color, cyan);
  EXPECT_EQ(v_costmap.mode, FlashMode::SLOW_FLASH);

  auto v_prog = vis("PROGRESS_STAGNATION");
  EXPECT_EQ(v_prog.color, cyan);
  EXPECT_EQ(v_prog.mode, FlashMode::SLOW_FLASH);

  auto v_dwb = vis("DWB_NO_TRAJECTORY");
  EXPECT_EQ(v_dwb.color, cyan);
  EXPECT_EQ(v_dwb.mode, FlashMode::SLOW_FLASH);

  auto v_cancel_unconfirmed = vis("CANCEL_UNCONFIRMED");
  EXPECT_EQ(v_cancel_unconfirmed.color, red);
  EXPECT_EQ(v_cancel_unconfirmed.mode, FlashMode::SOLID);

  auto v_goal = vis("GOAL_REACHED");
  EXPECT_EQ(v_goal.color, cyan);
  EXPECT_EQ(v_goal.mode, FlashMode::SOLID);

  auto v_cancel = vis("USER_CANCELLED");
  EXPECT_EQ(v_cancel.color, cyan);
  EXPECT_EQ(v_cancel.mode, FlashMode::SOLID);
}

TEST(NavigationIndicatorLogicTest, StateMachineStatusTimeout)
{
  IndicatorStateMachine sm;
  IndicatorStatusInput input;
  input.stage_code = "FOLLOWING";
  input.cause_code = "NONE";
  input.is_still = false;
  input.status_age_s = 2.5;

  IndicatorOutput output;
  sm.update(10.0, input, false, 2.0, kDefaultIndicatorBrightness, output);

  EXPECT_EQ(output.color, (ColorRGB{kDefaultIndicatorBrightness, 0, kDefaultIndicatorBrightness}));
  EXPECT_FALSE(output.beep_trigger);
  EXPECT_EQ(output.beep_duration_ms, 0);
  EXPECT_NE(output.display_desc.find("状态丢失超时"), std::string::npos);
}

TEST(NavigationIndicatorLogicTest, StateMachineCollapsedOperatorModes)
{
  IndicatorStateMachine sm;
  IndicatorStatusInput input;
  input.stage_code = "FOLLOWING";
  input.cause_code = "COSTMAP_OBSTACLE";
  input.is_still = true;
  input.status_age_s = 0.1;
  const uint8_t b = kDefaultIndicatorBrightness;
  const ColorRGB cyan{0, b, b};

  IndicatorOutput output;
  sm.update(10.0, input, false, 2.0, b, output);
  EXPECT_NE(output.display_desc.find("恢复或等待"), std::string::npos);
  EXPECT_EQ(output.color, cyan);
  EXPECT_FALSE(output.beep_trigger);

  sm.update(12.2, input, false, 2.0, b, output);
  EXPECT_NE(output.display_desc.find("恢复或等待"), std::string::npos);
  EXPECT_EQ(apply_flash(FlashMode::SLOW_FLASH, 12.2, cyan), output.color);
  EXPECT_FALSE(output.beep_trigger);

  input.is_still = false;
  input.cause_code = "NONE";
  sm.update(13.2, input, false, 2.0, b, output);
  EXPECT_NE(output.display_desc.find("导航运行"), std::string::npos);
  EXPECT_EQ(output.color, cyan);
  EXPECT_FALSE(output.beep_trigger);
}

TEST(NavigationIndicatorLogicTest, SoundDeduplicationAndCooldown)
{
  IndicatorStateMachine sm;
  IndicatorStatusInput input;
  input.stage_code = "FAILED";
  input.cause_code = "CANCEL_UNCONFIRMED";
  input.is_still = true;
  input.status_age_s = 0.05;
  const uint8_t b = kDefaultIndicatorBrightness;

  IndicatorOutput output;
  sm.update(100.0, input, false, 2.0, b, output);
  EXPECT_TRUE(output.beep_trigger);
  EXPECT_EQ(output.beep_duration_ms, 120);
  EXPECT_EQ(output.color, (ColorRGB{b, 0, 0}));

  sm.update(100.05, input, false, 2.0, b, output);
  EXPECT_FALSE(output.beep_trigger);
  sm.update(101.0, input, false, 2.0, b, output);
  EXPECT_FALSE(output.beep_trigger);
  sm.update(105.0, input, false, 2.0, b, output);
  EXPECT_FALSE(output.beep_trigger);
}

TEST(NavigationIndicatorLogicTest, MuteAudioSuppressesBeep)
{
  IndicatorStateMachine sm;
  IndicatorStatusInput input;
  input.stage_code = "SUCCEEDED";
  input.cause_code = "GOAL_REACHED";
  input.status_age_s = 0.05;
  IndicatorOutput output;
  sm.update(1.0, input, true, 2.0, kDefaultIndicatorBrightness, output);
  EXPECT_FALSE(output.beep_trigger);
  EXPECT_EQ(output.color, (ColorRGB{0, kDefaultIndicatorBrightness, kDefaultIndicatorBrightness}));
}

TEST(NavigationIndicatorLogicTest, QuietStagesDoNotBeep)
{
  IndicatorStateMachine sm;
  const uint8_t b = kDefaultIndicatorBrightness;
  IndicatorOutput output;
  auto input = stage_input("PLANNING");
  sm.update(1.0, input, false, 2.0, b, output);
  EXPECT_FALSE(output.beep_trigger);
  input = stage_input("FOLLOWING");
  sm.update(1.1, input, false, 2.0, b, output);
  EXPECT_FALSE(output.beep_trigger);
  input = stage_input("SPINNING");
  sm.update(1.2, input, false, 2.0, b, output);
  EXPECT_FALSE(output.beep_trigger);
  input = stage_input("SUCCEEDED");
  sm.update(1.3, input, false, 2.0, b, output);
  EXPECT_TRUE(output.beep_trigger);
  EXPECT_EQ(output.beep_duration_ms, 120);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
