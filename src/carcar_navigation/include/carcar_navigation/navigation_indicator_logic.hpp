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

#ifndef CARCAR_NAVIGATION__NAVIGATION_INDICATOR_LOGIC_HPP_
#define CARCAR_NAVIGATION__NAVIGATION_INDICATOR_LOGIC_HPP_

#include <cmath>
#include <cstdint>
#include <string>

namespace carcar_navigation
{

inline constexpr uint8_t kDefaultIndicatorBrightness = 85;

enum class FlashMode
{
  SOLID,
  SLOW_FLASH,
  FAST_FLASH
};

enum class OperatorLight
{
  OFF,
  NAV_CYAN,
  NAV_CYAN_SLOW_FLASH,
  FAIL_RED,
  LOST_FLASH
};

struct ColorRGB
{
  uint8_t r{0};
  uint8_t g{0};
  uint8_t b{0};

  bool operator==(const ColorRGB & other) const
  {
    return r == other.r && g == other.g && b == other.b;
  }
  bool operator!=(const ColorRGB & other) const
  {
    return !(*this == other);
  }
};

struct VisualState
{
  ColorRGB color;
  FlashMode mode{FlashMode::SOLID};
  std::string description;
};

struct IndicatorStatusInput
{
  std::string stage_code{"IDLE"};
  std::string cause_code{"NONE"};
  bool is_still{true};
  std::string task_status{"IDLE"};
  double status_age_s{0.0};
};

struct IndicatorOutput
{
  ColorRGB color{0, 0, 0};
  bool beep_trigger{false};
  uint16_t beep_duration_ms{0};
  std::string display_desc;
};

inline ColorRGB apply_flash(FlashMode mode, double time_sec, ColorRGB color)
{
  switch (mode) {
    case FlashMode::SOLID:
      return color;
    case FlashMode::SLOW_FLASH: {
      const double phase = std::fmod(time_sec, 1.0);
      return (phase < 0.5) ? color : ColorRGB{0, 0, 0};
    }
    case FlashMode::FAST_FLASH: {
      const double phase = std::fmod(time_sec, 0.5);
      return (phase < 0.25) ? color : ColorRGB{0, 0, 0};
    }
  }
  return color;
}

inline bool is_fail_code(const std::string & code)
{
  return code == "FAILED" || code == "CANCEL_UNCONFIRMED" || code == "STOP_NOT_CONFIRMED" ||
    code == "RECOVERY_EXHAUSTED" || code == "RECOVERY_TOTAL_TIMEOUT" ||
    code == "NAVIGATION_FAILED" || code == "ABORTED" || code == "TASK_FAILED" ||
    code == "CHASSIS_TIMEOUT" || code == "LOCK_STOP";
}

inline bool is_attention_stage(const std::string & stage_code)
{
  return stage_code == "LOC_WAIT" || stage_code == "LOC_RECOVER" ||
    stage_code == "RECOVERY_WAIT" || stage_code == "SPINNING" ||
    stage_code == "BACKING" || stage_code == "REAR_CHECK" || stage_code == "PARK_WAIT";
}

inline VisualState operator_visual(OperatorLight mode, uint8_t brightness)
{
  VisualState vs;
  const uint8_t b = brightness;
  switch (mode) {
    case OperatorLight::NAV_CYAN:
      vs.color = ColorRGB{0, b, b};
      vs.mode = FlashMode::SOLID;
      vs.description = "导航运行 (青色常亮)";
      break;
    case OperatorLight::NAV_CYAN_SLOW_FLASH:
      vs.color = ColorRGB{0, b, b};
      vs.mode = FlashMode::SLOW_FLASH;
      vs.description = "恢复或等待 (青色慢闪)";
      break;
    case OperatorLight::FAIL_RED:
      vs.color = ColorRGB{b, 0, 0};
      vs.mode = FlashMode::SOLID;
      vs.description = "失败锁定 (红色常亮)";
      break;
    case OperatorLight::LOST_FLASH:
      vs.color = ColorRGB{b, 0, b};
      vs.mode = FlashMode::FAST_FLASH;
      vs.description = "状态丢失超时 (紫色快闪)";
      break;
    case OperatorLight::OFF:
    default:
      vs.color = ColorRGB{0, 0, 0};
      vs.mode = FlashMode::SOLID;
      vs.description = "关闭";
      break;
  }
  return vs;
}

inline OperatorLight classify_operator_light(const IndicatorStatusInput & input, double status_timeout_s)
{
  if (input.status_age_s > status_timeout_s) {
    return OperatorLight::LOST_FLASH;
  }
  if (input.stage_code == "OFF" || input.stage_code == "SHUTDOWN") {
    return OperatorLight::OFF;
  }
  if (input.stage_code == "FAILED" || is_fail_code(input.cause_code) || is_fail_code(input.task_status)) {
    return OperatorLight::FAIL_RED;
  }
  if (is_attention_stage(input.stage_code) ||
    (input.is_still && !input.cause_code.empty() && input.cause_code != "NONE" &&
      input.cause_code != "GOAL_REACHED" && input.cause_code != "USER_CANCELLED")) {
    return OperatorLight::NAV_CYAN_SLOW_FLASH;
  }
  return OperatorLight::NAV_CYAN;
}

class IndicatorStateMachine
{
public:
  void update(
    double now_sec,
    const IndicatorStatusInput & input,
    bool mute_audio,
    double status_timeout_s,
    uint8_t brightness,
    IndicatorOutput & output)
  {
    const OperatorLight mode = classify_operator_light(input, status_timeout_s);
    const VisualState vs = operator_visual(mode, brightness);
    output.color = apply_flash(vs.mode, now_sec, vs.color);
    output.display_desc = vs.description;
    output.beep_trigger = false;
    output.beep_duration_ms = 0;

    const bool success = input.stage_code == "SUCCEEDED" || input.cause_code == "GOAL_REACHED";
    const bool fail = mode == OperatorLight::FAIL_RED;
    if (mode != OperatorLight::LOST_FLASH && !mute_audio && now_sec >= last_sound_completion_time_sec_) {
      if ((success && !last_success_) || (fail && !last_fail_)) {
        output.beep_trigger = true;
        output.beep_duration_ms = 120;
        last_sound_completion_time_sec_ = now_sec + 0.12 + 3.0;
      }
    }
    last_success_ = success;
    last_fail_ = fail;
  }

  void reset()
  {
    last_sound_completion_time_sec_ = 0.0;
    last_success_ = false;
    last_fail_ = false;
  }

private:
  double last_sound_completion_time_sec_{0.0};
  bool last_success_{false};
  bool last_fail_{false};
};

}  // namespace carcar_navigation

#endif  // CARCAR_NAVIGATION__NAVIGATION_INDICATOR_LOGIC_HPP_
