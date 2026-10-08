// Copyright 2026 carcar maintainers
// Licensed under the Apache License, Version 2.0.

#include <cmath>
#include <cstdint>
#include <string>
#include <vector>

#include "gtest/gtest.h"

#include "carcar_camera/flicker_logic.hpp"

using carcar_camera::bin_index;
using carcar_camera::count_runs;
using carcar_camera::count_transitions;
using carcar_camera::run_length_encode;
using carcar_camera::valid_resolution;
using carcar_camera::voxel_center;
using carcar_camera::voxel_key;

TEST(FlickerLogic, ResolutionGuard)
{
  EXPECT_TRUE(valid_resolution(0.02));
  EXPECT_TRUE(valid_resolution(1.0));
  EXPECT_FALSE(valid_resolution(0.0));
  EXPECT_FALSE(valid_resolution(-0.02));
  EXPECT_FALSE(valid_resolution(1.5));
  EXPECT_FALSE(valid_resolution(std::nan("")));
}

TEST(FlickerLogic, VoxelKeyUsesFloorAndRoundTripsThroughCenter)
{
  // Nav2 体素层同样使用向下取整，负坐标必须落到更小的格而不是靠近零。
  EXPECT_EQ(voxel_key(0.02, 0.0, 0.0, 0.02).x, 1);
  EXPECT_EQ(voxel_key(-0.01, 0.0, 0.0, 0.02).x, -1);
  EXPECT_EQ(voxel_key(-0.02, 0.0, 0.0, 0.02).x, -1);
  // 体素中心反解必须回到同一格，保证标记点与输入点使用同一 key。
  for (int32_t index = -50; index <= 50; ++index) {
    const double center = voxel_center(index, 0.02);
    EXPECT_EQ(voxel_key(center, 0.0, 0.0, 0.02).x, index);
  }
}

TEST(FlickerLogic, RunsDetectSingleContiguousBlock)
{
  EXPECT_EQ(count_runs({}), 0);
  EXPECT_EQ(count_runs({0}), 1);
  EXPECT_EQ(count_runs({3, 4, 5}), 1);
  EXPECT_EQ(count_runs({0, 1, 2, 3, 4}), 1);
}

TEST(FlickerLogic, RunsCountEachReappearance)
{
  // 出现、消失、再出现 = 两段，即一次闪烁。
  EXPECT_EQ(count_runs({0, 1, 3, 4}), 2);
  EXPECT_EQ(count_runs({0, 2, 4}), 3);
  EXPECT_EQ(count_runs({1, 2, 5, 6, 9}), 3);
}

TEST(FlickerLogic, TransitionsDistinguishOngoingFromEnded)
{
  // 整段观测都出现：只算开始那一次翻转。
  EXPECT_EQ(count_transitions({0, 1, 2}, 3), 1);
  // 出现过又消失且未再出现：开始与结束各一次。
  EXPECT_EQ(count_transitions({0, 1, 2}, 5), 2);
  EXPECT_EQ(count_transitions({0, 1}, 5), 2);
  // 两段且末段延续到结束：2*2-1。
  EXPECT_EQ(count_transitions({0, 2, 3, 4}, 5), 3);
  EXPECT_EQ(count_transitions({}, 5), 0);
}

TEST(FlickerLogic, RunLengthEncodingIsReadableAndComplete)
{
  EXPECT_EQ(run_length_encode({}, 3), "off3");
  EXPECT_EQ(run_length_encode({0, 1, 2}, 3), "on3");
  EXPECT_EQ(run_length_encode({2, 3}, 5), "off2,on2,off1");
  EXPECT_EQ(run_length_encode({0, 2}, 4), "on1,off1,on1,off1");
  // 编码中的 on 总数必须等于出现样本数，off 总数必须等于缺失样本数。
  const std::vector<int64_t> samples{1, 2, 5, 9, 10, 11};
  const int64_t total = 14;
  const std::string rle = run_length_encode(samples, total);
  // 注意 "on"/"off" 前缀长度不同，必须按首个数字切分，不能统一 substr(2)。
  int64_t on_total = 0, off_total = 0;
  size_t cursor = 0;
  while (cursor < rle.size()) {
    const size_t comma = rle.find(',', cursor);
    const std::string token = rle.substr(cursor, comma - cursor);
    const bool is_on = !token.compare(0, 2, "on") && token.compare(0, 3, "off") != 0;
    const int64_t value = std::stoll(token.substr(token.find_first_of("0123456789")));
    if (is_on) {on_total += value;} else {off_total += value;}
    if (comma == std::string::npos) {break;}
    cursor = comma + 1;
  }
  EXPECT_EQ(on_total, static_cast<int64_t>(samples.size()));
  EXPECT_EQ(on_total + off_total, total);
}

TEST(FlickerLogic, BinIndexRejectsInvalidInputButKeepsNegativeBins)
{
  EXPECT_EQ(bin_index(0.55, 0.0, 0.10), 5);
  // 车体右侧 y<0 是合法分箱，不能被当成无效输入丢掉。
  EXPECT_EQ(bin_index(-0.01, 0.0, 0.10), -1);
  EXPECT_EQ(bin_index(-0.35, 0.0, 0.10), -4);
  EXPECT_EQ(bin_index(std::nan(""), 0.0, 0.10), carcar_camera::kInvalidBin);
  EXPECT_EQ(bin_index(1.0, 0.0, 0.0), carcar_camera::kInvalidBin);
  EXPECT_EQ(bin_index(1.0, std::nan(""), 0.10), carcar_camera::kInvalidBin);
}
