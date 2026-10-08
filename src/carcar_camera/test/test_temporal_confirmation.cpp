// Copyright 2026 carcar maintainers
// Licensed under the Apache License, Version 2.0.

#include <cstdint>

#include "gtest/gtest.h"

#include "carcar_camera/temporal_confirmation.hpp"

using carcar_camera::TemporalConfirmation;

TEST(TemporalConfirmation, RejectsInvalidParameters)
{
  EXPECT_TRUE((TemporalConfirmation{0.02, 3, 2}.valid()));
  EXPECT_TRUE((TemporalConfirmation{0.02, 1, 1}.valid()));
  EXPECT_FALSE((TemporalConfirmation{0.0, 3, 2}.valid()));
  EXPECT_FALSE((TemporalConfirmation{0.02, 0, 1}.valid()));
  EXPECT_FALSE((TemporalConfirmation{0.02, 3, 0}.valid()));
  // min_hits 不能超过窗口长度，否则永远无法确认。
  EXPECT_FALSE((TemporalConfirmation{0.02, 2, 3}.valid()));
}

TEST(TemporalConfirmation, SingleFrameBlipIsNeverAccepted)
{
  // 这正是要解决的问题：每帧都出现在不同位置的孤立飞点（轮廓 mixed pixel），
  // 任何一个体素都只被命中一次，因此永远不能进入标障点云。
  TemporalConfirmation confirmation(0.02, 3, 2);
  for (int64_t frame = 0; frame < 20; ++frame) {
    confirmation.begin_frame(frame);
    const double x = 1.0 + 0.05 * static_cast<double>(frame);  // 每帧换一个体素
    EXPECT_FALSE(confirmation.accept(x, 0.0, 0.10, frame))
      << "第 " << frame << " 帧的孤立飞点不应被接受";
  }
}

TEST(TemporalConfirmation, PersistentSurfaceIsAcceptedFromSecondFrame)
{
  TemporalConfirmation confirmation(0.02, 3, 2);
  confirmation.begin_frame(0);
  EXPECT_FALSE(confirmation.accept(1.0, 0.0, 0.10, 0)) << "首帧还不满足 2 次命中";
  for (int64_t frame = 1; frame < 10; ++frame) {
    confirmation.begin_frame(frame);
    EXPECT_TRUE(confirmation.accept(1.0, 0.0, 0.10, frame))
      << "持续存在的表面从第 2 帧起就应被接受";
  }
}

TEST(TemporalConfirmation, SameFrameRepeatedPointsCountOnce)
{
  // 同一帧内一个体素被多个点命中只能算一次，否则稠密表面会立刻“自证”。
  TemporalConfirmation confirmation(0.02, 3, 2);
  confirmation.begin_frame(0);
  for (int i = 0; i < 50; ++i) {
    EXPECT_FALSE(confirmation.accept(1.0, 0.0, 0.10, 0))
      << "同一帧内的重复命中不应累加";
  }
  confirmation.begin_frame(1);
  EXPECT_TRUE(confirmation.accept(1.0, 0.0, 0.10, 1));
}

TEST(TemporalConfirmation, HitsMustBeInsideTheWindow)
{
  // 窗口 2 帧、需要 2 次命中：隔帧出现（第 0、2 帧）不满足。
  TemporalConfirmation consecutive(0.02, 2, 2);
  consecutive.begin_frame(0);
  EXPECT_FALSE(consecutive.accept(1.0, 0.0, 0.10, 0));
  consecutive.begin_frame(1);
  EXPECT_TRUE(consecutive.accept(1.0, 0.0, 0.10, 1)) << "连续两帧应满足";
  TemporalConfirmation spaced(0.02, 2, 2);
  spaced.begin_frame(0);
  EXPECT_FALSE(spaced.accept(1.0, 0.0, 0.10, 0));
  spaced.begin_frame(2);
  EXPECT_FALSE(spaced.accept(1.0, 0.0, 0.10, 2)) << "第 0 帧的命中已出窗口";
}

TEST(TemporalConfirmation, DifferentVoxelsAreTrackedIndependently)
{
  TemporalConfirmation confirmation(0.02, 3, 2);
  for (int64_t frame = 0; frame < 5; ++frame) {
    confirmation.begin_frame(frame);
    // 只有 1.00 m 处持续被命中；1.05 m 处只在第 2 帧出现过一次。
    // 持续表面从第 2 帧起通过，首帧因只有 1 次命中而被拒（即 1 帧确认延迟）。
    EXPECT_EQ(confirmation.accept(1.00, 0.0, 0.10, frame), frame >= 1)
      << "第 " << frame << " 帧的持续表面判定不符";
    if (frame == 1) {
      EXPECT_FALSE(confirmation.accept(1.05, 0.0, 0.10, frame)) << "单帧跳变不应通过";
    }
  }
}

TEST(TemporalConfirmation, ExpiredVoxelsAreEvictedToBoundMemory)
{
  TemporalConfirmation confirmation(0.02, 3, 2);
  for (int64_t frame = 0; frame < 6; ++frame) {
    confirmation.begin_frame(frame);
    confirmation.accept(1.0, 0.0, 0.10, frame);
  }
  EXPECT_LT(confirmation.tracked_voxels(), 20U);
  // 长时间不再出现后应被清空。
  for (int64_t frame = 6; frame < 30; ++frame) {confirmation.begin_frame(frame);}
  EXPECT_EQ(confirmation.tracked_voxels(), 0U);
}
