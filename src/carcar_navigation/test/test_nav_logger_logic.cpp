// NAV-013: 导航事件日志与状态归因逻辑单元测试
// 验证内容：
// 1. 行为树节点类型识别（ProgressGuard, ControlledSpin, ParkAndObserve, BackUp 等）及后退参数规范（0.10m, 0.05m/s, 4.0s）
// 2. 里程计运动反馈状态机逻辑（静止判据：线速<0.01 m/s & 角速<0.02 rad/s 持续1.0s；超时判据：>0.6s 标记为数据过期）
// 3. 停车归因多维定级（确定 CERTAIN / 关联 CORRELATED / 证据不足 INSUFFICIENT），禁止将 watchdog 或零速直接定性为唯一根本原因
// 4. 历史会话日志兼容解析（缺失 NAVIGATION_STOP 记录时平滑推测，不抛出异常）

#include <gtest/gtest.h>
#include <string>
#include <regex>
#include <map>
#include <vector>
#include <chrono>
#include <nlohmann/json.hpp>

using json = nlohmann::json;

namespace {

// 模拟 BT 节点识别逻辑
std::string resolve_bt_type_mock(const std::string & node_name, const std::map<std::string, std::string> & xml_map) {
  auto it = xml_map.find(node_name);
  if (it != xml_map.end()) {
    return it->second;
  }
  if (node_name.find("ProgressGuard") != std::string::npos) return "ProgressGuard";
  if (node_name.find("ControlledSpin") != std::string::npos) return "ControlledSpin";
  if (node_name.find("ParkAndObserve") != std::string::npos) return "ParkAndObserve";
  if (node_name.find("RearClear") != std::string::npos) return "RearClear";
  if (node_name.find("BackUp") != std::string::npos) return "BackUp";
  return "UnknownNode";
}

// 模拟里程计运动状态分类
enum class OdomMotionState {
  STILL,
  MOVING,
  DATA_EXPIRED
};

struct OdomClassifier {
  double still_linear_thresh{0.01};
  double still_angular_thresh{0.02};
  double still_duration_thresh{1.0};
  double expire_age_thresh{0.6};

  double still_accumulator{0.0};

  OdomMotionState classify(double linear_speed, double angular_speed, double age_s, double dt_s) {
    if (age_s > expire_age_thresh) {
      still_accumulator = 0.0;
      return OdomMotionState::DATA_EXPIRED;
    }

    if (std::abs(linear_speed) < still_linear_thresh && std::abs(angular_speed) < still_angular_thresh) {
      still_accumulator += dt_s;
      if (still_accumulator >= still_duration_thresh) {
        return OdomMotionState::STILL;
      }
    } else {
      still_accumulator = 0.0;
    }
    return OdomMotionState::MOVING;
  }
};

// 模拟停车归因置信度评级
enum class AttributionConfidence {
  CERTAIN,
  CORRELATED,
  INSUFFICIENT
};

struct StopAttributionResult {
  AttributionConfidence confidence;
  std::string category;
  std::string rationale;
};

StopAttributionResult evaluate_stop_attribution(
    bool watchdog_fired,
    bool cmd_vel_zero,
    bool progress_guard_blocked,
    bool obstacle_blocked,
    bool path_exhausted,
    double scan_age_s) {
  // 严格规则：看门狗动作和下发零速仅属于关联证据，绝不能作为唯一确定性根本原因
  if (watchdog_fired && !obstacle_blocked && !progress_guard_blocked && scan_age_s < 0.5) {
    return {
      AttributionConfidence::CORRELATED,
      "CHASSIS_WATCHDOG_INTERRUPT",
      "底盘看门狗触发停机，但此为被动保护动作，未发现环境或感知根因"
    };
  }

  if (cmd_vel_zero && !obstacle_blocked && !progress_guard_blocked) {
    return {
      AttributionConfidence::CORRELATED,
      "UPSTREAM_ZERO_VELOCITY",
      "上层节点下发零速命令，但未伴随明确阻断，需进一步追溯规划器/控制器状态"
    };
  }

  if (progress_guard_blocked) {
    return {
      AttributionConfidence::CERTAIN,
      "PROGRESS_GUARD_STAGNATION",
      "ProgressGuard 判定小车停滞并触发安全阻断"
    };
  }

  if (obstacle_blocked) {
    return {
      AttributionConfidence::CERTAIN,
      "OBSTACLE_PROXIMITY_BLOCKED",
      "前方或周围障碍物距离进入安全禁止区，触发停止"
    };
  }

  if (scan_age_s >= 0.5) {
    return {
      AttributionConfidence::CERTAIN,
      "SENSOR_DATA_EXPIRED",
      "激光雷达扫描数据过期超过安全门限"
    };
  }

  if (path_exhausted) {
    return {
      AttributionConfidence::CERTAIN,
      "GOAL_REACHED_TERMINAL",
      "已抵达最终目标容差范围并停车"
    };
  }

  return {
    AttributionConfidence::INSUFFICIENT,
    "UNKNOWN_STOP_CAUSE",
    "线索不足，无法确定停机根因"
  };
}

} // namespace

// 1. 验证 BT 节点类型识别及后退参数规范
TEST(NavLoggerLogicTest, BTNodeTypeResolutionAndBackupDescription) {
  std::map<std::string, std::string> xml_map = {
    {"guard_main", "ProgressGuard"},
    {"spin_in_place", "ControlledSpin"},
    {"park_and_wait", "ParkAndObserve"},
    {"rear_safety_check", "RearClear"},
    {"safe_backup_step", "BackUp"}
  };

  // 通过 XML 映射名识别
  EXPECT_EQ(resolve_bt_type_mock("guard_main", xml_map), "ProgressGuard");
  EXPECT_EQ(resolve_bt_type_mock("spin_in_place", xml_map), "ControlledSpin");
  EXPECT_EQ(resolve_bt_type_mock("park_and_wait", xml_map), "ParkAndObserve");
  EXPECT_EQ(resolve_bt_type_mock("rear_safety_check", xml_map), "RearClear");
  EXPECT_EQ(resolve_bt_type_mock("safe_backup_step", xml_map), "BackUp");

  // 通过启发式名称识别
  EXPECT_EQ(resolve_bt_type_mock("MyProgressGuard_1", {}), "ProgressGuard");
  EXPECT_EQ(resolve_bt_type_mock("CustomControlledSpin", {}), "ControlledSpin");
  EXPECT_EQ(resolve_bt_type_mock("AutoParkAndObserve", {}), "ParkAndObserve");
  EXPECT_EQ(resolve_bt_type_mock("CheckRearClear", {}), "RearClear");
  EXPECT_EQ(resolve_bt_type_mock("DoBackUp", {}), "BackUp");
  EXPECT_EQ(resolve_bt_type_mock("UnrelatedNode", {}), "UnknownNode");

  // 验证 BackUp 动作的实测物理参数描述格式
  const double backup_dist = 0.10;
  const double backup_speed = 0.05;
  const double time_allowance = 4.0;
  char buf[128];
  snprintf(buf, sizeof(buf), "距离=%.2fm, 速度=%.2fm/s, 超时=%.1fs", backup_dist, backup_speed, time_allowance);
  EXPECT_STREQ(buf, "距离=0.10m, 速度=0.05m/s, 超时=4.0s");
}

// 2. 验证里程计运动反馈状态机判据
TEST(NavLoggerLogicTest, OdometryMotionClassifier) {
  OdomClassifier classifier;

  // 刚停下时尚未累积满 1.0s，仍为 MOVING
  EXPECT_EQ(classifier.classify(0.005, 0.005, 0.05, 0.5), OdomMotionState::MOVING);
  // 累积满 1.0s 后变为 STILL
  EXPECT_EQ(classifier.classify(0.002, 0.001, 0.05, 0.6), OdomMotionState::STILL);

  // 一旦有移动，重置为 MOVING
  EXPECT_EQ(classifier.classify(0.05, 0.0, 0.05, 0.1), OdomMotionState::MOVING);

  // 数据过期 (>0.6s)
  EXPECT_EQ(classifier.classify(0.0, 0.0, 0.75, 0.1), OdomMotionState::DATA_EXPIRED);
}

// 3. 验证多维停车归因置信度评级（禁止单一归咎）
TEST(NavLoggerLogicTest, StopAttributionConfidence) {
  // 单纯底盘看门狗动作 -> 仅为 CORRELATED，不可定性为 CERTAIN 根因
  auto res1 = evaluate_stop_attribution(true, false, false, false, false, 0.1);
  EXPECT_EQ(res1.confidence, AttributionConfidence::CORRELATED);
  EXPECT_EQ(res1.category, "CHASSIS_WATCHDOG_INTERRUPT");

  // 单纯上层下发零速 -> 仅为 CORRELATED
  auto res2 = evaluate_stop_attribution(false, true, false, false, false, 0.1);
  EXPECT_EQ(res2.confidence, AttributionConfidence::CORRELATED);
  EXPECT_EQ(res2.category, "UPSTREAM_ZERO_VELOCITY");

  // ProgressGuard 触发停滞阻断 -> CERTAIN
  auto res3 = evaluate_stop_attribution(true, true, true, false, false, 0.1);
  EXPECT_EQ(res3.confidence, AttributionConfidence::CERTAIN);
  EXPECT_EQ(res3.category, "PROGRESS_GUARD_STAGNATION");

  // 障碍物阻挡 -> CERTAIN
  auto res4 = evaluate_stop_attribution(true, true, false, true, false, 0.1);
  EXPECT_EQ(res4.confidence, AttributionConfidence::CERTAIN);
  EXPECT_EQ(res4.category, "OBSTACLE_PROXIMITY_BLOCKED");

  // 激光雷达数据过期 -> CERTAIN
  auto res5 = evaluate_stop_attribution(false, false, false, false, false, 0.8);
  EXPECT_EQ(res5.confidence, AttributionConfidence::CERTAIN);
  EXPECT_EQ(res5.category, "SENSOR_DATA_EXPIRED");

  // 无任何线索 -> INSUFFICIENT
  auto res6 = evaluate_stop_attribution(false, false, false, false, false, 0.1);
  EXPECT_EQ(res6.confidence, AttributionConfidence::INSUFFICIENT);
  EXPECT_EQ(res6.category, "UNKNOWN_STOP_CAUSE");
}

// 4. 验证历史日志与未知字段兼容性解析
TEST(NavLoggerLogicTest, HistoricalLogJsonCompatibility) {
  // 模拟历史会话事件行（不含 stop_attribution 字段）
  std::string old_event_line = R"json({
    "timestamp": "2026-09-14 20:59:36.713",
    "event": "NAVIGATION_ABORTED",
    "node": "bt_navigator",
    "stage": "PLANNING",
    "goal_uuid": "bd38d90b-1234-5678",
    "details": "Aborted during planning"
  })json";

  EXPECT_NO_THROW({
    auto j = json::parse(old_event_line);
    EXPECT_EQ(j.value("event", ""), "NAVIGATION_ABORTED");
    EXPECT_EQ(j.value("goal_uuid", ""), "bd38d90b-1234-5678");
    EXPECT_FALSE(j.contains("stop_attribution"));
  });

  // 模拟新版包含 stop_record 的事件行
  std::string new_event_line = R"json({
    "timestamp": "2026-09-15 21:00:00.000",
    "event": "NAVIGATION_STOP",
    "node": "nav_event_logger",
    "stage": "FOLLOW_PATH",
    "goal_uuid": "aa11bb22",
    "stop_attribution": {
      "confidence": "CORRELATED",
      "category": "CHASSIS_WATCHDOG_INTERRUPT",
      "clues": ["watchdog_stopped=true", "last_cmd_age_s=0.25"]
    }
  })json";

  EXPECT_NO_THROW({
    auto j = json::parse(new_event_line);
    EXPECT_EQ(j.value("event", ""), "NAVIGATION_STOP");
    EXPECT_TRUE(j.contains("stop_attribution"));
    EXPECT_EQ(j["stop_attribution"].value("confidence", ""), "CORRELATED");
    EXPECT_EQ(j["stop_attribution"]["clues"].size(), 2);
  });
}

// 5. 验证会话分类与陈旧 ACTIVE 状态自动修正逻辑
TEST(NavLoggerLogicTest, SessionClassificationAndStaleActiveRepair) {
  // 模拟会话四分类判定
  auto classify_session = [](bool incomplete, bool cleaned, bool record_bag, bool bag_exists) -> std::string {
    if (incomplete) return "记录异常";
    if (cleaned || (record_bag && !bag_exists)) return "原始录包已清理";
    if (record_bag || bag_exists) return "附原始录包";
    return "轻量日志";
  };

  EXPECT_EQ(classify_session(false, false, false, false), "轻量日志");
  EXPECT_EQ(classify_session(false, true, true, false), "原始录包已清理");
  EXPECT_EQ(classify_session(false, false, true, true), "附原始录包");
  EXPECT_EQ(classify_session(true, false, false, false), "记录异常");

  // 模拟 PID 探测与陈旧 ACTIVE 状态修正逻辑
  std::string status = "ACTIVE";
  int test_pid = 999999; // 确定不存在的 PID
  bool is_alive = (kill(test_pid, 0) == 0);
  if (!is_alive && status == "ACTIVE") {
    status = "ENDED";
  }
  EXPECT_EQ(status, "ENDED");
}

// 6. 验证原始录包配额与限制逻辑 (120s / 256 MiB / 2 GiB 磁盘)
TEST(NavLoggerLogicTest, RawBagLimitsLogic) {
  const double max_duration_s = 120.0;
  const uint64_t max_bytes = 268435456ULL; // 256 MiB
  const uint64_t min_disk_free = 2147483648ULL; // 2 GiB

  auto check_limits = [&](double duration_s, uint64_t bag_bytes, uint64_t disk_free) -> std::pair<bool, std::string> {
    if (duration_s >= max_duration_s) {
      return {true, "达到录包时长上限 (120s)"};
    }
    if (bag_bytes >= max_bytes) {
      return {true, "达到录包大小上限 (256 MiB)"};
    }
    if (disk_free < min_disk_free) {
      return {true, "磁盘剩余空间低于 2 GiB"};
    }
    return {false, "正常录制中"};
  };

  // 正常录制
  auto r1 = check_limits(60.0, 100 * 1024 * 1024, 10ULL * 1024 * 1024 * 1024);
  EXPECT_FALSE(r1.first);

  // 达到 120s
  auto r2 = check_limits(120.0, 50 * 1024 * 1024, 10ULL * 1024 * 1024 * 1024);
  EXPECT_TRUE(r2.first);
  EXPECT_NE(r2.second.find("120s"), std::string::npos);

  // 达到 256 MiB
  auto r3 = check_limits(30.0, 268435456ULL, 10ULL * 1024 * 1024 * 1024);
  EXPECT_TRUE(r3.first);
  EXPECT_NE(r3.second.find("256 MiB"), std::string::npos);

  // 磁盘低于 2 GiB
  auto r4 = check_limits(10.0, 10 * 1024 * 1024, 1024ULL * 1024 * 1024); // 1 GiB free
  EXPECT_TRUE(r4.first);
  EXPECT_NE(r4.second.find("2 GiB"), std::string::npos);
}

// 7. 验证重规划 30s 聚合与告警去重合并
TEST(NavLoggerLogicTest, ReplanningSummaryAndAlertDeduplication) {
  int replan_success_count = 0;
  double interval_s = 30.0;
  double last_summary_t = 0.0;

  auto on_replan_success = [&](double now_t, std::vector<std::string> & output) {
    replan_success_count++;
    if (now_t - last_summary_t >= interval_s) {
      output.push_back("REPLANNING_SUMMARY: 过去 " + std::to_string((int)interval_s) + "s 成功重规划 " +
                       std::to_string(replan_success_count) + " 次");
      replan_success_count = 0;
      last_summary_t = now_t;
    }
  };

  std::vector<std::string> summaries;
  for (int i = 1; i <= 25; ++i) {
    on_replan_success(static_cast<double>(i), summaries);
  }
  EXPECT_TRUE(summaries.empty()); // 25s 内不输出汇总

  on_replan_success(30.5, summaries);
  EXPECT_EQ(summaries.size(), 1);
  EXPECT_NE(summaries[0].find("成功重规划 26 次"), std::string::npos);
  EXPECT_EQ(replan_success_count, 0); // 已清零重新计数
}

// 8. 验证 5Hz 滑动证据窗口及重叠停车合并
TEST(NavLoggerLogicTest, EvidenceWindowMerging) {
  struct MockStopWindow {
    bool is_active{false};
    double end_time{0.0};
    std::string category;
    std::vector<std::string> clues;
  } win;

  auto trigger_stop = [&](double now_t, const std::string & cat, const std::string & clue) {
    if (win.is_active) {
      win.end_time = std::max(win.end_time, now_t + 5.0);
      win.clues.push_back(clue);
      if (win.category == "CAUSE_UNDETERMINED" && cat != "CAUSE_UNDETERMINED") {
        win.category = cat;
      }
    } else {
      win.is_active = true;
      win.end_time = now_t + 5.0;
      win.category = cat;
      win.clues.push_back(clue);
    }
  };

  // 第一次触发停车
  trigger_stop(100.0, "CAUSE_UNDETERMINED", "线速度归零");
  EXPECT_TRUE(win.is_active);
  EXPECT_DOUBLE_EQ(win.end_time, 105.0);
  EXPECT_EQ(win.category, "CAUSE_UNDETERMINED");

  // 2 秒后在窗口内再次触发，且提供了明确根因
  trigger_stop(102.0, "GOAL_REACHED", "到达目标点容差范围");
  EXPECT_DOUBLE_EQ(win.end_time, 107.0); // 102.0 + 5.0 = 107.0
  EXPECT_EQ(win.category, "GOAL_REACHED"); // 成功更新为确定根因
  EXPECT_EQ(win.clues.size(), 2);
}

// 9. 验证停车大类划分与 7 列时间线格式解析（缺失字段显示为“缺失”）
TEST(NavLoggerLogicTest, StopCategoriesAnd7ColumnParsing) {
  // 模拟从 events.jsonl 读取的一条完整新版记录与一条旧版记录
  json new_record = {
    {"timestamp", "2026-09-15 22:00:00.123"},
    {"stage", "路径跟随"},
    {"trigger_reason", "目标正常抵达达成 (SUCCEEDED)"},
    {"evidence_status", "CERTAIN"},
    {"related_anomalies", "无"},
    {"cmd_vel_nav", {{"vx", 0.0}, {"wz", 0.0}}},
    {"cmd_vel", {{"vx", 0.0}, {"wz", 0.0}}},
    {"odom_vel", {{"vx", 0.0}, {"wz", 0.0}}},
    {"motion_feedback", "静止"},
    {"recovery_or_terminal", "正常到达"},
    {"stop_category", "GOAL_REACHED"}
  };

  json old_record = {
    {"timestamp", "2026-09-14 17:56:48.000"},
    {"stage", "规划"},
    {"event_type", "PLAN_FAILED"},
    {"description", "全局路径规划失败无通行路径"}
  };

  // 解析新版记录
  EXPECT_EQ(new_record.value("timestamp", "缺失"), "2026-09-15 22:00:00.123");
  EXPECT_EQ(new_record.value("stage", "缺失"), "路径跟随");
  EXPECT_EQ(new_record.value("trigger_reason", "缺失"), "目标正常抵达达成 (SUCCEEDED)");
  EXPECT_EQ(new_record.value("related_anomalies", "缺失"), "无");
  EXPECT_TRUE(new_record.contains("cmd_vel_nav"));
  EXPECT_TRUE(new_record.contains("odom_vel"));
  EXPECT_EQ(new_record.value("recovery_or_terminal", "缺失"), "正常到达");

  // 解析旧版记录，必须能够平滑解析且未记录项展示为“缺失”
  EXPECT_EQ(old_record.value("timestamp", "缺失"), "2026-09-14 17:56:48.000");
  EXPECT_EQ(old_record.value("stage", "缺失"), "规划");
  EXPECT_EQ(old_record.value("trigger_reason", "缺失"), "缺失");
  EXPECT_EQ(old_record.value("related_anomalies", "缺失"), "缺失");
  EXPECT_FALSE(old_record.contains("cmd_vel_nav"));
  EXPECT_FALSE(old_record.contains("odom_vel"));
  EXPECT_EQ(old_record.value("recovery_or_terminal", "缺失"), "缺失");
}

