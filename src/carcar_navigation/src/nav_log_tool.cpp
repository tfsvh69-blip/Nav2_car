// NAV-008/NAV-011: 导航会话分析、时间线审查、故障汇总与离线回看工具 (nav_log_tool)
// 遵循约定：全 C++ 实现；零驱动输出，绝不向 ROS 网络发布任何控制或目标指令。
// 支持会话清单 (--sessions)、时间线 (--timeline)、故障汇总 (--failures)、
// 离线会话回看 (--replay-session)、归档导出 (--export)；
// 同时保留旧版目标命令提取 (--replay) 并明确标注为旧功能。

#include <chrono>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <regex>
#include <sstream>
#include <string>
#include <thread>
#include <vector>
#include <map>
#include <csignal>
#include <sys/types.h>

#include <nlohmann/json.hpp>

namespace
{
const std::string SESSIONS_DIR = "/home/jetson/luhao/my_nav_carcar/log/nav_sessions";
const std::string LEGACY_DIR = "/home/jetson/luhao/my_nav_carcar/log/nav_events";

struct SessionSummary
{
  std::string session_id;
  std::string start_time{"未知"};
  std::string status{"未知"};
  std::string session_type_display{"轻量日志"};
  bool is_active{false};
  bool evidence_incomplete{false};
  std::string incomplete_reason;
  uint64_t total_bag_bytes{0};
  size_t total_events{0};
  size_t total_goals{0};
  int pid{0};
  bool record_raw_bag{false};
  bool raw_bag_cleaned{false};
};

SessionSummary load_session_summary(const std::filesystem::path & dir)
{
  SessionSummary s;
  s.session_id = dir.filename().string();

  const auto info_file = dir / "session_info.json";
  if (std::filesystem::exists(info_file)) {
    try {
      std::ifstream in(info_file);
      nlohmann::json j;
      in >> j;
      if (j.contains("start_time")) s.start_time = j["start_time"].get<std::string>();
      if (j.contains("pid")) s.pid = j["pid"].get<int>();
    } catch (...) {}
  }

  const auto state_file = dir / "session_state.json";
  nlohmann::json j_state;
  if (std::filesystem::exists(state_file)) {
    try {
      std::ifstream in(state_file);
      nlohmann::json j;
      in >> j;
      if (j.contains("status")) s.status = j["status"].get<std::string>();
      if (j.contains("evidence_incomplete")) s.evidence_incomplete = j["evidence_incomplete"].get<bool>();
      if (j.contains("evidence_incomplete_reason")) s.incomplete_reason = j["evidence_incomplete_reason"].get<std::string>();
      if (j.contains("total_bag_bytes")) s.total_bag_bytes = j["total_bag_bytes"].get<uint64_t>();
      if (j.contains("total_events")) s.total_events = j["total_events"].get<size_t>();
      if (j.contains("total_goals")) s.total_goals = j["total_goals"].get<size_t>();
      in >> j_state;
      if (j_state.contains("status")) s.status = j_state["status"].get<std::string>();
      if (j_state.contains("evidence_incomplete")) s.evidence_incomplete = j_state["evidence_incomplete"].get<bool>();
      if (j_state.contains("evidence_incomplete_reason")) s.incomplete_reason = j_state["evidence_incomplete_reason"].get<std::string>();
      if (j_state.contains("total_bag_bytes")) s.total_bag_bytes = j_state["total_bag_bytes"].get<uint64_t>();
      if (j_state.contains("total_events")) s.total_events = j_state["total_events"].get<size_t>();
      if (j_state.contains("total_goals")) s.total_goals = j_state["total_goals"].get<size_t>();
      if (j_state.contains("record_raw_bag")) s.record_raw_bag = j_state["record_raw_bag"].get<bool>();
      if (j_state.contains("raw_bag_cleaned")) s.raw_bag_cleaned = j_state["raw_bag_cleaned"].get<bool>();
    } catch (...) {}
  }

  s.is_active = (s.status == "ACTIVE");
  // 检查残留的 ACTIVE 状态：若记录进程已不在运行，自动修复为 ENDED 并更新文件
  if (s.status == "ACTIVE") {
    bool is_alive = false;
    if (s.pid > 0) {
      if (kill(s.pid, 0) == 0) {
        is_alive = true;
      }
    }
    if (!is_alive) {
      s.status = "ENDED";
      s.is_active = false;
      if (std::filesystem::exists(state_file)) {
        j_state["status"] = "ENDED";
        std::ofstream out(state_file);
        if (out.is_open()) {
          out << j_state.dump(2) << "\n";
        }
      }
    } else {
      s.is_active = true;
    }
  } else {
    s.is_active = (s.status == "ACTIVE");
  }

  // 四种会话类型分类：“轻量日志／附原始录包／原始录包已清理／记录异常”
  const bool bag_exists = std::filesystem::exists(dir / "bag");
  if (s.evidence_incomplete) {
    s.session_type_display = "记录异常";
  } else if (s.raw_bag_cleaned || (s.record_raw_bag && !bag_exists)) {
    s.session_type_display = "原始录包已清理";
  } else if (s.record_raw_bag || bag_exists) {
    s.session_type_display = "附原始录包";
  } else {
    s.session_type_display = "轻量日志";
  }

  return s;
}

std::vector<SessionSummary> list_all_sessions()
{
  std::vector<SessionSummary> list;
  if (!std::filesystem::exists(SESSIONS_DIR)) {
    return list;
  }
  for (const auto & entry : std::filesystem::directory_iterator(SESSIONS_DIR)) {
    if (entry.is_directory()) {
      list.push_back(load_session_summary(entry.path()));
    }
  }
  std::sort(list.begin(), list.end(), [](const SessionSummary & a, const SessionSummary & b) {
    return a.session_id < b.session_id;
  });
  return list;
}

std::filesystem::path resolve_session_path(const std::string & spec)
{
  const auto all = list_all_sessions();
  if (spec.empty() || spec == "latest" || spec == "LATEST") {
    if (!all.empty()) {
      return std::filesystem::path(SESSIONS_DIR) / all.back().session_id;
    }
    return {};
  }
  if (std::filesystem::exists(spec) && std::filesystem::is_directory(spec)) {
    return std::filesystem::path(spec);
  }
  const auto direct = std::filesystem::path(SESSIONS_DIR) / spec;
  if (std::filesystem::exists(direct)) {
    return direct;
  }
  // 查找匹配前缀或后缀
  for (const auto & s : all) {
    if (s.session_id.find(spec) != std::string::npos) {
      return std::filesystem::path(SESSIONS_DIR) / s.session_id;
    }
  }
  return {};
}

// 旧版目标点结构体
struct LegacyProblemGoal
{
  std::string goal_id;
  std::string timestamp;
  std::string failure_type;
  std::string failed_node;
  std::string reason;
  double target_x{0.0};
  double target_y{0.0};
  double target_yaw{0.0};
  double robot_x{0.0};
  double robot_y{0.0};
  double min_obs{0.0};
  double front_obs{0.0};
  double rear_obs{0.0};
  std::string replay_command;
};

std::string utf8_safe_truncate(const std::string & s, size_t max_bytes)
{
  if (s.size() <= max_bytes) return s;
  size_t cut = max_bytes > 3 ? max_bytes - 3 : max_bytes;
  while (cut > 0 && (static_cast<unsigned char>(s[cut]) & 0xC0) == 0x80) {
    --cut;
  }
  return s.substr(0, cut) + "...";
}

std::string extract_string(const std::string & block, const std::string & key)
{
  const std::string pattern = "\"" + key + "\"[ \\t]*:[ \\t]*\"([^\"]*)\"";
  std::regex re(pattern);
  std::smatch match;
  if (std::regex_search(block, match, re) && match.size() > 1) {
    return match[1].str();
  }
  return "";
}

double extract_double(const std::string & block, const std::string & key)
{
  const std::string pattern = "\"" + key + "\"[ \\t]*:[ \\t]*([0-9.-]+)";
  std::regex re(pattern);
  std::smatch match;
  if (std::regex_search(block, match, re) && match.size() > 1) {
    try { return std::stod(match[1].str()); } catch (...) { return 0.0; }
  }
  return 0.0;
}

std::vector<LegacyProblemGoal> parse_legacy_goals(const std::string & file_path)
{
  std::vector<LegacyProblemGoal> goals;
  if (!std::filesystem::exists(file_path)) return goals;
  std::ifstream in(file_path);
  std::stringstream ss;
  ss << in.rdbuf();
  const std::string content = ss.str();

  size_t pos = 0;
  while ((pos = content.find("\"goal_id\"", pos)) != std::string::npos) {
    const size_t start = content.rfind('{', pos);
    if (start == std::string::npos) break;
    int depth = 0;
    size_t cur = start;
    for (; cur < content.size(); ++cur) {
      if (content[cur] == '{') ++depth;
      else if (content[cur] == '}') {
        --depth;
        if (depth == 0) break;
      }
    }
    const std::string block = content.substr(start, cur - start + 1);
    LegacyProblemGoal g;
    g.goal_id = extract_string(block, "goal_id");
    g.timestamp = extract_string(block, "timestamp");
    g.failure_type = extract_string(block, "failure_type");
    g.failed_node = extract_string(block, "failed_node");
    g.reason = extract_string(block, "reason");
    g.target_x = extract_double(block, "x");
    g.target_y = extract_double(block, "y");
    g.target_yaw = extract_double(block, "yaw");
    g.min_obs = extract_double(block, "min_dist");
    g.front_obs = extract_double(block, "min_front");
    g.rear_obs = extract_double(block, "min_rear");
    g.replay_command = extract_string(block, "replay_command");

    const size_t robot_pos = block.find("\"robot_pose_at_failure\"");
    if (robot_pos != std::string::npos) {
      const std::string sub = block.substr(robot_pos);
      g.robot_x = extract_double(sub, "x");
      g.robot_y = extract_double(sub, "y");
    }
    goals.push_back(g);
    pos = cur + 1;
  }
  return goals;
}

void print_help()
{
  std::cout << "======================================================================\n"
            << "  NAV-011/NAV-013 导航会话分析、时间线与离线回看诊断工具 (nav_log_tool)\n"
            << "  [零驱动输出声明：本工具为只读分析工具，绝不向 ROS 网络发布指令]\n"
            << "======================================================================\n"
            << "用法:\n"
            << "  ros2 run carcar_navigation nav_log_tool [选项] [会话ID|latest]\n\n"
            << "会话管理与离线分析选项:\n"
            << "  --sessions, -s               列出已记录的所有导航会话、大小与状态\n"
            << "  --stops, --stop [会话ID]     【NAV-013】呈现指定会话的停车时间线、触发归因与关键线索\n"
            << "  --timeline, -t [会话ID]      呈现指定会话的完整事件时序流转时间线 (默认 latest)\n"
            << "  --failures, -f [会话ID]      汇总指定会话的所有规划/控制/倒车/停滞失败原因 (默认 latest)\n"
            << "  --replay-session [会话ID]    纯离线回看指定会话流水 (直接读文件，零ROS发布，默认 latest)\n"
            << "  --export <会话ID> [输出路径] 对已结束会话打包归档 (.tar.gz)\n\n"
            << "旧版目标命令提取与流水选项 (NAV-008 保留):\n"
            << "  --list, -l                   列出历史记录的问题目标点清单\n"
            << "  --latest                     查看最近一次受阻现场的诊断与一键命令\n"
            << "  --replay <序号/ID>           【旧目标命令提取】输出历史单目标的重放命令\n"
            << "  --tail <N>                   查看历史日志末尾 N 行\n"
            << "  --clear                      清空历史事件日志\n"
            << "  --help, -h                   显示此帮助信息\n"
            << "======================================================================\n";
}
}  // namespace

int main(int argc, char ** argv)
{
  if (argc < 2) {
    print_help();
    return 0;
  }

  const std::string cmd = argv[1];

  if (cmd == "--help" || cmd == "-h") {
    print_help();
    return 0;
  }

  // 1. --sessions: 列出全部会话
  if (cmd == "--sessions" || cmd == "-s") {
    const auto sessions = list_all_sessions();
    std::cout << "\n======================== [导航历史会话列表 (NAV-011)] ========================\n";
    if (sessions.empty()) {
      std::cout << "当前未找到任何导航会话目录 (路径: " << SESSIONS_DIR << ")\n\n";
      return 0;
    }

    std::cout << std::left
              << std::setw(26) << "会话 ID"
              << std::setw(24) << "开始时间"
              << std::setw(22) << "类型"
              << std::setw(10) << "状态"
              << std::setw(10) << "目标数"
              << std::setw(10) << "事件数"
              << std::setw(14) << "录包大小"
              << "证据完整度\n";
    std::cout << std::string(105, '-') << "\n";
    std::cout << std::string(120, '-') << "\n";

    for (const auto & s : sessions) {
      const double mb = s.total_bag_bytes / (1024.0 * 1024.0);
      std::ostringstream size_ss;
      size_ss << std::fixed << std::setprecision(1) << mb << " MB";

      std::string evidence_str = "完整 (100%)";
      if (s.evidence_incomplete) {
        evidence_str = "不完整: " + s.incomplete_reason;
      }

      std::cout << std::left
                << std::setw(26) << s.session_id
                << std::setw(24) << s.start_time
                << std::setw(22) << s.session_type_display
                << std::setw(10) << s.status
                << std::setw(10) << s.total_goals
                << std::setw(10) << s.total_events
                << std::setw(14) << size_ss.str()
                << evidence_str << "\n";
    }
    std::cout << "=================================================================================\n";
    std::cout << "提示: 使用 `nav_log_tool --stops latest` 查看最新会话停车与阻断归因 (7列时间线)。\n"
              << "      使用 `nav_log_tool --timeline latest` 查看指定会话的完整时间线。\n"
              << "      使用 `nav_log_tool --replay-session latest` 纯离线终端回看。\n\n";
    return 0;
  }

  // 2. --stops: 查看会话停车与阻断时间线 (NAV-013)
  // 2. --stops: 查看会话停车与阻断时间线 (NAV-013: 7列时间线与多维归因)
  if (cmd == "--stops" || cmd == "--stop") {
    std::string spec = "latest";
    if (argc >= 3) {
      spec = argv[2];
    }
    const auto session_path = resolve_session_path(spec);
    if (session_path.empty()) {
      std::cerr << "错误: 未找到匹配的会话: " << spec << "\n";
      return 1;
    }
    const auto jsonl_file = session_path / "events.jsonl";
    if (!std::filesystem::exists(jsonl_file)) {
      std::cerr << "错误: 会话中缺少 events.jsonl 文件: " << jsonl_file << "\n";
      return 1;
    }

    std::cout << "\n===================== [会话停车与阻断时间线 (NAV-013): " << session_path.filename().string() << "] =====================\n";
    std::ifstream in(jsonl_file);
    std::string line;
    size_t count = 0;
    std::vector<nlohmann::json> structured_stops;
    std::vector<nlohmann::json> fallback_stops;

    while (std::getline(in, line)) {
      if (line.empty()) continue;
      try {
        auto j = nlohmann::json::parse(line);
        std::string ev = j.value("event_type", "");
        if (ev == "NAVIGATION_STOP") {
          structured_stops.push_back(j);
        } else if (ev == "NAVIGATION_SUCCEEDED" || ev == "NAVIGATION_ABORTED" ||
                   ev == "NAVIGATION_CANCELED" || ev == "REAR_CLEAR_DENIED" ||
                   ev == "PROGRESS_GUARD_STAGNATION" || ev == "CONTROLLER_FAULT" ||
                   ev == "PLAN_FAILED" || ev == "CONTROL_FAILED") {
          fallback_stops.push_back(j);
        }
      } catch (...) {}
    }

    std::map<std::string, int> category_counts = {
      {"正常到达 (GOAL_REACHED)", 0},
      {"用户取消 (USER_CANCELLED)", 0},
      {"主动等待 (ACTIVE_WAIT)", 0},
      {"导航失败 (NAVIGATION_FAILED)", 0},
      {"底盘命令超时 (CHASSIS_TIMEOUT)", 0},
      {"原因未确定 (CAUSE_UNDETERMINED)", 0}
    };

    auto map_category = [](const std::string & cat) -> std::string {
      if (cat == "GOAL_REACHED") return "正常到达 (GOAL_REACHED)";
      if (cat == "USER_CANCELLED") return "用户取消 (USER_CANCELLED)";
      if (cat == "ACTIVE_WAIT") return "主动等待 (ACTIVE_WAIT)";
      if (cat == "NAVIGATION_FAILED") return "导航失败 (NAVIGATION_FAILED)";
      if (cat == "CHASSIS_TIMEOUT") return "底盘命令超时 (CHASSIS_TIMEOUT)";
      return "原因未确定 (CAUSE_UNDETERMINED)";
    };

    std::cout << std::left
              << std::setw(24) << "停车时间"
              << std::setw(16) << "当时行为"
              << std::setw(32) << "直接触发信息"
              << std::setw(26) << "相关异常"
              << std::setw(30) << "两级速度"
              << std::setw(24) << "里程计反馈"
              << "恢复或任务终态\n";
    std::cout << std::string(170, '-') << "\n";

    size_t total_stops_count = 0;

    if (!structured_stops.empty()) {
      for (const auto & j : structured_stops) {
        count++;
        total_stops_count++;
        std::string ts = j.value("timestamp", "缺失");
        std::string stage = j.value("stage", "缺失");
        std::string reason = j.value("trigger_reason", "缺失");
        std::string ev_status = j.value("evidence_status", "");
        if (!ev_status.empty() && ev_status != "INSUFFICIENT") {
          reason += " [" + ev_status + "]";
        }
        std::string anomaly = j.value("related_anomalies", "无");
        if (anomaly.empty()) anomaly = "无";

        std::string cmd_str = "缺失";
        if (j.contains("cmd_vel_nav") && j.contains("cmd_vel")) {
          auto nav = j["cmd_vel_nav"];
          auto cmd = j["cmd_vel"];
          std::ostringstream ss;
          ss << std::fixed << std::setprecision(2)
             << "nav(" << nav.value("vx", 0.0) << "," << nav.value("wz", 0.0) << ") "
             << "cmd(" << cmd.value("vx", 0.0) << "," << cmd.value("wz", 0.0) << ")";
          cmd_str = ss.str();
        }

        std::string odom_str = "缺失";
        if (j.contains("odom_vel")) {
          auto odom = j["odom_vel"];
          std::string fb = j.value("motion_feedback", "静止");
          std::ostringstream ss;
          ss << std::fixed << std::setprecision(2)
             << "vx=" << odom.value("vx", 0.0) << ",wz=" << odom.value("wz", 0.0) << " (" << fb << ")";
          odom_str = ss.str();
        }

        std::string rec_term = j.value("recovery_or_terminal", "");
        if (rec_term.empty()) {
          rec_term = j.value("stop_category", "缺失");
        }

        std::string cat = j.value("stop_category", "CAUSE_UNDETERMINED");
        category_counts[map_category(cat)]++;

        std::string reason_disp = utf8_safe_truncate(reason, 30);
        std::string anom_disp = utf8_safe_truncate(anomaly, 24);
        std::string cmd_disp = utf8_safe_truncate(cmd_str, 28);
        std::string odom_disp = utf8_safe_truncate(odom_str, 22);

        std::cout << std::left
                  << std::setw(24) << ts
                  << std::setw(16) << stage
                  << std::setw(32) << reason_disp
                  << std::setw(26) << anom_disp
                  << std::setw(30) << cmd_disp
                  << std::setw(24) << odom_disp
                  << rec_term << "\n";
      }
      std::cout << std::string(170, '-') << "\n";
      std::cout << "共查询到 " << count << " 条结构化停车/阻断事件记录。\n";
    } else if (!fallback_stops.empty()) {
      std::cout << "[历史会话兼容模式] 提示：该会话未记录 NAVIGATION_STOP 结构化诊断，以下为推测的历史停顿/阻断/终态时间点 (缺失字段显式标记为“缺失”)：\n";
      for (const auto & j : fallback_stops) {
        count++;
        total_stops_count++;
        std::string ts = j.value("timestamp", "缺失");
        std::string stage = j.value("stage", "缺失");
        std::string ev = j.value("event_type", "缺失");
        std::string desc = j.value("description", ev);
        std::string anomaly = "缺失";
        std::string cmd_str = "缺失";
        std::string odom_str = "缺失";
        std::string rec_term = "缺失";
        std::string cat_key = "原因未确定 (CAUSE_UNDETERMINED)";

        if (ev == "NAVIGATION_SUCCEEDED") {
          rec_term = "正常到达";
          cat_key = "正常到达 (GOAL_REACHED)";
        } else if (ev == "NAVIGATION_CANCELED") {
          rec_term = "用户取消";
          cat_key = "用户取消 (USER_CANCELLED)";
        } else if (ev == "NAVIGATION_ABORTED" || ev == "PLAN_FAILED" || ev == "CONTROL_FAILED") {
          rec_term = "导航失败";
          cat_key = "导航失败 (NAVIGATION_FAILED)";
        } else if (ev == "PROGRESS_GUARD_STAGNATION" || ev == "REAR_CLEAR_DENIED") {
          rec_term = "恢复等待";
          cat_key = "主动等待 (ACTIVE_WAIT)";
        }
        category_counts[cat_key]++;

        std::string reason_disp = utf8_safe_truncate(desc, 30);
        std::string anom_disp = utf8_safe_truncate(anomaly, 24);
        std::string cmd_disp = utf8_safe_truncate(cmd_str, 28);
        std::string odom_disp = utf8_safe_truncate(odom_str, 22);

        std::cout << std::left
                  << std::setw(24) << ts
                  << std::setw(16) << stage
                  << std::setw(32) << reason_disp
                  << std::setw(26) << anom_disp
                  << std::setw(30) << cmd_disp
                  << std::setw(24) << odom_disp
                  << rec_term << "\n";
      }
      std::cout << std::string(170, '-') << "\n";
      std::cout << "共推测检测到 " << count << " 项历史停顿/异常/终态记录 (历史缺失信息已标为缺失，未补造推测结论)。\n";
    } else {
      std::cout << "此会话未记录到任何停车或阻断事件。\n";
    }
    std::cout << "=============================================================================================================================\n\n";

    std::cout << std::string(170, '-') << "\n";
    std::cout << "共查询到 " << total_stops_count << " 项停车/阻断事件。\n\n";

    std::cout << "【停车大类归因汇总】\n";
    for (const auto & pair : category_counts) {
      std::cout << "  - " << std::left << std::setw(34) << pair.first << ": " << pair.second << " 次\n";
    }

    std::cout << "\n【多维度归因与线索分析说明】\n"
              << "  1. 非单一归因原则：底层里程计零速度、单次路径规划失败或看门狗停机仅视为现场线索，绝不粗暴定为唯一根因。\n"
              << "  2. 综合研判链路：结合上游控制器指令 (nav)、平滑后指令 (cmd)、底盘里程计反馈 (odom)、激光雷达障碍分布及底盘看门狗延时。\n"
              << "  3. 历史会话兼容：旧会话中未记录的字段显式显示为“缺失”，忠实记录原始事实，绝不凭空推造虚假数据。\n"
              << "=============================================================================================================================\n\n";
    return 0;
  }

  // 3. --timeline: 查看会话时序时间线
  if (cmd == "--timeline" || cmd == "-t") {
    std::string spec = "latest";
    if (argc >= 3) {
      spec = argv[2];
    }
    const auto session_path = resolve_session_path(spec);
    if (session_path.empty()) {
      std::cerr << "错误: 未找到匹配的会话: " << spec << "\n";
      return 1;
    }
    const auto jsonl_file = session_path / "events.jsonl";
    if (!std::filesystem::exists(jsonl_file)) {
      std::cerr << "错误: 会话中缺少 events.jsonl 文件: " << jsonl_file << "\n";
      return 1;
    }

    std::cout << "\n===================== [会话时间线: " << session_path.filename().string() << "] =====================\n";
    std::ifstream in(jsonl_file);
    std::string line;
    std::cout << std::left
              << std::setw(24) << "时间戳"
              << std::setw(24) << "事件类型"
              << std::setw(12) << "目标UUID"
              << std::setw(16) << "当前阶段"
              << "详细描述与状态\n";
    std::cout << std::string(110, '-') << "\n";

    while (std::getline(in, line)) {
      if (line.empty()) continue;
      try {
        auto j = nlohmann::json::parse(line);
        std::string ts = j.value("timestamp", "");
        std::string ev = j.value("event_type", "");
        std::string uuid = j.value("goal_uuid", "NONE");
        if (uuid.size() > 8 && uuid != "NONE") uuid = uuid.substr(0, 8);
        std::string stage = j.value("stage", "");
        std::string desc = j.value("description", "");

        std::cout << std::left
                  << std::setw(24) << ts
                  << std::setw(24) << ev
                  << std::setw(12) << uuid
                  << std::setw(16) << stage
                  << desc << "\n";
      } catch (...) {}
    }
    std::cout << "=================================================================================\n\n";
    return 0;
  }

  // 4. --failures: 故障与拒绝诊断汇总
  if (cmd == "--failures" || cmd == "-f") {
    std::string spec = "latest";
    if (argc >= 3) {
      spec = argv[2];
    }
    const auto session_path = resolve_session_path(spec);
    if (session_path.empty()) {
      std::cerr << "错误: 未找到匹配的会话: " << spec << "\n";
      return 1;
    }
    const auto jsonl_file = session_path / "events.jsonl";
    if (!std::filesystem::exists(jsonl_file)) {
      std::cerr << "错误: 会话中缺少 events.jsonl 文件\n";
      return 1;
    }

    std::cout << "\n================= [会话故障与拒绝原因汇总: " << session_path.filename().string() << "] =================\n";
    std::ifstream in(jsonl_file);
    std::string line;
    size_t count = 0;

    while (std::getline(in, line)) {
      if (line.empty()) continue;
      try {
        auto j = nlohmann::json::parse(line);
        std::string ev = j.value("event_type", "");
        if (ev.find("FAILED") != std::string::npos ||
            ev.find("DENIED") != std::string::npos ||
            ev.find("ABORTED") != std::string::npos ||
            ev.find("FAULT") != std::string::npos ||
            ev.find("STAGNATION") != std::string::npos ||
            ev.find("TIMEOUT") != std::string::npos) {
          count++;
          std::cout << "[" << j.value("timestamp", "") << "] "
                    << "【" << ev << "】 "
                    << "节点: " << j.value("node", "未知") << " | "
                    << "阶段: " << j.value("stage", "") << " | "
                    << j.value("description", "") << "\n";
        }
      } catch (...) {}
    }
    if (count == 0) {
      std::cout << "恭喜！此会话未记录到任何失败或阻断事件 (全过程顺利通过)。\n";
    } else {
      std::cout << "---------------------------------------------------------------------------------\n";
      std::cout << "共检测到 " << count << " 项故障/阻断/拒绝事件记录。\n";
    }
    std::cout << "=================================================================================\n\n";
    return 0;
  }

  // 5. --replay-session: 纯离线回看 (零 ROS 发布)
  if (cmd == "--replay-session") {
    std::string spec = "latest";
    if (argc >= 3) {
      spec = argv[2];
    }
    const auto session_path = resolve_session_path(spec);
    if (session_path.empty()) {
      std::cerr << "错误: 未找到匹配的会话: " << spec << "\n";
      return 1;
    }
    const auto jsonl_file = session_path / "events.jsonl";
    if (!std::filesystem::exists(jsonl_file)) {
      std::cerr << "错误: 会话中缺少 events.jsonl 文件\n";
      return 1;
    }

    std::cout << "\n================== [离线会话终端回看 (零 ROS 发布，纯文件读取)] ==================\n"
              << "正在读取会话文件: " << session_path << "\n\n";

    std::ifstream in(jsonl_file);
    std::string line;
    size_t idx = 0;
    while (std::getline(in, line)) {
      if (line.empty()) continue;
      try {
        auto j = nlohmann::json::parse(line);
        std::string ts = j.value("timestamp", "");
        std::string ev = j.value("event_type", "");
        std::string stage = j.value("stage", "");
        std::string desc = j.value("description", "");
        double min_obs = 99.0;
        if (j.contains("obstacles") && j["obstacles"].contains("min_dist")) {
          min_obs = j["obstacles"]["min_dist"].get<double>();
        }

        std::cout << "[" << ts << "] [步骤 " << std::setw(3) << ++idx << "] "
                  << "[" << std::setw(14) << stage << "] "
                  << ">>> " << ev << ": " << desc
                  << " (最近障碍=" << std::fixed << std::setprecision(2) << min_obs << "m)\n";
        // 轻微延时模拟时序播放感（离线阅读体验良好）
        std::this_thread::sleep_for(std::chrono::milliseconds(30));
      } catch (...) {}
    }
    std::cout << "\n============================ [离线会话回看结束] ============================\n\n";
    return 0;
  }

  // 6. --export: 归档已结束的会话
  if (cmd == "--export") {
    if (argc < 3) {
      std::cerr << "错误: 请指定要归档的会话 ID\n";
      return 1;
    }
    const auto session_path = resolve_session_path(argv[2]);
    if (session_path.empty()) {
      std::cerr << "错误: 未找到匹配的会话: " << argv[2] << "\n";
      return 1;
    }

    const auto summary = load_session_summary(session_path);
    if (summary.is_active) {
      std::cerr << "错误: 该会话当前仍处于 ACTIVE 状态！仅允许归档已结束的会话。\n";
      return 1;
    }

    std::string dest_file = session_path.filename().string() + ".tar.gz";
    if (argc >= 4) {
      dest_file = argv[3];
    }

    std::cout << "正在归档会话 [" << session_path.filename().string() << "] 至 " << dest_file << "...\n";
    std::string tar_cmd = "tar -czf \"" + dest_file + "\" -C \"" +
                          session_path.parent_path().string() + "\" \"" +
                          session_path.filename().string() + "\"";
    const int ret = std::system(tar_cmd.c_str());
    if (ret == 0) {
      std::cout << "归档完成！生成文件: " << dest_file << "\n\n";
      return 0;
    } else {
      std::cerr << "归档打包失败，错误码: " << ret << "\n";
      return 1;
    }
  }

  // 6. --list, -l (旧版问题点清单保留)
  if (cmd == "--list" || cmd == "-l") {
    const std::string problem_file = LEGACY_DIR + "/problematic_goals.json";
    const auto goals = parse_legacy_goals(problem_file);
    std::cout << "\n================ [NAV-008 历史问题目标点清单 (旧版)] ================\n";
    if (goals.empty()) {
      std::cout << "当前未记录到任何受阻或失败的目标点 (日志为空或均顺利到达)。\n\n";
      return 0;
    }
    std::cout << std::left
              << std::setw(6)  << "#"
              << std::setw(14) << "目标ID"
              << std::setw(22) << "发生时间"
              << std::setw(18) << "阻断类型"
              << std::setw(20) << "目标坐标 (X, Y)"
              << std::setw(18) << "周围最近障碍"
              << "失败原因\n";
    std::cout << std::string(110, '-') << "\n";

    for (size_t i = 0; i < goals.size(); ++i) {
      const auto & g = goals[i];
      std::ostringstream target_str, obs_str;
      target_str << "(" << std::fixed << std::setprecision(2) << g.target_x << ", " << g.target_y << ")";
      obs_str << std::fixed << std::setprecision(2) << g.min_obs << "m (前:" << g.front_obs << "m)";

      std::cout << std::left
                << std::setw(6)  << (std::to_string(i + 1))
                << std::setw(14) << g.goal_id
                << std::setw(22) << g.timestamp
                << std::setw(18) << g.failure_type
                << std::setw(20) << target_str.str()
                << std::setw(18) << obs_str.str()
                << g.reason << "\n";
    }
    std::cout << "======================================================================\n";
    std::cout << "提示: 使用 `nav_log_tool --replay <#序号>` 可提取当时目标点的下发命令。\n\n";
    return 0;
  }

  // 7. --latest (旧版最近问题诊断)
  if (cmd == "--latest") {
    const std::string problem_file = LEGACY_DIR + "/problematic_goals.json";
    const auto goals = parse_legacy_goals(problem_file);
    if (goals.empty()) {
      std::cout << "当前无任何历史失败目标点记录。\n";
      return 0;
    }
    const auto & g = goals.back();
    std::cout << "\n=========== [最近一次受阻/停止事件现场诊断 (旧版)] ===========\n"
              << "目标 ID        : " << g.goal_id << "\n"
              << "时间戳         : " << g.timestamp << "\n"
              << "失败类型       : " << g.failure_type << " (" << g.failed_node << ")\n"
              << "失败原因       : " << g.reason << "\n"
              << "目标点坐标     : X=" << std::fixed << std::setprecision(3) << g.target_x
              << " m, Y=" << g.target_y << " m, Yaw=" << g.target_yaw << " rad\n"
              << "现场小车位姿   : X=" << g.robot_x << " m, Y=" << g.robot_y << " m\n"
              << "现场障碍物特征 : 最近距离=" << g.min_obs << " m (前方=" << g.front_obs
              << " m, 后方=" << g.rear_obs << " m)\n"
              << "--------------------------------------------------------\n"
              << "【一键重放此目标点命令】:\n"
              << g.replay_command << "\n"
              << "========================================================\n\n";
    return 0;
  }

  // 8. --replay: 明确标注为【旧目标命令提取】
  if (cmd == "--replay") {
    std::cout << "\n[提示] `--replay` 为【旧目标命令提取】功能，仅用于提取历史目标的下发命令，不作日志时序回放。\n"
              << "       如需离线时序回看日志，请使用 `--replay-session <会话ID>`。\n";
    const std::string problem_file = LEGACY_DIR + "/problematic_goals.json";
    if (argc < 3) {
      std::cerr << "错误: 请指定要提取的目标序号或目标 ID，例如 `nav_log_tool --replay 1`\n";
      return 1;
    }
    const auto goals = parse_legacy_goals(problem_file);
    if (goals.empty()) {
      std::cerr << "错误: 历史问题目标库为空。\n";
      return 1;
    }
    const std::string target_spec = argv[2];
    size_t target_idx = 0;
    try {
      target_idx = std::stoul(target_spec);
      if (target_idx >= 1 && target_idx <= goals.size()) {
        target_idx -= 1;
      } else {
        target_idx = goals.size() - 1;
      }
    } catch (...) {
      bool found = false;
      for (size_t i = 0; i < goals.size(); ++i) {
        if (goals[i].goal_id == target_spec) {
          target_idx = i;
          found = true;
          break;
        }
      }
      if (!found) {
        std::cerr << "未找到目标 ID: " << target_spec << "\n";
        return 1;
      }
    }

    const auto & g = goals[target_idx];
    std::cout << "\n已提取目标 [" << g.goal_id << "] 重放命令:\n\n"
              << g.replay_command << "\n\n"
              << "可直接复制上方命令并在终端执行重放测试。\n\n";
    return 0;
  }

  // 9. --tail (旧版查看尾部日志)
  if (cmd == "--tail") {
    int lines = 10;
    if (argc >= 3) {
      try { lines = std::stoi(argv[2]); } catch (...) {}
    }
    const std::string text_file = LEGACY_DIR + "/nav_events.log";
    if (!std::filesystem::exists(text_file)) {
      std::cout << "旧版日志文件尚未生成。\n";
      return 0;
    }
    std::ifstream in(text_file);
    std::string line;
    std::vector<std::string> all_lines;
    while (std::getline(in, line)) {
      all_lines.push_back(line);
    }
    const int start = std::max(0, static_cast<int>(all_lines.size()) - lines);
    std::cout << "\n=== [最新 " << (all_lines.size() - start) << " 条事件流水] ===\n";
    for (size_t i = start; i < all_lines.size(); ++i) {
      std::cout << all_lines[i] << "\n";
    }
    std::cout << "====================================\n\n";
    return 0;
  }

  // 10. --clear (清理历史旧日志)
  if (cmd == "--clear") {
    std::error_code ec;
    std::filesystem::remove(LEGACY_DIR + "/problematic_goals.json", ec);
    std::filesystem::remove(LEGACY_DIR + "/nav_events.jsonl", ec);
    std::filesystem::remove(LEGACY_DIR + "/nav_events.log", ec);
    std::cout << "已清空旧版导航事件与问题目标日志。\n";
    return 0;
  }

  print_help();
  std::cerr << "错误: 未知选项 " << cmd << "\n";
  return 1;
}
