// Copyright 2026 carcar maintainers. SPDX-License-Identifier: Apache-2.0
#pragma once
#include "carcar_inspection/domain.hpp"
namespace carcar_inspection {
// 网页只请求下一步；命令确认、图像新鲜度和人工确认在同一车端流程检查。
class Teaching {
public:
  std::string phase = "IDLE", error, owner, driver_epoch, command_id;
  double target = 140.5, command_at = 0;
  int board = 94, pending_board = 94;
  uint64_t revision = 0, frame_sequence = 0, displayed_frame = 0;
  void begin(const std::string & session) {
    require(phase == "IDLE" || phase == "CONFIRMED" || phase == "FAULT", "请先完成或结束当前角度调整");
    owner = session; phase = "ACQUIRING_HOLD"; error.clear(); ++revision;
  }
  void acquired(const std::string & epoch, int current_board) {
    require(phase == "ACQUIRING_HOLD", "停车保持状态已变化");
    require(current_board >= 57 && current_board <= 106, "板端档位超出当前安全范围");
    driver_epoch = epoch; board = current_board;
    target = board == 94 ? 140.5 : board * 1.5; phase = "WAITING_STOP"; ++revision;
  }
  void stopped(bool fresh_and_still) {
    if (phase == "WAITING_STOP" && fresh_and_still) {phase = "READY"; ++revision;}
  }
  int step(double requested, uint64_t expected_revision, bool fresh_and_still,
           uint64_t current_frame, double now) {
    require(revision == expected_revision, "调整状态已变化，请按最新状态操作");
    require(phase == "READY" || phase == "AWAITING_CONFIRMATION", "请等待停车或上一条命令结果");
    require(fresh_and_still, "轮式反馈过期或车体尚未停稳");
    require(std::abs(requested - target) <= 1.000001, "示教每次最多调整1°目标值");
    pending_board = servo_command(requested);
    require(std::abs(pending_board - board) <= 1, "板端每次最多调整一档");
    target = requested; command_at = now; frame_sequence = current_frame; displayed_frame = 0;
    phase = "SENDING"; ++revision; return pending_board;
  }
  void sent(bool ok, int actual_command, const std::string & reason, double now) {
    if (phase != "SENDING") return;
    if (!ok || actual_command != pending_board) {fault(reason.empty() ? "舵机发送结果不一致" : reason); return;}
    board = actual_command; command_at = now; phase = "WAITING_FRAME"; ++revision;
  }
  void image(uint64_t sequence, bool fresh, double now, double settle_seconds) {
    if (phase == "WAITING_FRAME" && fresh && sequence > frame_sequence && now - command_at >= settle_seconds) {
      phase = "AWAITING_CONFIRMATION"; ++revision;
    }
  }
  void viewed(uint64_t sequence) {displayed_frame = sequence;}
  void check_confirm(uint64_t expected_revision, uint64_t displayed, bool fresh_and_still,
                     bool fresh_image, double now, double settle_seconds) const {
    require(expected_revision == revision, "确认状态已变化，请重新查看画面");
    require(phase == "AWAITING_CONFIRMATION", "请先下发目标，并等待新的稳定画面");
    require(fresh_and_still && fresh_image && now - command_at >= settle_seconds, "车体或相机数据尚未满足确认条件");
    require(displayed == displayed_frame && displayed > frame_sequence, "请先查看本次调角后的新画面");
  }
  void confirmed() {phase = "CONFIRMED"; ++revision;}
  void fault(const std::string & reason) {phase = "FAULT"; error = reason; ++revision; displayed_frame = 0;}
  void disconnect(const std::string & session) {
    if (owner == session && phase != "IDLE" && phase != "CONFIRMED") fault("网页已断开，停车保持保留；重连后重新开始确认流程");
  }
  json state() const {return {{"phase",phase},{"error",error},{"revision",revision},
    {"target_degrees",target},{"board_command",board},{"position_feedback",false}};}
};
}  // namespace carcar_inspection
