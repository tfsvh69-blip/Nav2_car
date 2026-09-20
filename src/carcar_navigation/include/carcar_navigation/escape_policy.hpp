#pragma once

namespace carcar_navigation {

// 当前包络已压住致死格时的逃生优先级：先倒车把车头让开，再尝试短转。
enum class EscapeAction { None, Backup, RotatePositive, RotateNegative };

inline EscapeAction choose_escape(
  bool in_collision, bool backup_clear, bool rotate_pos_clear, bool rotate_neg_clear)
{
  if (!in_collision) {return EscapeAction::None;}
  if (backup_clear) {return EscapeAction::Backup;}
  if (rotate_pos_clear) {return EscapeAction::RotatePositive;}
  if (rotate_neg_clear) {return EscapeAction::RotateNegative;}
  return EscapeAction::None;
}

}  // namespace carcar_navigation
