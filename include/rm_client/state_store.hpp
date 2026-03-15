#pragma once

#include <chrono>
#include <mutex>
#include <optional>
#include <string>
#include <unordered_map>

#include "judge_system.pb.h"
#include "guard_ctrl.pb.h"

namespace rm_client
{

class StateStore
{
public:
  using Clock = std::chrono::steady_clock;
  using TimePoint = Clock::time_point;

  StateStore() = default;
  ~StateStore() = default;

  // ===== update =====
  void update_game_status(const GameStatus & msg);
  void update_global_unit_status(const GlobalUnitStatus & msg);
  void update_robot_dynamic_status(const RobotDynamicStatus & msg);
  void update_guard_ctrl_result(const GuardCtrlResult & msg);

  // 按 topic 名更新时间；适合你以后扩展更多 topic 时先复用
  void touch_topic(const std::string & topic_name);

  // ===== has =====
  bool has_game_status() const;
  bool has_global_unit_status() const;
  bool has_robot_dynamic_status() const;
  bool has_guard_ctrl_result() const;

  bool has_topic(const std::string & topic_name) const;

  // ===== get =====
  std::optional<GameStatus> get_game_status() const;
  std::optional<GlobalUnitStatus> get_global_unit_status() const;
  std::optional<RobotDynamicStatus> get_robot_dynamic_status() const;
  std::optional<GuardCtrlResult> get_guard_ctrl_result() const;

  // ===== last update =====
  std::optional<TimePoint> get_topic_last_update(const std::string & topic_name) const;

  double seconds_since_topic_update(const std::string & topic_name) const;

  bool is_topic_stale(const std::string & topic_name, double stale_threshold_sec) const;

  // ===== convenience =====
  bool can_send_guard_ctrl() const;

  // 调试辅助：拿一份状态摘要字符串
  std::string debug_summary() const;

private:
  void set_last_update_locked(const std::string & topic_name, TimePoint now);

private:
  mutable std::mutex mutex_;

  std::optional<GameStatus> game_status_;
  std::optional<GlobalUnitStatus> global_unit_status_;
  std::optional<RobotDynamicStatus> robot_dynamic_status_;
  std::optional<GuardCtrlResult> guard_ctrl_result_;

  std::unordered_map<std::string, TimePoint> last_update_map_;
};

}  // namespace rm_client