#include "rm_client/state_store.hpp"

#include <sstream>

namespace rm_client
{

void StateStore::update_game_status(const GameStatus & msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  game_status_ = msg;
  set_last_update_locked("GameStatus", Clock::now());
}

void StateStore::update_global_unit_status(const GlobalUnitStatus & msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  global_unit_status_ = msg;
  set_last_update_locked("GlobalUnitStatus", Clock::now());
}

void StateStore::update_robot_dynamic_status(const RobotDynamicStatus & msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  robot_dynamic_status_ = msg;
  set_last_update_locked("RobotDynamicStatus", Clock::now());
}

void StateStore::update_guard_ctrl_result(const GuardCtrlResult & msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  guard_ctrl_result_ = msg;
  set_last_update_locked("GuardCtrlResult", Clock::now());
}

void StateStore::touch_topic(const std::string & topic_name)
{
  std::lock_guard<std::mutex> lock(mutex_);
  set_last_update_locked(topic_name, Clock::now());
}

bool StateStore::has_game_status() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return game_status_.has_value();
}

bool StateStore::has_global_unit_status() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return global_unit_status_.has_value();
}

bool StateStore::has_robot_dynamic_status() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return robot_dynamic_status_.has_value();
}

bool StateStore::has_guard_ctrl_result() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return guard_ctrl_result_.has_value();
}

bool StateStore::has_topic(const std::string & topic_name) const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return last_update_map_.find(topic_name) != last_update_map_.end();
}

std::optional<GameStatus> StateStore::get_game_status() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return game_status_;
}

std::optional<GlobalUnitStatus> StateStore::get_global_unit_status() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return global_unit_status_;
}

std::optional<RobotDynamicStatus> StateStore::get_robot_dynamic_status() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return robot_dynamic_status_;
}

std::optional<GuardCtrlResult> StateStore::get_guard_ctrl_result() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return guard_ctrl_result_;
}

std::optional<StateStore::TimePoint> StateStore::get_topic_last_update(
  const std::string & topic_name) const
{
  std::lock_guard<std::mutex> lock(mutex_);
  const auto it = last_update_map_.find(topic_name);
  if (it == last_update_map_.end()) {
    return std::nullopt;
  }
  return it->second;
}

double StateStore::seconds_since_topic_update(const std::string & topic_name) const
{
  std::lock_guard<std::mutex> lock(mutex_);
  const auto it = last_update_map_.find(topic_name);
  if (it == last_update_map_.end()) {
    return -1.0;
  }

  const auto now = Clock::now();
  const std::chrono::duration<double> dt = now - it->second;
  return dt.count();
}

bool StateStore::is_topic_stale(const std::string & topic_name, double stale_threshold_sec) const
{
  const double age_sec = seconds_since_topic_update(topic_name);
  if (age_sec < 0.0) {
    return true;
  }
  return age_sec > stale_threshold_sec;
}

bool StateStore::can_send_guard_ctrl() const
{
  std::lock_guard<std::mutex> lock(mutex_);

  // 最保守策略：
  // 1. 没收到 GameStatus 就不发
  // 2. GameStatus 太久没更新也不发（这里用 2 秒）
  const auto it = last_update_map_.find("GameStatus");
  if (!game_status_.has_value() || it == last_update_map_.end()) {
    return false;
  }

  const std::chrono::duration<double> dt = Clock::now() - it->second;
  if (dt.count() > 2.0) {
    return false;
  }

  // 这里先不硬编码 game phase 具体枚举值，
  // 因为你当前 proto 的具体字段命名/枚举定义我没有逐行展开核验。
  // 第一版先只保证：必须收到且必须新鲜。
  return true;
}

std::string StateStore::debug_summary() const
{
  std::lock_guard<std::mutex> lock(mutex_);

  std::ostringstream oss;
  oss << "StateStore{";

  oss << "has_game_status=" << (game_status_.has_value() ? "true" : "false") << ", ";
  oss << "has_global_unit_status=" << (global_unit_status_.has_value() ? "true" : "false") << ", ";
  oss << "has_robot_dynamic_status=" << (robot_dynamic_status_.has_value() ? "true" : "false") << ", ";
  oss << "has_guard_ctrl_result=" << (guard_ctrl_result_.has_value() ? "true" : "false");

  if (robot_dynamic_status_.has_value()) {
    oss << ", current_health=" << robot_dynamic_status_->current_health();
    oss << ", remaining_ammo=" << robot_dynamic_status_->remaining_ammo();
    oss << ", can_remote_heal=" << (robot_dynamic_status_->can_remote_heal() ? "true" : "false");
    oss << ", can_remote_ammo=" << (robot_dynamic_status_->can_remote_ammo() ? "true" : "false");
  }

  if (guard_ctrl_result_.has_value()) {
    oss << ", last_guard_command_id=" << guard_ctrl_result_->command_id();
    oss << ", last_guard_result_code=" << guard_ctrl_result_->result_code();
  }

  oss << "}";
  return oss.str();
}

void StateStore::set_last_update_locked(const std::string & topic_name, TimePoint now)
{
  last_update_map_[topic_name] = now;
}

}  // namespace rm_client