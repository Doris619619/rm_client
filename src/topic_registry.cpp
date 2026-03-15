#include "rm_client/topic_registry.hpp"

namespace rm_client
{

TopicRegistry::TopicRegistry()
{
  // ===== 当前第一阶段最小闭环 =====
  // 服务器 -> 客户端
  register_topic({
    "GameStatus",
    "GameStatus",
    TopicDirection::ServerToClient,
    5.0,
    true,
    StoreKind::MatchStore
  });

  register_topic({
    "GlobalUnitStatus",
    "GlobalUnitStatus",
    TopicDirection::ServerToClient,
    1.0,
    true,
    StoreKind::GlobalStore
  });

  register_topic({
    "RobotDynamicStatus",
    "RobotDynamicStatus",
    TopicDirection::ServerToClient,
    10.0,
    true,
    StoreKind::RobotStore
  });

  register_topic({
    "GuardCtrlResult",
    "GuardCtrlResult",
    TopicDirection::ServerToClient,
    1.0,
    true,
    StoreKind::SentinelStore
  });

  // 以下为 judge_receiver.cpp 中 legacy 订阅的其余下行 topic
  register_topic({
    "GlobalLogisticsStatus",
    "GlobalLogisticsStatus",
    TopicDirection::ServerToClient,
    1.0,
    false,
    StoreKind::GlobalStore
  });

  register_topic({
    "GlobalSpecialMechanism",
    "GlobalSpecialMechanism",
    TopicDirection::ServerToClient,
    1.0,
    false,
    StoreKind::GlobalStore
  });

  register_topic({
    "Event",
    "Event",
    TopicDirection::ServerToClient,
    0.0,
    false,
    StoreKind::MatchStore
  });

  register_topic({
    "RobotInjuryStat",
    "RobotInjuryStat",
    TopicDirection::ServerToClient,
    1.0,
    false,
    StoreKind::RobotStore
  });

  register_topic({
    "RobotRespawnStatus",
    "RobotRespawnStatus",
    TopicDirection::ServerToClient,
    1.0,
    false,
    StoreKind::RobotStore
  });

  register_topic({
    "RobotStaticStatus",
    "RobotStaticStatus",
    TopicDirection::ServerToClient,
    1.0,
    false,
    StoreKind::RobotStore
  });

  register_topic({
    "RobotModuleStatus",
    "RobotModuleStatus",
    TopicDirection::ServerToClient,
    1.0,
    false,
    StoreKind::RobotStore
  });

  register_topic({
    "RobotPosition",
    "RobotPosition",
    TopicDirection::ServerToClient,
    1.0,
    false,
    StoreKind::RobotStore
  });

  register_topic({
    "Buff",
    "Buff",
    TopicDirection::ServerToClient,
    1.0,
    false,
    StoreKind::RobotStore
  });

  register_topic({
    "PenaltyInfo",
    "PenaltyInfo",
    TopicDirection::ServerToClient,
    0.0,
    false,
    StoreKind::MatchStore
  });

  register_topic({
    "RobotPathPlanInfo",
    "RobotPathPlanInfo",
    TopicDirection::ServerToClient,
    1.0,
    false,
    StoreKind::SentinelStore
  });

  register_topic({
    "RaderInfoToClient",
    "RaderInfoToClient",
    TopicDirection::ServerToClient,
    1.0,
    false,
    StoreKind::GlobalStore
  });

  register_topic({
    "CustomByteBlock",
    "CustomByteBlock",
    TopicDirection::RobotToClient,
    50.0,
    false,
    StoreKind::MatchStore
  });

  register_topic({
    "TechCoreMotionStateSync",
    "TechCoreMotionStateSync",
    TopicDirection::ServerToClient,
    1.0,
    false,
    StoreKind::MatchStore
  });

  register_topic({
    "RobotPerformanceSelectionSync",
    "RobotPerformanceSelectionSync",
    TopicDirection::ServerToClient,
    1.0,
    false,
    StoreKind::RobotStore
  });

  register_topic({
    "DeployModeStatusSync",
    "DeployModeStatusSync",
    TopicDirection::ServerToClient,
    1.0,
    false,
    StoreKind::RobotStore
  });

  register_topic({
    "RuneStatusSync",
    "RuneStatusSync",
    TopicDirection::ServerToClient,
    1.0,
    false,
    StoreKind::MatchStore
  });

  register_topic({
    "SentinelStatusSync",
    "SentinelStatusSync",
    TopicDirection::ServerToClient,
    1.0,
    false,
    StoreKind::SentinelStore
  });

  register_topic({
    "DartSelectTargetStatusSync",
    "DartSelectTargetStatusSync",
    TopicDirection::ServerToClient,
    1.0,
    false,
    StoreKind::SentinelStore
  });

  register_topic({
    "AirSupportStatusSync",
    "AirSupportStatusSync",
    TopicDirection::ServerToClient,
    1.0,
    false,
    StoreKind::MatchStore
  });

  // 客户端 -> 服务器
  register_topic({
    "GuardCtrlCommand",
    "GuardCtrlCommand",
    TopicDirection::ClientToServer,
    1.0,
    true,
    StoreKind::SentinelStore
  });
}

void TopicRegistry::register_topic(const TopicMeta & meta)
{
  topics_[meta.topic_name] = meta;
}

const TopicMeta * TopicRegistry::find(const std::string & topic_name) const
{
  const auto it = topics_.find(topic_name);
  if (it == topics_.end()) {
    return nullptr;
  }
  return &it->second;
}

bool TopicRegistry::contains(const std::string & topic_name) const
{
  return topics_.find(topic_name) != topics_.end();
}

std::vector<std::string> TopicRegistry::get_all_topics() const
{
  std::vector<std::string> out;
  out.reserve(topics_.size());

  for (const auto & kv : topics_) {
    out.push_back(kv.first);
  }
  return out;
}

std::vector<std::string> TopicRegistry::get_subscribe_topics() const
{
  std::vector<std::string> out;
  out.reserve(topics_.size());

  for (const auto & kv : topics_) {
    if (
      kv.second.direction == TopicDirection::ServerToClient ||
      kv.second.direction == TopicDirection::RobotToClient)
    {
      out.push_back(kv.first);
    }
  }
  return out;
}

std::vector<std::string> TopicRegistry::get_publish_topics() const
{
  std::vector<std::string> out;
  out.reserve(topics_.size());

  for (const auto & kv : topics_) {
    if (kv.second.direction == TopicDirection::ClientToServer) {
      out.push_back(kv.first);
    }
  }
  return out;
}

std::size_t TopicRegistry::size() const
{
  return topics_.size();
}

std::string TopicRegistry::to_string(TopicDirection direction)
{
  switch (direction) {
    case TopicDirection::ServerToClient:
      return "ServerToClient";
    case TopicDirection::ClientToServer:
      return "ClientToServer";
    case TopicDirection::RobotToClient:
      return "RobotToClient";
    default:
      return "Unknown";
  }
}

std::string TopicRegistry::to_string(StoreKind store_kind)
{
  switch (store_kind) {
    case StoreKind::MatchStore:
      return "MatchStore";
    case StoreKind::GlobalStore:
      return "GlobalStore";
    case StoreKind::RobotStore:
      return "RobotStore";
    case StoreKind::SentinelStore:
      return "SentinelStore";
    default:
      return "Unknown";
  }
}

}  // namespace rm_client