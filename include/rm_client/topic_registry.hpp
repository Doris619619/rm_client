#pragma once

#include <cstddef>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

namespace rm_client
{

enum class TopicDirection
{
  ServerToClient,
  ClientToServer,
  RobotToClient,
  Unknown
};

enum class StoreKind
{
  MatchStore,
  GlobalStore,
  RobotStore,
  SentinelStore,
  Unknown
};

struct TopicMeta
{
  std::string topic_name;
  std::string message_type;
  TopicDirection direction{TopicDirection::Unknown};
  double frequency_hz{0.0};
  bool is_critical{false};
  StoreKind store_kind{StoreKind::Unknown};
};

class TopicRegistry
{
public:
  TopicRegistry();

  const TopicMeta * find(const std::string & topic_name) const;

  bool contains(const std::string & topic_name) const;

  std::vector<std::string> get_all_topics() const;

  std::vector<std::string> get_subscribe_topics() const;

  std::vector<std::string> get_publish_topics() const;

  std::size_t size() const;

  static std::string to_string(TopicDirection direction);
  static std::string to_string(StoreKind store_kind);

private:
  void register_topic(const TopicMeta & meta);

private:
  std::unordered_map<std::string, TopicMeta> topics_;
};

}  // namespace rm_client