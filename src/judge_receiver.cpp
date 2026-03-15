// src/judge_receiver.cpp
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/u_int32.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/u_int8_multi_array.hpp>
#include <std_msgs/msg/u_int64.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/string.hpp>

#include <mqtt/async_client.h>
#include "judge_system.pb.h"
#include "guard_ctrl.pb.h"
#include "rm_client/topic_registry.hpp"
#include "rm_client/state_store.hpp"

#include <algorithm>
#include <chrono>
#include <cctype>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_set>
#include <vector>

// ROS 消息容器 (用于数组整合消息)
#include <geometry_msgs/msg/point.hpp>

class JudgeReceiverNode : public rclcpp::Node {
public:
  JudgeReceiverNode() : Node("judge_receiver")
  {
    // ---------- MQTT 连接参数 ----------
    this->declare_parameter<std::string>("judge_host", "127.0.0.1");
    this->declare_parameter<int>("judge_port", 3333);
    this->declare_parameter<int>("qos", 1);
    this->declare_parameter<std::string>("client_id", "rm_semi_auto_client_rx");
    this->declare_parameter<double>("reconnect_period_sec", 2.0);

    // ---------- ROS 前缀 ----------
    this->declare_parameter<std::string>("ros_prefix", "/judge");

    host_ = this->get_parameter("judge_host").as_string();
    port_ = this->get_parameter("judge_port").as_int();
    qos_ = this->get_parameter("qos").as_int();
    client_id_ = this->get_parameter("client_id").as_string();
    reconnect_period_sec_ = this->get_parameter("reconnect_period_sec").as_double();
    ros_prefix_ = this->get_parameter("ros_prefix").as_string();

    server_uri_ = "tcp://" + host_ + ":" + std::to_string(port_);

    init_publishers();

    // ---------- MQTT client ----------
    mqtt_client_ = std::make_unique<mqtt::async_client>(server_uri_, client_id_);
    mqtt_cb_ = std::make_shared<MqttCallback>(this);
    mqtt_client_->set_callback(*mqtt_cb_);

    connect_and_subscribe_once();

    reconnect_timer_ = this->create_wall_timer(
      std::chrono::duration<double>(reconnect_period_sec_),
      std::bind(&JudgeReceiverNode::reconnect_tick, this)
    );

    const auto receiver_topics = get_receiver_topics();
    RCLCPP_INFO(
      this->get_logger(),
      "%s started, server_uri=%s, client_id=%s, qos=%d, subscribe_topics=%zu",
      this->get_name(),
      server_uri_.c_str(),
      client_id_.c_str(),
      qos_,
      receiver_topics.size());

    RCLCPP_INFO(this->get_logger(),
      "JudgeReceiver initialized (24 message types).\n"
      "MQTT uri=%s client_id=%s qos=%d\n"
      "ros_prefix=%s reconnect=%.1fs",
      server_uri_.c_str(), client_id_.c_str(), qos_,
      ros_prefix_.c_str(), reconnect_period_sec_);
  }

  ~JudgeReceiverNode() override
  {
    std::lock_guard<std::mutex> lock(mqtt_mtx_);
    try {
      if (mqtt_client_ && mqtt_client_->is_connected()) {
        mqtt_client_->disconnect()->wait();
      }
    } catch (...) {}
  }

private:
  class MqttCallback : public virtual mqtt::callback {
  public:
    explicit MqttCallback(JudgeReceiverNode* node) : node_(node) {}

    void connection_lost(const std::string& cause) override
    {
      if (!node_) return;
      node_->subscribed_ = false;
      RCLCPP_WARN(node_->get_logger(), "MQTT connection lost: %s", cause.c_str());
    }

    void message_arrived(mqtt::const_message_ptr msg) override
    {
      if (!node_ || !msg) return;
      node_->on_mqtt_message(msg);
    }

  private:
    JudgeReceiverNode* node_;
  };

  static std::string sanitize_ros_name(const std::string& s)
  {
    std::string out;
    out.reserve(s.size());
    for (char c : s) {
      if (std::isalnum(static_cast<unsigned char>(c)) || c == '_') out.push_back(c);
      else out.push_back('_');
    }
    if (out.empty()) out = "topic";
    return out;
  }

  void init_publishers()
  {
    // 1. GameStatus
    pub_game_status_ = this->create_publisher<std_msgs::msg::String>(ros_prefix_ + "/game_status", 10);

    // 2. GlobalUnitStatus
    pub_global_unit_status_ = this->create_publisher<std_msgs::msg::String>(ros_prefix_ + "/global_unit_status", 10);

    // 3. GlobalLogisticsStatus
    pub_global_logistics_status_ = this->create_publisher<std_msgs::msg::String>(ros_prefix_ + "/global_logistics_status", 10);

    // 4. GlobalSpecialMechanism
    pub_global_special_mechanism_ = this->create_publisher<std_msgs::msg::String>(ros_prefix_ + "/global_special_mechanism", 10);

    // 5. Event
    pub_event_ = this->create_publisher<std_msgs::msg::String>(ros_prefix_ + "/event", 10);

    // 6. RobotInjuryStat
    pub_robot_injury_stat_ = this->create_publisher<std_msgs::msg::String>(ros_prefix_ + "/robot_injury_stat", 10);

    // 7. RobotRespawnStatus
    pub_robot_respawn_status_ = this->create_publisher<std_msgs::msg::String>(ros_prefix_ + "/robot_respawn_status", 10);

    // 8. RobotStaticStatus
    pub_robot_static_status_ = this->create_publisher<std_msgs::msg::String>(ros_prefix_ + "/robot_static_status", 10);

    // 9. RobotDynamicStatus
    pub_robot_dynamic_status_ = this->create_publisher<std_msgs::msg::String>(ros_prefix_ + "/robot_dynamic_status", 10);

    // 10. RobotModuleStatus
    pub_robot_module_status_ = this->create_publisher<std_msgs::msg::String>(ros_prefix_ + "/robot_module_status", 10);

    // 11. RobotPosition
    pub_robot_position_ = this->create_publisher<geometry_msgs::msg::Point>(ros_prefix_ + "/robot_position", 10);

    // 12. Buff
    pub_buff_ = this->create_publisher<std_msgs::msg::String>(ros_prefix_ + "/buff", 10);

    // 13. PenaltyInfo
    pub_penalty_info_ = this->create_publisher<std_msgs::msg::String>(ros_prefix_ + "/penalty_info", 10);

    // 14. RobotPathPlanInfo
    pub_robot_path_plan_info_ = this->create_publisher<std_msgs::msg::String>(ros_prefix_ + "/robot_path_plan_info", 10);

    // 15. RaderInfoToClient
    pub_rader_info_ = this->create_publisher<std_msgs::msg::String>(ros_prefix_ + "/rader_info", 10);

    // 16. CustomByteBlock
    pub_custom_byte_block_ = this->create_publisher<std_msgs::msg::UInt8MultiArray>(ros_prefix_ + "/custom_byte_block", 10);

    // 17. TechCoreMotionStateSync
    pub_tech_core_motion_ = this->create_publisher<std_msgs::msg::String>(ros_prefix_ + "/tech_core_motion", 10);

    // 18. RobotPerformanceSelectionSync
    pub_robot_performance_ = this->create_publisher<std_msgs::msg::String>(ros_prefix_ + "/robot_performance", 10);

    // 19. DeployModeStatusSync
    pub_deploy_mode_status_ = this->create_publisher<std_msgs::msg::UInt32>(ros_prefix_ + "/deploy_mode_status", 10);

    // 20. RuneStatusSync
    pub_rune_status_ = this->create_publisher<std_msgs::msg::String>(ros_prefix_ + "/rune_status", 10);

    // 21. SentinelStatusSync
    pub_sentinel_status_ = this->create_publisher<std_msgs::msg::String>(ros_prefix_ + "/sentinel_status", 10);

    // 22. DartSelectTargetStatusSync
    pub_dart_select_target_ = this->create_publisher<std_msgs::msg::String>(ros_prefix_ + "/dart_select_target", 10);

    // 23. GuardCtrlResult
    pub_guard_ctrl_result_ = this->create_publisher<std_msgs::msg::String>(ros_prefix_ + "/guard_ctrl_result", 10);

    // 24. AirSupportStatusSync
    pub_air_support_status_ = this->create_publisher<std_msgs::msg::String>(ros_prefix_ + "/air_support_status", 10);
  }

  void reconnect_tick()
  {
    std::lock_guard<std::mutex> lock(mqtt_mtx_);
    if (!mqtt_client_) return;

    if (!mqtt_client_->is_connected() || !subscribed_) {
      connect_and_subscribe_once_locked();
    }
  }

  void connect_and_subscribe_once()
  {
    std::lock_guard<std::mutex> lock(mqtt_mtx_);
    connect_and_subscribe_once_locked();
  }

  void connect_and_subscribe_once_locked()
  {
    if (!mqtt_client_) return;

    mqtt::connect_options opts;
    opts.set_clean_session(true);
    std::string subscribing_topic;

    try {
      if (!mqtt_client_->is_connected()) {
        RCLCPP_INFO(this->get_logger(), "Connecting MQTT broker: %s", server_uri_.c_str());
        mqtt_client_->connect(opts)->wait();
        RCLCPP_INFO(this->get_logger(), "MQTT connected.");
      }

      const auto all_topics = get_receiver_topics();
      RCLCPP_INFO(this->get_logger(), "Start subscribing MQTT topics: count=%zu", all_topics.size());

      for (const auto& t : all_topics) {
        subscribing_topic = t;
        mqtt_client_->subscribe(t, qos_)->wait();
      }
      subscribed_ = true;

      RCLCPP_INFO(this->get_logger(), "MQTT subscribe completed: count=%zu", all_topics.size());
    }
    catch (const mqtt::exception &e) {
      subscribed_ = false;
      RCLCPP_WARN(
        this->get_logger(),
        "MQTT connect/subscribe failed (topic=%s): %s",
        (subscribing_topic.empty() ? "<none>" : subscribing_topic.c_str()),
        e.what());
    }
  }

  void on_mqtt_message(const mqtt::const_message_ptr& msg)
  {
    const std::string& topic = msg->get_topic();
    const auto& bytes = msg->get_payload_ref();
    std::string payload(bytes.data(), bytes.data() + bytes.length());

    if (topic == "GameStatus") {
      handle_game_status(payload);
    } else if (topic == "GlobalUnitStatus") {
      handle_global_unit_status(payload);
    } else if (topic == "GlobalLogisticsStatus") {
      handle_global_logistics_status(payload);
    } else if (topic == "GlobalSpecialMechanism") {
      handle_global_special_mechanism(payload);
    } else if (topic == "Event") {
      handle_event(payload);
    } else if (topic == "RobotInjuryStat") {
      handle_robot_injury_stat(payload);
    } else if (topic == "RobotRespawnStatus") {
      handle_robot_respawn_status(payload);
    } else if (topic == "RobotStaticStatus") {
      handle_robot_static_status(payload);
    } else if (topic == "RobotDynamicStatus") {
      handle_robot_dynamic_status(payload);
    } else if (topic == "RobotModuleStatus") {
      handle_robot_module_status(payload);
    } else if (topic == "RobotPosition") {
      handle_robot_position(payload);
    } else if (topic == "Buff") {
      handle_buff(payload);
    } else if (topic == "PenaltyInfo") {
      handle_penalty_info(payload);
    } else if (topic == "RobotPathPlanInfo") {
      handle_robot_path_plan_info(payload);
    } else if (topic == "RaderInfoToClient") {
      handle_rader_info(payload);
    } else if (topic == "CustomByteBlock") {
      handle_custom_byte_block(payload);
    } else if (topic == "TechCoreMotionStateSync") {
      handle_tech_core_motion(payload);
    } else if (topic == "RobotPerformanceSelectionSync") {
      handle_robot_performance(payload);
    } else if (topic == "DeployModeStatusSync") {
      handle_deploy_mode_status(payload);
    } else if (topic == "RuneStatusSync") {
      handle_rune_status(payload);
    } else if (topic == "SentinelStatusSync") {
      handle_sentinel_status(payload);
    } else if (topic == "DartSelectTargetStatusSync") {
      handle_dart_select_target(payload);
    } else if (topic == "GuardCtrlResult") {
      handle_guard_ctrl_result(payload);
    } else if (topic == "AirSupportStatusSync") {
      handle_air_support_status(payload);
    } else {
      RCLCPP_DEBUG(this->get_logger(), "Unknown topic received: %s", topic.c_str());
    }
  }

  std::vector<std::string> get_receiver_topics() const
  {
    std::vector<std::string> topics = topic_registry_.get_subscribe_topics();

    // 保留当前 receiver 的完整 24 topic 订阅能力，逐步迁移到 registry。
    const std::vector<std::string> legacy_topics = {
      "GameStatus",
      "GlobalUnitStatus",
      "GlobalLogisticsStatus",
      "GlobalSpecialMechanism",
      "Event",
      "RobotInjuryStat",
      "RobotRespawnStatus",
      "RobotStaticStatus",
      "RobotDynamicStatus",
      "RobotModuleStatus",
      "RobotPosition",
      "Buff",
      "PenaltyInfo",
      "RobotPathPlanInfo",
      "RaderInfoToClient",
      "CustomByteBlock",
      "TechCoreMotionStateSync",
      "RobotPerformanceSelectionSync",
      "DeployModeStatusSync",
      "RuneStatusSync",
      "SentinelStatusSync",
      "DartSelectTargetStatusSync",
      "GuardCtrlResult",
      "AirSupportStatusSync"
    };

    topics.insert(topics.end(), legacy_topics.begin(), legacy_topics.end());

    std::vector<std::string> deduped;
    deduped.reserve(topics.size());
    std::unordered_set<std::string> seen;
    for (const auto & t : topics) {
      if (seen.insert(t).second) {
        deduped.push_back(t);
      }
    }
    return deduped;
  }

  void log_core_state_snapshot()
  {
    RCLCPP_INFO(this->get_logger(), "%s", state_store_.debug_summary().c_str());
  }

  #define PUBLISH_JSON(publisher, json_content) \
    if (publisher) { \
      std_msgs::msg::String msg; \
      msg.data = json_content; \
      publisher->publish(msg); \
    }

  void handle_game_status(const std::string& payload)
  {
    GameStatus s;
    if (!s.ParseFromString(payload)) {
      RCLCPP_ERROR(this->get_logger(), "Parse GameStatus failed (%zu bytes)", payload.size());
      return;
    }

    state_store_.update_game_status(s);
    RCLCPP_DEBUG(this->get_logger(), "StateStore updated: GameStatus");
    if (!has_logged_first_game_status_snapshot_) {
      log_core_state_snapshot();
      has_logged_first_game_status_snapshot_ = true;
    }

    std::string json = R"({"current_round":)" + std::to_string(s.current_round()) +
      R"(,"total_rounds":)" + std::to_string(s.total_rounds()) +
      R"(,"red_score":)" + std::to_string(s.red_score()) +
      R"(,"blue_score":)" + std::to_string(s.blue_score()) +
      R"(,"current_stage":)" + std::to_string(s.current_stage()) +
      R"(,"stage_countdown_sec":)" + std::to_string(s.stage_countdown_sec()) +
      R"(,"stage_elapsed_sec":)" + std::to_string(s.stage_elapsed_sec()) +
      R"(,"is_paused":)" + (s.is_paused() ? "true" : "false") + "}";
    PUBLISH_JSON(pub_game_status_, json);
  }

  void handle_global_unit_status(const std::string& payload)
  {
    GlobalUnitStatus s;
    if (!s.ParseFromString(payload)) {
      RCLCPP_ERROR(this->get_logger(), "Parse GlobalUnitStatus failed");
      return;
    }

    state_store_.update_global_unit_status(s);
    RCLCPP_DEBUG(this->get_logger(), "StateStore updated: GlobalUnitStatus");

    std::string json = R"({"base_health":)" + std::to_string(s.base_health()) +
      R"(,"base_status":)" + std::to_string(s.base_status()) +
      R"(,"base_shield":)" + std::to_string(s.base_shield()) +
      R"(,"outpost_health":)" + std::to_string(s.outpost_health()) +
      R"(,"outpost_status":)" + std::to_string(s.outpost_status()) +
      R"(,"total_damage_red":)" + std::to_string(s.total_damage_red()) +
      R"(,"total_damage_blue":)" + std::to_string(s.total_damage_blue());

    json += R"(,"robot_health":[)";
    for (int i = 0; i < s.robot_health_size(); ++i) {
      if (i > 0) json += ",";
      json += std::to_string(s.robot_health(i));
    }
    json += "]";

    json += R"(,"robot_bullets":[)";
    for (int i = 0; i < s.robot_bullets_size(); ++i) {
      if (i > 0) json += ",";
      json += std::to_string(s.robot_bullets(i));
    }
    json += "]}";

    PUBLISH_JSON(pub_global_unit_status_, json);
  }

  void handle_global_logistics_status(const std::string& payload)
  {
    GlobalLogisticsStatus s;
    if (!s.ParseFromString(payload)) {
      RCLCPP_ERROR(this->get_logger(), "Parse GlobalLogisticsStatus failed");
      return;
    }

    std::string json = R"({"remaining_economy":)" + std::to_string(s.remaining_economy()) +
      R"(,"total_economy_obtained":)" + std::to_string(s.total_economy_obtained()) +
      R"(,"tech_level":)" + std::to_string(s.tech_level()) +
      R"(,"encryption_level":)" + std::to_string(s.encryption_level()) + "}";
    PUBLISH_JSON(pub_global_logistics_status_, json);
  }

  void handle_global_special_mechanism(const std::string& payload)
  {
    GlobalSpecialMechanism s;
    if (!s.ParseFromString(payload)) {
      RCLCPP_ERROR(this->get_logger(), "Parse GlobalSpecialMechanism failed");
      return;
    }

    std::string json = R"({"mechanism_id":[)";
    for (int i = 0; i < s.mechanism_id_size(); ++i) {
      if (i > 0) json += ",";
      json += std::to_string(s.mechanism_id(i));
    }
    json += "],\"mechanism_time_sec\":[";
    for (int i = 0; i < s.mechanism_time_sec_size(); ++i) {
      if (i > 0) json += ",";
      json += std::to_string(s.mechanism_time_sec(i));
    }
    json += "]}";

    PUBLISH_JSON(pub_global_special_mechanism_, json);
  }

  void handle_event(const std::string& payload)
  {
    Event e;
    if (!e.ParseFromString(payload)) {
      RCLCPP_ERROR(this->get_logger(), "Parse Event failed");
      return;
    }

    std::string json = R"({"event_id":)" + std::to_string(e.event_id()) +
      R"(,"param":")" + e.param() + "\"}";
    PUBLISH_JSON(pub_event_, json);
  }

  void handle_robot_injury_stat(const std::string& payload)
  {
    RobotInjuryStat s;
    if (!s.ParseFromString(payload)) {
      RCLCPP_ERROR(this->get_logger(), "Parse RobotInjuryStat failed");
      return;
    }

    std::string json = R"({"total_damage":)" + std::to_string(s.total_damage()) +
      R"(,"collision_damage":)" + std::to_string(s.collision_damage()) +
      R"(,"small_projectile_damage":)" + std::to_string(s.small_projectile_damage()) +
      R"(,"large_projectile_damage":)" + std::to_string(s.large_projectile_damage()) +
      R"(,"dart_splash_damage":)" + std::to_string(s.dart_splash_damage()) +
      R"(,"module_offline_damage":)" + std::to_string(s.module_offline_damage()) +
      R"(,"wifi_offline_damage":)" + std::to_string(s.wifi_offline_damage()) +
      R"(,"penalty_damage":)" + std::to_string(s.penalty_damage()) +
      R"(,"server_kill_damage":)" + std::to_string(s.server_kill_damage()) +
      R"(,"killer_id":)" + std::to_string(s.killer_id()) + "}";
    PUBLISH_JSON(pub_robot_injury_stat_, json);
  }

  void handle_robot_respawn_status(const std::string& payload)
  {
    RobotRespawnStatus s;
    if (!s.ParseFromString(payload)) {
      RCLCPP_ERROR(this->get_logger(), "Parse RobotRespawnStatus failed");
      return;
    }

    std::string json = R"({"is_pending_respawn":)" + (s.is_pending_respawn() ? std::string("true") : std::string("false")) +
      R"(,"total_respawn_progress":)" + std::to_string(s.total_respawn_progress()) +
      R"(,"current_respawn_progress":)" + std::to_string(s.current_respawn_progress()) +
      R"(,"can_free_respawn":)" + (s.can_free_respawn() ? std::string("true") : std::string("false")) +
      R"(,"gold_cost_for_respawn":)" + std::to_string(s.gold_cost_for_respawn()) +
      R"(,"can_pay_for_respawn":)" + (s.can_pay_for_respawn() ? std::string("true") : std::string("false")) + "}";
    PUBLISH_JSON(pub_robot_respawn_status_, json);
  }

  void handle_robot_static_status(const std::string& payload)
  {
    RobotStaticStatus s;
    if (!s.ParseFromString(payload)) {
      RCLCPP_ERROR(this->get_logger(), "Parse RobotStaticStatus failed");
      return;
    }

    std::string json = R"({"connection_state":)" + std::to_string(s.connection_state()) +
      R"(,"field_state":)" + std::to_string(s.field_state()) +
      R"(,"alive_state":)" + std::to_string(s.alive_state()) +
      R"(,"robot_id":)" + std::to_string(s.robot_id()) +
      R"(,"robot_type":)" + std::to_string(s.robot_type()) +
      R"(,"performance_system_shooter":)" + std::to_string(s.performance_system_shooter()) +
      R"(,"performance_system_chassis":)" + std::to_string(s.performance_system_chassis()) +
      R"(,"level":)" + std::to_string(s.level()) +
      R"(,"max_health":)" + std::to_string(s.max_health()) +
      R"(,"max_heat":)" + std::to_string(s.max_heat()) +
      R"(,"heat_cooldown_rate":)" + std::to_string(s.heat_cooldown_rate()) +
      R"(,"max_power":)" + std::to_string(s.max_power()) +
      R"(,"max_buffer_energy":)" + std::to_string(s.max_buffer_energy()) +
      R"(,"max_chassis_energy":)" + std::to_string(s.max_chassis_energy()) + "}";
    PUBLISH_JSON(pub_robot_static_status_, json);
  }

  void handle_robot_dynamic_status(const std::string& payload)
  {
    RobotDynamicStatus s;
    if (!s.ParseFromString(payload)) {
      RCLCPP_ERROR(this->get_logger(), "Parse RobotDynamicStatus failed");
      return;
    }

    state_store_.update_robot_dynamic_status(s);
    RCLCPP_DEBUG(this->get_logger(), "StateStore updated: RobotDynamicStatus");

    std::string json = R"({"current_health":)" + std::to_string(s.current_health()) +
      R"(,"current_heat":)" + std::to_string(s.current_heat()) +
      R"(,"last_projectile_fire_rate":)" + std::to_string(s.last_projectile_fire_rate()) +
      R"(,"current_chassis_energy":)" + std::to_string(s.current_chassis_energy()) +
      R"(,"current_buffer_energy":)" + std::to_string(s.current_buffer_energy()) +
      R"(,"current_experience":)" + std::to_string(s.current_experience()) +
      R"(,"experience_for_upgrade":)" + std::to_string(s.experience_for_upgrade()) +
      R"(,"total_projectiles_fired":)" + std::to_string(s.total_projectiles_fired()) +
      R"(,"remaining_ammo":)" + std::to_string(s.remaining_ammo()) +
      R"(,"is_out_of_combat":)" + (s.is_out_of_combat() ? std::string("true") : std::string("false")) +
      R"(,"out_of_combat_countdown":)" + std::to_string(s.out_of_combat_countdown()) +
      R"(,"can_remote_heal":)" + (s.can_remote_heal() ? std::string("true") : std::string("false")) +
      R"(,"can_remote_ammo":)" + (s.can_remote_ammo() ? std::string("true") : std::string("false")) + "}";
    PUBLISH_JSON(pub_robot_dynamic_status_, json);
  }

  void handle_robot_module_status(const std::string& payload)
  {
    RobotModuleStatus s;
    if (!s.ParseFromString(payload)) {
      RCLCPP_ERROR(this->get_logger(), "Parse RobotModuleStatus failed");
      return;
    }

    std::string json = R"({"power_manager":)" + std::to_string(s.power_manager()) +
      R"(,"rfid":)" + std::to_string(s.rfid()) +
      R"(,"light_strip":)" + std::to_string(s.light_strip()) +
      R"(,"small_shooter":)" + std::to_string(s.small_shooter()) +
      R"(,"big_shooter":)" + std::to_string(s.big_shooter()) +
      R"(,"uwb":)" + std::to_string(s.uwb()) +
      R"(,"armor":)" + std::to_string(s.armor()) +
      R"(,"video_transmission":)" + std::to_string(s.video_transmission()) +
      R"(,"capacitor":)" + std::to_string(s.capacitor()) +
      R"(,"main_controller":)" + std::to_string(s.main_controller()) + "}";
    PUBLISH_JSON(pub_robot_module_status_, json);
  }

  void handle_robot_position(const std::string& payload)
  {
    RobotPosition s;
    if (!s.ParseFromString(payload)) {
      RCLCPP_ERROR(this->get_logger(), "Parse RobotPosition failed");
      return;
    }

    geometry_msgs::msg::Point p;
    p.x = s.x();
    p.y = s.y();
    p.z = s.z();
    if (pub_robot_position_) {
      pub_robot_position_->publish(p);
    }
  }

  void handle_buff(const std::string& payload)
  {
    Buff b;
    if (!b.ParseFromString(payload)) {
      RCLCPP_ERROR(this->get_logger(), "Parse Buff failed");
      return;
    }

    std::string json = R"({"robot_id":)" + std::to_string(b.robot_id()) +
      R"(,"buff_type":)" + std::to_string(b.buff_type()) +
      R"(,"buff_level":)" + std::to_string(b.buff_level()) +
      R"(,"buff_max_time":)" + std::to_string(b.buff_max_time()) +
      R"(,"buff_left_time":)" + std::to_string(b.buff_left_time()) +
      R"(,"msg_params":")" + b.msg_params() + "\"}";
    PUBLISH_JSON(pub_buff_, json);
  }

  void handle_penalty_info(const std::string& payload)
  {
    PenaltyInfo p;
    if (!p.ParseFromString(payload)) {
      RCLCPP_ERROR(this->get_logger(), "Parse PenaltyInfo failed");
      return;
    }

    std::string json = R"({"penalty_type":)" + std::to_string(p.penalty_type()) +
      R"(,"penalty_effect_sec":)" + std::to_string(p.penalty_effect_sec()) +
      R"(,"total_penalty_num":)" + std::to_string(p.total_penalty_num()) + "}";
    PUBLISH_JSON(pub_penalty_info_, json);
  }

  void handle_robot_path_plan_info(const std::string& payload)
  {
    RobotPathPlanInfo r;
    if (!r.ParseFromString(payload)) {
      RCLCPP_ERROR(this->get_logger(), "Parse RobotPathPlanInfo failed");
      return;
    }

    std::string json = R"({"intention":)" + std::to_string(r.intention()) +
      R"(,"start_pos_x":)" + std::to_string(r.start_pos_x()) +
      R"(,"start_pos_y":)" + std::to_string(r.start_pos_y()) +
      R"(,"sender_id":)" + std::to_string(r.sender_id());

    json += R"(,"offset_x":[)";
    for (int i = 0; i < r.offset_x_size(); ++i) {
      if (i > 0) json += ",";
      json += std::to_string(r.offset_x(i));
    }
    json += "],\"offset_y\":[";
    for (int i = 0; i < r.offset_y_size(); ++i) {
      if (i > 0) json += ",";
      json += std::to_string(r.offset_y(i));
    }
    json += "]}";

    PUBLISH_JSON(pub_robot_path_plan_info_, json);
  }

  void handle_rader_info(const std::string& payload)
  {
    RaderInfoToClient r;
    if (!r.ParseFromString(payload)) {
      RCLCPP_ERROR(this->get_logger(), "Parse RaderInfoToClient failed");
      return;
    }

    std::string json = R"({"target_robot_id":)" + std::to_string(r.target_robot_id()) +
      R"(,"target_pos_x":)" + std::to_string(r.target_pos_x()) +
      R"(,"target_pos_y":)" + std::to_string(r.target_pos_y()) +
      R"(,"torward_angle":)" + std::to_string(r.torward_angle()) +
      R"(,"is_high_light":)" + std::to_string(r.is_high_light()) + "}";
    PUBLISH_JSON(pub_rader_info_, json);
  }

  void handle_custom_byte_block(const std::string& payload)
  {
    CustomByteBlock c;
    if (!c.ParseFromString(payload)) {
      RCLCPP_ERROR(this->get_logger(), "Parse CustomByteBlock failed");
      return;
    }

    std_msgs::msg::UInt8MultiArray out;
    out.data.assign(c.data().begin(), c.data().end());
    if (pub_custom_byte_block_) {
      pub_custom_byte_block_->publish(out);
    }
  }

  void handle_tech_core_motion(const std::string& payload)
  {
    TechCoreMotionStateSync t;
    if (!t.ParseFromString(payload)) {
      RCLCPP_ERROR(this->get_logger(), "Parse TechCoreMotionStateSync failed");
      return;
    }

    std::string json = R"({"maximum_difficulty_level":)" + std::to_string(t.maximum_difficulty_level()) +
      R"(,"status":)" + std::to_string(t.status()) + "}";
    PUBLISH_JSON(pub_tech_core_motion_, json);
  }

  void handle_robot_performance(const std::string& payload)
  {
    RobotPerformanceSelectionSync r;
    if (!r.ParseFromString(payload)) {
      RCLCPP_ERROR(this->get_logger(), "Parse RobotPerformanceSelectionSync failed");
      return;
    }

    std::string json = R"({"shooter":)" + std::to_string(r.shooter()) +
      R"(,"chassis":)" + std::to_string(r.chassis()) + "}";
    PUBLISH_JSON(pub_robot_performance_, json);
  }

  void handle_deploy_mode_status(const std::string& payload)
  {
    DeployModeStatusSync d;
    if (!d.ParseFromString(payload)) {
      RCLCPP_ERROR(this->get_logger(), "Parse DeployModeStatusSync failed");
      return;
    }

    std_msgs::msg::UInt32 u32;
    u32.data = d.status();
    if (pub_deploy_mode_status_) {
      pub_deploy_mode_status_->publish(u32);
    }
  }

  void handle_rune_status(const std::string& payload)
  {
    RuneStatusSync r;
    if (!r.ParseFromString(payload)) {
      RCLCPP_ERROR(this->get_logger(), "Parse RuneStatusSync failed");
      return;
    }

    std::string json = R"({"rune_status":)" + std::to_string(r.rune_status()) +
      R"(,"activated_arms":)" + std::to_string(r.activated_arms()) +
      R"(,"average_rings":)" + std::to_string(r.average_rings()) + "}";
    PUBLISH_JSON(pub_rune_status_, json);
  }

  void handle_sentinel_status(const std::string& payload)
  {
    SentinelStatusSync s;
    if (!s.ParseFromString(payload)) {
      RCLCPP_ERROR(this->get_logger(), "Parse SentinelStatusSync failed");
      return;
    }

    std::string json = R"({"posture_id":)" + std::to_string(s.posture_id()) +
      R"(,"is_weakened":)" + (s.is_weakened() ? std::string("true") : std::string("false")) + "}";
    PUBLISH_JSON(pub_sentinel_status_, json);
  }

  void handle_dart_select_target(const std::string& payload)
  {
    DartSelectTargetStatusSync d;
    if (!d.ParseFromString(payload)) {
      RCLCPP_ERROR(this->get_logger(), "Parse DartSelectTargetStatusSync failed");
      return;
    }

    std::string json = R"({"target_id":)" + std::to_string(d.target_id()) +
      R"(,"open":)" + (d.open() ? std::string("true") : std::string("false")) + "}";
    PUBLISH_JSON(pub_dart_select_target_, json);
  }

  void handle_guard_ctrl_result(const std::string& payload)
  {
    GuardCtrlResult g;
    if (!g.ParseFromString(payload)) {
      RCLCPP_ERROR(this->get_logger(), "Parse GuardCtrlResult failed");
      return;
    }

    state_store_.update_guard_ctrl_result(g);
    RCLCPP_INFO(this->get_logger(), "StateStore updated: GuardCtrlResult");
    log_core_state_snapshot();

    std::string json = R"({"command_id":)" + std::to_string(g.command_id()) +
      R"(,"result_code":)" + std::to_string(g.result_code()) + "}";
    PUBLISH_JSON(pub_guard_ctrl_result_, json);
  }

  void handle_air_support_status(const std::string& payload)
  {
    AirSupportStatusSync a;
    if (!a.ParseFromString(payload)) {
      RCLCPP_ERROR(this->get_logger(), "Parse AirSupportStatusSync failed");
      return;
    }

    std::string json = R"({"airsupport_status":)" + std::to_string(a.airsupport_status()) +
      R"(,"left_time":)" + std::to_string(a.left_time()) +
      R"(,"cost_coins":)" + std::to_string(a.cost_coins()) + "}";
    PUBLISH_JSON(pub_air_support_status_, json);
  }

private:
  // params
  std::string host_;
  int port_{3333};
  int qos_{1};
  std::string client_id_;
  double reconnect_period_sec_{2.0};
  std::string ros_prefix_{"/judge"};

  // mqtt
  std::string server_uri_;
  std::unique_ptr<mqtt::async_client> mqtt_client_;
  std::shared_ptr<MqttCallback> mqtt_cb_;
  std::mutex mqtt_mtx_;
  bool subscribed_{false};
  rm_client::TopicRegistry topic_registry_;
  rm_client::StateStore state_store_;
  bool has_logged_first_game_status_snapshot_{false};

  // ROS publishers (24 total)
  rclcpp::TimerBase::SharedPtr reconnect_timer_;

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_game_status_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_global_unit_status_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_global_logistics_status_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_global_special_mechanism_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_event_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_robot_injury_stat_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_robot_respawn_status_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_robot_static_status_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_robot_dynamic_status_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_robot_module_status_;
  rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr pub_robot_position_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_buff_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_penalty_info_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_robot_path_plan_info_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_rader_info_;
  rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr pub_custom_byte_block_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_tech_core_motion_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_robot_performance_;
  rclcpp::Publisher<std_msgs::msg::UInt32>::SharedPtr pub_deploy_mode_status_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_rune_status_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_sentinel_status_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_dart_select_target_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_guard_ctrl_result_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_air_support_status_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<JudgeReceiverNode>());
  rclcpp::shutdown();
  return 0;
}