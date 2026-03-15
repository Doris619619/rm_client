// src/judge_sender.cpp
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/u_int32.hpp>

#include <mqtt/async_client.h>
#include "guard_ctrl.pb.h"

#include <cstdint>
#include <string>
#include <memory>
#include <mutex>

class JudgeSenderNode : public rclcpp::Node {
public:
  JudgeSenderNode()
  : Node("judge_sender")//说明节点的名字叫做judgersender
  {
    // ---------- 参数 ----------
    // 平时自测：127.0.0.1（本机起 mosquitto）
    // 比赛现场：改成 192.168.12.1（或按现场文档）
    this->declare_parameter<std::string>("judge_host", "127.0.0.1");
    this->declare_parameter<int>("judge_port", 3333);
    this->declare_parameter<std::string>("topic_cmd", "GuardCtrlCommand");
    this->declare_parameter<int>("qos", 1);
    this->declare_parameter<std::string>("client_id", "rm_semi_auto_client");
    this->declare_parameter<double>("reconnect_period_sec", 2.0);

    host_ = this->get_parameter("judge_host").as_string();
    port_ = this->get_parameter("judge_port").as_int();
    topic_ = this->get_parameter("topic_cmd").as_string();
    qos_ = this->get_parameter("qos").as_int();
    client_id_ = this->get_parameter("client_id").as_string();
    reconnect_period_sec_ = this->get_parameter("reconnect_period_sec").as_double();

    server_uri_ = "tcp://" + host_ + ":" + std::to_string(port_);

    // ---------- ROS 订阅 ----------
    sub_ = this->create_subscription<std_msgs::msg::UInt32>(
      "/semi_auto/command_id", 10,
      std::bind(&JudgeSenderNode::on_command, this, std::placeholders::_1)
    );

    // ---------- MQTT client ----------
    mqtt_client_ = std::make_unique<mqtt::async_client>(server_uri_, client_id_);

    // 尝试首次连接
    connect_mqtt_once();

    // 定时重连：如果没连上，每隔一段时间尝试一次
    reconnect_timer_ = this->create_wall_timer(
      std::chrono::duration<double>(reconnect_period_sec_),
      std::bind(&JudgeSenderNode::reconnect_tick, this)
    );

    RCLCPP_INFO(this->get_logger(),
      "JudgeSender ready.\n"
      "Subscribe: /semi_auto/command_id\n"
      "MQTT: uri=%s client_id=%s topic=%s qos=%d\n"
      "Reconnect every %.1fs when disconnected.",
      server_uri_.c_str(), client_id_.c_str(), topic_.c_str(), qos_, reconnect_period_sec_);
  }

  ~JudgeSenderNode() override
  {
    std::lock_guard<std::mutex> lock(mqtt_mtx_);
    try {
      if (mqtt_client_ && mqtt_client_->is_connected()) {
        mqtt_client_->disconnect()->wait();
      }
    } catch (...) {
      // ignore
    }
  }

private:
  void on_command(const std_msgs::msg::UInt32::SharedPtr msg)
  {
    const uint32_t cmd_id = msg->data;
    send_to_judge(cmd_id);
  }

  void reconnect_tick()
  {
    std::lock_guard<std::mutex> lock(mqtt_mtx_);
    if (!mqtt_client_) return;

    if (!mqtt_client_->is_connected()) {
      connect_mqtt_once_locked();
    }
  }

  void connect_mqtt_once()
  {
    std::lock_guard<std::mutex> lock(mqtt_mtx_);
    connect_mqtt_once_locked();
  }

  void connect_mqtt_once_locked()
  {
    if (!mqtt_client_) return;
    if (mqtt_client_->is_connected()) return;

    mqtt::connect_options opts;
    opts.set_clean_session(true);

    try {
      RCLCPP_INFO(this->get_logger(), "Connecting MQTT broker: %s", server_uri_.c_str());
      mqtt_client_->connect(opts)->wait();
      RCLCPP_INFO(this->get_logger(), "MQTT connected.");
    } catch (const mqtt::exception &e) {
      RCLCPP_WARN(this->get_logger(), "MQTT connect failed: %s", e.what());
    }
  }

  void send_to_judge(uint32_t command_id)
  {
    // 你可以在这里加任何“更聪明的半自动逻辑”
    // 例如：某些条件不发、限制重复、组合指令等

    std::lock_guard<std::mutex> lock(mqtt_mtx_);

    if (!mqtt_client_ || !mqtt_client_->is_connected()) {
      RCLCPP_WARN(this->get_logger(), "MQTT not connected. Drop command_id=%u", command_id);
      return;
    }

    // 1) 构造 Protobuf
    GuardCtrlCommand cmd;
    cmd.set_command_id(command_id);

    // 2) 序列化为二进制 payload
    std::string payload;
    if (!cmd.SerializeToString(&payload)) {
      RCLCPP_ERROR(this->get_logger(), "Protobuf SerializeToString failed.");
      return;
    }

    // 3) MQTT publish
    try {
      auto m = mqtt::make_message(topic_, payload);
      m->set_qos(qos_);
      mqtt_client_->publish(m)->wait();

      RCLCPP_INFO(this->get_logger(), "[SENT] %s command_id=%u (payload=%zu bytes)",
                  topic_.c_str(), command_id, payload.size());
    } catch (const mqtt::exception &e) {
      RCLCPP_ERROR(this->get_logger(), "MQTT publish failed: %s", e.what());
    }
  }

private:
  // ROS
  rclcpp::Subscription<std_msgs::msg::UInt32>::SharedPtr sub_;
  rclcpp::TimerBase::SharedPtr reconnect_timer_;

  // params
  std::string host_;
  int port_;
  std::string topic_;
  int qos_;
  std::string client_id_;
  double reconnect_period_sec_;

  // mqtt
  std::string server_uri_;
  std::unique_ptr<mqtt::async_client> mqtt_client_;
  std::mutex mqtt_mtx_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<JudgeSenderNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
