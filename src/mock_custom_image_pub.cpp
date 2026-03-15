#include <rclcpp/rclcpp.hpp>

#include <mqtt/async_client.h>
#include "judge_system.pb.h"

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

class MockCustomImagePubNode : public rclcpp::Node {
public:
  MockCustomImagePubNode()
  : Node("mock_custom_image_pub"),
    server_uri_("tcp://127.0.0.1:3333"),
    topic_("CustomByteBlock"),
    qos_(1),
    client_id_("mock_custom_image_pub"),
    frame_id_(0),
    image_(build_test_image())
  {
    mqtt_client_ = std::make_unique<mqtt::async_client>(server_uri_, client_id_);

    connect_mqtt_once();

    reconnect_timer_ = this->create_wall_timer(
      std::chrono::seconds(2),
      std::bind(&MockCustomImagePubNode::reconnect_tick, this));

    publish_timer_ = this->create_wall_timer(
      std::chrono::seconds(1),
      std::bind(&MockCustomImagePubNode::publish_one_frame, this));

    RCLCPP_INFO(
      this->get_logger(),
      "mock_custom_image_pub started. MQTT=%s topic=%s qos=%d fps=1 image=48x48 gray8",
      server_uri_.c_str(), topic_.c_str(), qos_);
  }

  ~MockCustomImagePubNode() override
  {
    std::lock_guard<std::mutex> lock(mqtt_mtx_);
    try {
      if (mqtt_client_ && mqtt_client_->is_connected()) {
        mqtt_client_->disconnect()->wait();
      }
    } catch (...) {
    }
  }

private:
  static constexpr uint8_t kMagicR = 0x52;
  static constexpr uint8_t kMagicM = 0x4D;
  static constexpr uint8_t kVersion = 0x01;
  static constexpr uint8_t kMsgTypeImage = 0x01;
  static constexpr uint8_t kWidth = 48;
  static constexpr uint8_t kHeight = 48;
  static constexpr uint8_t kEncodingGray8 = 1;
  static constexpr uint8_t kReserved = 0;
  static constexpr std::size_t kImageBytes = 48 * 48;
  static constexpr std::size_t kChunkPayloadBytes = 120;
  static constexpr std::size_t kHeaderBytes = 12;

  static std::vector<uint8_t> build_test_image()
  {
    std::vector<uint8_t> img(kImageBytes, 0);

    for (int y = 0; y < 48; ++y) {
      for (int x = 0; x < 48; ++x) {
        if (x == 24 || y == 24) {
          img[static_cast<std::size_t>(y) * 48 + static_cast<std::size_t>(x)] = 255;
        }

        if (x >= 6 && x <= 11 && y >= 6 && y <= 11) {
          img[static_cast<std::size_t>(y) * 48 + static_cast<std::size_t>(x)] = 220;
        }

        if (x == y && x >= 30) {
          img[static_cast<std::size_t>(y) * 48 + static_cast<std::size_t>(x)] = 180;
        }
      }
    }

    return img;
  }

  void reconnect_tick()
  {
    std::lock_guard<std::mutex> lock(mqtt_mtx_);
    if (!mqtt_client_) {
      return;
    }

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
    if (!mqtt_client_ || mqtt_client_->is_connected()) {
      return;
    }

    mqtt::connect_options opts;
    opts.set_clean_session(true);

    try {
      RCLCPP_INFO(this->get_logger(), "Connecting MQTT: %s", server_uri_.c_str());
      mqtt_client_->connect(opts)->wait();
      RCLCPP_INFO(this->get_logger(), "MQTT connected.");
    } catch (const mqtt::exception &e) {
      RCLCPP_WARN(this->get_logger(), "MQTT connect failed: %s", e.what());
    }
  }

  void publish_one_frame()
  {
    std::lock_guard<std::mutex> lock(mqtt_mtx_);
    if (!mqtt_client_ || !mqtt_client_->is_connected()) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 3000, "MQTT not connected, skip frame.");
      return;
    }

    const std::size_t total_chunks_sz =
      (image_.size() + kChunkPayloadBytes - 1) / kChunkPayloadBytes;
    if (total_chunks_sz == 0 || total_chunks_sz > 255) {
      RCLCPP_ERROR(this->get_logger(), "Invalid total_chunks: %zu", total_chunks_sz);
      return;
    }

    const uint8_t total_chunks = static_cast<uint8_t>(total_chunks_sz);
    bool ok = true;

    for (uint8_t chunk_id = 0; chunk_id < total_chunks; ++chunk_id) {
      const std::size_t offset = static_cast<std::size_t>(chunk_id) * kChunkPayloadBytes;
      const std::size_t remain = image_.size() - offset;
      const std::size_t payload_size = std::min(kChunkPayloadBytes, remain);

      std::string data;
      data.resize(kHeaderBytes + payload_size);

      data[0] = static_cast<char>(kMagicR);
      data[1] = static_cast<char>(kMagicM);
      data[2] = static_cast<char>(kVersion);
      data[3] = static_cast<char>(kMsgTypeImage);
      data[4] = static_cast<char>(frame_id_ & 0xFFu);
      data[5] = static_cast<char>((frame_id_ >> 8) & 0xFFu);
      data[6] = static_cast<char>(chunk_id);
      data[7] = static_cast<char>(total_chunks);
      data[8] = static_cast<char>(kWidth);
      data[9] = static_cast<char>(kHeight);
      data[10] = static_cast<char>(kEncodingGray8);
      data[11] = static_cast<char>(kReserved);

      std::copy_n(
        image_.begin() + static_cast<std::ptrdiff_t>(offset),
        static_cast<std::ptrdiff_t>(payload_size),
        data.begin() + static_cast<std::ptrdiff_t>(kHeaderBytes));

      CustomByteBlock msg;
      msg.set_data(data);

      std::string payload;
      if (!msg.SerializeToString(&payload)) {
        RCLCPP_ERROR(this->get_logger(), "Serialize CustomByteBlock failed.");
        ok = false;
        break;
      }

      try {
        auto mqtt_msg = mqtt::make_message(topic_, payload);
        mqtt_msg->set_qos(qos_);
        mqtt_client_->publish(mqtt_msg)->wait();
      } catch (const mqtt::exception &e) {
        RCLCPP_WARN(this->get_logger(), "Publish chunk failed: %s", e.what());
        ok = false;
        break;
      }
    }

    if (ok) {
      RCLCPP_INFO(
        this->get_logger(),
        "Sent frame_id=%u chunks=%u image_bytes=%zu",
        frame_id_,
        total_chunks,
        image_.size());
      frame_id_ = static_cast<uint16_t>(frame_id_ + 1);
    }
  }

private:
  std::string server_uri_;
  std::string topic_;
  int qos_;
  std::string client_id_;

  std::unique_ptr<mqtt::async_client> mqtt_client_;
  std::mutex mqtt_mtx_;

  rclcpp::TimerBase::SharedPtr reconnect_timer_;
  rclcpp::TimerBase::SharedPtr publish_timer_;

  uint16_t frame_id_;
  std::vector<uint8_t> image_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MockCustomImagePubNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}