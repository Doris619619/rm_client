#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/u_int8_multi_array.hpp>

#include <opencv2/opencv.hpp>

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

class CameraCustomImagePubNode : public rclcpp::Node {
public:
  CameraCustomImagePubNode()
  : Node("camera_custom_image_pub"),
    device_index_(this->declare_parameter<int>("device_index", 0)),
    publish_fps_(this->declare_parameter<double>("publish_fps", 5.0)),
    chunk_payload_size_(this->declare_parameter<int>("chunk_payload_size", 200)),
    frame_id_(0)
  {
    if (publish_fps_ <= 0.0) {
      RCLCPP_WARN(this->get_logger(), "publish_fps<=0 is invalid, fallback to 5.0");
      publish_fps_ = 5.0;
    }

    if (chunk_payload_size_ <= 0) {
      RCLCPP_WARN(this->get_logger(), "chunk_payload_size<=0 is invalid, fallback to 200");
      chunk_payload_size_ = 200;
    }

    // total_chunks is one byte, so payload size cannot be too small.
    const int min_payload_size =
      static_cast<int>((kImageBytes + kMaxChunks - 1) / kMaxChunks);
    if (chunk_payload_size_ < min_payload_size) {
      RCLCPP_WARN(
        this->get_logger(),
        "chunk_payload_size=%d too small, raise to %d to keep total_chunks<=255",
        chunk_payload_size_,
        min_payload_size);
      chunk_payload_size_ = min_payload_size;
    }

    image_pub_ = this->create_publisher<std_msgs::msg::UInt8MultiArray>(
      "/judge/custom_byte_block", rclcpp::QoS(10));

    open_camera_once();

    const std::size_t total_chunks =
      (kImageBytes + static_cast<std::size_t>(chunk_payload_size_) - 1) /
      static_cast<std::size_t>(chunk_payload_size_);

    RCLCPP_INFO(
      this->get_logger(),
      "camera_custom_image_pub config: device_index=%d fps=%.2f image=%dx%d chunk_payload_size=%d chunks=%zu",
      device_index_,
      publish_fps_,
      static_cast<int>(kWidth),
      static_cast<int>(kHeight),
      chunk_payload_size_,
      total_chunks);

    const auto period = std::chrono::duration<double>(1.0 / publish_fps_);
    timer_ = this->create_wall_timer(
      std::chrono::duration_cast<std::chrono::milliseconds>(period),
      std::bind(&CameraCustomImagePubNode::on_timer, this));
  }

  ~CameraCustomImagePubNode() override
  {
    if (cap_.isOpened()) {
      cap_.release();
    }
  }

private:
  static constexpr uint8_t kMagicR = 0x52;
  static constexpr uint8_t kMagicM = 0x4D;
  static constexpr uint8_t kVersion = 0x01;
  static constexpr uint8_t kMsgTypeImage = 0x01;
  static constexpr uint8_t kWidth = 48;
  static constexpr uint8_t kHeight = 48;
  static constexpr uint8_t kEncodingGray8 = 0x01;
  static constexpr uint8_t kReserved = 0x00;
  static constexpr std::size_t kHeaderBytes = 12;
  static constexpr std::size_t kImageBytes = static_cast<std::size_t>(kWidth) * kHeight;
  static constexpr int kMaxChunks = 255;

  bool open_camera_once()
  {
    if (cap_.isOpened()) {
      return true;
    }

    if (!cap_.open(device_index_)) {
      RCLCPP_WARN(
        this->get_logger(),
        "Failed to open camera /dev/video%d (device_index=%d)",
        device_index_,
        device_index_);
      return false;
    }

    RCLCPP_INFO(
      this->get_logger(),
      "Camera opened: /dev/video%d (device_index=%d)",
      device_index_,
      device_index_);
    return true;
  }

  bool read_processed_frame(std::vector<uint8_t> &gray_48x48)
  {
    cv::Mat frame;
    cap_ >> frame;
    if (frame.empty()) {
      RCLCPP_WARN(this->get_logger(), "Camera frame read failed.");
      return false;
    }

    cv::Mat gray;
    if (frame.channels() == 3) {
      cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
    } else if (frame.channels() == 4) {
      cv::cvtColor(frame, gray, cv::COLOR_BGRA2GRAY);
    } else if (frame.channels() == 1) {
      gray = frame;
    } else {
      RCLCPP_WARN(this->get_logger(), "Unsupported camera channels: %d", frame.channels());
      return false;
    }

    cv::Mat resized;
    cv::resize(
      gray,
      resized,
      cv::Size(static_cast<int>(kWidth), static_cast<int>(kHeight)),
      0.0,
      0.0,
      cv::INTER_AREA);

    if (!resized.isContinuous()) {
      resized = resized.clone();
    }

    gray_48x48.assign(resized.data, resized.data + static_cast<std::ptrdiff_t>(kImageBytes));
    return true;
  }

  void publish_chunks(const std::vector<uint8_t> &image_bytes)
  {
    const std::size_t chunk_payload_size = static_cast<std::size_t>(chunk_payload_size_);
    const std::size_t total_chunks_sz =
      (image_bytes.size() + chunk_payload_size - 1) / chunk_payload_size;

    if (total_chunks_sz == 0 || total_chunks_sz > static_cast<std::size_t>(kMaxChunks)) {
      RCLCPP_ERROR(this->get_logger(), "Invalid total_chunks=%zu", total_chunks_sz);
      return;
    }

    const uint8_t total_chunks = static_cast<uint8_t>(total_chunks_sz);

    for (uint8_t chunk_id = 0; chunk_id < total_chunks; ++chunk_id) {
      const std::size_t offset = static_cast<std::size_t>(chunk_id) * chunk_payload_size;
      const std::size_t remain = image_bytes.size() - offset;
      const std::size_t payload_size = std::min(chunk_payload_size, remain);

      std_msgs::msg::UInt8MultiArray msg;
      msg.data.reserve(kHeaderBytes + payload_size);

      msg.data.push_back(kMagicR);
      msg.data.push_back(kMagicM);
      msg.data.push_back(kVersion);
      msg.data.push_back(kMsgTypeImage);
      msg.data.push_back(static_cast<uint8_t>(frame_id_ & 0xFFu));
      msg.data.push_back(static_cast<uint8_t>((frame_id_ >> 8) & 0xFFu));
      msg.data.push_back(chunk_id);
      msg.data.push_back(total_chunks);
      msg.data.push_back(kWidth);
      msg.data.push_back(kHeight);
      msg.data.push_back(kEncodingGray8);
      msg.data.push_back(kReserved);

      msg.data.insert(
        msg.data.end(),
        image_bytes.begin() + static_cast<std::ptrdiff_t>(offset),
        image_bytes.begin() + static_cast<std::ptrdiff_t>(offset + payload_size));

      image_pub_->publish(msg);
    }

    frame_id_ = static_cast<uint16_t>(frame_id_ + 1);
  }

  void on_timer()
  {
    if (!open_camera_once()) {
      return;
    }

    std::vector<uint8_t> image_bytes;
    if (!read_processed_frame(image_bytes)) {
      return;
    }

    publish_chunks(image_bytes);
  }

private:
  int device_index_;
  double publish_fps_;
  int chunk_payload_size_;
  uint16_t frame_id_;

  cv::VideoCapture cap_;

  rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr image_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<CameraCustomImagePubNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
