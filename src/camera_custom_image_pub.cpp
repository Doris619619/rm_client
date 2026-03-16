#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/u_int8_multi_array.hpp>

#include <MvCameraControl.h>
#include <opencv2/opencv.hpp>

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <cstring>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

class CameraCustomImagePubNode : public rclcpp::Node {
public:
  CameraCustomImagePubNode()
  : Node("camera_custom_image_pub"),
    camera_index_(this->declare_parameter<int>("camera_index", 0)),
    camera_serial_(this->declare_parameter<std::string>("camera_serial", "")),
    publish_fps_(this->declare_parameter<double>("publish_fps", 5.0)),
    roi_size_(this->declare_parameter<int>("roi_size", 480)),
    enable_clahe_(this->declare_parameter<bool>("enable_clahe", true)),
    clahe_clip_limit_(this->declare_parameter<double>("clahe_clip_limit", 3.0)),
    chunk_payload_size_(this->declare_parameter<int>("chunk_payload_size", 200)),
    grab_timeout_ms_(this->declare_parameter<int>("grab_timeout_ms", 1000)),
    exposure_time_(this->declare_parameter<double>("exposure_time", -1.0)),
    frame_id_(0)
  {
    this->declare_parameter<int>("device_index", -1);

    if (publish_fps_ <= 0.0) {
      RCLCPP_WARN(this->get_logger(), "publish_fps<=0 is invalid, fallback to 5.0");
      publish_fps_ = 5.0;
    }
    if (publish_fps_ > 50.0) {
      RCLCPP_WARN(this->get_logger(), "publish_fps=%.2f exceeds 50Hz, clamp to 50.0", publish_fps_);
      publish_fps_ = 50.0;
    }

    if (roi_size_ < 0) {
      RCLCPP_WARN(this->get_logger(), "roi_size<0 is invalid, fallback to 0(full frame)");
      roi_size_ = 0;
    }

    if (clahe_clip_limit_ <= 0.0) {
      RCLCPP_WARN(this->get_logger(), "clahe_clip_limit<=0 is invalid, fallback to 3.0");
      clahe_clip_limit_ = 3.0;
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

    if (camera_index_ < 0) {
      RCLCPP_WARN(this->get_logger(), "camera_index<0 is invalid, fallback to 0");
      camera_index_ = 0;
    }

    if (grab_timeout_ms_ <= 0) {
      RCLCPP_WARN(this->get_logger(), "grab_timeout_ms<=0 is invalid, fallback to 1000");
      grab_timeout_ms_ = 1000;
    }

    image_pub_ = this->create_publisher<std_msgs::msg::UInt8MultiArray>(
      "/judge/custom_byte_block", rclcpp::QoS(10));

    open_camera_once();

    const std::size_t total_chunks =
      (kImageBytes + static_cast<std::size_t>(chunk_payload_size_) - 1) /
      static_cast<std::size_t>(chunk_payload_size_);

    RCLCPP_INFO(
      this->get_logger(),
      "camera_custom_image_pub config: camera_index=%d camera_serial='%s' fps=%.2f image=%dx%d roi_size=%d enable_clahe=%s clahe_clip_limit=%.2f chunk_payload_size=%d chunks=%zu grab_timeout_ms=%d exposure_time=%.2f",
      camera_index_,
      camera_serial_.c_str(),
      publish_fps_,
      static_cast<int>(kWidth),
      static_cast<int>(kHeight),
      roi_size_,
      enable_clahe_ ? "true" : "false",
      clahe_clip_limit_,
      chunk_payload_size_,
      total_chunks,
      grab_timeout_ms_,
      exposure_time_);

    RCLCPP_INFO(this->get_logger(), "output image protocol fixed to 160x120 GRAY8 on /judge/custom_byte_block");

    const auto period = std::chrono::duration<double>(1.0 / publish_fps_);
    timer_ = this->create_wall_timer(
      std::chrono::duration_cast<std::chrono::milliseconds>(period),
      std::bind(&CameraCustomImagePubNode::on_timer, this));
  }

  ~CameraCustomImagePubNode() override
  {
    release_camera();
  }

private:
  static constexpr uint8_t kMagicR = 0x52;
  static constexpr uint8_t kMagicM = 0x4D;
  static constexpr uint8_t kVersion = 0x01;
  static constexpr uint8_t kMsgTypeImage = 0x01;
  static constexpr uint8_t kWidth = 160;
  static constexpr uint8_t kHeight = 120;
  static constexpr uint8_t kEncodingGray8 = 0x01;
  static constexpr uint8_t kReserved = 0x00;
  static constexpr std::size_t kHeaderBytes = 12;
  static constexpr std::size_t kImageBytes = static_cast<std::size_t>(kWidth) * kHeight;
  static constexpr int kMaxChunks = 255;

  static std::string as_hex(uint32_t value)
  {
    std::ostringstream oss;
    oss << "0x" << std::hex << std::uppercase << value;
    return oss.str();
  }

  static std::string device_type_to_string(unsigned int type)
  {
    if (type == MV_GIGE_DEVICE) {
      return "GigE";
    }
    if (type == MV_USB_DEVICE) {
      return "USB3";
    }
    return "Unknown";
  }

  static std::string cstr_from_u8(const unsigned char *data)
  {
    if (data == nullptr) {
      return std::string();
    }
    return std::string(reinterpret_cast<const char *>(data));
  }

  std::string device_model(const MV_CC_DEVICE_INFO *dev) const
  {
    if (dev == nullptr) {
      return "";
    }
    if (dev->nTLayerType == MV_GIGE_DEVICE) {
      return cstr_from_u8(dev->SpecialInfo.stGigEInfo.chModelName);
    }
    if (dev->nTLayerType == MV_USB_DEVICE) {
      return cstr_from_u8(dev->SpecialInfo.stUsb3VInfo.chModelName);
    }
    return "";
  }

  std::string device_serial(const MV_CC_DEVICE_INFO *dev) const
  {
    if (dev == nullptr) {
      return "";
    }
    if (dev->nTLayerType == MV_GIGE_DEVICE) {
      return cstr_from_u8(dev->SpecialInfo.stGigEInfo.chSerialNumber);
    }
    if (dev->nTLayerType == MV_USB_DEVICE) {
      return cstr_from_u8(dev->SpecialInfo.stUsb3VInfo.chSerialNumber);
    }
    return "";
  }

  bool select_device(const MV_CC_DEVICE_INFO_LIST &device_list, unsigned int &out_index)
  {
    if (device_list.nDeviceNum == 0U) {
      return false;
    }

    if (!camera_serial_.empty()) {
      for (unsigned int i = 0; i < device_list.nDeviceNum; ++i) {
        const auto *dev = device_list.pDeviceInfo[i];
        if (dev == nullptr) {
          continue;
        }
        if (device_serial(dev) == camera_serial_) {
          out_index = i;
          return true;
        }
      }
      RCLCPP_WARN(
        this->get_logger(),
        "camera_serial='%s' not found, fallback to camera_index=%d",
        camera_serial_.c_str(),
        camera_index_);
    }

    if (static_cast<unsigned int>(camera_index_) >= device_list.nDeviceNum) {
      RCLCPP_WARN(
        this->get_logger(),
        "camera_index=%d out of range (n=%u), fallback to 0",
        camera_index_,
        device_list.nDeviceNum);
      out_index = 0;
      return true;
    }

    out_index = static_cast<unsigned int>(camera_index_);
    return true;
  }

  void log_device_list(const MV_CC_DEVICE_INFO_LIST &device_list)
  {
    RCLCPP_INFO(this->get_logger(), "Enumerated Hikvision devices: %u", device_list.nDeviceNum);
    for (unsigned int i = 0; i < device_list.nDeviceNum; ++i) {
      const auto *dev = device_list.pDeviceInfo[i];
      if (dev == nullptr) {
        continue;
      }
      const std::string model = device_model(dev);
      const std::string serial = device_serial(dev);
      RCLCPP_INFO(
        this->get_logger(),
        "Device[%u]: type=%s model='%s' serial='%s'",
        i,
        device_type_to_string(dev->nTLayerType).c_str(),
        model.c_str(),
        serial.c_str());
    }
  }

  void release_camera()
  {
    if (camera_handle_ != nullptr) {
      if (is_grabbing_) {
        const int ret = MV_CC_StopGrabbing(camera_handle_);
        if (ret != MV_OK) {
          RCLCPP_WARN(
            this->get_logger(),
            "MV_CC_StopGrabbing failed: %s",
            as_hex(static_cast<uint32_t>(ret)).c_str());
        }
        is_grabbing_ = false;
      }

      if (is_opened_) {
        const int ret = MV_CC_CloseDevice(camera_handle_);
        if (ret != MV_OK) {
          RCLCPP_WARN(
            this->get_logger(),
            "MV_CC_CloseDevice failed: %s",
            as_hex(static_cast<uint32_t>(ret)).c_str());
        }
        is_opened_ = false;
      }

      const int ret = MV_CC_DestroyHandle(camera_handle_);
      if (ret != MV_OK) {
        RCLCPP_WARN(
          this->get_logger(),
          "MV_CC_DestroyHandle failed: %s",
          as_hex(static_cast<uint32_t>(ret)).c_str());
      }
      camera_handle_ = nullptr;
    }

    raw_buffer_.clear();
  }

  bool open_camera_once()
  {
    if (camera_handle_ != nullptr && is_opened_ && is_grabbing_) {
      return true;
    }

    release_camera();

    MV_CC_DEVICE_INFO_LIST device_list;
    std::memset(&device_list, 0, sizeof(device_list));

    const unsigned int tlayer = MV_GIGE_DEVICE | MV_USB_DEVICE;

    int ret = MV_CC_EnumDevices(tlayer, &device_list);
    if (ret != MV_OK) {
      RCLCPP_ERROR(
        this->get_logger(),
        "MV_CC_EnumDevices failed: %s",
        as_hex(static_cast<uint32_t>(ret)).c_str());
      return false;
    }

    log_device_list(device_list);

    if (device_list.nDeviceNum == 0U) {
      RCLCPP_WARN(this->get_logger(), "No Hikvision camera found.");
      return false;
    }

    unsigned int selected_index = 0;
    if (!select_device(device_list, selected_index)) {
      RCLCPP_WARN(this->get_logger(), "Failed to select camera.");
      return false;
    }

    MV_CC_DEVICE_INFO *selected = device_list.pDeviceInfo[selected_index];
    if (selected == nullptr) {
      RCLCPP_ERROR(this->get_logger(), "Selected camera info is null.");
      return false;
    }

    RCLCPP_INFO(
      this->get_logger(),
      "Connecting camera[%u]: type=%s model='%s' serial='%s'",
      selected_index,
      device_type_to_string(selected->nTLayerType).c_str(),
      device_model(selected).c_str(),
      device_serial(selected).c_str());

    ret = MV_CC_CreateHandle(&camera_handle_, selected);
    if (ret != MV_OK) {
      camera_handle_ = nullptr;
      RCLCPP_ERROR(
        this->get_logger(),
        "MV_CC_CreateHandle failed: %s",
        as_hex(static_cast<uint32_t>(ret)).c_str());
      return false;
    }

    ret = MV_CC_OpenDevice(camera_handle_);
    if (ret != MV_OK) {
      RCLCPP_ERROR(
        this->get_logger(),
        "MV_CC_OpenDevice failed: %s",
        as_hex(static_cast<uint32_t>(ret)).c_str());
      release_camera();
      return false;
    }
    is_opened_ = true;
    RCLCPP_INFO(this->get_logger(), "MV_CC_OpenDevice succeeded.");

    if (selected->nTLayerType == MV_GIGE_DEVICE) {
      const int packet_size = MV_CC_GetOptimalPacketSize(camera_handle_);
      if (packet_size > 0) {
        const int packet_ret = MV_CC_SetIntValue(camera_handle_, "GevSCPSPacketSize", packet_size);
        if (packet_ret != MV_OK) {
          RCLCPP_WARN(
            this->get_logger(),
            "MV_CC_SetIntValue(GevSCPSPacketSize=%d) failed: %s",
            packet_size,
            as_hex(static_cast<uint32_t>(packet_ret)).c_str());
        }
      } else {
        RCLCPP_WARN(
          this->get_logger(),
          "MV_CC_GetOptimalPacketSize returned %d",
          packet_size);
      }
    }

    ret = MV_CC_SetEnumValue(camera_handle_, "TriggerMode", 0);
    if (ret != MV_OK) {
      RCLCPP_WARN(
        this->get_logger(),
        "MV_CC_SetEnumValue(TriggerMode=Off) failed: %s",
        as_hex(static_cast<uint32_t>(ret)).c_str());
    }

    if (exposure_time_ > 0.0) {
      const int auto_ret = MV_CC_SetEnumValue(camera_handle_, "ExposureAuto", 0);
      if (auto_ret != MV_OK) {
        RCLCPP_WARN(
          this->get_logger(),
          "MV_CC_SetEnumValue(ExposureAuto=Off) failed: %s",
          as_hex(static_cast<uint32_t>(auto_ret)).c_str());
      }
      const int exp_ret = MV_CC_SetFloatValue(camera_handle_, "ExposureTime", static_cast<float>(exposure_time_));
      if (exp_ret != MV_OK) {
        RCLCPP_WARN(
          this->get_logger(),
          "MV_CC_SetFloatValue(ExposureTime=%.2f) failed: %s",
          exposure_time_,
          as_hex(static_cast<uint32_t>(exp_ret)).c_str());
      } else {
        RCLCPP_INFO(this->get_logger(), "ExposureTime set to %.2f us", exposure_time_);
      }
    }

    MVCC_INTVALUE payload_size_info;
    std::memset(&payload_size_info, 0, sizeof(payload_size_info));
    ret = MV_CC_GetIntValue(camera_handle_, "PayloadSize", &payload_size_info);
    if (ret != MV_OK || payload_size_info.nCurValue == 0U) {
      RCLCPP_ERROR(
        this->get_logger(),
        "MV_CC_GetIntValue(PayloadSize) failed: %s",
        as_hex(static_cast<uint32_t>(ret)).c_str());
      release_camera();
      return false;
    }

    raw_buffer_.assign(payload_size_info.nCurValue, 0);
    RCLCPP_INFO(this->get_logger(), "PayloadSize=%u bytes", payload_size_info.nCurValue);

    ret = MV_CC_StartGrabbing(camera_handle_);
    if (ret != MV_OK) {
      RCLCPP_ERROR(
        this->get_logger(),
        "MV_CC_StartGrabbing failed: %s",
        as_hex(static_cast<uint32_t>(ret)).c_str());
      release_camera();
      return false;
    }
    is_grabbing_ = true;
    first_frame_logged_ = false;
    RCLCPP_INFO(this->get_logger(), "MV_CC_StartGrabbing succeeded.");

    RCLCPP_INFO(
      this->get_logger(),
      "Hikvision camera opened and grabbing. publish output=%dx%d",
      static_cast<int>(kWidth),
      static_cast<int>(kHeight));

    return true;
  }

  bool convert_hik_frame_to_mat(
    const unsigned char *data,
    const MV_FRAME_OUT_INFO_EX &info,
    cv::Mat &out_bgr_or_gray)
  {
    const int width = static_cast<int>(info.nWidth);
    const int height = static_cast<int>(info.nHeight);

    if (width <= 0 || height <= 0 || data == nullptr) {
      RCLCPP_WARN(this->get_logger(), "Invalid frame data from camera.");
      return false;
    }

    const unsigned int pixel_type = info.enPixelType;

    switch (pixel_type) {
      case PixelType_Gvsp_Mono8: {
          cv::Mat mono(height, width, CV_8UC1, const_cast<unsigned char *>(data));
          out_bgr_or_gray = mono.clone();
          return true;
        }
      case PixelType_Gvsp_BayerRG8: {
          cv::Mat bayer(height, width, CV_8UC1, const_cast<unsigned char *>(data));
          cv::cvtColor(bayer, out_bgr_or_gray, cv::COLOR_BayerRG2BGR);
          return true;
        }
      case PixelType_Gvsp_BayerBG8: {
          cv::Mat bayer(height, width, CV_8UC1, const_cast<unsigned char *>(data));
          cv::cvtColor(bayer, out_bgr_or_gray, cv::COLOR_BayerBG2BGR);
          return true;
        }
      case PixelType_Gvsp_BayerGR8: {
          cv::Mat bayer(height, width, CV_8UC1, const_cast<unsigned char *>(data));
          cv::cvtColor(bayer, out_bgr_or_gray, cv::COLOR_BayerGR2BGR);
          return true;
        }
      case PixelType_Gvsp_BayerGB8: {
          cv::Mat bayer(height, width, CV_8UC1, const_cast<unsigned char *>(data));
          cv::cvtColor(bayer, out_bgr_or_gray, cv::COLOR_BayerGB2BGR);
          return true;
        }
      case PixelType_Gvsp_RGB8_Packed: {
          cv::Mat rgb(height, width, CV_8UC3, const_cast<unsigned char *>(data));
          cv::cvtColor(rgb, out_bgr_or_gray, cv::COLOR_RGB2BGR);
          return true;
        }
      case PixelType_Gvsp_BGR8_Packed: {
          cv::Mat bgr(height, width, CV_8UC3, const_cast<unsigned char *>(data));
          out_bgr_or_gray = bgr.clone();
          return true;
        }
      default:
        RCLCPP_WARN(
          this->get_logger(),
          "Unsupported pixel format: %s",
          as_hex(pixel_type).c_str());
        return false;
    }
  }

  bool read_processed_frame(std::vector<uint8_t> &gray_out)
  {
    if (camera_handle_ == nullptr || !is_opened_ || !is_grabbing_) {
      RCLCPP_WARN(this->get_logger(), "Camera is not ready when grabbing frame.");
      return false;
    }

    MV_FRAME_OUT_INFO_EX frame_info;
    std::memset(&frame_info, 0, sizeof(frame_info));

    const int ret = MV_CC_GetOneFrameTimeout(
      camera_handle_,
      raw_buffer_.data(),
      static_cast<unsigned int>(raw_buffer_.size()),
      &frame_info,
      grab_timeout_ms_);

    if (ret != MV_OK) {
      RCLCPP_WARN(
        this->get_logger(),
        "MV_CC_GetOneFrameTimeout failed(timeout=%d ms): %s",
        grab_timeout_ms_,
        as_hex(static_cast<uint32_t>(ret)).c_str());
      return false;
    }

    if (!first_frame_logged_) {
      RCLCPP_INFO(
        this->get_logger(),
        "First frame ok: width=%u height=%u pixel_type=%s frame_len=%u",
        frame_info.nWidth,
        frame_info.nHeight,
        as_hex(frame_info.enPixelType).c_str(),
        frame_info.nFrameLen);
      first_frame_logged_ = true;
    }

    cv::Mat frame;
    if (!convert_hik_frame_to_mat(raw_buffer_.data(), frame_info, frame) || frame.empty()) {
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

    cv::Mat roi_or_full = gray;
    const int min_side = std::min(gray.cols, gray.rows);
    int crop_side = min_side;
    if (roi_size_ > 0) {
      crop_side = std::min(roi_size_, min_side);
    }

    if (crop_side > 0 && crop_side < min_side) {
      const int x0 = (gray.cols - crop_side) / 2;
      const int y0 = (gray.rows - crop_side) / 2;
      cv::Rect center_roi(x0, y0, crop_side, crop_side);
      roi_or_full = gray(center_roi);
    }

    cv::Mat resized;
    cv::resize(
      roi_or_full,
      resized,
      cv::Size(static_cast<int>(kWidth), static_cast<int>(kHeight)),
      0.0,
      0.0,
      cv::INTER_AREA);

    if (enable_clahe_) {
      auto clahe = cv::createCLAHE(clahe_clip_limit_, cv::Size(8, 8));
      cv::Mat enhanced;
      clahe->apply(resized, enhanced);
      resized = enhanced;
    }

    if (!resized.isContinuous()) {
      resized = resized.clone();
    }

    gray_out.assign(resized.data, resized.data + static_cast<std::ptrdiff_t>(kImageBytes));
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

    RCLCPP_INFO_THROTTLE(
      this->get_logger(),
      *this->get_clock(),
      2000,
      "Publishing frame_id=%u as %u chunks (payload=%d, output=%dx%d gray8)",
      frame_id_,
      total_chunks,
      chunk_payload_size_,
      static_cast<int>(kWidth),
      static_cast<int>(kHeight));

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
  int camera_index_;
  std::string camera_serial_;
  double publish_fps_;
  int roi_size_;
  bool enable_clahe_;
  double clahe_clip_limit_;
  int chunk_payload_size_;
  int grab_timeout_ms_;
  double exposure_time_;
  uint16_t frame_id_;

  void *camera_handle_{nullptr};
  bool is_opened_{false};
  bool is_grabbing_{false};
  bool first_frame_logged_{false};
  std::vector<unsigned char> raw_buffer_;

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
