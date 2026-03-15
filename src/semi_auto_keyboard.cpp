// src/semi_auto_keyboard.cpp
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/u_int32.hpp>

#include <unordered_map>
#include <chrono>

// Linux 终端读取按键（无回显、无需回车）
#include <termios.h>
#include <unistd.h>
#include <sys/select.h>

class TerminalRawMode {
public:
  TerminalRawMode() {
    tcgetattr(STDIN_FILENO, &old_);
    termios raw = old_;
    raw.c_lflag &= ~(ICANON | ECHO);  // 关闭规范模式和回显
    tcsetattr(STDIN_FILENO, TCSANOW, &raw);
  }
  ~TerminalRawMode() {
    tcsetattr(STDIN_FILENO, TCSANOW, &old_);
  }
private:
  termios old_{};
};

static bool read_key_nonblocking(char &out)
{
  fd_set set;
  FD_ZERO(&set);
  FD_SET(STDIN_FILENO, &set);

  timeval tv;
  tv.tv_sec = 0;
  tv.tv_usec = 0;

  int rv = select(STDIN_FILENO + 1, &set, nullptr, nullptr, &tv);
  if (rv > 0 && FD_ISSET(STDIN_FILENO, &set)) {
    char c = 0;
    if (read(STDIN_FILENO, &c, 1) == 1) {
      out = c;
      return true;
    }
  }
  return false;
}

class SemiAutoKeyboardNode : public rclcpp::Node {
public:
  SemiAutoKeyboardNode()
  : Node("semi_auto_keyboard")
  {
    pub_ = this->create_publisher<std_msgs::msg::UInt32>("/semi_auto/command_id", 10);

    // 1Hz 限频：默认 1000ms，可用参数调
    this->declare_parameter<int>("cooldown_ms", 1000);
    cooldown_ = std::chrono::milliseconds(this->get_parameter("cooldown_ms").as_int());

    // 键盘映射：qasdfghjkl -> 1..10
    key_map_ = {
      {'q', 1}, {'a', 2}, {'s', 3}, {'d', 4}, {'f', 5},
      {'g', 6}, {'h', 7}, {'j', 8}, {'k', 9}, {'l', 10}
    };

    last_send_ = std::chrono::steady_clock::now() - std::chrono::seconds(10);

    RCLCPP_INFO(this->get_logger(),
      "SemiAutoKeyboard ready.\n"
      "Press: q a s d f g h j k l -> command_id 1..10\n"
      "Press: ESC to exit\n"
      "Topic: /semi_auto/command_id");

    // 50Hz 轮询读取键盘
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(20),
      std::bind(&SemiAutoKeyboardNode::on_timer, this)
    );
  }

private:
  void on_timer()
  {
    char c = 0;
    if (!read_key_nonblocking(c)) return;

    if (c == 27) { // ESC
      RCLCPP_INFO(this->get_logger(), "ESC pressed, shutting down.");
      rclcpp::shutdown();
      return;
    }

    // 大写转小写
    if (c >= 'A' && c <= 'Z') c = static_cast<char>(c - 'A' + 'a');

    auto it = key_map_.find(c);
    if (it == key_map_.end()) return;

    auto now = std::chrono::steady_clock::now();
    if (now - last_send_ < cooldown_) {
      RCLCPP_WARN(this->get_logger(),
        "Rate limited (<=1Hz). Ignored key '%c'.", c);
      return;
    }

    last_send_ = now;

    std_msgs::msg::UInt32 msg;
    msg.data = it->second;
    pub_->publish(msg);

    RCLCPP_INFO(this->get_logger(),
      "Published command_id=%u (key=%c)", msg.data, c);
  }

private:
  rclcpp::Publisher<std_msgs::msg::UInt32>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::unordered_map<char, uint32_t> key_map_;
  std::chrono::milliseconds cooldown_;
  std::chrono::steady_clock::time_point last_send_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  // 进入 raw mode：保证按键无需回车；退出时自动恢复终端
  TerminalRawMode raw_mode;

  auto node = std::make_shared<SemiAutoKeyboardNode>();
  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}
