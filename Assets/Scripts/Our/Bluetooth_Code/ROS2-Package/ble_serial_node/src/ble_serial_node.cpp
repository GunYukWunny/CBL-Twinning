#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>
#include <fstream>
#include <sstream>
#include <string>

using namespace std::chrono_literals;

class BLESerialNode : public rclcpp::Node {
public:
  BLESerialNode()
  : Node("ble_serial_node")
  {
    publisher_ = this->create_publisher<std_msgs::msg::Float32>("bluetooth_distance", 10);
    timer_ = this->create_wall_timer(500ms, std::bind(&BLESerialNode::read_serial, this));
    serial_.open("/dev/ttyUSB0", std::ios::in);

    if (!serial_.is_open()) {
      RCLCPP_ERROR(this->get_logger(), "Failed to open /dev/ttyUSB0");
      rclcpp::shutdown();
    } else {
      RCLCPP_INFO(this->get_logger(), "Serial port /dev/ttyUSB0 opened.");
      // Optional: set unbuffered mode
      serial_.rdbuf()->pubsetbuf(0, 0);
    }
  }

private:
  void read_serial() {
    std::string line;
    if (std::getline(serial_, line)) {
      RCLCPP_INFO(this->get_logger(), "Read line: '%s'", line.c_str());
      std::istringstream iss(line);
      std::string token;
      while (iss >> token) {
        try {
          float distance = std::stof(token);
          auto msg = std_msgs::msg::Float32();
          msg.data = distance;
          publisher_->publish(msg);
          RCLCPP_INFO(this->get_logger(), "Published: %.2f m", distance);
        } catch (...) {
          RCLCPP_WARN(this->get_logger(), "Invalid float token: '%s'", token.c_str());
        }
      }
    } else {
      RCLCPP_WARN(this->get_logger(), "No data read from serial");
    }
  }

  std::ifstream serial_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<BLESerialNode>());
  rclcpp::shutdown();
  return 0;
}
