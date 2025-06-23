#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/camera_info.hpp"

class CameraInfoPublisher : public rclcpp::Node {
public:
  CameraInfoPublisher() : Node("camera_info_pub") {
    pub_ = create_publisher<sensor_msgs::msg::CameraInfo>("/kinect2/qhd/camera_info", 10);
    timer_ = create_wall_timer(
        std::chrono::milliseconds(100), std::bind(&CameraInfoPublisher::publish_info, this));
  }

private:
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  void publish_info() {
    auto msg = sensor_msgs::msg::CameraInfo();
    msg.header.stamp = now();
    msg.header.frame_id = "kinect2_color_optical_frame";
    msg.width = 512;
    msg.height = 424;
    msg.k = {365.456, 0.0, 254.878, 0.0, 365.456, 205.395, 0.0, 0.0, 1.0}; // fx, fy, cx, cy
    msg.p = {365.456, 0.0, 254.878, 0.0, 0.0, 365.456, 205.395, 0.0, 0.0, 0.0, 1.0, 0.0};
    pub_->publish(msg);
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CameraInfoPublisher>());
  rclcpp::shutdown();
  return 0;
}
