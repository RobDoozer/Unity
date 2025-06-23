#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/point_field.hpp"
#include "std_msgs/msg/header.hpp"
#include "builtin_interfaces/msg/time.hpp"

#include <thread>
#include <vector>
#include <chrono>
#include <cstring>
#include <arpa/inet.h>
#include <unistd.h>
#include <mutex>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>

using namespace std::chrono_literals;

class KinectReceiver : public rclcpp::Node {
public:
    KinectReceiver() : Node("kinect_receiver2") {
        this->declare_parameter<std::string>("kinect_id", "kinect3");
        this->declare_parameter<int>("depth_port", 5011);
        this->declare_parameter<int>("rgb_port", 5012);

        this->get_parameter("kinect_id", kinect_id_);
        this->get_parameter("depth_port", depth_port_);
        this->get_parameter("rgb_port", rgb_port_);

        depth_topic_ = "/" + kinect_id_ + "/depth";
        rgb_topic_   = "/" + kinect_id_ + "/rgb";
        points_topic_ = "/" + kinect_id_ + "/points";

        depth_pub_ = create_publisher<sensor_msgs::msg::Image>(depth_topic_, 10);
        rgb_pub_   = create_publisher<sensor_msgs::msg::Image>(rgb_topic_, 10);
        pc_pub_    = create_publisher<sensor_msgs::msg::PointCloud2>(points_topic_, 10);

        depth_data_.resize(width_ * height_);
        rgb_data_.resize(width_ * height_ * 3);

        std::thread(&KinectReceiver::depth_server, this).detach();
        std::thread(&KinectReceiver::rgb_server, this).detach();
    }

private:
    std::string kinect_id_;
    int depth_port_, rgb_port_;
    std::string depth_topic_, rgb_topic_, points_topic_;

    int width_ = 512;
    int height_ = 424;
    float fx_ = 365.456, fy_ = 365.456, cx_ = 254.878, cy_ = 205.395;

    std::vector<float> depth_data_;
    std::vector<uint8_t> rgb_data_;

    std::mutex data_mutex_;
    bool has_depth_ = false, has_rgb_ = false;

    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr depth_pub_, rgb_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pc_pub_;

    void depth_server() {
        int server_fd = socket(AF_INET, SOCK_STREAM, 0);
        sockaddr_in addr{};
        addr.sin_family = AF_INET;
        addr.sin_addr.s_addr = INADDR_ANY;
        addr.sin_port = htons(depth_port_);
        bind(server_fd, (struct sockaddr *)&addr, sizeof(addr));
        listen(server_fd, 1);
        RCLCPP_INFO(this->get_logger(), "Depth Server läuft auf Port %d", depth_port_);

        while (rclcpp::ok()) {
            int client_fd = accept(server_fd, nullptr, nullptr);
            std::vector<uint8_t> buffer(width_ * height_ * 4);
            recv_all(client_fd, buffer.data(), buffer.size());
            close(client_fd);

            std::lock_guard<std::mutex> lock(data_mutex_);
            std::memcpy(depth_data_.data(), buffer.data(), buffer.size());
            has_depth_ = true;

            publish_depth();
            try_publish_pointcloud();
        }
    }

    void rgb_server() {
        int server_fd = socket(AF_INET, SOCK_STREAM, 0);
        sockaddr_in addr{};
        addr.sin_family = AF_INET;
        addr.sin_addr.s_addr = INADDR_ANY;
        addr.sin_port = htons(rgb_port_);
        bind(server_fd, (struct sockaddr *)&addr, sizeof(addr));
        listen(server_fd, 1);
        RCLCPP_INFO(this->get_logger(), "RGB Server läuft auf Port %d", rgb_port_);

        while (rclcpp::ok()) {
            int client_fd = accept(server_fd, nullptr, nullptr);
            recv_all(client_fd, rgb_data_.data(), rgb_data_.size());
            close(client_fd);

            std::lock_guard<std::mutex> lock(data_mutex_);
            has_rgb_ = true;

            publish_rgb();
            try_publish_pointcloud();
        }
    }

    void recv_all(int socket, void *buffer, size_t length) {
        uint8_t *ptr = static_cast<uint8_t*>(buffer);
        size_t total = 0;
        while (total < length) {
            ssize_t n = recv(socket, ptr + total, length - total, 0);
            if (n <= 0) break;
            total += n;
        }
    }

    void publish_depth() {
        auto msg = sensor_msgs::msg::Image();
        msg.header.stamp = now();
        msg.header.frame_id = "camera_link";
        msg.height = height_;
        msg.width = width_;
        msg.encoding = "32FC1";
        msg.step = width_ * 4;
        msg.data.resize(width_ * height_ * 4);
        std::memcpy(msg.data.data(), depth_data_.data(), msg.data.size());
        depth_pub_->publish(msg);
    }

    void publish_rgb() {
        auto msg = sensor_msgs::msg::Image();
        msg.header.stamp = now();
        msg.header.frame_id = "camera_link";
        msg.height = height_;
        msg.width = width_;
        msg.encoding = "rgb8";
        msg.step = width_ * 3;
        msg.data = rgb_data_;
        rgb_pub_->publish(msg);
    }

    void try_publish_pointcloud() {
        if (!has_depth_ || !has_rgb_) return;

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
        cloud->header.frame_id = "camera_link";
        cloud->is_dense = false;
        cloud->points.reserve(width_ * height_);

        for (int v = 0; v < height_; ++v) {
            for (int u = 0; u < width_; ++u) {
                float Z = depth_data_[v * width_ + u];
                if (Z == 0.0f || !std::isfinite(Z)) continue;

                float X = (u - cx_) * Z / fx_;
                float Y = (v - cy_) * Z / fy_;

                int idx = (v * width_ + u) * 3;
                uint8_t r = rgb_data_[idx + 0];
                uint8_t g = rgb_data_[idx + 1];
                uint8_t b = rgb_data_[idx + 2];

                pcl::PointXYZRGB point;
                point.x = X;
                point.y = Y;
                point.z = Z;
                point.r = r;
                point.g = g;
                point.b = b;

                cloud->points.push_back(point);
            }
        }

        cloud->width = static_cast<uint32_t>(cloud->points.size());
        cloud->height = 1;

        sensor_msgs::msg::PointCloud2 output;
        pcl::toROSMsg(*cloud, output);
        output.header.stamp = now();
        output.header.frame_id = "camera_link";
        pc_pub_->publish(output);
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<KinectReceiver>());
    rclcpp::shutdown();
    return 0;
}

