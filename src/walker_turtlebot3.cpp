/**
 * @file walker_turtlebot3.cpp
 * @author Mudit Singal (muditsingal@gmail.com)
 * @brief Cpp script for
 * @version 0.1
 * @date 2023-11-30
 *
 * @copyright Copyright (c) 2023
 *
 */
#include <chrono>
#include <fstream>
#include <functional>
#include <memory>
#include <string>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

using namespace std::chrono_literals;


class WalkerTurtebot3 : public rclcpp::Node {
 public:
  WalkerTurtebot3() : Node("walker_turtlebot3") {
    subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "scan", 10,
        std::bind(&WalkerTurtebot3::main_ctrl_cb, this, std::placeholders::_1));
    vel_publisher_ =
              this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
  }

 private:
  void main_ctrl_cb(const sensor_msgs::msg::LaserScan& lidar_msg) {
    auto lidar_scan = lidar_msg.ranges;

    for (int ray_angle = 360 - sweep_span; ray_angle < 360; ray_angle++) {
      if (lidar_scan[ray_angle] < threshold) {
        turn();
      } else {
        straight();
      }
    }
    for (int ray_angle = 0; ray_angle < sweep_span; ray_angle++) {
      if (lidar_scan[ray_angle] < threshold) {
        turn();
      } else {
        straight();
      }
    }
  }
  void turn() {
    auto tbot_vel = geometry_msgs::msg::Twist();
    tbot_vel.linear.x = 0.0;
    tbot_vel.angular.z = turn_vel;
    vel_publisher_->publish(tbot_vel);
    return;
  }

  void straight() {
    auto tbot_vel = geometry_msgs::msg::Twist();
    tbot_vel.linear.x = fwd_vel;
    tbot_vel.angular.z = 0.0;
    vel_publisher_->publish(tbot_vel);
    return;
  }
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_publisher_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
  rclcpp::TimerBase::SharedPtr timer;
  // sensor_msgs::msg::LaserScan subscription_;
  size_t count_;
  int sweep_span = 20;
  float threshold = 0.6;
  float fwd_vel = 0.15;
  float turn_vel = -0.1;
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<WalkerTurtebot3>());
  rclcpp::shutdown();
  return 0;
}
