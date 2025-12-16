// Copyright 2016 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

#include  "diff_drive_control.hpp"

using std::placeholders::_1;


using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
 * member function as a callback from the timer. */

namespace diff_drive_control {

DiffDriveControl::DiffDriveControl()
  : Node("diff_drive_control"), count_(0)
{
  publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/diff_drive_base_controller/cmd_vel_unstamped", 10);
  timer_ = this->create_wall_timer(
    50ms, std::bind(&DiffDriveControl::timer_callback, this));

  subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
    "/scan", 10, std::bind(&DiffDriveControl::topic_callback, this, _1));
}

DiffDriveControl::~DiffDriveControl() = default;

void DiffDriveControl::timer_callback()
{
  geometry_msgs::msg::Twist command;

  command.linear.x = 0.1;
  command.linear.y = 0.0;
  command.linear.z = 0.0;

  command.angular.x = 0.0;
  command.angular.y = 0.0;
  command.angular.z = 0.1;

  RCLCPP_INFO(this->get_logger(), "Publishing linear.x: '%f', angular.z: '%f'", command.linear.x, command.angular.z);
  // publisher_->publish(command);
}

void DiffDriveControl::topic_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
  scan_buffer_.push(*msg);
}

} // namespace diff_drive_control

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<diff_drive_control::DiffDriveControl>());
  rclcpp::shutdown();
  return 0;
}
