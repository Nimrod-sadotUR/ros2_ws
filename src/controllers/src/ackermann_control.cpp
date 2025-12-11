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
#include <geometry_msgs/msg/twist_stamped.hpp>


using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
 * member function as a callback from the timer. */

class AckermannControl : public rclcpp::Node
{
public:
  AckermannControl()
  : Node("ackermann_control"), count_(0)
  {
    this->set_parameter(rclcpp::Parameter("use_sim_time", true));
    publisher_ = this->create_publisher<geometry_msgs::msg::TwistStamped>("/ackermann_steering_controller/reference", 10);
    timer_ = this->create_wall_timer(
      50ms, std::bind(&AckermannControl::timer_callback, this));
  }

private:
  void timer_callback()
  {
    geometry_msgs::msg::Twist tw;

    tw.linear.x = 0.5;
    tw.linear.y = 0.0;
    tw.linear.z = 0.0;

    tw.angular.x = 0.0;
    tw.angular.y = 0.0;
    tw.angular.z = 0.3;

    geometry_msgs::msg::TwistStamped command;
    command.twist = tw;
    command.header.stamp = this->now();

    RCLCPP_INFO(this->get_logger(), "Publishing linear.x: '%f', angular.z: '%f'", tw.linear.x, tw.angular.z);
    publisher_->publish(command);
  }
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr publisher_;
  size_t count_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AckermannControl>());
  rclcpp::shutdown();
  return 0;
}
