#ifndef DIFF_DRIVE_CONTROL_HPP
#define DIFF_DRIVE_CONTROL_HPP

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include "ring_beffer.hpp"

namespace diff_drive_control
{

class DiffDriveControl : public rclcpp::Node
{
public:
    DiffDriveControl();
    ~DiffDriveControl();

private:
    void timer_callback();
    void topic_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg);

    RingBuffer<sensor_msgs::msg::LaserScan, 10> scan_buffer_;

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
    int count_;



};

}  // namespace diff_drive_control

#endif  // DIFF_DRIVE_CONTROL_HPP