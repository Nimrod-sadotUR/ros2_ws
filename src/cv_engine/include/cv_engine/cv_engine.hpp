#ifndef CV_ENGINE_CV_ENGINE_HPP
#define CV_ENGINE_CV_ENGINE_HPP

#include <string>
#include <vector>
#include <memory>
#include <opencv2/opencv.hpp>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/imu.hpp"


namespace cv_engine {

struct RealsenseData {
  sensor_msgs::msg::Image::SharedPtr image_raw;
  sensor_msgs::msg::Image::SharedPtr image_rect_raw;
  sensor_msgs::msg::PointCloud2::SharedPtr points;
  sensor_msgs::msg::Imu::SharedPtr imu;
  cv::Mat frame;
};

class CvEngine : public rclcpp::Node
{
public:
    CvEngine();
    ~CvEngine();
 



    // Image processing methods
    // cv::Mat processImage(const cv::Mat& input);
    // cv::Mat detectFeatures(const cv::Mat& input);
    // std::vector<cv::Mat> splitChannels(const cv::Mat& input);

    // Configuration methods
    // void setParameter(const std::string& key, const std::string& value);
    // std::string getParameter(const std::string& key) const;

    // Utility methods
    // bool isInitialized() const;
    // void reset();

private:
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
  rclcpp::TimerBase::SharedPtr classification_timer_;
  std::vector<std::string> class_names_;
  cv::dnn::Net net_;
  bool net_loaded_{false};
  cv::Mat frame;

  RealsenseData realsense_data_;
  void image_callback(const sensor_msgs::msg::Image::SharedPtr msg);
  void timer_callback();



};

}  // namespace cv_engine

#endif  // CV_ENGINE_CV_ENGINE_HPP