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

#include <functional>
#include <memory>

// #include "cv_engine/cv_engine.hpp"


#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/image.hpp"
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/objdetect.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/dnn.hpp>



using std::placeholders::_1;



class cv_engine : public rclcpp::Node
{
public:
  cv_engine()
  : Node("cv_engine")
  {
    subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
      "/camera/camera/color/image_raw", 10, std::bind(&cv_engine::image_callback, this, _1));

      net_ = cv::dnn::readNetFromONNX("/home/nimrod/models/yolov5s.onnx");

  }

  private:
  void image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    frame = cv_bridge::toCvCopy(msg, "bgr8")->image;

    // detect_and_draw(frame);
    // RCLCPP_INFO(this->get_logger(), "Processed frame");
    cv::imshow("RealSense Color Image", frame);
    cv::waitKey(1);

    // RCLCPP_INFO(this->get_logger(), "got raw data");
  }


  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
  std::vector<std::string> class_names_;
  cv::dnn::Net net_;
  cv::Mat frame;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<cv_engine>());
  rclcpp::shutdown();
  return 0;
}
