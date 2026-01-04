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

#include "cv_engine/cv_engine.hpp"


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



namespace cv_engine {

  CvEngine::CvEngine() : Node("cv_engine_node")
  {



    subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
      "/camera/camera/color/image_raw", 10, std::bind(&CvEngine::image_callback, this, _1));
    
    classification_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(static_cast<int>(1000 / 30)), // 30 Hz
      std::bind(&CvEngine::timer_callback, this));

    // Attempt to load ONNX model. OpenCV may throw on parsing unsupported ONNX ops
    try {
      net_ = cv::dnn::readNetFromONNX("/home/nimrod/models/yolov5s.onnx");
      net_loaded_ = true;
      RCLCPP_INFO(this->get_logger(), "ONNX model loaded successfully");
    } catch (const cv::Exception & e) {
      net_loaded_ = false;
      RCLCPP_ERROR(this->get_logger(), "Failed to load ONNX model: %s", e.what());
      RCLCPP_ERROR(this->get_logger(), "Suggestions: try simplifying the ONNX model (onnx-simplifier), or upgrade OpenCV to a newer version with better ONNX support.");
    }

  }

  CvEngine::~CvEngine() = default;


  void CvEngine::image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    frame = cv_bridge::toCvCopy(msg, "bgr8")->image;

    // detect_and_draw(frame);
    // RCLCPP_INFO(this->get_logger(), "Processed frame");
    cv::imshow("RealSense Color Image", frame);
    cv::waitKey(1);

    // RCLCPP_INFO(this->get_logger(), "got raw data");
  }

  void CvEngine::timer_callback()
  {
    if (frame.empty()) {
      RCLCPP_WARN(this->get_logger(), "No frame available for classification");
      return;
    }

    // Preprocess the frame for classification

    // cv::Mat blob = cv::dnn::blobFromImage(frame, 1.0 / 255.0, cv::Size(640, 640), cv::Scalar(), true, false);

    // net_.setInput(blob);
    // cv::Mat outputs = net_.forward();

    // Process outputs (this is a placeholder, actual processing depends on the model)
    RCLCPP_INFO(this->get_logger(), "Frame classified");

    // Optionally, display the frame with classification results
    cv::imshow("Classified Frame", frame);
    cv::waitKey(1);
  }

  

}  // namespace cv_engine

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<cv_engine::CvEngine>());
  rclcpp::shutdown();
  return 0;
}
