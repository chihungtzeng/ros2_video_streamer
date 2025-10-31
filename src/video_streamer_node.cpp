#include "ros2_video_streamer/video_streamer_node.hpp"

#include <unistd.h>

#include <chrono>
#include <ctime>
#include <fstream>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <queue>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <string>
#include <thread>
#include <vector>

VideoStreamerNode::VideoStreamerNode(const rclcpp::NodeOptions & node_options)
: Node("video_streamer", node_options)
{
  video_path_ = declare_parameter<std::string>("video_path", "/tmp/dummy.avi");
  loop_ = declare_parameter<int>("loop", 1);
  int fps = declare_parameter<int>("fps", 20);

  RCLCPP_INFO(
    this->get_logger(), "Use video file %s, loop: %d, fps: %d", video_path_.c_str(), loop_, fps);

  rclcpp::QoS publisher_qos = rclcpp::QoS(rclcpp::KeepLast(10));
  std::string image_out = declare_parameter<std::string>("image_out", "/cam/dummy");
  publisher_ = this->create_publisher<sensor_msgs::msg::Image>(image_out, publisher_qos);
  RCLCPP_INFO(this->get_logger(), "Publish image at topic %s", publisher_->get_topic_name());

  // Pack the OpenCV image into the ROS image.
  timer_ =
    this->create_wall_timer(std::chrono::milliseconds(1000 / fps), [this] { publish_raw(); });
}

VideoStreamerNode::~VideoStreamerNode() = default;

void VideoStreamerNode::publish_raw()
{
  if (!video_capture_.isOpened()) {
    video_capture_.open(video_path_);
    if (!video_capture_.isOpened()) {
      RCLCPP_WARN(this->get_logger(), "Unable to open %s", video_path_.c_str());
      return;
    }
  }

  cv::Mat frame;
  video_capture_.read(frame);
  if (frame.empty() && (loop_ != 0)) {
    video_capture_.release();
    video_capture_.open(video_path_);
    video_capture_.read(frame);
  }
  if (frame.empty()) {
    RCLCPP_WARN(this->get_logger(), "Empty frame");
    return;
  }

  RCLCPP_INFO_ONCE(
    this->get_logger(), "published image size: %dx%d, total %ld KB", frame.cols, frame.rows,
    (frame.total() * 3) / 1024);

  sensor_msgs::msg::Image::UniquePtr raw_msg{new sensor_msgs::msg::Image};
  RCLCPP_DEBUG(this->get_logger(), "image ptr: %p", static_cast<void *>(raw_msg.get()));
  raw_msg->height = frame.rows;
  raw_msg->width = frame.cols;
  raw_msg->encoding = "bgr8";
  raw_msg->is_bigendian = 0u;
  raw_msg->step = static_cast<sensor_msgs::msg::Image::_step_type>(frame.step);
  raw_msg->data.assign(frame.datastart, frame.dataend);
  raw_msg->header.stamp = this->get_clock()->now();
  publisher_->publish(std::move(raw_msg));
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  rclcpp::spin(std::make_shared<VideoStreamerNode>(options));
  rclcpp::shutdown();
  return 0;
}

