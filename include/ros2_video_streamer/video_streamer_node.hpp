#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <string>

class VideoStreamerNode : public rclcpp::Node
{
public:
  explicit VideoStreamerNode(const rclcpp::NodeOptions & node_options);
  ~VideoStreamerNode();

private:
  void publish_raw();
  void prepare_images();

  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  int loop_ = 0;
  std::string video_path_;
  cv::VideoCapture video_capture_;
};  // class VideoStreamerNode
