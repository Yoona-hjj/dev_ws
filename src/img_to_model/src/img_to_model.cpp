#include <rclcpp/rclcpp.hpp>
#include "hbm_img_msgs/msg/hbm_msg1080_p.hpp"
#include <opencv2/opencv.hpp>
#include <std_msgs/msg/int32.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <mutex>

class Image_large_model : public rclcpp::Node
{
public:
  Image_large_model() : Node("image_large_model"), send_image_flag_(0)
  {
    rclcpp::QoS qos(1);
    qos.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);

    subscriber_hbmem_ = this->create_subscription<hbm_img_msgs::msg::HbmMsg1080P>(
        "/nv12_img", qos, std::bind(&Image_large_model::subscription_callback, this, std::placeholders::_1));

    get_picture_sub_ = this->create_subscription<std_msgs::msg::Int32>(
        "/get_picture", 10, std::bind(&Image_large_model::get_picture_callback, this, std::placeholders::_1));

    image_pub_ = this->create_publisher<sensor_msgs::msg::CompressedImage>("/model_image", 10);
  }

private:
  void subscription_callback(const hbm_img_msgs::msg::HbmMsg1080P::SharedPtr msg)
  {
    if (!msg)
      return;

    bool should_process = false;
    {
      std::lock_guard<std::mutex> lock(msg_mutex_);
      if (send_image_flag_ == 1)
      {
        should_process = true;
        send_image_flag_ = 0;
      }
    }

    if (should_process)
    {
      cv::Mat nv12_mat(msg->height * 3 / 2, msg->width, CV_8UC1, msg->data.data());
      cv::Mat bgr_mat;
      cv::cvtColor(nv12_mat, bgr_mat, cv::COLOR_YUV2BGR_NV12);

      std::vector<uchar> jpeg_buffer;
      std::vector<int> params = {cv::IMWRITE_JPEG_QUALITY, 50};
      if (!cv::imencode(".jpg", bgr_mat, jpeg_buffer, params))
      {
        RCLCPP_ERROR(this->get_logger(), "JPEG encoding failed");
        return;
      }

      auto compressed_msg = sensor_msgs::msg::CompressedImage();
      compressed_msg.header.stamp = this->get_clock()->now();
      compressed_msg.header.frame_id = "camera";
      compressed_msg.format = "jpeg";
      compressed_msg.data = std::vector<uint8_t>(jpeg_buffer.begin(), jpeg_buffer.end());

      image_pub_->publish(compressed_msg);
    }
  }

  void get_picture_callback(const std_msgs::msg::Int32::SharedPtr msg)
  {
    if (!msg)
      return;
    std::lock_guard<std::mutex> lock(msg_mutex_);
    send_image_flag_ = 1;
  }

  rclcpp::Subscription<hbm_img_msgs::msg::HbmMsg1080P>::SharedPtr subscriber_hbmem_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr get_picture_sub_;
  rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr image_pub_;

  std::mutex msg_mutex_;
  int send_image_flag_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Image_large_model>());
  rclcpp::shutdown();
  return 0;
}