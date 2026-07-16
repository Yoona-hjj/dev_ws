#include <rclcpp/rclcpp.hpp>
#include "hbm_img_msgs/msg/hbm_msg1080_p.hpp"
#include <opencv2/opencv.hpp>
#include <zbar.h>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/int32.hpp>
#include "origincar_msg/msg/sign.hpp"

class Qrcode : public rclcpp::Node
{
public:
  Qrcode() : Node("qrcode"), prev_qr_data(""), number_i_(0)
  {
    rclcpp::QoS qos(1);
    qos.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);

    subscriber_hbmem_ = this->create_subscription<hbm_img_msgs::msg::HbmMsg1080P>(
        "/nv12_img", qos, std::bind(&Qrcode::subscription_callback, this, std::placeholders::_1));

    publisher_ = this->create_publisher<origincar_msg::msg::Sign>("/sign_switch", 10);

    qrcode_number_publisher_ = this->create_publisher<std_msgs::msg::String>("/zbar_number", 10);

    qrcode_bottom_publisher_ = this->create_publisher<std_msgs::msg::Int32>("/qrcode_bottom", 10);
  }

private:
  void subscription_callback(const hbm_img_msgs::msg::HbmMsg1080P::SharedPtr msg)
  {
    if (!msg)
    {
      return;
    }

    number_i_ += 1;
    number_i_ = number_i_ % 2;
    if (number_i_ != 0)
    {
      return;
    }

    int height = msg->height;
    int width = msg->width;
    size_t step = msg->step;

    cv::Mat y_plane(height, width, CV_8UC1, msg->data.data(), step);
    cv::Mat gray = y_plane;
    zbar::ImageScanner scanner;
    scanner.set_config(zbar::ZBAR_NONE, zbar::ZBAR_CFG_ENABLE, 1);
    zbar::Image zbar_image(width, height, "Y800", gray.data, width * height);

    int result = scanner.scan(zbar_image);
    if (result > 0)
    {
      for (zbar::Image::SymbolIterator symbol = zbar_image.symbol_begin(); symbol != zbar_image.symbol_end(); ++symbol)
      {
        // 发布二维码边界框 bottom 坐标，供 racing_control 减速使用
        int bottom_y = 0;
        for (int i = 0; i < static_cast<int>(symbol->get_location_size()); ++i)
        {
          int y = symbol->get_location_y(i);
          if (y > bottom_y) bottom_y = y;
        }
        std_msgs::msg::Int32 bottom_msg;
        bottom_msg.data = bottom_y;
        qrcode_bottom_publisher_->publish(bottom_msg);
        std::string qr_data = symbol->get_data();
        if (qr_data == prev_qr_data)
        {
          std_msgs::msg::String qrcode_number_msg;
          origincar_msg::msg::Sign sign_msg;
          if (qr_data == "ClockWise") // 顺时针
          {
            sign_msg.sign_data = 3;
            qrcode_number_msg.data = qr_data;
          }
          else if (qr_data == "AntiClockWise") // 逆时针
          {
            sign_msg.sign_data = 4;
            qrcode_number_msg.data = qr_data;
          }
          else
          {
            try
            {
              int number = std::stoi(qr_data);
              if (number >= 1 && number <= 9999)
              {
                sign_msg.sign_data = (number % 2 == 0) ? 4 : 3;
                qrcode_number_msg.data = qr_data;
              }
              else
              {
                RCLCPP_WARN(this->get_logger(), "Recognized number out of range (1-9999): %d", number);
                continue;
              }
            }
            catch (const std::invalid_argument &e)
            {
              RCLCPP_WARN(this->get_logger(), "Unrecognized content: %s", qr_data.c_str());
              continue;
            }
          }
          publisher_->publish(sign_msg);
          qrcode_number_publisher_->publish(qrcode_number_msg);
        }
        prev_qr_data = qr_data;
      }
    }
    else
    {
      prev_qr_data = "";
    }
  }

  rclcpp::Subscription<hbm_img_msgs::msg::HbmMsg1080P>::SharedPtr subscriber_hbmem_;
  rclcpp::Publisher<origincar_msg::msg::Sign>::SharedPtr publisher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr qrcode_number_publisher_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr qrcode_bottom_publisher_;
  std::string prev_qr_data;
  int number_i_ = 0;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Qrcode>());
  rclcpp::shutdown();
  return 0;
}