#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <functional>
#include <mutex>
#include <vector>

#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>

#include "ai_msgs/msg/perception_targets.hpp"
#include "hbm_img_msgs/msg/hbm_msg1080_p.hpp"
#include <sensor_msgs/msg/compressed_image.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/int32.hpp>

namespace
{
constexpr double kMinPersonConfidence = 0.6;
constexpr double kRoiScale = 1.5;
constexpr int kFullOutputWidth = 640;
constexpr int kFullOutputHeight = 360;
constexpr int kRoiTargetHeight = 640;
constexpr int kExpectedWidth = 1280;
constexpr int kExpectedHeight = 720;

struct PersonRoiRect
{
  int x_offset = 0;
  int y_offset = 0;
  int width = 0;
  int height = 0;
};
}  // namespace

class PersonImageCropper : public rclcpp::Node
{
public:
  PersonImageCropper() : Node("person_image_cropper")
  {
    rclcpp::QoS qos(1);
    qos.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);

    image_sub_ = create_subscription<hbm_img_msgs::msg::HbmMsg1080P>(
      "/nv12_img", qos,
      std::bind(&PersonImageCropper::image_callback, this, std::placeholders::_1));

    target_sub_ = create_subscription<ai_msgs::msg::PerceptionTargets>(
      "/racing_obstacle_detection", qos,
      std::bind(&PersonImageCropper::target_callback, this, std::placeholders::_1));

    get_picture_sub_ = create_subscription<std_msgs::msg::Int32>(
      "/get_picture", 10,
      std::bind(&PersonImageCropper::get_picture_callback, this, std::placeholders::_1));

    chassis_start_button_sub_ = create_subscription<std_msgs::msg::Bool>(
      "/chassis_start_button", 10,
      std::bind(&PersonImageCropper::chassis_start_button_callback, this, std::placeholders::_1));

    image_pub_ = create_publisher<sensor_msgs::msg::CompressedImage>("/model_image", 10);

    RCLCPP_INFO(get_logger(), "person_image_cropper started");
  }

private:
  void target_callback(const ai_msgs::msg::PerceptionTargets::SharedPtr msg)
  {
    PersonRoiRect best_rect;
    const bool has_person = select_best_person_rect(msg, best_rect);

    std::lock_guard<std::mutex> lock(target_mutex_);
    has_person_ = has_person;
    latest_person_rect_ = best_rect;
  }

  bool select_best_person_rect(
    const ai_msgs::msg::PerceptionTargets::SharedPtr & msg,
    PersonRoiRect & best_rect) const
  {
    if (!msg) {
      return false;
    }

    bool has_best = false;
    uint64_t max_area = 0;

    for (const auto & target : msg->targets) {
      if (target.type != "person") {
        continue;
      }

      for (const auto & roi : target.rois) {
        if (roi.confidence <= kMinPersonConfidence) {
          continue;
        }

        int x_min = static_cast<int>(roi.rect.x_offset);
        int y_min = static_cast<int>(roi.rect.y_offset);
        const int box_width = static_cast<int>(roi.rect.width);
        const int box_height = static_cast<int>(roi.rect.height);

        if (box_width <= 0 || box_height <= 0) {
          continue;
        }

        int x_max = x_min + box_width - 1;
        int y_max = y_min + box_height - 1;

        if (x_max < 0 || y_max < 0 ||
            x_min > (kExpectedWidth - 1) ||
            y_min > (kExpectedHeight - 1)) {
          continue;
        }

        x_min = std::max(0, x_min);
        y_min = std::max(0, y_min);
        x_max = std::min(kExpectedWidth - 1, x_max);
        y_max = std::min(kExpectedHeight - 1, y_max);

        const uint64_t area =
          static_cast<uint64_t>(x_max - x_min + 1) *
          static_cast<uint64_t>(y_max - y_min + 1);

        if (!has_best || area > max_area) {
          has_best = true;
          max_area = area;
          best_rect.x_offset = x_min;
          best_rect.y_offset = y_min;
          best_rect.width = x_max - x_min + 1;
          best_rect.height = y_max - y_min + 1;
        }
      }
    }

    return has_best;
  }

  void image_callback(const hbm_img_msgs::msg::HbmMsg1080P::SharedPtr msg)
  {
    {
      std::lock_guard<std::mutex> lock(flag_mutex_);
      if (send_image_flag_ != 1) {
        return;
      }
      send_image_flag_ = 0;
    }

    if (!msg || msg->width <= 0 || msg->height <= 0 || msg->step == 0 || msg->data.empty()) {
      RCLCPP_WARN(get_logger(), "invalid nv12 image message");
      return;
    }

    cv::Mat bgr_image;
    if (!convert_nv12_to_bgr(*msg, bgr_image)) {
      return;
    }

    PersonRoiRect person_rect;
    bool has_person = false;
    {
      std::lock_guard<std::mutex> lock(target_mutex_);
      has_person = has_person_;
      person_rect = latest_person_rect_;
    }

    cv::Mat output_image;
    bool used_roi = false;

    PersonRoiRect expanded_rect;
    if (has_person && expand_and_clip_roi(person_rect, bgr_image.cols, bgr_image.rows, expanded_rect)) {
      const cv::Rect crop_rect(
        expanded_rect.x_offset,
        expanded_rect.y_offset,
        expanded_rect.width,
        expanded_rect.height);
      cv::Mat roi_image = bgr_image(crop_rect);
      resize_roi(roi_image, output_image);
      used_roi = true;
    } else {
      cv::resize(bgr_image, output_image, cv::Size(kFullOutputWidth, kFullOutputHeight), 0, 0, cv::INTER_AREA);
    }

    std::vector<uint8_t> jpeg_buffer;
    if (!cv::imencode(".jpg", output_image, jpeg_buffer)) {
      RCLCPP_WARN(get_logger(), "failed to encode model image");
      return;
    }

    sensor_msgs::msg::CompressedImage compressed_msg;
    compressed_msg.header = msg->header;
    compressed_msg.format = "jpeg";
    compressed_msg.data = std::move(jpeg_buffer);
    image_pub_->publish(compressed_msg);

    RCLCPP_INFO(
      get_logger(),
      "published /model_image: mode=%s, width=%d, height=%d",
      used_roi ? "person_roi" : "full_image",
      output_image.cols,
      output_image.rows);
  }

  bool convert_nv12_to_bgr(const hbm_img_msgs::msg::HbmMsg1080P & msg, cv::Mat & bgr_image)
  {
    const int width = static_cast<int>(msg.width);
    const int height = static_cast<int>(msg.height);
    const size_t step = static_cast<size_t>(msg.step);
    const int nv12_rows = height + height / 2;

    if ((width % 2) != 0 || (height % 2) != 0) {
      RCLCPP_WARN(get_logger(), "NV12 image dimensions must be even");
      return false;
    }

    if (step < static_cast<size_t>(width)) {
      RCLCPP_WARN(get_logger(), "NV12 image step is smaller than width");
      return false;
    }

    const size_t min_required_size =
      static_cast<size_t>(nv12_rows - 1) * step + static_cast<size_t>(width);
    if (msg.data.size() < min_required_size) {
      RCLCPP_WARN(get_logger(), "NV12 image data is too small");
      return false;
    }

    try {
      if (step == static_cast<size_t>(width)) {
        cv::Mat nv12_mat(nv12_rows, width, CV_8UC1, const_cast<uint8_t *>(msg.data.data()));
        cv::cvtColor(nv12_mat, bgr_image, cv::COLOR_YUV2BGR_NV12);
      } else {
        nv12_buffer_.resize(static_cast<size_t>(nv12_rows) * static_cast<size_t>(width));
        for (int row = 0; row < nv12_rows; ++row) {
          std::memcpy(
            nv12_buffer_.data() + static_cast<size_t>(row) * static_cast<size_t>(width),
            msg.data.data() + static_cast<size_t>(row) * step,
            static_cast<size_t>(width));
        }
        cv::Mat nv12_mat(nv12_rows, width, CV_8UC1, nv12_buffer_.data());
        cv::cvtColor(nv12_mat, bgr_image, cv::COLOR_YUV2BGR_NV12);
      }
    } catch (const cv::Exception & error) {
      RCLCPP_ERROR(get_logger(), "OpenCV NV12 conversion failed: %s", error.what());
      return false;
    }

    return true;
  }

  bool expand_and_clip_roi(
    const PersonRoiRect & rect,
    int image_width,
    int image_height,
    PersonRoiRect & out_rect) const
  {
    if (rect.width <= 0 || rect.height <= 0 || image_width <= 0 || image_height <= 0) {
      return false;
    }

    const double center_x = rect.x_offset + rect.width / 2.0;
    const double center_y = rect.y_offset + rect.height / 2.0;
    const double scaled_width = rect.width * kRoiScale;
    const double scaled_height = rect.height * kRoiScale;

    int x_min = static_cast<int>(std::round(center_x - scaled_width / 2.0));
    int y_min = static_cast<int>(std::round(center_y - scaled_height / 2.0));
    int x_max = static_cast<int>(std::round(center_x + scaled_width / 2.0)) - 1;
    int y_max = static_cast<int>(std::round(center_y + scaled_height / 2.0)) - 1;

    x_min = std::max(0, x_min);
    y_min = std::max(0, y_min);
    x_max = std::min(image_width - 1, x_max);
    y_max = std::min(image_height - 1, y_max);

    if (x_max <= x_min || y_max <= y_min) {
      return false;
    }

    out_rect.x_offset = x_min;
    out_rect.y_offset = y_min;
    out_rect.width = x_max - x_min + 1;
    out_rect.height = y_max - y_min + 1;
    return true;
  }

  void resize_roi(const cv::Mat & roi_image, cv::Mat & output_image) const
  {
    if (roi_image.empty()) {
      output_image = roi_image;
      return;
    }

    const double scale = static_cast<double>(kRoiTargetHeight) / static_cast<double>(roi_image.rows);
    const int target_width = std::max(1, static_cast<int>(std::round(roi_image.cols * scale)));
    const int interpolation = scale < 1.0 ? cv::INTER_AREA : cv::INTER_LINEAR;
    cv::resize(roi_image, output_image, cv::Size(target_width, kRoiTargetHeight), 0, 0, interpolation);
  }

  void get_picture_callback(const std_msgs::msg::Int32::SharedPtr msg)
  {
    if (!msg || msg->data != 1) {
      return;
    }

    std::lock_guard<std::mutex> lock(flag_mutex_);
    send_image_flag_ = 1;
  }

  void reset_state_by_button()
  {
    {
      std::lock_guard<std::mutex> lock(flag_mutex_);
      send_image_flag_ = 0;
    }

    {
      std::lock_guard<std::mutex> lock(target_mutex_);
      has_person_ = false;
      latest_person_rect_ = PersonRoiRect();
    }

    nv12_buffer_.clear();
    RCLCPP_INFO(get_logger(), "person_image_cropper state reset");
  }

  void chassis_start_button_callback(const std_msgs::msg::Bool::SharedPtr msg)
  {
    if (!msg) {
      return;
    }

    const bool button_pressed = msg->data;
    const bool rising_edge = button_pressed && !last_start_button_state_;
    last_start_button_state_ = button_pressed;

    if (rising_edge) {
      reset_state_by_button();
    }
  }

  rclcpp::Subscription<hbm_img_msgs::msg::HbmMsg1080P>::SharedPtr image_sub_;
  rclcpp::Subscription<ai_msgs::msg::PerceptionTargets>::SharedPtr target_sub_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr get_picture_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr chassis_start_button_sub_;
  rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr image_pub_;

  std::mutex flag_mutex_;
  std::mutex target_mutex_;
  int send_image_flag_ = 0;
  bool last_start_button_state_ = false;
  bool has_person_ = false;
  PersonRoiRect latest_person_rect_;
  std::vector<uint8_t> nv12_buffer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PersonImageCropper>());
  rclcpp::shutdown();
  return 0;
}
