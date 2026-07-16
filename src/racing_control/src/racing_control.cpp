#include <algorithm>
#include <mutex>
#include <vector>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <ai_msgs/msg/perception_targets.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/int32.hpp>

class RacingControlNode : public rclcpp::Node
{
public:
  RacingControlNode(const std::string &node_name, const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

private:
  void subscription_callback_point(const ai_msgs::msg::PerceptionTargets::SharedPtr msg);
  void subscription_callback_target(const ai_msgs::msg::PerceptionTargets::SharedPtr msg);

  void line_following(const ai_msgs::msg::Target &point_msg);
  void timer_callback();
  double qr_speed_factor();

  ai_msgs::msg::PerceptionTargets::SharedPtr latest_point_msg_;
  ai_msgs::msg::PerceptionTargets::SharedPtr latest_target_msg_;

  std::mutex point_msg_mutex_;
  std::mutex target_msg_mutex_;

  int end_y_ = 0;
  int is_avoid_ = 0;
  double angular_z_ = 0.0;

  double line_x_ = 0.0;
  double line_kp_ = 0.0;
  double line_center_offset_ = 20.0;
  double avoid_x_ = 0.0;
  double avoid_kp_ = 0.0;
  int end_y_p_ = 0;
  bool line_return_ = false;
  double p_target_x_ = 300.0;
  double p_align_tolerance_ = 25.0;
  double p_approach_x_ = 0.12;
  double p_kp_ = 0.004;
  double p_max_angular_ = 0.8;
  int p_force_stop_y_ = 455;
  int p_blind_start_y_ = 390;
  double p_blind_x_ = 0.10;
  double p_blind_duration_ = 1.0;
  bool p_ready_for_blind_ = false;
  bool p_blind_forward_ = false;
  rclcpp::Time p_blind_start_time_;

  double last_p_error_out_ = 0.0;
  double last_point_error_out_ = 0.0;
  double last_avoid_error_out_ = 0.0;

  double last_avoid_error_sign_ = 0.0;

  int lost_line_count_ = 0;       // 连续丢线帧数
  bool line_ever_detected_ = false; // 首次检测到线后才启用丢线恢复旋转
  int avoid_direction_ = -1;       // 障碍物规避方向 (-1:未初始化, 0:左, 1:右)
  int avoid_counter_ = 0;          // 规避计数器

  // QR 减速参数与状态
  double qr_slow_x_ = 0.12;
  int qr_slow_y_min_ = 60;
  int qr_slow_y_max_ = 180;
  int qr_stop_y_ = 210;
  double qr_bottom_timeout_ = 0.8;
  int qr_bottom_ = -1;
  rclcpp::Time qr_bottom_time_;
  std::mutex qr_bottom_mutex_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr qr_bottom_subscriber_;

  std::string pub_control_topic_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_p_;
  rclcpp::Subscription<ai_msgs::msg::PerceptionTargets>::SharedPtr point_subscriber_;
  rclcpp::Subscription<ai_msgs::msg::PerceptionTargets>::SharedPtr target_subscriber_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

RacingControlNode::RacingControlNode(const std::string &node_name, const rclcpp::NodeOptions &options) : Node(node_name, options)
{
  declare_parameter("pub_control_topic", "racing");
  declare_parameter("end_y", end_y_);
  declare_parameter("line_x", line_x_);
  declare_parameter("line_kp", line_kp_);
  declare_parameter("line_center_offset", line_center_offset_);
  declare_parameter("avoid_x", avoid_x_);
  declare_parameter("avoid_kp", avoid_kp_);
  declare_parameter("end_y_p", end_y_p_);
  declare_parameter("p_target_x", p_target_x_);
  declare_parameter("p_align_tolerance", p_align_tolerance_);
  declare_parameter("p_approach_x", p_approach_x_);
  declare_parameter("p_kp", p_kp_);
  declare_parameter("p_max_angular", p_max_angular_);
  declare_parameter("p_force_stop_y", p_force_stop_y_);
  declare_parameter("p_blind_start_y", p_blind_start_y_);
  declare_parameter("p_blind_x", p_blind_x_);
  declare_parameter("p_blind_duration", p_blind_duration_);
  declare_parameter("qr_slow_x", qr_slow_x_);
  declare_parameter("qr_slow_y_min", qr_slow_y_min_);
  declare_parameter("qr_slow_y_max", qr_slow_y_max_);
  declare_parameter("qr_stop_y", qr_stop_y_);
  declare_parameter("qr_bottom_timeout", qr_bottom_timeout_);

  get_parameter("pub_control_topic", pub_control_topic_);
  get_parameter("end_y", end_y_);
  get_parameter("line_x", line_x_);
  get_parameter("line_kp", line_kp_);
  get_parameter("line_center_offset", line_center_offset_);
  get_parameter("avoid_x", avoid_x_);
  get_parameter("avoid_kp", avoid_kp_);
  get_parameter("end_y_p", end_y_p_);
  get_parameter("p_target_x", p_target_x_);
  get_parameter("p_align_tolerance", p_align_tolerance_);
  get_parameter("p_approach_x", p_approach_x_);
  get_parameter("p_kp", p_kp_);
  get_parameter("p_max_angular", p_max_angular_);
  get_parameter("p_force_stop_y", p_force_stop_y_);
  get_parameter("p_blind_start_y", p_blind_start_y_);
  get_parameter("p_blind_x", p_blind_x_);
  get_parameter("p_blind_duration", p_blind_duration_);
  get_parameter("qr_slow_x", qr_slow_x_);
  get_parameter("qr_slow_y_min", qr_slow_y_min_);
  get_parameter("qr_slow_y_max", qr_slow_y_max_);
  get_parameter("qr_stop_y", qr_stop_y_);
  get_parameter("qr_bottom_timeout", qr_bottom_timeout_);

  rclcpp::QoS qos(1);
  qos.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);

  point_subscriber_ = create_subscription<ai_msgs::msg::PerceptionTargets>(
      "/racing_track_center_detection", qos,
      std::bind(&RacingControlNode::subscription_callback_point, this, std::placeholders::_1));

  target_subscriber_ = create_subscription<ai_msgs::msg::PerceptionTargets>(
      "/racing_obstacle_detection", qos,
      std::bind(&RacingControlNode::subscription_callback_target, this, std::placeholders::_1));

  publisher_p_ = this->create_publisher<std_msgs::msg::String>("/p", 10);
  publisher_ = create_publisher<geometry_msgs::msg::Twist>(pub_control_topic_, qos);
  timer_ = create_wall_timer(std::chrono::milliseconds(30), std::bind(&RacingControlNode::timer_callback, this));

  qr_bottom_subscriber_ = create_subscription<std_msgs::msg::Int32>(
      "/qrcode_bottom", rclcpp::QoS(10).best_effort(),
      [this](const std_msgs::msg::Int32::SharedPtr msg) {
          std::unique_lock<std::mutex> lock(qr_bottom_mutex_);
          qr_bottom_ = msg->data;
          qr_bottom_time_ = this->now();
      });
}

double RacingControlNode::qr_speed_factor()
{
  int bottom;
  rclcpp::Time stamp;
  {
    std::unique_lock<std::mutex> lock(qr_bottom_mutex_);
    bottom = qr_bottom_;
    stamp = qr_bottom_time_;
  }
  if (bottom < 0)
    return 1.0;
  if ((this->now() - stamp).seconds() > qr_bottom_timeout_)
    return 1.0;
  if (qr_stop_y_ > 0 && bottom >= qr_stop_y_)
    return 0.0;
  if (bottom <= qr_slow_y_min_)
    return 1.0;

  double target_x = qr_slow_x_;
  if (bottom < qr_slow_y_max_ && qr_slow_y_max_ > qr_slow_y_min_)
  {
    double t = static_cast<double>(bottom - qr_slow_y_min_) /
               static_cast<double>(qr_slow_y_max_ - qr_slow_y_min_);
    target_x = line_x_ + (qr_slow_x_ - line_x_) * t;
  }
  if (line_x_ <= 0.0)
    return 1.0;
  return std::max(0.0, std::min(1.0, target_x / line_x_));
}

void RacingControlNode::subscription_callback_point(const ai_msgs::msg::PerceptionTargets::SharedPtr msg)
{
  std::unique_lock<std::mutex> lock(point_msg_mutex_);
  latest_point_msg_ = msg;
}

void RacingControlNode::subscription_callback_target(const ai_msgs::msg::PerceptionTargets::SharedPtr msg)
{
  std::unique_lock<std::mutex> lock(target_msg_mutex_);
  latest_target_msg_ = msg;
}

void RacingControlNode::timer_callback()
{
  line_return_ = false;
  ai_msgs::msg::PerceptionTargets::SharedPtr point_msg;
  ai_msgs::msg::PerceptionTargets::SharedPtr target_msg;

  {
    std::unique_lock<std::mutex> lock(point_msg_mutex_);
    if (latest_point_msg_)
      point_msg = latest_point_msg_;
  }
  {
    std::unique_lock<std::mutex> lock(target_msg_mutex_);
    if (latest_target_msg_)
      target_msg = latest_target_msg_;
  }

  std::vector<ai_msgs::msg::Target> filtered_obstacles;
  std::vector<ai_msgs::msg::Target> filtered_p;
  if (target_msg)
  {
    for (const auto &target : target_msg->targets)
    {
      if (!target.rois.empty() && target.type == "zt" && target.rois[0].confidence > 0.7)
      {
        filtered_obstacles.push_back(target);
      }

      if (!target.rois.empty() && target.type == "p" && target.rois[0].confidence > 0.7)
      {
        filtered_p.push_back(target);
      }
    }
  }

  if (!filtered_obstacles.empty())
  {
    auto max_area_target = std::max_element(
        filtered_obstacles.begin(), filtered_obstacles.end(),
        [](const ai_msgs::msg::Target &a, const ai_msgs::msg::Target &b)
        { return (a.rois[0].rect.width * a.rois[0].rect.height) <
                 (b.rois[0].rect.width * b.rois[0].rect.height); });
    const auto &target = *max_area_target;
    int bottom = target.rois[0].rect.y_offset + target.rois[0].rect.height;

    int obstacle_left = target.rois[0].rect.x_offset;
    int obstacle_right = obstacle_left + target.rois[0].rect.width;
    double center_x = (obstacle_left + obstacle_right) / 2.0;

    RCLCPP_INFO(this->get_logger(), "end_y:%d obstacle_left:%d obstacle_right:%d center_x:%lf",
                bottom, obstacle_left, obstacle_right, center_x);

    if (bottom >= end_y_ && bottom <= 480)
    {
      int current_dir = -1;
      if (avoid_direction_ == -1 || avoid_counter_ >= 2)
      {
        current_dir = (center_x > 320) ? 0 : 1;
        avoid_direction_ = current_dir;
        avoid_counter_ = 0;
      }
      else
      {
        current_dir = avoid_direction_;
        avoid_counter_++;
      }

      double avoid_error_now = 0.0;
      if (current_dir == 0)
        avoid_error_now = 640 - center_x;
      else
        avoid_error_now = 0 - center_x;

      if (std::abs(avoid_error_now) < 5.0)
      {
        avoid_error_now = 0.0;
        last_avoid_error_out_ = 0.0;
      }
      double avoid_error_out = 0.7 * avoid_error_now + 0.3 * last_avoid_error_out_;
      last_avoid_error_out_ = avoid_error_out;

      angular_z_ = avoid_kp_ * avoid_error_out;
      RCLCPP_INFO(this->get_logger(), "error:%lf  avoid_z:%lf", avoid_error_out, angular_z_);

      auto twist_msg = geometry_msgs::msg::Twist();
      twist_msg.linear.x = avoid_x_;
      twist_msg.angular.z = angular_z_;
      publisher_->publish(twist_msg);

      is_avoid_ = 3;

      if (avoid_error_out > 0)
      {
        last_avoid_error_sign_ = 1.0;
      }
      else if (avoid_error_out < 0)
      {
        last_avoid_error_sign_ = -1.0;
      }
      else
      {
        last_avoid_error_sign_ = 0.0;
      }
      return;
    }
  }

  // --- P-point blind forward phase (no P visibility required) ---
  if (p_blind_forward_)
  {
    double elapsed = (this->now() - p_blind_start_time_).seconds();
    if (elapsed >= p_blind_duration_)
    {
      // Blind forward done -> stop and publish /p
      auto twist_msg = geometry_msgs::msg::Twist();
      publisher_->publish(twist_msg);
      auto msg_str = std_msgs::msg::String();
      msg_str.data = "1";
      publisher_p_->publish(msg_str);
      RCLCPP_INFO(this->get_logger(), "P_BLIND done (%.2fs) -> STOP", elapsed);
      p_blind_forward_ = false;
      p_ready_for_blind_ = false;
    }
    else
    {
      auto twist_msg = geometry_msgs::msg::Twist();
      twist_msg.linear.x = p_blind_x_;
      publisher_->publish(twist_msg);
    }
    return;
  }

  // --- P-point alignment phase ---
  if (!filtered_p.empty())
  {
    auto max_area_target = std::max_element(
        filtered_p.begin(), filtered_p.end(),
        [](const ai_msgs::msg::Target &a, const ai_msgs::msg::Target &b)
        { return (a.rois[0].rect.width * a.rois[0].rect.height) <
                 (b.rois[0].rect.width * b.rois[0].rect.height); });
    const auto &target = *max_area_target;
    int bottom = target.rois[0].rect.y_offset + target.rois[0].rect.height;

    int obstacle_left = target.rois[0].rect.x_offset;
    int obstacle_right = obstacle_left + target.rois[0].rect.width;
    double center_x = (obstacle_left + obstacle_right) / 2.0;

    RCLCPP_INFO(this->get_logger(), "P_ALIGN: bottom=%d center_x=%.1f p_target_x=%.1f",
                bottom, center_x, p_target_x_);
    line_return_ = true;

    // Force stop if P is extremely close
    if (bottom >= p_force_stop_y_)
    {
      auto twist_msg = geometry_msgs::msg::Twist();
      publisher_->publish(twist_msg);
      auto msg_str = std_msgs::msg::String();
      msg_str.data = "1";
      publisher_p_->publish(msg_str);
      RCLCPP_INFO(this->get_logger(), "P_FORCE_STOP: bottom=%d >= %d", bottom, p_force_stop_y_);
      return;
    }

    // Lateral alignment: steer to make center_x match p_target_x
    double p_error = p_target_x_ - center_x;
    double p_error_out = 0.7 * p_error + 0.3 * last_p_error_out_;
    last_p_error_out_ = p_error_out;
    double p_angular = p_kp_ * p_error_out;
    p_angular = std::max(-p_max_angular_, std::min(p_max_angular_, p_angular));

    bool aligned = std::abs(p_error) <= p_align_tolerance_;

    // Check if ready to enter blind forward
    if (aligned && bottom >= p_blind_start_y_)
    {
      p_ready_for_blind_ = true;
    }

    if (p_ready_for_blind_ && bottom >= p_blind_start_y_)
    {
      // Enter blind forward mode
      p_blind_forward_ = true;
      p_blind_start_time_ = this->now();
      RCLCPP_INFO(this->get_logger(), "P_BLIND_START: aligned=true bottom=%d", bottom);
      auto twist_msg = geometry_msgs::msg::Twist();
      twist_msg.linear.x = p_blind_x_;
      publisher_->publish(twist_msg);
      return;
    }

    // Drive toward P with alignment correction
    auto twist_msg = geometry_msgs::msg::Twist();
    twist_msg.linear.x = p_approach_x_;
    twist_msg.angular.z = p_angular;
    publisher_->publish(twist_msg);
    RCLCPP_INFO(this->get_logger(), "P_APPROACH: err=%.1f ang=%.3f aligned=%d bottom=%d",
                p_error, p_angular, aligned ? 1 : 0, bottom);
    return;
  }

  if (!point_msg || point_msg->targets.empty() || point_msg->targets[0].points.empty() || point_msg->targets[0].points[0].point.empty())
  {
    // 首次检测到线之前不旋转（等待相机流水线初始化）
    if (!line_ever_detected_)
      return;

    geometry_msgs::msg::Twist twist_msg;
    lost_line_count_++;
    if (lost_line_count_ > 5) lost_line_count_ = 5;  // 防止无界增长

    double base_angular = -std::copysign(0.5, angular_z_);
    double current_angular = (lost_line_count_ == 1) ? base_angular : base_angular * (lost_line_count_ + 2) * 0.3;
    current_angular = std::max(-1.5, std::min(1.5, current_angular));  // 限幅

    twist_msg.angular.z = current_angular;
    publisher_->publish(twist_msg);
  }
  else
  {
    lost_line_count_ = 0;
    line_ever_detected_ = true;
    const auto &point_target = point_msg->targets[0];
    line_following(point_target);
  }
}

void RacingControlNode::line_following(const ai_msgs::msg::Target &point_msg)
{
  double x = point_msg.points[0].point[0].x;
   double point_error_now = 0.0;
  point_error_now = (320.0 + line_center_offset_) - x;
  if (std::abs(point_error_now) < 3.0)
  {
    point_error_now = 0.0;
    last_point_error_out_ = 0.0;
  }
  double point_error_out = 0.7 * point_error_now + 0.3 * last_point_error_out_;
  double line_z = line_kp_ * point_error_out;

  if (is_avoid_ > 0)
  {
    line_z *= 0.7;
    is_avoid_ -= 1;
  }

  auto twist_msg = geometry_msgs::msg::Twist();
  double speed_factor = qr_speed_factor();
  twist_msg.linear.x = line_x_ * speed_factor;  // QR 接近时减速
  twist_msg.angular.z = speed_factor <= 0.01 ? 0.0 : line_z;
  publisher_->publish(twist_msg);

  last_point_error_out_ = point_error_out;
}

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RacingControlNode>("RacingControlNode"));
  rclcpp::shutdown();
  return 0;
}