#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/bool.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <mutex>
#include <cmath>
#include <algorithm>
#include <limits>
#include <utility>
#include <vector>

class ChassisController : public rclcpp::Node
{
public:
    ChassisController() : Node("control_master"), qrcode_detected_(false), p_detected_(false), vision_active_(false)
    {
        // LiDAR avoidance parameters
        this->declare_parameter("enable_lidar_avoid", false);

        this->declare_parameter("lidar_avoid_speed", 0.46);
        this->declare_parameter("lidar_avoid_distance", 0.35);
        this->declare_parameter("lidar_avoid_clear_distance", 0.55);
        this->declare_parameter("lidar_front_angle_deg", 55.0);
        this->declare_parameter("lidar_center_angle_deg", 22.0);
        this->declare_parameter("lidar_side_guard_distance", 0.28);
        this->declare_parameter("lidar_turn_trigger_gain", 0.22);
        this->declare_parameter("lidar_turn_clear_gain", 0.12);
        this->declare_parameter("lidar_speed_trigger_gain", 0.45);
        this->declare_parameter("lidar_speed_clear_gain", 0.22);
        this->declare_parameter("lidar_bypass_release_margin", 0.15);
        this->declare_parameter("teb_lite_horizon", 1.30);
        this->declare_parameter("teb_lite_dt", 0.10);
        this->declare_parameter("teb_lite_safety_radius", 0.32);
        this->declare_parameter("teb_lite_collision_radius", 0.22);
        this->declare_parameter("teb_lite_max_wz", 2.4);
        this->declare_parameter("teb_lite_obstacle_weight", 95.0);
        // Follow-the-Gap (F1TENTH) parameters
        this->declare_parameter("ftg_bubble_radius", 0.32);
        this->declare_parameter("ftg_gap_min_range", 0.85);
        this->declare_parameter("ftg_max_range", 3.0);
        // Phased avoidance (S-curve) parameters
        this->declare_parameter("avoid_turn_cycles", 6);
        this->declare_parameter("avoid_forward_min_cycles", 6);
        this->declare_parameter("avoid_forward_max_cycles", 20);
        this->declare_parameter("avoid_return_cycles", 10);
        enable_lidar_avoid_ = this->get_parameter("enable_lidar_avoid").as_bool();
        lidar_avoid_speed_ = this->get_parameter("lidar_avoid_speed").as_double();
        lidar_avoid_distance_ = this->get_parameter("lidar_avoid_distance").as_double();
        lidar_avoid_clear_distance_ = this->get_parameter("lidar_avoid_clear_distance").as_double();
        lidar_front_angle_ = this->get_parameter("lidar_front_angle_deg").as_double() * M_PI / 180.0;
        lidar_center_angle_ = this->get_parameter("lidar_center_angle_deg").as_double() * M_PI / 180.0;
        lidar_side_guard_distance_ = this->get_parameter("lidar_side_guard_distance").as_double();
        lidar_turn_trigger_gain_ = this->get_parameter("lidar_turn_trigger_gain").as_double();
        lidar_turn_clear_gain_ = this->get_parameter("lidar_turn_clear_gain").as_double();
        lidar_speed_trigger_gain_ = this->get_parameter("lidar_speed_trigger_gain").as_double();
        lidar_speed_clear_gain_ = this->get_parameter("lidar_speed_clear_gain").as_double();
        lidar_bypass_release_margin_ = this->get_parameter("lidar_bypass_release_margin").as_double();
        teb_lite_horizon_ = this->get_parameter("teb_lite_horizon").as_double();
        teb_lite_dt_ = this->get_parameter("teb_lite_dt").as_double();
        teb_lite_safety_radius_ = this->get_parameter("teb_lite_safety_radius").as_double();
        teb_lite_collision_radius_ = this->get_parameter("teb_lite_collision_radius").as_double();
        teb_lite_max_wz_ = this->get_parameter("teb_lite_max_wz").as_double();
        teb_lite_obstacle_weight_ = this->get_parameter("teb_lite_obstacle_weight").as_double();
        ftg_bubble_radius_ = this->get_parameter("ftg_bubble_radius").as_double();
        ftg_gap_min_range_ = this->get_parameter("ftg_gap_min_range").as_double();
        ftg_max_range_ = this->get_parameter("ftg_max_range").as_double();
        avoid_turn_cycles_ = this->get_parameter("avoid_turn_cycles").as_int();
        avoid_forward_min_cycles_ = this->get_parameter("avoid_forward_min_cycles").as_int();
        avoid_forward_max_cycles_ = this->get_parameter("avoid_forward_max_cycles").as_int();
        avoid_return_cycles_ = this->get_parameter("avoid_return_cycles").as_int();

        rclcpp::QoS qos(1);
        qos.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);

        qrcode_sub_ = this->create_subscription<std_msgs::msg::String>(
            "/qrcode_result",
            10,
            std::bind(&ChassisController::qrcode_callback, this, std::placeholders::_1));

        p_sub_ = this->create_subscription<std_msgs::msg::String>(
            "/p",
            10,
            std::bind(&ChassisController::p_callback, this, std::placeholders::_1));

        racing_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/racing",
            qos,
            std::bind(&ChassisController::racing_callback, this, std::placeholders::_1));

        nav_cmd_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/nav_cmd_vel", 10,
            std::bind(&ChassisController::nav_cmd_callback, this, std::placeholders::_1));

        subscriber_car_go_ = this->create_subscription<std_msgs::msg::Int32>(
            "/car_go", 10, std::bind(&ChassisController::callback_car_go, this, std::placeholders::_1));

        vision_enable_sub_ = this->create_subscription<std_msgs::msg::Bool>(
            "/vision_enable", 10, std::bind(&ChassisController::callback_vision_enable, this, std::placeholders::_1));

        if (enable_lidar_avoid_)
        {
            scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
                "/scan", rclcpp::SensorDataQoS(),
                std::bind(&ChassisController::scan_callback, this, std::placeholders::_1));
        }

        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom_combined", rclcpp::SensorDataQoS(),
            std::bind(&ChassisController::odom_callback, this, std::placeholders::_1));

        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", qos);

        // Publish simplified obstacle info for waypoint_nav_node (avoids Python LaserScan crash)
        lidar_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/lidar_min_range", 10);

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(30),
            std::bind(&ChassisController::timer_callback, this));
    }

    // Publish zero velocity several times so the chassis driver (which latches
    // the last /cmd_vel) actually stops when the node is shutting down (Ctrl-C).
    void stop()
    {
        geometry_msgs::msg::Twist zero;
        for (int i = 0; i < 10 && rclcpp::ok(); ++i)
        {
            cmd_vel_pub_->publish(zero);
            rclcpp::sleep_for(std::chrono::milliseconds(20));
        }
    }

private:
    int compute_blend_cycles(double reference_speed) const
    {
        int cycles = 16 + static_cast<int>(std::round(22.0 * std::max(0.0, reference_speed)));
        return std::max(16, std::min(34, cycles));
    }

    void callback_car_go(const std_msgs::msg::Int32::SharedPtr msg)
    {
        if (!msg)
            return;
        if (msg->data == -10)
        {
            std::lock_guard<std::mutex> lock(mutex_);
            if (!vision_active_)
            {
                vision_active_ = true;
                RCLCPP_INFO(this->get_logger(), "control_master: /car_go=-10 -> vision_active=true");
            }
        }
    }

    void callback_vision_enable(const std_msgs::msg::Bool::SharedPtr msg)
    {
        if (!msg)
            return;
        std::lock_guard<std::mutex> lock(mutex_);
        if (msg->data)
        {
            // RISING EDGE ONLY: waypoint_nav republishes /vision_enable=true at
            // 20Hz during HANDOFF. Clearing the stop flags on every message
            // would wipe a just-latched p_detected_/qrcode_detected_ within 50ms
            // and the car would never actually stop at the P point. Only reset
            // when vision FIRST becomes active.
            if (!vision_active_)
            {
                vision_active_ = true;
                qrcode_detected_ = false;
                p_detected_ = false;
                RCLCPP_INFO(this->get_logger(), "control_master: vision_enable=true -> vision_active=true");
            }
        }
        else
        {
            // Hand control back to odometry nav: go silent and clear stop flags
            // so a later re-enable doesn't immediately latch a stop.
            if (vision_active_)
            {
                vision_active_ = false;
                qrcode_detected_ = false;
                p_detected_ = false;
                RCLCPP_INFO(this->get_logger(), "control_master: vision_enable=false -> vision_active=false (odom nav active)");
            }
        }
    }

    void qrcode_callback(const std_msgs::msg::String::SharedPtr msg)
    {
        if (!msg)
            return;
        std::lock_guard<std::mutex> lock(mutex_);
        // Only latch a QR stop while the vision stack is actually driving.
        // During the odometry leg a re-detected QR must not arm a stop that
        // would freeze the later vision return-to-P leg.
        if (!vision_active_)
            return;
        qrcode_detected_ = true;
    }

    void p_callback(const std_msgs::msg::String::SharedPtr msg)
    {
        if (!msg)
            return;
        std::lock_guard<std::mutex> lock(mutex_);
        p_detected_ = true;
    }
    void racing_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        if (!msg)
            return;
        std::lock_guard<std::mutex> lock(mutex_);
        racing_cmd_ = *msg;
    }

    void nav_cmd_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        if (!msg)
            return;
        std::lock_guard<std::mutex> lock(mutex_);
        nav_cmd_ = *msg;
        nav_cmd_fresh_ = true;
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
            "control_master: nav_cmd received vx=%.2f wz=%.2f vision_active=%s",
            nav_cmd_.linear.x, nav_cmd_.angular.z, vision_active_ ? "true" : "false");
    }

    static double normalize_angle(double a)
    {
        while (a > M_PI) a -= 2.0 * M_PI;
        while (a < -M_PI) a += 2.0 * M_PI;
        return a;
    }

    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        if (!msg)
            return;
        std::lock_guard<std::mutex> lock(mutex_);
        // Extract yaw from quaternion
        double qy = msg->pose.pose.orientation.y;
        double qz = msg->pose.pose.orientation.z;
        double qw = msg->pose.pose.orientation.w;
        double qx = msg->pose.pose.orientation.x;
        robot_heading_ = std::atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz));
    }

    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        if (!msg)
            return;
        std::lock_guard<std::mutex> lock(mutex_);

        double min_l = 1e9, min_r = 1e9, min_c = 1e9;
        double min_l_outer = 1e9, min_r_outer = 1e9;
        double side_l = 1e9, side_r = 1e9;
        double left_weight_sum = 0.0, right_weight_sum = 0.0;
        double left_angle_sum = 0.0, right_angle_sum = 0.0;
        double left_range_sum = 0.0, right_range_sum = 0.0;
        int left_count = 0, right_count = 0;
        double score_cap = lidar_avoid_clear_distance_ + 0.4;
        std::vector<std::pair<double, double>> obstacle_points;
        obstacle_points.reserve(msg->ranges.size());
        std::vector<std::pair<double, double>> fov_pts;  // (angle, clamped range) for FTG
        fov_pts.reserve(msg->ranges.size());

        for (size_t i = 0; i < msg->ranges.size(); ++i)
        {
            double angle = msg->angle_min + i * msg->angle_increment;
            double r = msg->ranges[i];
            if (r < msg->range_min || r > msg->range_max || std::isnan(r) || std::isinf(r))
                continue;
            double a = std::atan2(std::sin(angle), std::cos(angle));
            if (std::abs(a) <= lidar_center_angle_ && r < min_c)
                min_c = r;
            if (std::abs(a) <= lidar_front_angle_)
                fov_pts.emplace_back(a, std::min(r, ftg_max_range_));
            // Collect obstacle points over a wide arc (front + true sides) so
            // that a cone beside the car is still seen during the return phase.
            if (std::abs(a) <= lidar_front_angle_ && r <= lidar_avoid_clear_distance_ + 0.9)
                obstacle_points.emplace_back(r * std::cos(a), r * std::sin(a));
            else if (std::abs(a) <= 2.45 && r <= 1.0)
                obstacle_points.emplace_back(r * std::cos(a), r * std::sin(a));
            // True-side minima (beyond the front sector, up to ~140 deg)
            if (a > lidar_front_angle_ && a <= 2.45 && r < side_l)
                side_l = r;
            else if (a < -lidar_front_angle_ && a >= -2.45 && r < side_r)
                side_r = r;
            if (a > 0.0 && a <= lidar_front_angle_)
            {
                if (r < min_l) min_l = r;
                if (a > lidar_center_angle_ && r < min_l_outer) min_l_outer = r;
                double clearance = std::min(r, score_cap);
                double weight = clearance * clearance;
                left_weight_sum += weight;
                left_angle_sum += weight * a;
                left_range_sum += clearance;
                left_count++;
            }
            else if (a < 0.0 && a >= -lidar_front_angle_)
            {
                if (r < min_r) min_r = r;
                if (a < -lidar_center_angle_ && r < min_r_outer) min_r_outer = r;
                double clearance = std::min(r, score_cap);
                double weight = clearance * clearance;
                right_weight_sum += weight;
                right_angle_sum += weight * a;
                right_range_sum += clearance;
                right_count++;
            }
        }
        lidar_min_left_ = min_l;
        lidar_min_right_ = min_r;
        lidar_min_center_ = min_c;
        lidar_min_left_outer_ = min_l_outer;
        lidar_min_right_outer_ = min_r_outer;
        lidar_min_left_side_ = side_l;
        lidar_min_right_side_ = side_r;
        lidar_left_free_heading_ = left_weight_sum > 0.0 ? left_angle_sum / left_weight_sum : 0.70 * lidar_front_angle_;
        lidar_right_free_heading_ = right_weight_sum > 0.0 ? right_angle_sum / right_weight_sum : -0.70 * lidar_front_angle_;
        lidar_left_free_range_ = left_count > 0 ? left_range_sum / left_count : 0.0;
        lidar_right_free_range_ = right_count > 0 ? right_range_sum / right_count : 0.0;
        lidar_left_score_ = left_count > 0 ? (lidar_left_free_range_ - 0.12 * std::abs(lidar_left_free_heading_)) : -1.0;
        lidar_right_score_ = right_count > 0 ? (lidar_right_free_range_ - 0.12 * std::abs(lidar_right_free_heading_)) : -1.0;
        lidar_obstacles_ = std::move(obstacle_points);

        // --- Follow the Gap (F1TENTH): bubble out the closest obstacle, then
        // steer toward the widest/deepest free gap in the front FOV. ---
        lidar_ftg_valid_ = false;
        if (fov_pts.size() >= 8)
        {
            std::sort(fov_pts.begin(), fov_pts.end(),
                      [](const std::pair<double, double> &p1, const std::pair<double, double> &p2)
                      { return p1.first < p2.first; });

            // 1. Closest point in the FOV
            size_t min_i = 0;
            for (size_t i = 1; i < fov_pts.size(); ++i)
                if (fov_pts[i].second < fov_pts[min_i].second)
                    min_i = i;
            double closest_r = fov_pts[min_i].second;
            double closest_a = fov_pts[min_i].first;

            // 2. Safety bubble: mark every point whose euclidean distance to the
            // closest obstacle point is inside the bubble as blocked (range 0).
            std::vector<double> proc(fov_pts.size());
            double cx = closest_r * std::cos(closest_a);
            double cy = closest_r * std::sin(closest_a);
            for (size_t i = 0; i < fov_pts.size(); ++i)
            {
                double px = fov_pts[i].second * std::cos(fov_pts[i].first);
                double py = fov_pts[i].second * std::sin(fov_pts[i].first);
                proc[i] = (std::hypot(px - cx, py - cy) < ftg_bubble_radius_) ? 0.0 : fov_pts[i].second;
            }

            // 3. Largest contiguous gap where range clears the threshold
            size_t best_start = 0, best_len = 0;
            size_t cur_start = 0, cur_len = 0;
            for (size_t i = 0; i < proc.size(); ++i)
            {
                if (proc[i] > ftg_gap_min_range_)
                {
                    if (cur_len == 0)
                        cur_start = i;
                    cur_len++;
                    if (cur_len > best_len)
                    {
                        best_len = cur_len;
                        best_start = cur_start;
                    }
                }
                else
                {
                    cur_len = 0;
                }
            }

            if (best_len >= 5)
            {
                // 4. Best point: blend the gap centre (stable) with the deepest
                // point of the gap (progress), keeping Ackermann-feasible angles.
                size_t gap_end = best_start + best_len - 1;
                size_t deepest = best_start;
                for (size_t i = best_start; i <= gap_end; ++i)
                    if (proc[i] > proc[deepest])
                        deepest = i;
                double center_a = 0.5 * (fov_pts[best_start].first + fov_pts[gap_end].first);
                double deep_a = fov_pts[deepest].first;
                double target = 0.55 * center_a + 0.45 * deep_a;
                target = std::max(-1.05, std::min(1.05, target));
                // EMA smoothing to avoid frame-to-frame jitter at speed
                lidar_ftg_heading_ = lidar_ftg_valid_prev_
                                         ? (0.65 * target + 0.35 * lidar_ftg_heading_)
                                         : target;
                lidar_ftg_valid_ = true;
            }
        }
        lidar_ftg_valid_prev_ = lidar_ftg_valid_;

        geometry_msgs::msg::Twist range_msg;
        range_msg.linear.x = min_l;
        range_msg.linear.y = min_r;
        range_msg.linear.z = min_c;
        lidar_pub_->publish(range_msg);
    }

    // Follow-the-Gap based avoidance target: prefer the FTG best-gap heading
    // (reactive, high-speed friendly); fall back to the legacy left/right
    // free-space scores when no valid gap exists.
    void select_avoid_heading()
    {
        if (lidar_ftg_valid_)
        {
            lidar_avoid_direction_ = (lidar_ftg_heading_ >= 0.0) ? 1 : -1;
            lidar_local_target_heading_ = lidar_ftg_heading_;
        }
        else
        {
            lidar_avoid_direction_ = (lidar_left_score_ >= lidar_right_score_) ? 1 : -1;
            lidar_local_target_heading_ = lidar_avoid_direction_ > 0 ? lidar_left_free_heading_ : lidar_right_free_heading_;
        }
    }

    geometry_msgs::msg::Twist compute_teb_lite_avoidance(const geometry_msgs::msg::Twist &base_cmd, double min_front)
    {
        geometry_msgs::msg::Twist best_cmd;
        double urgency = (lidar_avoid_clear_distance_ - std::min(min_front, lidar_avoid_clear_distance_)) /
                         std::max(0.05, lidar_avoid_clear_distance_ - lidar_avoid_distance_);
        urgency = std::max(0.0, std::min(1.0, urgency));

        double base_v = std::max(0.30, base_cmd.linear.x * (1.0 - 0.18 * urgency));
        double vx_candidates[] = {
            std::max(0.28, base_v * 0.90),
            base_v,
            std::min(std::max(0.34, base_cmd.linear.x), base_v * 1.30)
        };
        double max_wz = std::max(0.8, teb_lite_max_wz_);
        double wz_candidates[] = {
            -max_wz, -1.9, -1.45, -1.05, -0.70, -0.40, -0.18,
             0.18,  0.40,  0.70,  1.05,  1.45,  1.9, max_wz
        };

        double best_cost = std::numeric_limits<double>::infinity();
        double best_clearance = 0.0;
        int candidate_count = 0;
        double desired_theta;
        {
            double path_theta = base_cmd.angular.z * teb_lite_horizon_;
            double avoid_theta = std::abs(lidar_local_target_heading_) > 0.05 ?
                                 lidar_local_target_heading_ :
                                 static_cast<double>(lidar_avoid_direction_) * 0.55;
            avoid_theta = std::max(-1.05, std::min(1.05, avoid_theta));
            if (lidar_use_heading_return_)
            {
                // Converge back to the original heading PLUS the track curvature,
                // so on curves we rejoin the (curving) path instead of a stale heading.
                double heading_err = normalize_angle(avoidance_start_heading_ - robot_heading_);
                desired_theta = std::max(-1.2, std::min(1.2, heading_err * 1.2 + path_theta));
            }
            else
            {
                double blend = (lidar_avoid_blend_override_ >= 0.0) ?
                                lidar_avoid_blend_override_ :
                                (0.35 + urgency * 0.55);
                desired_theta = avoid_theta * blend + path_theta * (1.0 - blend);
                desired_theta = std::max(-1.2, std::min(1.2, desired_theta));
            }
        }

        for (double vx : vx_candidates)
        {
            for (double wz : wz_candidates)
            {
                candidate_count++;
                double x = 0.0;
                double y = 0.0;
                double theta = 0.0;
                double min_clearance = std::numeric_limits<double>::infinity();
                double obstacle_cost = 0.0;

                for (double t = 0.0; t < teb_lite_horizon_; t += teb_lite_dt_)
                {
                    x += vx * std::cos(theta) * teb_lite_dt_;
                    y += vx * std::sin(theta) * teb_lite_dt_;
                    theta += wz * teb_lite_dt_;

                    for (const auto &obstacle : lidar_obstacles_)
                    {
                        double ox = obstacle.first;
                        double oy = obstacle.second;
                        if (ox < -0.05)
                            continue;
                        double d = std::hypot(x - ox, y - oy);
                        min_clearance = std::min(min_clearance, d);
                        if (d < teb_lite_collision_radius_)
                        {
                            double diff = teb_lite_collision_radius_ - d;
                            obstacle_cost += 1200.0 + 2500.0 * diff * diff;
                        }
                        else if (d < teb_lite_safety_radius_)
                        {
                            double diff = teb_lite_safety_radius_ - d;
                            obstacle_cost += teb_lite_obstacle_weight_ * diff * diff;
                        }
                    }
                }

                if (!std::isfinite(min_clearance))
                    min_clearance = 9.99;

                double direction_cost = 0.0;
                if (!lidar_use_heading_return_)
                {
                    if (wz * static_cast<double>(lidar_avoid_direction_) < 0.0)
                        direction_cost += 1.8 + 0.45 * std::abs(wz);
                    else
                        direction_cost -= 0.35 * std::abs(wz) * (0.4 + urgency);
                }

                double weak_turn_cost = 0.0;
                if (urgency > 0.35 && std::abs(wz) < 0.55)
                    weak_turn_cost = 4.0 * (0.55 - std::abs(wz));

                double lateral_cost = 0.0;
                if (!lidar_use_heading_return_)
                {
                    lateral_cost = 0.12 * std::abs(y);
                    if (y * static_cast<double>(lidar_avoid_direction_) < 0.0)
                        lateral_cost += 2.0 * std::abs(y);
                    else
                        lateral_cost -= 0.45 * std::min(std::abs(y), 0.60);
                }

                double heading_cost = 0.55 * (1.0 - urgency * 0.5) * std::abs(theta - desired_theta);
                double smooth_cost = 0.18 * std::abs(wz - lidar_avoid_wz_ema_);
                double speed_cost = 0.05 * std::abs(vx - base_cmd.linear.x);
                double progress_cost = -(4.4 + 1.8 * urgency) * x;
                double clearance_reward = -0.60 * std::min(min_clearance, 1.0);

                double cost = obstacle_cost + direction_cost + weak_turn_cost + lateral_cost +
                              heading_cost + smooth_cost + speed_cost + progress_cost + clearance_reward;

                if (cost < best_cost)
                {
                    best_cost = cost;
                    best_clearance = min_clearance;
                    best_cmd.linear.x = vx;
                    best_cmd.angular.z = wz;
                }
            }
        }

        if (!std::isfinite(best_cost))
        {
            best_cmd.linear.x = 0.30;
            best_cmd.angular.z = static_cast<double>(lidar_avoid_direction_) * max_wz;
            best_cost = 9999.0;
            best_clearance = 0.0;
        }

        lidar_avoid_wz_ema_ = 0.45 * lidar_avoid_wz_ema_ + 0.55 * best_cmd.angular.z;
        best_cmd.angular.z = std::max(-max_wz, std::min(max_wz, lidar_avoid_wz_ema_));

        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500,
            "[TEB-LITE] urg=%.2f dtheta=%.2f ret=%d bov=%.2f head=%.2f -> vx=%.2f wz=%.2f cost=%.1f clr=%.2f cand=%d obs=%zu",
            urgency, desired_theta, lidar_use_heading_return_ ? 1 : 0,
            lidar_avoid_blend_override_, robot_heading_,
            best_cmd.linear.x, best_cmd.angular.z, best_cost, best_clearance,
            candidate_count, lidar_obstacles_.size());

        return best_cmd;
    }

    // Phased avoidance: S-curve lane change (TURN → FORWARD → RETURN → BLEND)
    geometry_msgs::msg::Twist compute_phased_avoidance(
        const geometry_msgs::msg::Twist &base_cmd,
        double forward_risk, double side_risk,
        double dynamic_clear, double release_threshold,
        double dynamic_enter)
    {
        geometry_msgs::msg::Twist cmd;
        double vx = std::max(0.30, base_cmd.linear.x);

        if (lidar_avoid_phase_ == AVOID_TURN)
        {
            lidar_avoid_blend_override_ = -1.0;  // use urgency-based blend
            lidar_use_heading_return_ = false;
            // Follow the Gap: refresh the target heading every cycle so the
            // steering tracks the live best gap instead of a stale snapshot.
            // Keep the committed direction unless the gap clearly flipped sides.
            if (lidar_ftg_valid_)
            {
                if (lidar_ftg_heading_ * static_cast<double>(lidar_avoid_direction_) > -0.15)
                    lidar_local_target_heading_ = lidar_ftg_heading_;
                else if (std::abs(lidar_ftg_heading_) > 0.35)
                    select_avoid_heading();  // gap decisively on the other side
            }
            cmd = compute_teb_lite_avoidance(base_cmd, forward_risk);
            lidar_avoid_phase_cnt_++;

            if (lidar_avoid_phase_cnt_ >= avoid_turn_cycles_)
            {
                lidar_avoid_phase_ = AVOID_FORWARD;
                lidar_avoid_phase_cnt_ = 0;
            }

            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500,
                "[AVOID_TURN] cnt=%d/%d risk=%.2f heading=%.2f -> vx=%.2f wz=%.2f",
                lidar_avoid_phase_cnt_, avoid_turn_cycles_, forward_risk,
                robot_heading_, cmd.linear.x, cmd.angular.z);
        }
        else if (lidar_avoid_phase_ == AVOID_FORWARD)
        {
            lidar_avoid_blend_override_ = 0.08;  // mostly straight, minimal avoid bias
            lidar_use_heading_return_ = false;
            cmd = compute_teb_lite_avoidance(base_cmd, forward_risk);
            lidar_avoid_phase_cnt_++;

            if (lidar_min_center_ < dynamic_enter)
            {
                // Only a FRONT obstacle restarts the turn; side proximity is
                // expected while passing alongside the obstacle.
                lidar_avoid_phase_ = AVOID_TURN;
                lidar_avoid_phase_cnt_ = 0;
                select_avoid_heading();
                RCLCPP_INFO(this->get_logger(), "[AVOID_FORWARD->TURN] new obstacle min_C=%.2f ftg=%d head=%.2f",
                            lidar_min_center_, lidar_ftg_valid_ ? 1 : 0, lidar_local_target_heading_);
            }
            else
            {
                // The cone sits on the opposite side of the avoid direction.
                // Require that TRUE side to open up before cutting back, so the
                // return turn cannot clip the cone.
                double cone_side = lidar_avoid_direction_ > 0 ?
                                   lidar_min_right_side_ : lidar_min_left_side_;
                bool cone_passed = cone_side > 0.40;
                if (lidar_avoid_phase_cnt_ >= avoid_forward_min_cycles_ &&
                    forward_risk > release_threshold && side_risk > dynamic_clear && cone_passed)
                {
                    lidar_avoid_phase_ = AVOID_RETURN;
                    lidar_avoid_phase_cnt_ = 0;
                }
                else if (lidar_avoid_phase_cnt_ >= avoid_forward_max_cycles_ && cone_passed)
                {
                    lidar_avoid_phase_ = AVOID_RETURN;
                    lidar_avoid_phase_cnt_ = 0;
                }
                else if (lidar_avoid_phase_cnt_ >= avoid_forward_max_cycles_ + 10)
                {
                    // Failsafe: cone never "clears" (e.g. wall on that side)
                    lidar_avoid_phase_ = AVOID_RETURN;
                    lidar_avoid_phase_cnt_ = 0;
                    RCLCPP_WARN(this->get_logger(),
                        "[AVOID_FORWARD] failsafe return, cone_side=%.2f", cone_side);
                }
            }

            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500,
                "[AVOID_FORWARD] cnt=%d risk=%.2f release=%.2f side=%.2f sideL=%.2f sideR=%.2f -> vx=%.2f wz=%.2f",
                lidar_avoid_phase_cnt_, forward_risk, release_threshold, side_risk,
                lidar_min_left_side_, lidar_min_right_side_,
                cmd.linear.x, cmd.angular.z);
        }
        else if (lidar_avoid_phase_ == AVOID_RETURN)
        {
            lidar_avoid_blend_override_ = -1.0;
            lidar_use_heading_return_ = true;  // steer back toward original heading
            cmd = compute_teb_lite_avoidance(base_cmd, forward_risk);
            lidar_avoid_phase_cnt_++;

            double ret_heading_err = normalize_angle(avoidance_start_heading_ - robot_heading_);
            if (lidar_min_center_ < dynamic_enter)
            {
                lidar_avoid_phase_ = AVOID_TURN;
                lidar_avoid_phase_cnt_ = 0;
                lidar_use_heading_return_ = false;
                select_avoid_heading();
                RCLCPP_INFO(this->get_logger(), "[AVOID_RETURN->TURN] new obstacle min_C=%.2f ftg=%d head=%.2f",
                            lidar_min_center_, lidar_ftg_valid_ ? 1 : 0, lidar_local_target_heading_);
            }
            else if (lidar_avoid_phase_cnt_ >= avoid_return_cycles_ ||
                     (lidar_avoid_phase_cnt_ >= 3 && std::abs(ret_heading_err) < 0.12))
            {
                lidar_avoid_active_ = false;
                lidar_avoid_phase_ = AVOID_IDLE;
                lidar_avoid_phase_cnt_ = 0;
                lidar_avoid_blend_override_ = -1.0;
                lidar_use_heading_return_ = false;
                lidar_blend_total_cycles_ = compute_blend_cycles(vx);
                lidar_blend_counter_ = lidar_blend_total_cycles_;
                lidar_blend_start_wz_ = lidar_avoid_wz_ema_;
                RCLCPP_INFO(this->get_logger(),
                    "[AVOID_RETURN->BLEND] complete, heading_err=%.2f blend cnt=%d",
                    ret_heading_err, lidar_blend_total_cycles_);
            }

            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500,
                "[AVOID_RETURN] cnt=%d/%d heading_err=%.2f -> vx=%.2f wz=%.2f",
                lidar_avoid_phase_cnt_, avoid_return_cycles_, ret_heading_err,
                cmd.linear.x, cmd.angular.z);
        }
        else
        {
            lidar_avoid_blend_override_ = -1.0;
            lidar_use_heading_return_ = false;
            cmd = compute_teb_lite_avoidance(base_cmd, forward_risk);
        }

        return cmd;
    }

    // Compute LiDAR avoidance command for state 2
    geometry_msgs::msg::Twist compute_lidar_avoidance()
    {
        // Use the vision line-following command as the path reference so the
        // local plan stays glued to the track instead of assuming "straight".
        geometry_msgs::msg::Twist base_cmd;
        base_cmd.linear.x = lidar_avoid_speed_;
        base_cmd.angular.z = racing_cmd_.angular.z;

        double min_front = lidar_min_center_;
        double side_risk = std::min(lidar_min_left_outer_, lidar_min_right_outer_);
        double left_score = lidar_left_score_;
        double right_score = lidar_right_score_;
        double speed_term_enter = lidar_speed_trigger_gain_ * std::max(0.0, base_cmd.linear.x - 0.18);
        double speed_term_clear = lidar_speed_clear_gain_ * std::max(0.0, base_cmd.linear.x - 0.18);
        double dynamic_enter = lidar_avoid_distance_ + speed_term_enter;
        double dynamic_clear = lidar_avoid_clear_distance_ + speed_term_clear;
        double release_threshold = dynamic_clear + lidar_bypass_release_margin_;

        if (!lidar_avoid_active_ && std::min(min_front, side_risk) < dynamic_enter)
        {
            lidar_avoid_active_ = true;
            lidar_blend_counter_ = 0;
            lidar_blend_total_cycles_ = 0;
            if (lidar_ftg_valid_)
            {
                lidar_avoid_direction_ = (lidar_ftg_heading_ >= 0.0) ? 1 : -1;
                lidar_local_target_heading_ = lidar_ftg_heading_;
            }
            else
            {
                lidar_avoid_direction_ = (left_score >= right_score) ? 1 : -1;
                lidar_local_target_heading_ = lidar_avoid_direction_ > 0 ? lidar_left_free_heading_ : lidar_right_free_heading_;
            }
            lidar_avoid_phase_ = AVOID_TURN;
            lidar_avoid_phase_cnt_ = 0;
            lidar_use_heading_return_ = false;
            avoidance_start_heading_ = robot_heading_;
            RCLCPP_INFO(this->get_logger(),
                "[State2 AVOID START] heading=%.2f dir=%s ftg=%d ftg_head=%.2f min_C=%.2f side=%.2f enter=%.2f speed=%.2f",
                robot_heading_, lidar_avoid_direction_ > 0 ? "L" : "R",
                lidar_ftg_valid_ ? 1 : 0, lidar_ftg_heading_,
                min_front, side_risk, dynamic_enter, base_cmd.linear.x);
        }

        // Near-trigger warning for debugging
        if (!lidar_avoid_active_ && std::min(min_front, side_risk) < dynamic_enter * 1.3)
        {
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 300,
                "[State2 NEAR] min_C=%.2f side=%.2f enter=%.2f ratio=%.2f min_LO=%.2f min_RO=%.2f",
                min_front, side_risk, dynamic_enter,
                std::min(min_front, side_risk) / std::max(0.01, dynamic_enter),
                lidar_min_left_outer_, lidar_min_right_outer_);
        }

        if (!lidar_avoid_active_)
        {
            if (lidar_blend_counter_ > 0)
            {
                double alpha = 1.0 - static_cast<double>(lidar_blend_counter_) /
                    static_cast<double>(std::max(1, lidar_blend_total_cycles_));
                // Blend from avoidance wz back to the line-following wz
                double blended_wz = lidar_blend_start_wz_ * (1.0 - alpha) + racing_cmd_.angular.z * alpha;
                base_cmd.angular.z = blended_wz;
                lidar_blend_counter_--;
                if (lidar_blend_counter_ <= 0)
                    lidar_blend_total_cycles_ = 0;
                lidar_avoid_wz_ema_ = 0.85 * lidar_avoid_wz_ema_ + 0.15 * blended_wz;
                RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500,
                    "[State2 BLEND] cnt=%d alpha=%.2f race_wz=%.2f blend_wz=%.2f",
                    lidar_blend_counter_, alpha, racing_cmd_.angular.z, blended_wz);
                return base_cmd;
            }
            lidar_avoid_wz_ema_ = 0.85 * lidar_avoid_wz_ema_ + 0.15 * racing_cmd_.angular.z;
            return base_cmd;
        }

        double forward_risk = std::min(min_front, side_risk);
        geometry_msgs::msg::Twist cmd = compute_phased_avoidance(
            base_cmd, forward_risk, side_risk,
            dynamic_clear, release_threshold, dynamic_enter);
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500,
            "[State2 AVOID] phase=%d dir=%s min_C=%.2f min_LO=%.2f min_RO=%.2f -> vx=%.2f wz=%.2f",
            static_cast<int>(lidar_avoid_phase_), lidar_avoid_direction_ > 0 ? "L" : "R",
            lidar_min_center_, lidar_min_left_outer_, lidar_min_right_outer_,
            cmd.linear.x, cmd.angular.z);
        return cmd;
    }

    void timer_callback()
    {
        std::lock_guard<std::mutex> lock(mutex_);
        geometry_msgs::msg::Twist output_cmd;

        // === Odometry nav phase: forward /nav_cmd_vel with LiDAR override ===
        if (!vision_active_)
        {
            if (!nav_cmd_fresh_)
                return;  // No nav commands yet, stay silent

            double min_l = lidar_min_left_;
            double min_r = lidar_min_right_;
            double min_front = lidar_min_center_;
            double min_l_outer = lidar_min_left_outer_;
            double min_r_outer = lidar_min_right_outer_;
            double left_score = lidar_left_score_;
            double right_score = lidar_right_score_;
            double turn_mag = std::min(1.0, std::abs(nav_cmd_.angular.z) / 1.2);
            double speed_term_enter = lidar_speed_trigger_gain_ * std::max(0.0, nav_cmd_.linear.x - 0.18);
            double speed_term_clear = lidar_speed_clear_gain_ * std::max(0.0, nav_cmd_.linear.x - 0.18);
            double dynamic_enter = lidar_avoid_distance_ + lidar_turn_trigger_gain_ * turn_mag + speed_term_enter;
            double dynamic_clear = lidar_avoid_clear_distance_ + lidar_turn_clear_gain_ * turn_mag + speed_term_clear;
            double release_threshold = dynamic_clear + lidar_bypass_release_margin_;
            double forward_risk = min_front;  // only front-center triggers avoidance

            if (nav_cmd_.angular.z > 0.10)
            {
                left_score += 0.08;
                if (min_l_outer < dynamic_enter)
                    left_score -= 0.35;
            }
            else if (nav_cmd_.angular.z < -0.10)
            {
                right_score += 0.08;
                if (min_r_outer < dynamic_enter)
                    right_score -= 0.35;
            }

            if (nav_cmd_.linear.x <= 0.05)
            {
                lidar_avoid_active_ = false;
                lidar_blend_counter_ = 0;
                lidar_blend_total_cycles_ = 0;
                lidar_avoid_wz_ema_ = nav_cmd_.angular.z;
                lidar_local_target_heading_ = 0.0;
                lidar_avoid_phase_ = AVOID_IDLE;
                lidar_avoid_phase_cnt_ = 0;
                lidar_use_heading_return_ = false;
                output_cmd = nav_cmd_;
                cmd_vel_pub_->publish(output_cmd);
                return;
            }

            if (enable_lidar_avoid_ && !lidar_avoid_active_ && forward_risk < dynamic_enter)
            {
                lidar_avoid_active_ = true;
                lidar_blend_counter_ = 0;
                lidar_blend_total_cycles_ = 0;
                if (lidar_ftg_valid_)
                {
                    lidar_avoid_direction_ = (lidar_ftg_heading_ >= 0.0) ? 1 : -1;
                    lidar_local_target_heading_ = lidar_ftg_heading_;
                }
                else
                {
                    lidar_avoid_direction_ = (left_score >= right_score) ? 1 : -1;
                    lidar_local_target_heading_ = lidar_avoid_direction_ > 0 ? lidar_left_free_heading_ : lidar_right_free_heading_;
                }
                lidar_avoid_phase_ = AVOID_TURN;
                lidar_avoid_phase_cnt_ = 0;
                lidar_use_heading_return_ = false;
                avoidance_start_heading_ = robot_heading_;
                RCLCPP_INFO(this->get_logger(),
                    "[Odom AVOID START] heading=%.2f dir=%s min_C=%.2f enter=%.2f nav_vx=%.2f nav_wz=%.2f turn_mag=%.2f",
                    robot_heading_, lidar_avoid_direction_ > 0 ? "L" : "R",
                    min_front, dynamic_enter, nav_cmd_.linear.x, nav_cmd_.angular.z, turn_mag);
            }

            // Near-trigger warning for debugging
            if (enable_lidar_avoid_ && !lidar_avoid_active_ && forward_risk < dynamic_enter * 1.3)
            {
                RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 300,
                    "[NEAR TRIGGER] min_C=%.2f enter=%.2f ratio=%.2f turn_mag=%.2f nav_wz=%.2f min_LO=%.2f min_RO=%.2f",
                    min_front, dynamic_enter, min_front / std::max(0.01, dynamic_enter),
                    turn_mag, nav_cmd_.angular.z, min_l_outer, min_r_outer);
            }

            if (enable_lidar_avoid_ && lidar_avoid_active_)
            {
                double side_risk = std::min(min_l_outer, min_r_outer);
                output_cmd = compute_phased_avoidance(
                    nav_cmd_, forward_risk, side_risk,
                    dynamic_clear, release_threshold, dynamic_enter);

                RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500,
                    "[LiDAR AVOID] phase=%d dir=%s risk=%.2f enter=%.2f clear=%.2f release=%.2f min_C=%.2f min_LO=%.2f min_RO=%.2f -> vx=%.2f wz=%.2f",
                    static_cast<int>(lidar_avoid_phase_), lidar_avoid_direction_ > 0 ? "L" : "R",
                    forward_risk, dynamic_enter, dynamic_clear, release_threshold,
                    min_front, min_l_outer, min_r_outer,
                    output_cmd.linear.x, output_cmd.angular.z);
            }
            else
            {
                output_cmd = nav_cmd_;
                if (lidar_blend_counter_ > 0)
                {
                    double alpha = 1.0 - static_cast<double>(lidar_blend_counter_) /
                        static_cast<double>(std::max(1, lidar_blend_total_cycles_));
                    double blended_wz = lidar_blend_start_wz_ * (1.0 - alpha) + nav_cmd_.angular.z * alpha;
                    output_cmd.angular.z = blended_wz;
                    lidar_blend_counter_--;
                    if (lidar_blend_counter_ <= 0)
                        lidar_blend_total_cycles_ = 0;
                    if (min_l < lidar_side_guard_distance_ && forward_risk > dynamic_enter)
                        output_cmd.angular.z = std::max(-1.0, output_cmd.angular.z - 0.45 * (lidar_side_guard_distance_ - min_l) / lidar_side_guard_distance_);
                    else if (min_r < lidar_side_guard_distance_ && forward_risk > dynamic_enter)
                        output_cmd.angular.z = std::min(1.0, output_cmd.angular.z + 0.45 * (lidar_side_guard_distance_ - min_r) / lidar_side_guard_distance_);
                    lidar_avoid_wz_ema_ = 0.85 * lidar_avoid_wz_ema_ + 0.15 * blended_wz;
                    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500,
                        "control_master: BLEND cnt=%d alpha=%.2f nav_wz=%.2f blend_wz=%.2f -> wz=%.2f",
                        lidar_blend_counter_, alpha, nav_cmd_.angular.z, blended_wz, output_cmd.angular.z);
                }
                else
                {
                    if (min_l < lidar_side_guard_distance_ && forward_risk > dynamic_enter)
                        output_cmd.angular.z = std::max(-1.0, output_cmd.angular.z - 0.45 * (lidar_side_guard_distance_ - min_l) / lidar_side_guard_distance_);
                    else if (min_r < lidar_side_guard_distance_ && forward_risk > dynamic_enter)
                        output_cmd.angular.z = std::min(1.0, output_cmd.angular.z + 0.45 * (lidar_side_guard_distance_ - min_r) / lidar_side_guard_distance_);
                    lidar_avoid_wz_ema_ = 0.85 * lidar_avoid_wz_ema_ + 0.15 * nav_cmd_.angular.z;
                    lidar_local_target_heading_ = 0.0;
                    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                        "control_master: odom pass-through vx=%.2f wz=%.2f min_C=%.2f min_L=%.2f min_R=%.2f min_LO=%.2f min_RO=%.2f heading=%.2f",
                        output_cmd.linear.x, output_cmd.angular.z, min_front, min_l, min_r, min_l_outer, min_r_outer, robot_heading_);
                }
            }
            cmd_vel_pub_->publish(output_cmd);
            return;
        }
        if (qrcode_detected_ || p_detected_)
        {
            output_cmd.linear.x = 0.0;
            output_cmd.angular.z = 0.0;
        }
        else
        {
            output_cmd = enable_lidar_avoid_ ? compute_lidar_avoidance() : racing_cmd_;
        }

        cmd_vel_pub_->publish(output_cmd);
    }

    std::mutex mutex_;
    geometry_msgs::msg::Twist racing_cmd_ = geometry_msgs::msg::Twist();
    geometry_msgs::msg::Twist nav_cmd_ = geometry_msgs::msg::Twist();
    bool nav_cmd_fresh_ = false;
    bool qrcode_detected_;
    bool p_detected_;
    bool vision_active_;
    bool enable_lidar_avoid_ = false;
    double lidar_avoid_speed_;
    double lidar_avoid_distance_;
    double lidar_avoid_clear_distance_;
    double lidar_front_angle_;
    double lidar_center_angle_;
    double lidar_side_guard_distance_;
    double lidar_turn_trigger_gain_;
    double lidar_turn_clear_gain_;
    double lidar_speed_trigger_gain_;
    double lidar_speed_clear_gain_;
    double lidar_bypass_release_margin_;
    double teb_lite_horizon_;
    double teb_lite_dt_;
    double teb_lite_safety_radius_;
    double teb_lite_collision_radius_;
    double teb_lite_max_wz_;
    double teb_lite_obstacle_weight_;
    double lidar_min_left_ = 1e9;
    double lidar_min_right_ = 1e9;
    double lidar_min_center_ = 1e9;
    double lidar_min_left_outer_ = 1e9;
    double lidar_min_right_outer_ = 1e9;
    double lidar_left_free_heading_ = 0.0;
    double lidar_right_free_heading_ = 0.0;
    double lidar_left_free_range_ = 0.0;
    double lidar_right_free_range_ = 0.0;
    double lidar_left_score_ = -1.0;
    double lidar_right_score_ = -1.0;
    bool lidar_avoid_active_ = false;
    int lidar_avoid_direction_ = 1;
    double lidar_local_target_heading_ = 0.0;
    double lidar_avoid_wz_ema_ = 0.0;
    int lidar_blend_counter_ = 0;
    int lidar_blend_total_cycles_ = 0;
    double lidar_blend_start_wz_ = 0.0;
    // Phased avoidance (S-curve)
    enum AvoidPhase { AVOID_IDLE, AVOID_TURN, AVOID_FORWARD, AVOID_RETURN };
    AvoidPhase lidar_avoid_phase_ = AVOID_IDLE;
    int lidar_avoid_phase_cnt_ = 0;
    int avoid_turn_cycles_ = 10;
    int avoid_forward_min_cycles_ = 6;
    int avoid_forward_max_cycles_ = 20;
    int avoid_return_cycles_ = 10;
    double lidar_avoid_blend_override_ = -1.0;  // >=0 overrides TEB-lite blend
    // Robot pose from odometry
    double robot_heading_ = 0.0;
    // True-side minima (beyond front sector), used to gate the return phase
    double lidar_min_left_side_ = 1e9;
    double lidar_min_right_side_ = 1e9;
    // Heading-based return: steer back toward the heading when avoidance started
    double avoidance_start_heading_ = 0.0;
    bool lidar_use_heading_return_ = false;
    // Follow-the-Gap (F1TENTH) state
    double ftg_bubble_radius_ = 0.32;
    double ftg_gap_min_range_ = 0.85;
    double ftg_max_range_ = 3.0;
    bool lidar_ftg_valid_ = false;
    bool lidar_ftg_valid_prev_ = false;
    double lidar_ftg_heading_ = 0.0;
    std::vector<std::pair<double, double>> lidar_obstacles_;

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr p_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr qrcode_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr racing_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr nav_cmd_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr lidar_pub_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr subscriber_car_go_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr vision_enable_sub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ChassisController>();

    // On Ctrl-C, push a stop onto /cmd_vel before the context tears down,
    // otherwise the chassis driver keeps running the last latched velocity.
    auto pre_shutdown_handle =
        node->get_node_base_interface()->get_context()->add_pre_shutdown_callback(
            [node]() { node->stop(); });

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
