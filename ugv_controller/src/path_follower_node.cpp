#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2/exceptions.h"

#include "std_msgs/msg/bool.hpp"

#include <fstream>
#include <sstream>
#include <vector>
#include <string>
#include <cmath>
#include <memory>
#include <algorithm>
#include <chrono>
#include <optional>

class PID {
public:
    PID(double Kp, double Ki, double Kd) : Kp_(Kp), Ki_(Ki), Kd_(Kd), prev_error_(0.0), integral_(0.0) {}
    double compute(double error, double dt) {
        integral_ += error * dt;
        double derivative = dt > 0.0 ? (error - prev_error_) / dt : 0.0;
        double output = Kp_ * error + Ki_ * integral_ + Kd_ * derivative;
        prev_error_ = error;
        return output;
    }
private:
    double Kp_, Ki_, Kd_, prev_error_, integral_;
};

struct Waypoint {
    double x, y;
    int mission_state;
    double curvature{0.0};
};

class PathFollower : public rclcpp::Node {
public:
    PathFollower() : Node("path_follower"), current_target_index_(0), pose_received_(false),
      mission_finished_(false), is_ready_to_start_(false), 
      takeoff_signal_received_(false), takeoff_command_sent_(false),
      is_stopping_for_takeoff_(false), is_waiting_for_takeoff_signal_(false),
      pid_speed_(
          this->declare_parameter<double>("speed.kp", 1.5),
          this->declare_parameter<double>("speed.ki", 0.0),
          this->declare_parameter<double>("speed.kd", 0.2)
      ),
      pid_steer_(
          this->declare_parameter<double>("steer.kp", 4.0),
          this->declare_parameter<double>("steer.ki", 0.0),
          this->declare_parameter<double>("steer.kd", 0.1)
      )
    {
        // 기본 파라미터 선언
        this->declare_parameter<std::string>("mission_file", "mission.csv");
        this->declare_parameter<double>("max_speed", 5.5);
        this->declare_parameter<double>("waypoint_threshold", 0.7);
        this->declare_parameter<double>("speed_smoothing_factor", 0.08);
        this->declare_parameter<double>("curvature_gain", 1.5);

        // [추가] 정차 로직 파라미터 선언
        this->declare_parameter<double>("stop_logic.takeoff_cmd_speed_threshold", 3.0);
        this->declare_parameter<double>("stop_logic.deceleration_multiplier", 2.0);
        this->declare_parameter<double>("stop_logic.stop_speed_threshold", 0.05);

        // [추가] 속도 프로파일 파라미터 선언
        this->declare_parameter<double>("speed_profile.section_1_speed", 4.0);
        this->declare_parameter<double>("speed_profile.section_2_speed", 5.5);
        this->declare_parameter<double>("speed_profile.section_3_speed", 3.0);
        this->declare_parameter<double>("speed_profile.section_4_speed", 2.0);
        this->declare_parameter<double>("speed_profile.final_section_speed", 1.0);


        // 파라미터 값 가져오기
        this->get_parameter("mission_file", mission_file_);
        this->get_parameter("max_speed", max_speed_);
        this->get_parameter("waypoint_threshold", waypoint_threshold_);
        this->get_parameter("speed_smoothing_factor", speed_smoothing_factor_);
        this->get_parameter("curvature_gain", curvature_gain_);

        // [추가] 새로 선언한 파라미터 값 가져오기
        this->get_parameter("stop_logic.takeoff_cmd_speed_threshold", takeoff_cmd_speed_threshold_);
        this->get_parameter("stop_logic.deceleration_multiplier", deceleration_multiplier_);
        this->get_parameter("stop_logic.stop_speed_threshold", stop_speed_threshold_);
        this->get_parameter("speed_profile.section_1_speed", speed_profile_sec1_);
        this->get_parameter("speed_profile.section_2_speed", speed_profile_sec2_);
        this->get_parameter("speed_profile.section_3_speed", speed_profile_sec3_);
        this->get_parameter("speed_profile.section_4_speed", speed_profile_sec4_);
        this->get_parameter("speed_profile.final_section_speed", speed_profile_final_);


        cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/model/X1_asp/cmd_vel", 10);
        pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("current_pose", 10);
        marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("target_marker", 10);

        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        takeoff_pub_ = this->create_publisher<std_msgs::msg::Bool>("/command/takeoff", rclcpp::QoS(10));
        
        rendezvous_pub_ = this->create_publisher<std_msgs::msg::Bool>(
            "/command/rendezvous",
            rclcpp::QoS(rclcpp::KeepLast(1)).transient_local()
        );
        
        takeoff_status_sub_ = this->create_subscription<std_msgs::msg::Bool>(
            "/status/takeoff_complete",
            rclcpp::QoS(1).transient_local(),
            std::bind(&PathFollower::takeoff_status_callback, this, std::placeholders::_1)
        );

        drone_ready_sub_ = this->create_subscription<std_msgs::msg::Bool>(
            "/status/drone_ready",
            rclcpp::QoS(1).transient_local(),
            std::bind(&PathFollower::drone_ready_callback, this, std::placeholders::_1)
        );

        timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&PathFollower::control_loop, this));
        
        RCLCPP_INFO(this->get_logger(), "Path Follower node started. Waiting for drone ready signal on /status/drone_ready...");
        load_and_process_path();
    }

private:
    // 이 아래 함수들은 그대로 유지됩니다.
    void drone_ready_callback(const std_msgs::msg::Bool::SharedPtr msg) {
        if (msg->data && !is_ready_to_start_) {
            RCLCPP_INFO(this->get_logger(), "Received drone_ready signal! Vehicle mission will now start.");
            is_ready_to_start_ = true; 
        }
    }

    void takeoff_status_callback(const std_msgs::msg::Bool::SharedPtr msg) {
        if (msg->data && !takeoff_signal_received_) {
            RCLCPP_INFO(this->get_logger(), "Received UAV takeoff complete signal.");
            takeoff_signal_received_ = true;

            if (is_stopping_for_takeoff_) {
                RCLCPP_INFO(this->get_logger(), "Signal received while stopping. Aborting stop and proceeding to next waypoint.");
                is_stopping_for_takeoff_ = false;
                current_target_index_++;         
            }
            else if (is_waiting_for_takeoff_signal_) {
                RCLCPP_INFO(this->get_logger(), "Signal received while waiting. Proceeding to next waypoint.");
                is_waiting_for_takeoff_signal_ = false;
                current_target_index_++;               
            }
        }
    }

    double calculate_curvature(const Waypoint& p_prev, const Waypoint& p_curr, const Waypoint& p_next) {
        double dx1 = p_curr.x - p_prev.x;
        double dy1 = p_curr.y - p_prev.y;
        double dx2 = p_next.x - p_curr.x;
        double dy2 = p_next.y - p_curr.y;
        double area = std::abs(dx1 * dy2 - dx2 * dy1);
        double dist1 = std::hypot(dx1, dy1);
        double dist2 = std::hypot(dx2, dy2);
        double dist3 = std::hypot(p_next.x - p_prev.x, p_next.y - p_prev.y);
        if (dist1 < 1e-6 || dist2 < 1e-6 || dist3 < 1e-6) return 0.0;
        return 4.0 * area / (dist1 * dist2 * dist3);
    }

    void load_and_process_path() {
        path_.clear();
        current_target_index_ = 0;
        mission_2_waypoint_index_.reset();

        std::vector<Waypoint> temp_path;
        std::ifstream file(mission_file_);
        if (!file.is_open()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open path file: %s", mission_file_.c_str());
            rclcpp::shutdown();
            return;
        }
        std::string line;
        while (std::getline(file, line)) {
            std::stringstream ss(line);
            std::string x_str, y_str, state_str;
            if (std::getline(ss, x_str, ',') && std::getline(ss, y_str, ',') && std::getline(ss, state_str)) {
                temp_path.push_back({std::stod(x_str), std::stod(y_str), std::stoi(state_str), 0.0});
            }
        }

        if (temp_path.size() < 3) {
            path_ = temp_path;
        } else {
            path_ = temp_path;
            for (size_t i = 1; i < path_.size() - 1; ++i) {
                path_[i].curvature = calculate_curvature(path_[i-1], path_[i], path_[i+1]);
            }
        }
        
        for (size_t i = 0; i < path_.size(); ++i) {
            if (path_[i].mission_state == 2) {
                mission_2_waypoint_index_ = i;
                RCLCPP_INFO(this->get_logger(), "Takeoff-related waypoint (mission_state=2) found at index %zu", i);
                break;
            }
        }

        RCLCPP_INFO(this->get_logger(), "Loaded and processed %zu waypoints from %s.", path_.size(), mission_file_.c_str());
    }

    void control_loop() {
        if (!is_ready_to_start_) {
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "Waiting for '/status/drone_ready' topic to be true...");
            return;
        }

        geometry_msgs::msg::TransformStamped t;
        try {
            t = tf_buffer_->lookupTransform("map", "X1_asp/base_link", tf2::TimePointZero);
        } catch (const tf2::TransformException & ex) {
            RCLCPP_WARN(this->get_logger(), "Could not get vehicle transform: %s", ex.what());
            return;
        }
        current_x_ = t.transform.translation.x;
        current_y_ = t.transform.translation.y;
        tf2::Quaternion q;
        tf2::fromMsg(t.transform.rotation, q);
        double roll, pitch;
        tf2::Matrix3x3(q).getRPY(roll, pitch, current_yaw_);

        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Pos: (%.1f, %.1f), Vel: %.1f m/s, Target WP: %zu", current_x_, current_y_, smoothed_speed_, current_target_index_);

        if (!mission_finished_) {
            if (is_waiting_for_takeoff_signal_) {
                RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "Stopped at waypoint. Waiting for UAV takeoff complete signal...");
                stop_vehicle();
                return;
            }
            
            if (is_stopping_for_takeoff_) {
                RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Slowing down to stop at waypoint...");
                
                // [수정] 하드코딩된 값(1.0)을 파라미터로 대체
                if (!takeoff_command_sent_ && smoothed_speed_ < takeoff_cmd_speed_threshold_) {
                    RCLCPP_INFO(this->get_logger(), "Speed is below %.1f m/s. Sending takeoff command to UAV!", takeoff_cmd_speed_threshold_);
                    std_msgs::msg::Bool takeoff_msg;
                    takeoff_msg.data = true;
                    takeoff_pub_->publish(takeoff_msg);
                    takeoff_command_sent_ = true;
                }

                // [수정] 하드코딩된 값(2.0)을 파라미터로 대체
                smoothed_speed_ *= (1.0 - speed_smoothing_factor_ * deceleration_multiplier_);
                
                geometry_msgs::msg::Twist cmd;
                cmd.linear.x = smoothed_speed_;
                cmd.angular.z = 0.0; 
                cmd_pub_->publish(cmd);

                // [수정] 하드코딩된 값(0.05)을 파라미터로 대체
                if (smoothed_speed_ < stop_speed_threshold_) {
                    RCLCPP_INFO(this->get_logger(), "Vehicle stop complete.");
                    is_stopping_for_takeoff_ = false;
                    is_waiting_for_takeoff_signal_ = true;
                }
                return;
            }

            if (current_target_index_ >= path_.size()) {
                RCLCPP_INFO(this->get_logger(), "All missions finished! Sending rendezvous command.");
                stop_vehicle();
                std_msgs::msg::Bool msg;
                msg.data = true;
                rendezvous_pub_->publish(msg);
                mission_finished_ = true;
            } else { 
                const auto& target_wp = path_[current_target_index_];
                double dist_to_target = std::hypot(target_wp.x - current_x_, target_wp.y - current_y_);

                if (dist_to_target < waypoint_threshold_) {
                    RCLCPP_INFO(this->get_logger(), "Reached waypoint #%zu.", current_target_index_);

                    if (mission_2_waypoint_index_.has_value() && current_target_index_ == mission_2_waypoint_index_.value()) {
                        if (!takeoff_signal_received_) {
                            RCLCPP_INFO(this->get_logger(), "Initiating stop procedure at waypoint #%zu.", current_target_index_);
                            is_stopping_for_takeoff_ = true;
                        } else {
                            RCLCPP_INFO(this->get_logger(), "Takeoff signal already received. Skipping stop procedure.");
                            current_target_index_++;
                        }
                    } else {
                        current_target_index_++;
                    }
                } else {
                    bool should_decelerate = mission_2_waypoint_index_.has_value() && current_target_index_ == mission_2_waypoint_index_.value();
                    drive_to_waypoint(target_wp, should_decelerate);
                }
                publish_visualizations(target_wp.x, target_wp.y);
            }
        }
    }

    void drive_to_waypoint(const Waypoint& target, bool should_decelerate) {
        double ex = target.x - current_x_;
        double ey = target.y - current_y_;
        double dist = std::hypot(ex, ey);
        double target_ang = std::atan2(ey, ex);
        double ang_err = std::atan2(std::sin(target_ang - current_yaw_), std::cos(target_ang - current_yaw_));
        double dt = 0.1;
        double target_speed;

        if (should_decelerate) {
            target_speed = pid_speed_.compute(dist, dt);
        } else {
            // [수정] 하드코딩된 속도 값들을 파라미터로 대체
            if (current_target_index_ >= 3 && current_target_index_ <= 5) {
                max_speed_ = speed_profile_sec1_;
                double curvature = target.curvature;
                target_speed = max_speed_ / (1.0 + curvature_gain_ * curvature);
            }
            else if (current_target_index_ >= 6 && current_target_index_ <= 8) {
                max_speed_ = speed_profile_sec2_;
                target_speed = speed_profile_sec2_;
            }
            else if (current_target_index_ >=9 and current_target_index_ <=14){
                target_speed = speed_profile_sec3_;
            }
            else if (current_target_index_ >=15 and current_target_index_ <=21){
                target_speed = speed_profile_sec4_;
            }
            else if (current_target_index_ >= 23) {
                target_speed = speed_profile_final_;
            } 
            else {
                double curvature = target.curvature;
                target_speed = max_speed_ / (1.0 + curvature_gain_ * curvature);
            }
        }

        double raw_speed_cmd = std::clamp(target_speed, 0.0, max_speed_);
        double steer_cmd = pid_steer_.compute(ang_err, dt);
        smoothed_speed_ = speed_smoothing_factor_ * raw_speed_cmd + (1.0 - speed_smoothing_factor_) * smoothed_speed_;
        geometry_msgs::msg::Twist cmd;
        cmd.linear.x = smoothed_speed_;
        cmd.angular.z = steer_cmd;
        cmd_pub_->publish(cmd);
    }

    void stop_vehicle() {
        smoothed_speed_ = 0.0;
        geometry_msgs::msg::Twist cmd;
        cmd.linear.x = 0.0;
        cmd.angular.z = 0.0;
        cmd_pub_->publish(cmd);
    }

    void publish_visualizations(double goal_x, double goal_y) {
        geometry_msgs::msg::PoseStamped pose_msg;
        pose_msg.header.stamp = this->now();
        pose_msg.header.frame_id = "map";
        pose_msg.pose.position.x = current_x_;
        pose_msg.pose.position.y = current_y_;
        tf2::Quaternion q;
        q.setRPY(0, 0, current_yaw_);
        pose_msg.pose.orientation = tf2::toMsg(q);
        pose_pub_->publish(pose_msg);

        visualization_msgs::msg::Marker marker;
        marker.header = pose_msg.header;
        marker.ns = "target";
        marker.id = 0;
        marker.type = visualization_msgs::msg::Marker::SPHERE;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.pose.position.x = goal_x;
        marker.pose.position.y = goal_y;
        marker.scale.x = 0.5; marker.scale.y = 0.5; marker.scale.z = 0.5;
        marker.color.a = 1.0; marker.color.r = 1.0; marker.color.g = 0.0; marker.color.b = 0.0;
        marker_pub_->publish(marker);
    }

    // 멤버 변수
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr takeoff_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr rendezvous_pub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr takeoff_status_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr drone_ready_sub_;

    std::string mission_file_;
    std::vector<Waypoint> path_;
    std::optional<size_t> mission_2_waypoint_index_;

    size_t current_target_index_;
    bool pose_received_;
    bool mission_finished_;
    bool is_ready_to_start_;
    bool takeoff_signal_received_;
    bool takeoff_command_sent_;
    bool is_stopping_for_takeoff_;
    bool is_waiting_for_takeoff_signal_;

    double current_x_{0.0}, current_y_{0.0}, current_yaw_{0.0};
    double max_speed_, waypoint_threshold_;
    PID pid_speed_, pid_steer_;

    double smoothed_speed_{0.0};
    double speed_smoothing_factor_;
    double curvature_gain_;

    // [추가] 새로 추가된 파라미터를 저장할 멤버 변수
    double takeoff_cmd_speed_threshold_;
    double deceleration_multiplier_;
    double stop_speed_threshold_;
    double speed_profile_sec1_;
    double speed_profile_sec2_;
    double speed_profile_sec3_;
    double speed_profile_sec4_;
    double speed_profile_final_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PathFollower>());
    rclcpp::shutdown();
    return 0;
}
