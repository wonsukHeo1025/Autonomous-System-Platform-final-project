// uav_final.cpp
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/msg/pose.hpp>
#include <chrono>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include <std_msgs/msg/bool.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <string>
#include <vector>
#include <fstream>
#include <sstream>
#include <cmath> // sqrt 사용을 위해
#include <filesystem>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>               // ArUco 헤더 추가
#include <px4_msgs/msg/vehicle_status.hpp> // <<< 이 헤더를 추가합니다.
#include <std_msgs/msg/float32.hpp>
#include <px4_msgs/msg/vehicle_land_detected.hpp>

using namespace std::chrono_literals;

enum class RendezvousStep
{
  GO_TO_AREA,         // 초기 랑데부 지역으로 이동
  WAIT_FOR_STABLE,    // <-- 새로운 안정화 대기 단계 추가
  TRACK_AND_APPROACH, // 마커 추적 및 착륙 지점 접근
  LAND                // 착륙 명령
};

// --- '탐색 액션' 하나를 정의하는 새로운 구조체 ---
struct SearchAction
{
  double drone_yaw_rad = 0.0;
  double gimbal_pitch_deg = 0.0;
  int num_markers_to_find = 0;
};

// 이제 하나의 경로점은 '위치'와 '탐색 액션 목록'을 가집니다.
struct WaypointTask
{
  geometry_msgs::msg::Pose waypoint_pose;
  std::vector<SearchAction> search_actions;
};

// handleSearchForMarker의 세부 단계를 정의
enum class SearchStep
{
  SET_POSE,        // 드론/짐벌 자세 설정
  WAIT_FOR_STABLE, // 자세가 안정화되기를 대기
  SEARCHING,       // 이미지 콜백에서 마커 탐색
};

// 미션 상태 정의 (스코프드 enum)
enum class MissionPhase
{
  Init = 0,        // 부팅 대기
  TakeOff,         // 이륙
  PlanRoute,       // 경로 계획
  FlyToWaypoint,   // 웨이포인트 비행
  SearchForMarker, // 마커 검색
  Rendezvous,      // 합류
  Completed        // 미션 종료
};

class UAVController : public rclcpp::Node
{
public:
  UAVController()
      : Node("uav_controller")
  {
    // ─── 0. Parameter declaration & initialization ───────────────────
    // (These names & defaults must match your YAML)
    target_takeoff_altitude_ = this->declare_parameter<float>("target_takeoff_altitude", 5.0f);
    ugv_start_altitude_threshold_ = this->declare_parameter<float>("ugv_start_altitude_threshold", 5.0f);
    takeoff_altitude_tolerance_ = this->declare_parameter<double>("takeoff_altitude_tolerance", 0.2);
    waypoint_arrival_tolerance_ = this->declare_parameter<double>("waypoint_arrival_tolerance", 0.5);
    STOPPED_VELOCITY_THRESHOLD_ = this->declare_parameter<float>("stopped_velocity_threshold", 0.1f);
    STOPPED_VELOCITY_THRESHOLD_2 = this->declare_parameter<float>("stopped_velocity_threshold2", 0.1f);

    int pose_wait_sec = this->declare_parameter<int>("pose_stable_wait_seconds", 2); 
    int pose_wait_sec2 = this->declare_parameter<int>("pose_stable_wait_seconds2", 2); 
    int marker_timeout_s = this->declare_parameter<int>("marker_search_timeout_seconds", 5);
    pose_stable_wait_duration_ = rclcpp::Duration(std::chrono::seconds(pose_wait_sec));
    pose_stable_wait_duration_2 = rclcpp::Duration(std::chrono::seconds(pose_wait_sec2));
    marker_search_timeout_ = rclcpp::Duration(std::chrono::seconds(marker_timeout_s));

    RENDEZVOUS_WAITING_ALTITUDE = this->declare_parameter<double>("rendezvous_waiting_altitude", 5.0);
    RENDEZVOUS_LANDING_ALTITUDE = this->declare_parameter<double>("rendezvous_landing_altitude", 1.0);
    rendezvous_arrival_tolerance_ = this->declare_parameter<double>("rendezvous_arrival_tolerance", 1.0);
    rendezvous_descent_distance_ = this->declare_parameter<double>("rendezvous_descent_distance", 1.0);

    // ─── 1. Clock & TF 초기화 ───────────────────────────────
    clock_ = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(clock_);
    tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);
    static_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

    // ─── 2. QoS 설정 ───────────────────────────────────────
    auto reliable_qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local();
    auto best_effort_qos = rclcpp::QoS(1).best_effort();
    auto default_qos10 = rclcpp::QoS(10);

    // ─── 3. Subscribers ─────────────────────────────────────
    local_pos_sub_ = this->create_subscription<px4_msgs::msg::VehicleLocalPosition>(
        "/fmu/out/vehicle_local_position",
        rclcpp::SensorDataQoS(),
        std::bind(&UAVController::local_position_callback, this, std::placeholders::_1));

    takeoff_command_sub_ = this->create_subscription<std_msgs::msg::Bool>(
        "/command/takeoff",
        default_qos10,
        std::bind(&UAVController::takeoff_command_callback, this, std::placeholders::_1));

    image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
        "/world/default/model/x500_gimbal_0/link/camera_link/sensor/camera/image",
        best_effort_qos,
        std::bind(&UAVController::image_callback, this, std::placeholders::_1));

    camera_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
        "/world/default/model/x500_gimbal_0/link/camera_link/sensor/camera/camera_info",
        best_effort_qos,
        std::bind(&UAVController::camera_info_callback, this, std::placeholders::_1));

    rendezvous_command_sub_ = this->create_subscription<std_msgs::msg::Bool>(
        "/command/rendezvous",
        reliable_qos,
        std::bind(&UAVController::rendezvous_command_callback, this, std::placeholders::_1));

    land_detected_sub_ = this->create_subscription<px4_msgs::msg::VehicleLandDetected>(
        "/fmu/out/vehicle_land_detected",
        rclcpp::SensorDataQoS(),
        std::bind(&UAVController::land_detected_callback, this, std::placeholders::_1));

    // ─── 4. Publishers ──────────────────────────────────────
    drone_ready_status_pub_ = this->create_publisher<std_msgs::msg::Bool>(
        "/status/drone_ready",
        reliable_qos);

    takeoff_status_pub_ = this->create_publisher<std_msgs::msg::Bool>(
        "/status/takeoff_complete",
        reliable_qos);

    target_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
        "/command/pose",
        default_qos10);

    vehicle_cmd_pub_ = this->create_publisher<px4_msgs::msg::VehicleCommand>(
        "/fmu/in/vehicle_command",
        default_qos10);

    processed_image_pub_ = this->create_publisher<sensor_msgs::msg::Image>(
        "/offboard_control/image_proc",
        best_effort_qos);

    gimbal_pitch_pub_ = this->create_publisher<std_msgs::msg::Float32>(
        "/gimbal_pitch_degree",
        default_qos10);

    land_pub_ = this->create_publisher<std_msgs::msg::Bool>("/command/land", 10);
    disarm_pub_ = this->create_publisher<std_msgs::msg::Bool>("/command/disarm", 10);

    // ─── 5. ArUco 탐지기 초기화 ─────────────────────────────
    aruco_dictionary_ = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);
    aruco_detector_params_ = cv::aruco::DetectorParameters::create();

    // ─── 6. Main Loop Timer ─────────────────────────────────
    timer_ = this->create_wall_timer(
        100ms,
        std::bind(&UAVController::timer_callback, this));
  }

private:
  // ─── Initialization Flags ───────────────────────────
  bool local_origin_set_{false}; // 생성자에서 초기화한 멤버

  // ─── Timers & Clocks & Time ─────────────────────────────
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Clock::SharedPtr clock_;
  rclcpp::Time mission_start_time_;
  rclcpp::Time pose_stable_wait_start_time_;
  rclcpp::Time marker_find_wait_start_time_;

  // ─── Mission State & Phases ─────────────────────────────
  MissionPhase current_phase_{MissionPhase::Init};
  SearchStep current_search_step_{SearchStep::SET_POSE};
  RendezvousStep current_rendezvous_step_{RendezvousStep::GO_TO_AREA};
  bool takeoff_commanded_{false};
  bool ugv_start_signal_sent_{false};
  bool route_planned_{false};
  bool process_image_{false};
  int markers_found_this_action_{0};
  std::vector<int> found_marker_ids_this_action_; // 마커 ID 저장용
  bool rendezvous_command_received_{false};
  bool allow_marker_lock_{false};
  bool mission_time_logged_{false};
  bool is_drone_stable_and_stopped_{false};
  bool is_drone_stable_and_stopped_2{false};

  // ─── Pose & Navigation Data ──────────────────────────────
  geometry_msgs::msg::Pose drone_map_pose_;
  geometry_msgs::msg::Pose takeoff_target_pose_map_;
  geometry_msgs::msg::Pose current_target_pose_map_;
  geometry_msgs::msg::Pose rendezvous_pose_map_;
  std::vector<WaypointTask> mission_route_;
  size_t current_waypoint_index_{0};
  size_t current_search_action_index_{0};

  // ─── TF & Static Transforms ──────────────────────────────
  tf2_ros::Buffer::SharedPtr tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_broadcaster_;

  // ─── Subscriptions ───────────────────────────────────────
  rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr local_pos_sub_;
  px4_msgs::msg::VehicleLocalPosition::SharedPtr px4_local_pos_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr takeoff_command_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr rendezvous_command_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleLandDetected>::SharedPtr land_detected_sub_;

  // ─── Publishers ──────────────────────────────────────────
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr drone_ready_status_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr takeoff_status_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr target_pose_pub_;
  rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr vehicle_cmd_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr processed_image_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr gimbal_pitch_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr land_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr disarm_pub_;

  // ─── Camera & ArUco Detection ────────────────────────────
  cv::Mat camera_matrix_;
  cv::Mat dist_coeffs_;
  cv::Ptr<cv::aruco::Dictionary> aruco_dictionary_;
  cv::Ptr<cv::aruco::DetectorParameters> aruco_detector_params_;
  std::ofstream detection_log_file_;

  // ─── Locking & Optional Values ───────────────────────────
  std::optional<geometry_msgs::msg::Point> locked_marker_position;
  std::optional<double> locked_marker_relative_yaw_;

  // ─── Parameters ─────────────────────────────
  float target_takeoff_altitude_ = 5.0f;
  float ugv_start_altitude_threshold_ = 1.0f;
  double waypoint_arrival_tolerance_ = 0.5;
  rclcpp::Duration pose_stable_wait_duration_{5, 0};
  rclcpp::Duration pose_stable_wait_duration_2{5, 0};
  rclcpp::Duration marker_search_timeout_{5, 0};
  double RENDEZVOUS_WAITING_ALTITUDE = 5.0;
  double RENDEZVOUS_LANDING_ALTITUDE = 1.0;
  float STOPPED_VELOCITY_THRESHOLD_ = 0.1f;
  float STOPPED_VELOCITY_THRESHOLD_2 = 0.1f;
  double takeoff_altitude_tolerance_ = {0.5}; // 이륙 허용 오차 (초기값 0.5m)
  double rendezvous_arrival_tolerance_{1.0};  // 랑데부 도착 허용 반경 (m)
  double rendezvous_descent_distance_{1.2};   // 한 번에 내려갈 거리 (랑데부 착륙 시 수직 하강 스텝, m)

  // ─── Constants ─────────────────────────────
  float marker_size_ = 1.0f;
  double start_takeoff_altitude_ = {0.0};
  bool landed_ = false;

  // ─── System ID Constants ─────────────────────────────────
  static constexpr uint8_t MY_SYSID = 46;
  static constexpr uint8_t MY_COMPID = 47;
  static constexpr uint8_t TARGET_SYSID = 1;
  static constexpr uint8_t TARGET_COMPID = 1;

  // 메인 루프 콜백
  // --- 전체가 수정될 timer_callback 함수 ---
  void timer_callback()
  {
    // 1. 매 루프마다 드론 위치·자세 업데이트
    updateDronePose();

    // 2. 이륙 시작 후 ~ 임무 완료 전까지 매 루프마다 목표 자세 발행 (Offboard 유지)
    if (current_phase_ > MissionPhase::Init && current_phase_ < MissionPhase::Completed)
    {
      publish_target_pose(current_target_pose_map_);
    }

    // 3. 현재 미션 단계에 맞는 핸들러 실행
    switch (current_phase_)
    {
    case MissionPhase::Init:
      handleInit();
      break;
    case MissionPhase::TakeOff:
      handleTakeOff();
      break;
    case MissionPhase::PlanRoute:
      handlePlanRoute();
      break;
    case MissionPhase::FlyToWaypoint:
      handleFlyToWaypoint();
      break;
    case MissionPhase::SearchForMarker:
      handleSearchForMarker();
      break;
    case MissionPhase::Rendezvous:
      handleRendezvous();
      break;
    case MissionPhase::Completed:
      handleCompleted();
      break;
    }
  }

  // 드론의 맵상 위치·자세를 TF에서 가져와서 저장
  void updateDronePose()
  {
    try
    {
      auto t = tf_buffer_->lookupTransform(
          "map",
          "x500_gimbal_0/base_link",
          tf2::TimePointZero);
      drone_map_pose_.position.x = t.transform.translation.x;
      drone_map_pose_.position.y = t.transform.translation.y;
      drone_map_pose_.position.z = t.transform.translation.z;
      drone_map_pose_.orientation = t.transform.rotation;
    }
    catch (const tf2::TransformException &ex)
    {
      RCLCPP_WARN(this->get_logger(), "tf lookup failed: %s", ex.what());
    }
  }

  // --- 전체가 수정될 handleInit 함수 ---
  void handleInit()
  {
    // 1. 드론 준비 작업 (아직 수행되지 않았다면 실행)
    if (!local_origin_set_)
    {
      // PX4 위치가 유효한지 체크
      if (px4_local_pos_ != nullptr && px4_local_pos_->xy_valid && px4_local_pos_->z_valid)
      {

        // --- 이 안의 내용은 기존과 동일합니다 ---
        geometry_msgs::msg::TransformStamped t;
        t.header.stamp = clock_->now();
        t.header.frame_id = "map";
        t.child_frame_id = "local_ned";
        t.transform.translation.x = drone_map_pose_.position.x;
        t.transform.translation.y = drone_map_pose_.position.y;
        t.transform.translation.z = drone_map_pose_.position.z;
        tf2::Quaternion q;
        q.setRPY(M_PI, 0, M_PI_2);
        t.transform.rotation = tf2::toMsg(q);
        static_broadcaster_->sendTransform(t);
        // --- 여기까지 동일 ---

        local_origin_set_ = true; // 준비 작업이 끝났음을 표시

        // ✨ 여기서 미션 시작 시간을 일관된 시계로 기록합니다.
        mission_start_time_ = clock_->now();
        RCLCPP_INFO(this->get_logger(), "Local origin set. Mission timer started.");
      }
      else
      {
        RCLCPP_INFO_ONCE(this->get_logger(), "Phase: Init - Waiting for valid local position from PX4...");
        return; // PX4 위치가 아직 유효하지 않으면 더 진행하지 않음
      }
    }

    // 2. 상태 전이 결정
    // 드론 준비가 끝났고, 이륙 명령도 받았다면 TakeOff 단계로 전환
    if (local_origin_set_ && takeoff_commanded_)
    {
      RCLCPP_INFO_ONCE(this->get_logger(), "Takeoff command confirmed. Transitioning to TakeOff phase.");
      current_phase_ = MissionPhase::TakeOff;
    }
    else
    {
      // 첫 번째 UGV 출발 신호 발행 (수정한 이름 기준)
      std_msgs::msg::Bool ready_msg;
      ready_msg.data = true;
      drone_ready_status_pub_->publish(ready_msg);
      RCLCPP_INFO_THROTTLE(
          this->get_logger(), *clock_, 5000,
          "Phase: Init - Ready. Published status on /status/drone_ready, Waiting for takeoff command...");
    }
  }

  void handleTakeOff()
  {
    RCLCPP_INFO_ONCE(this->get_logger(), "Phase: TakeOff - Ascending to %.1f meters.", target_takeoff_altitude_);

    // // --- UGV 출발 신호 발행 로직 (이륙 초기에 한 번만 실행) ---
    // // --- UGV 출발 신호 발행 로직 (단순화된 버전) ---
    // if (!ugv_start_signal_sent_)
    // {
    // 현재 고도가 (시작 고도 + 임계값) 보다 높아지면
    if (drone_map_pose_.position.z >= start_takeoff_altitude_ + ugv_start_altitude_threshold_)
    {
      RCLCPP_INFO_ONCE(this->get_logger(), "Altitude threshold for UGV start reached (%.1fm).", ugv_start_altitude_threshold_);

      // UGV 출발 신호 발행
      std_msgs::msg::Bool ugv_start_msg;
      ugv_start_msg.data = true;
      takeoff_status_pub_->publish(ugv_start_msg);
      RCLCPP_INFO_ONCE(this->get_logger(), "Published UGV start signal on /status/takeoff_complete");

      // // 플래그를 true로 설정하여 다시는 신호를 보내지 않도록 함
      // ugv_start_signal_sent_ = true;
    }
    // }

    // 현재 고도와 목표 고도 체크 (오차 범위 0.3m)
    double current_altitude = drone_map_pose_.position.z;
    double target_altitude = takeoff_target_pose_map_.position.z;

    if (std::abs(current_altitude - target_altitude) < takeoff_altitude_tolerance_)
    {
      RCLCPP_INFO(this->get_logger(), "Takeoff complete. Altitude reached.");

      // 다음 단계로 전환
      current_phase_ = MissionPhase::PlanRoute;
    }
  }
  // --- 전체가 수정될 handlePlanRoute 함수 ---
  void handlePlanRoute()
  {
    if (route_planned_)
    {
      return;
    }

    RCLCPP_INFO(this->get_logger(), "Phase: PlanRoute - Reading mission files.");

    try
    {
      std::filesystem::path package_path = PACKAGE_SOURCE_DIR;
      std::string line; // <<<< line 변수를 여기서 한 번만 선언합니다.

      // --- 1. uav_path.csv 파싱 (쉼표 기준, 안전장치 포함) ---
      std::filesystem::path path_csv_path = package_path / "path" / "uav_path.csv";
      RCLCPP_INFO(this->get_logger(), "Reading path file from: %s", path_csv_path.c_str());
      std::ifstream path_file(path_csv_path);
      if (!path_file.is_open())
      {
        RCLCPP_ERROR(this->get_logger(), "Failed to open uav_path.csv file from src. Is the path correct?");
        return;
      }

      while (std::getline(path_file, line))
      {
        if (line.empty() || line.find_first_not_of(" \t\n\r") == std::string::npos)
          continue;

        std::stringstream ss(line);
        ss.imbue(std::locale("C"));

        std::string token;
        std::vector<double> values;
        try
        {
          while (std::getline(ss, token, ','))
          {
            values.push_back(std::stod(token));
          }
        }
        catch (const std::exception &e)
        {
          RCLCPP_WARN(this->get_logger(), "Skipping malformed line in uav_path.csv: %s", line.c_str());
          continue;
        }

        if (values.size() < 3)
        {
          RCLCPP_WARN(this->get_logger(), "Skipping malformed line in uav_path.csv (not enough values): %s", line.c_str());
          continue;
        }

        WaypointTask new_task;
        new_task.waypoint_pose.position.x = values[0];
        new_task.waypoint_pose.position.y = values[1];
        new_task.waypoint_pose.position.z = values[2];
        new_task.waypoint_pose.orientation.w = 1.0;

        for (size_t i = 3; i + 2 < values.size(); i += 3)
        {
          SearchAction new_action;
          new_action.drone_yaw_rad = values[i];
          new_action.gimbal_pitch_deg = values[i + 1];
          new_action.num_markers_to_find = static_cast<int>(values[i + 2]);
          new_task.search_actions.push_back(new_action);
        }
        mission_route_.push_back(new_task);
      }
      RCLCPP_INFO(this->get_logger(), "Path plan complete. Loaded %zu waypoints.", mission_route_.size());
      path_file.close();

      // --- 2. rendezvous.csv 파싱 (쉼표 기준, 안전장치 포함) ---
      std::filesystem::path rendezvous_csv_path = package_path / "path" / "rendezvous.csv";
      RCLCPP_INFO(this->get_logger(), "Reading rendezvous file from: %s", rendezvous_csv_path.c_str());
      std::ifstream rendezvous_file(rendezvous_csv_path);
      if (!rendezvous_file.is_open())
      {
        RCLCPP_ERROR(this->get_logger(), "FATAL: Failed to open rendezvous.csv file. Cannot start mission.");
        return;
      }

      if (std::getline(rendezvous_file, line) && !line.empty())
      {
        std::stringstream ss(line);
        ss.imbue(std::locale("C"));
        std::string token;
        std::vector<double> values;
        try
        {
          while (std::getline(ss, token, ','))
          {
            values.push_back(std::stod(token));
          }
        }
        catch (const std::exception &e)
        {
          RCLCPP_ERROR(this->get_logger(), "FATAL: Malformed value in rendezvous.csv. Cannot start mission.");
          return;
        }

        if (values.size() == 3)
        {
          rendezvous_pose_map_.position.x = values[0];
          rendezvous_pose_map_.position.y = values[1];
          rendezvous_pose_map_.position.z = values[2];
          rendezvous_pose_map_.orientation.w = 1.0;
          RCLCPP_INFO(this->get_logger(), "Rendezvous point loaded.");
        }
        else
        {
          RCLCPP_ERROR(this->get_logger(), "FATAL: rendezvous.csv must contain exactly 3 values. Cannot start mission.");
          return;
        }
      }
      else
      {
        RCLCPP_ERROR(this->get_logger(), "FATAL: rendezvous.csv is empty. Cannot start mission.");
        return;
      }
      rendezvous_file.close();

      // --- 3. 탐지 로그 파일 준비 ---
      std::filesystem::path log_csv_path = package_path / "path" / "marker_detections.csv";
      detection_log_file_.open(log_csv_path.c_str());
      if (!detection_log_file_.is_open())
      {
        RCLCPP_ERROR(this->get_logger(), "Failed to open detection log file!");
      }
      else
      {
        detection_log_file_.seekp(0, std::ios::end);
        if (detection_log_file_.tellp() == 0)
        {
          detection_log_file_ << "timestamp_s,marker_id,x,y,z\n"; // 헤더에 timestamp 추가
        }
      }

      // 모든 계획 완료
      route_planned_ = true;
      current_phase_ = MissionPhase::FlyToWaypoint;
    }
    catch (const std::exception &e)
    {
      RCLCPP_ERROR(this->get_logger(), "Exception while planning route: %s", e.what());
    }
  }

  void handleFlyToWaypoint()
  {
    if (current_waypoint_index_ >= mission_route_.size())
    {
      RCLCPP_INFO(this->get_logger(), "All waypoints visited. Proceeding to Rendezvous phase.");
      current_rendezvous_step_ = RendezvousStep::GO_TO_AREA;
      current_phase_ = MissionPhase::Rendezvous;
      return;
    }

    // 현재 목표 경로점 설정
    const auto &current_task = mission_route_[current_waypoint_index_];
    // 1. 목표 위치 설정
    current_target_pose_map_.position = current_task.waypoint_pose.position;

    // 2. 목표 방향(yaw)을 동적으로 계산 및 설정
    // 만약 해당 경로점에 탐색 액션이 있다면, 첫 번째 액션의 yaw를 바라봄
    if (!current_task.search_actions.empty())
    {
      tf2::Quaternion q;
      q.setRPY(0, 0, current_task.search_actions[0].drone_yaw_rad);
      current_target_pose_map_.orientation = tf2::toMsg(q);

    // ✅ 추가: 이동 중 짐벌 pitch 각도 미리 설정
    send_gimbal_command(static_cast<float>(current_task.search_actions[0].gimbal_pitch_deg));
    }
    // 탐색 액션이 없는 단순 경유지라면, 이전 비행의 마지막 방향을 그대로 유지

    RCLCPP_INFO_THROTTLE(this->get_logger(), *clock_, 2000,
                         "Phase: FlyToWaypoint - Flying to waypoint #%zu", current_waypoint_index_);

    // 현재 드론 위치와 목표 경로점 사이의 거리 계산
    double dx = drone_map_pose_.position.x - current_target_pose_map_.position.x;
    double dy = drone_map_pose_.position.y - current_target_pose_map_.position.y;
    double dz = drone_map_pose_.position.z - current_target_pose_map_.position.z;
    double distance = std::sqrt(dx * dx + dy * dy + dz * dz);

    // 도착 판정
    if (distance < waypoint_arrival_tolerance_)
    {
      RCLCPP_INFO(this->get_logger(), "Arrived at waypoint #%zu.", current_waypoint_index_);
      // 탐색 액션이 없으면 다음 웨이포인트로 바로 진행
      if (current_task.search_actions.empty())
      {
        ++current_waypoint_index_;
        RCLCPP_INFO(this->get_logger(), "No search action: moving to waypoint #%zu.", current_waypoint_index_);
        // 그대로 FlyToWaypoint 상태를 유지해서 다음 목표로 비행
      }
      else
      {
        // 탐색 액션이 있으면 기존 로직대로 SearchForMarker 단계로 전환
        current_search_action_index_ = 0;
        current_search_step_        = SearchStep::SET_POSE;
        current_phase_              = MissionPhase::SearchForMarker;
      }


    }
  }

  // --- 전체가 수정될 handleSearchForMarker 함수 ---
  void handleSearchForMarker()
  {
    // 현재 웨이포인트의 과업 정보를 가져옴
    const auto &current_task = mission_route_[current_waypoint_index_];

    // 이 경로점에서 수행할 모든 탐색을 마쳤거나, 탐색 액션이 없는 경우 다음 웨이포인트로
    if (current_task.search_actions.empty() || current_search_action_index_ >= current_task.search_actions.size())
    {
      RCLCPP_INFO(this->get_logger(), "All actions at waypoint #%zu are complete. Moving to next waypoint.", current_waypoint_index_);
      current_waypoint_index_++;
      current_phase_ = MissionPhase::FlyToWaypoint;
      return;
    }

    const auto &current_action = current_task.search_actions[current_search_action_index_];

    // 현재 탐색 세부 단계에 따라 동작 수행
    switch (current_search_step_)
    {
    case SearchStep::SET_POSE:
    {
      RCLCPP_INFO(this->get_logger(), "SearchStep: SET_POSE for action #%zu", current_search_action_index_);
      // 1. 목표 방향(yaw) 및 위치(호버링) 설정
      tf2::Quaternion q;
      q.setRPY(0, 0, current_action.drone_yaw_rad);
      current_target_pose_map_.orientation = tf2::toMsg(q);
      current_target_pose_map_.position = current_task.waypoint_pose.position;

      // 2. 짐벌 제어 명령 전송
      send_gimbal_command(static_cast<float>(current_action.gimbal_pitch_deg));

      // 3. 안정화 대기 시작 시간 기록 및 다음 단계로 전환
      pose_stable_wait_start_time_ = clock_->now();
      current_search_step_ = SearchStep::WAIT_FOR_STABLE;
      break;
    }

    case SearchStep::WAIT_FOR_STABLE:
    {
      RCLCPP_INFO_THROTTLE(this->get_logger(), *clock_, 1000, "SearchStep: WAIT_FOR_STABLE - Waiting for drone to stop or timeout...");

      // --- ✨ 두 조건을 OR (||) 로 연결하여 하나만 만족해도 통과하도록 수정 ✨ ---
      bool timed_out = (clock_->now() - pose_stable_wait_start_time_) > pose_stable_wait_duration_;
      if (is_drone_stable_and_stopped_ || timed_out)
      {
        if (timed_out && !is_drone_stable_and_stopped_)
        {
          RCLCPP_WARN(this->get_logger(), "Wait for stable timed out. Proceeding anyway.");
        }
        RCLCPP_INFO(this->get_logger(), "Drone is stable or wait timed out. Starting image processing.");
        process_image_ = true; // 이미지 콜백 활성화
        current_search_step_ = SearchStep::SEARCHING;
      }
      break;
    }

    case SearchStep::SEARCHING:
    {
      RCLCPP_INFO_THROTTLE(this->get_logger(), *clock_, 5000, "SearchStep: SEARCHING for markers...");
      // 이 단계에서는 아무것도 하지 않음. image_callback이 모든 작업을 처리하고
      // 완료되면 current_search_action_index_ 를 증가시키고 current_search_step_ 를 리셋함.
      break;
    }
    }

    RCLCPP_INFO_THROTTLE(this->get_logger(), *clock_, 5000,
                         "Phase: SearchForMarker - At waypoint #%zu, performing search action #%zu (Yaw: %.1f, Pitch: %.1f)",
                         current_waypoint_index_, current_search_action_index_, current_action.drone_yaw_rad, current_action.gimbal_pitch_deg);
  }

  // --- 전체가 수정될 handleRendezvous 함수 ---
  void handleRendezvous()
  {
    // 1. 랑데부 시작 명령을 아직 받지 못한 경우 (대기 상태)
    if (!rendezvous_command_received_)
    {
      // 목표 위치를 초기 랑데부 지점의 XY와 지정된 대기 고도로 설정
      current_target_pose_map_.position.x = rendezvous_pose_map_.position.x;
      current_target_pose_map_.position.y = rendezvous_pose_map_.position.y;
      current_target_pose_map_.position.z = RENDEZVOUS_WAITING_ALTITUDE;
      allow_marker_lock_ = false; // 이 상태에서는 마커를 찾지 않음

      RCLCPP_INFO_THROTTLE(this->get_logger(), *clock_, 5000,
                           "Phase: Rendezvous (Waiting) - Flying to rendezvous area and holding at %.1f m for command.", RENDEZVOUS_WAITING_ALTITUDE);
      return;
    }

    // --- 2. 랑데부 시작 명령을 받은 이후 ---
    // 내부 상태에 따라 다른 동작 수행
    switch (current_rendezvous_step_)
    {
    case RendezvousStep::GO_TO_AREA:
    {
      // 2a. 초기 랑데부 지점 상공으로 이동 및 호버링
      allow_marker_lock_ = false; // 이동 중에는 마커를 찾지 않음
      RCLCPP_INFO_THROTTLE(this->get_logger(), *clock_, 2000, "Rendezvous: Flying to initial rendezvous area.");

      // 목표는 초기 랑데부 지점의 XY와 현재 고도 유지
      current_target_pose_map_.position.x = rendezvous_pose_map_.position.x;
      current_target_pose_map_.position.y = rendezvous_pose_map_.position.y;
      current_target_pose_map_.position.z = RENDEZVOUS_WAITING_ALTITUDE;
      // 고도는 현재 고도를 유지하며 수평 이동만 하도록 함

      // 랑데부 지역 도착 판정
      double dx = drone_map_pose_.position.x - rendezvous_pose_map_.position.x;
      double dy = drone_map_pose_.position.y - rendezvous_pose_map_.position.y;
      double dz = drone_map_pose_.position.z - RENDEZVOUS_WAITING_ALTITUDE; // 목표 고도와의 차이
      double distance = std::sqrt(dx * dx + dy * dy + dz * dz);             // Z축을 포함한 3D 거리

      // 수평 거리가 1.0m 이내로 들어오면 다음 단계(안정화 대기)로 전환
      if (distance < rendezvous_arrival_tolerance_)
      {
        send_gimbal_command(-90.0);
        RCLCPP_INFO(this->get_logger(), "Arrived at rendezvous area. Waiting for stable hover.");
        // 안정화 대기 시작 시간 기록
        pose_stable_wait_start_time_ = clock_->now();
        current_rendezvous_step_ = RendezvousStep::WAIT_FOR_STABLE;
      }
      break;
    }

    case RendezvousStep::WAIT_FOR_STABLE:
    {
      // ✨ 새로 추가된 안정화 대기 단계 ✨
      allow_marker_lock_ = false; // 안정화 중에는 마커를 찾지 않음
      RCLCPP_INFO_THROTTLE(this->get_logger(), *clock_, 1000, "Rendezvous Step: WAIT_FOR_STABLE");

      // 드론이 정지하거나, 타임아웃 시간이 지나면 다음 단계로
      bool timed_out = (clock_->now() - pose_stable_wait_start_time_) > pose_stable_wait_duration_2;
      if (is_drone_stable_and_stopped_2 || timed_out)
      {
        if (timed_out && !is_drone_stable_and_stopped_2)

        {
          RCLCPP_WARN(this->get_logger(), "Rendezvous wait for stable timed out. Proceeding anyway.");
        }
        else
        {
          RCLCPP_INFO(this->get_logger(), "Drone is stable at Z=%.2f.", drone_map_pose_.position.z);
        }

        // 먼저 착륙 고도인지 확인
        if (drone_map_pose_.position.z < RENDEZVOUS_LANDING_ALTITUDE)
        {
          RCLCPP_INFO(this->get_logger(), "Landing altitude reached. Switching to LAND step.");
          current_rendezvous_step_ = RendezvousStep::LAND;
        }
        else
        {
          RCLCPP_INFO(this->get_logger(), "Ready to search for marker.");
          allow_marker_lock_ = true; // ✨ 이 상태에서만 마커 찾기 허용 ✨
          marker_find_wait_start_time_ = clock_->now();
          current_rendezvous_step_ = RendezvousStep::TRACK_AND_APPROACH;
          marker_size_ = 0.5;
        }
      }
      break;
    }

    case RendezvousStep::TRACK_AND_APPROACH:
    {
      // 이 상태에서는 image_callback이 마커를 찾도록 허용 (allow_marker_lock_ = true)
      RCLCPP_INFO_THROTTLE(this->get_logger(), *clock_, 2000, "Rendezvous: Tracking marker...");

      // image_callback이 마커를 발견하여 위치를 갱신해 주었는지 확인
      if (locked_marker_position.has_value() && locked_marker_relative_yaw_.has_value())
      {
        RCLCPP_INFO(this->get_logger(), "Marker locked. Setting next descent target.");

        //=========================== 상대 좌표 사용 =============================//
        // --- ✨ 목표 방향(Yaw) 계산 ---

        double marker_yaw_cam = locked_marker_relative_yaw_.value();

        // 1. 드론의 현재 Yaw(방향) 구하기 (라디안)
        tf2::Quaternion q_drone_map;
        tf2::fromMsg(drone_map_pose_.orientation, q_drone_map);
        double drone_roll, drone_pitch, drone_yaw;
        tf2::Matrix3x3(q_drone_map).getRPY(drone_roll, drone_pitch, drone_yaw);
        double target_yaw = drone_yaw - marker_yaw_cam;
        tf2::Quaternion q_target;
        q_target.setRPY(0, 0, target_yaw);
        current_target_pose_map_.orientation = tf2::toMsg(q_target);

        // --- ✨ 목표 위치(Position) 계산 ---
        // 1. 드론 중심(base_link)에서 카메라(camera_link)까지의 정적 TF를 조회
        double camera_forward_offset = 0.0;
        double camera_right_offset = 0.0;    // right 오프셋을 저장할 변수
        double camera_vertical_offset = 0.0; // Z축 오프셋을 저장할 변수

        try
        {
          geometry_msgs::msg::TransformStamped t_base_to_cam;
          t_base_to_cam = tf_buffer_->lookupTransform(
              "x500_gimbal_0",             // 기준 프레임
              "x500_gimbal_0/camera_link", // 대상 프레임
              tf2::TimePointZero);         // 가장 최신 정보 조회

          // base_link 기준 camera_link의 X축(정면) 거리
          camera_forward_offset = t_base_to_cam.transform.translation.x;
          camera_right_offset = t_base_to_cam.transform.translation.y;    // <<< Y축 오프셋 가져오기
          camera_vertical_offset = t_base_to_cam.transform.translation.z; // <<< Z축 오프셋 가져오기

          // --- ✨ 이 로그 라인을 추가합니다 ✨ ---
          RCLCPP_INFO(this->get_logger(),
                      "Static Camera Offset from BaseLink (m) -> Forward: %.3f, Right: %.3f, Up: %.3f",
                      camera_forward_offset, camera_right_offset, camera_vertical_offset);
          // ------------------------------------
        }
        catch (const tf2::TransformException &ex)
        {
          RCLCPP_WARN(this->get_logger(), "Could not get camera offset TF: %s", ex.what());
        }

        // 2. 주석으로 설명해주신 '드론 기준 마커의 상대 XY 위치' 정의
        // "드론 baselink에서 바라볼때 마커의 x 좌표 위치" -> 카메라 Z축(정면) 사용
        // 2. 최종 forward_offset 계산: (카메라-마커 거리) + (드론중심-카메라 거리)
        // double forward_offset = locked_marker_position->z + camera_forward_offset;
        double forward_offset = locked_marker_position->z;
        // "드론 baselink에서 바라볼때 마커의 y 좌표 위치" -> 카메라 Y축(왼쪽)의 반대 방향 사용
        // "드론 baselink에서 바라볼때 마커의 y 좌표 위치" + "드론중심-카메라의 y 오프셋"
        double right_offset = -locked_marker_position->y - camera_vertical_offset;
        // double right_offset = -locked_marker_position->y;

        // 3. 2D 회전 변환을 직접 계산하여, 드론 기준의 상대 오차를 map 기준의 절대 오차로 변환
        // offset_map_x = forward_offset * cos(yaw) - right_offset * sin(yaw)
        // offset_map_y = forward_offset * sin(yaw) + right_offset * cos(yaw)
        double offset_map_x = forward_offset * std::cos(drone_yaw) - right_offset * std::sin(drone_yaw);
        double offset_map_y = forward_offset * std::sin(drone_yaw) + right_offset * std::cos(drone_yaw);

        // 4. 다음 목표 위치 설정: "현재 위치에서 오차를 빼준다"
        // -> 의도하신 바는 '오차를 없애는 방향으로 이동'이므로, 수학적으로는 '현재 위치 + 계산된 오차 벡터'가 됩니다.
        current_target_pose_map_.position.x = drone_map_pose_.position.x + offset_map_x;
        current_target_pose_map_.position.y = drone_map_pose_.position.y + offset_map_y;
        current_target_pose_map_.position.z = drone_map_pose_.position.z - rendezvous_descent_distance_;

        // 다음 이동을 위해 상태 변경
        locked_marker_position.reset();
        pose_stable_wait_start_time_ = clock_->now();
        current_rendezvous_step_ = RendezvousStep::WAIT_FOR_STABLE; // 새로 설정된 목표 지점으로 이동 시작
        //=====================================================================//

      } // 2. 타임아웃 확인 시, 새로 추가한 변수 사용
      else if ((clock_->now() - marker_find_wait_start_time_) > marker_search_timeout_)
      { // <<< 이 부분을 수정
        RCLCPP_WARN(this->get_logger(), "Marker search timed out. Returning to initial rendezvous point.");
        allow_marker_lock_ = false;
        current_target_pose_map_ = rendezvous_pose_map_;
        current_target_pose_map_.position.z = RENDEZVOUS_WAITING_ALTITUDE;
        current_rendezvous_step_ = RendezvousStep::GO_TO_AREA;
      }
      break;
    }

    case RendezvousStep::LAND:
    {
      send_disarm_command();
      // current_target_pose_map_.position.z = 0.1;
      // ✨ 지시하신 대로, 드론이 완전히 정지했는지 확인합니다. ✨
      if (is_drone_stable_and_stopped_2)
      {
        RCLCPP_INFO_THROTTLE(this->get_logger(), *clock_, 5000, "RendezvousStep: LAND - Vehicle has stopped. Mission phase is now Completed.");
        current_phase_ = MissionPhase::Completed;
      }
      break;
    }
    }
  }
  // --- 전체가 수정될 handleCompleted 함수 ---
  void handleCompleted()
  {
    // 기체가 착륙했을 때만 완료 로직 실행
    // if (landed_)
    // {
    send_disarm_command();

    // 최종 시간을 아직 로깅하지 않았다면 로깅 실행
    if (!mission_time_logged_)
    {
      send_disarm_command();
      rclcpp::Time end_time = clock_->now();
      rclcpp::Duration mission_duration = end_time - mission_start_time_;

      long total_seconds = mission_duration.seconds();
      long minutes = total_seconds / 60;
      long seconds = total_seconds % 60;

      RCLCPP_INFO(this->get_logger(), "===========================================");
      RCLCPP_INFO(this->get_logger(), "           MISSION ACCOMPLISHED");
      RCLCPP_INFO(this->get_logger(), " ");
      RCLCPP_INFO(this->get_logger(), " Total Mission Time: %ld minutes and %ld seconds.", minutes, seconds);
      RCLCPP_INFO(this->get_logger(), "===========================================");

      // 로그를 다시 출력하지 않도록 플래그 설정
      mission_time_logged_ = true;

      // 로그 파일 닫기
      if (detection_log_file_.is_open())
      {
        detection_log_file_.close();
      }
    }
    // 이미 로깅했다면 대기 메시지만 출력
    else
    {
      RCLCPP_INFO_THROTTLE(this->get_logger(), *clock_, 30000, "Node is idling. Press Ctrl+C to exit.");
    }

    // }
    // // 아직 착륙하지 않았다면 착륙 대기 중임을 알림
    // else
    // {
    //   RCLCPP_INFO_THROTTLE(this->get_logger(), *clock_, 5000, "Mission sequence finished, waiting for landing confirmation...");
    // }
  }

  void local_position_callback(const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg)
  {
    px4_local_pos_ = msg; // PX4 로컬 위치 업데이트

    // 3D 속도의 크기(speed)를 계산
    float speed = std::sqrt(msg->vx * msg->vx + msg->vy * msg->vy + msg->vz * msg->vz);

    // 계산된 속력이 설정된 임계값보다 작으면 정지 상태로 판단
    if (speed < STOPPED_VELOCITY_THRESHOLD_)
    {
      is_drone_stable_and_stopped_ = true;
    }
    else
    {
      is_drone_stable_and_stopped_ = false;
    }

        // 계산된 속력이 설정된 임계값보다 작으면 정지 상태로 판단
    if (speed < STOPPED_VELOCITY_THRESHOLD_2)
    {
      is_drone_stable_and_stopped_2 = true;
    }
    else
    {
      is_drone_stable_and_stopped_2 = false;
    }
  }

  // map(ENU) -> local_ned(NED) 변환 및 목표 자세 발행 함수
  void publish_target_pose(const geometry_msgs::msg::Pose &target_pose_map)
  {
    geometry_msgs::msg::PoseStamped input_pose_stamped;
    input_pose_stamped.header.frame_id = "map";
    input_pose_stamped.header.stamp = clock_->now();
    input_pose_stamped.pose = target_pose_map;

    try
    {
      // "local_ned" 프레임으로 변환
      auto transformed_pose_stamped = tf_buffer_->transform(input_pose_stamped, "local_ned", tf2::durationFromSec(1.0));

      // 최종 발행할 메시지
      geometry_msgs::msg::PoseStamped final_command = transformed_pose_stamped;
      final_command.header.stamp = clock_->now(); // 타임스탬프는 현재 시간으로

      target_pose_pub_->publish(final_command);
    }
    catch (const tf2::TransformException &ex)
    {
      RCLCPP_WARN(this->get_logger(), "Could not transform 'map' to 'local_ned': %s", ex.what());
    }
  }

  // 이륙 명령 수신 콜백
  void takeoff_command_callback(const std_msgs::msg::Bool::SharedPtr msg)
  {
    if (msg->data && !takeoff_commanded_)
    {
      takeoff_commanded_ = true;
      RCLCPP_INFO(this->get_logger(), "Takeoff command received!");

      start_takeoff_altitude_ = drone_map_pose_.position.z;

      // 이륙 목표 위치 설정 (map 프레임 기준)
      // X, Y는 드론의 시작 위치와 동일
      takeoff_target_pose_map_.position.x = drone_map_pose_.position.x;
      takeoff_target_pose_map_.position.y = drone_map_pose_.position.y;
      // Z는 시작 위치의 Z + 목표 고도
      takeoff_target_pose_map_.position.z = drone_map_pose_.position.z + target_takeoff_altitude_;
      // 방향은 시작 방향과 동일
      takeoff_target_pose_map_.orientation = drone_map_pose_.orientation;

      current_target_pose_map_ = takeoff_target_pose_map_;
    }
  }

  // --- 새로 설계될 image_callback 함수 ---
  void image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    // --- 섹션 0: 사전 준비 ---
    // 0a. 카메라 정보가 아직 수신되지 않았다면 아무 처리도 하지 않음
    if (camera_matrix_.empty() || dist_coeffs_.empty())
    {
      return;
    }

    // 0b. ROS 이미지를 OpenCV 이미지(cv::Mat)로 변환. 실패 시 중단.
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception &e)
    {
      RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
      return;
    }

    // --- 섹션 1: 미션 단계에 따른 로직 분기 ---
    if (current_phase_ == MissionPhase::SearchForMarker || current_rendezvous_step_ == RendezvousStep::TRACK_AND_APPROACH || current_rendezvous_step_ == RendezvousStep::WAIT_FOR_STABLE)
    {
      // ===================================================================
      // === 마커 탐색 모드 (SearchForMarker 또는 Rendezvous 단계) ===
      // ===================================================================

      // 1a. (SearchForMarker 전용) 메인 루프에서 탐색을 허가하지 않았다면(플래그가 꺼져있다면) 아무것도 하지 않음
      if (current_phase_ == MissionPhase::SearchForMarker && !process_image_)
      {
        // 이 경우, 원본 이미지를 그대로 발행하여 현재 카메라 화면을 계속 볼 수 있게 함
        processed_image_pub_->publish(*cv_ptr->toImageMsg());
        return;
      }

      // --- ArUco 마커 탐색 로직 (사용자 코드 스타일 반영) ---
      std::vector<int> ids;
      std::vector<std::vector<cv::Point2f>> corners;
      // detectMarkers 함수에 parameters 인자 추가
      cv::aruco::detectMarkers(cv_ptr->image, aruco_dictionary_, corners, ids, aruco_detector_params_);

      if (!ids.empty())
      {
        // 마커가 탐지되었을 경우의 처리
        cv::aruco::drawDetectedMarkers(cv_ptr->image, corners, ids);

        std::vector<cv::Vec3d> rvecs, tvecs;
        cv::aruco::estimatePoseSingleMarkers(
            corners, marker_size_, camera_matrix_, dist_coeffs_, rvecs, tvecs);

        // --- 이 for 루프 전체를 아래 내용으로 교체합니다 ---
        for (size_t i = 0; i < ids.size(); ++i)
        {
          // 시각화를 위해 좌표축을 이미지에 그림
          cv::drawFrameAxes(cv_ptr->image, camera_matrix_, dist_coeffs_, rvecs[i], tvecs[i], marker_size_);

          // 3. TF 버퍼를 사용해 마커의 카메라 기준 좌표를 map 기준 좌표로 변환
          try
          {

            // ########################### 카메라 위치 baselink에 붙인 버전 #########################//

            // 1. 필요한 벡터 및 쿼터니언 준비
            // 1a. camera_link -> base_link 변환 정보 (위치와 방향)
            geometry_msgs::msg::TransformStamped t_cam_to_base_stamped = tf_buffer_->lookupTransform(
                "x500_gimbal_0/camera_link", "x500_gimbal_0", tf2::TimePointZero);
            tf2::Transform t_cam_to_base;
            tf2::fromMsg(t_cam_to_base_stamped.transform, t_cam_to_base);

            // 3a. `t_cam_to_base`에서 위치 벡터와 회전 쿼터니언을 분리
            tf2::Vector3 v_cam_to_base = t_cam_to_base.getOrigin();
            tf2::Quaternion q_cam_to_base = t_cam_to_base.getRotation();

            // 2. "tvecs, rvecs는 baselink 기준"이라는 가정 하에 상대 자세(Pose) 준비
            // 2a. 위치 벡터 (tvecs)
            tf2::Vector3 v_base_to_marker_assumed(-tvecs[i][2], tvecs[i][0], -tvecs[i][1]);

            // 1c. base_link 기준 카메라 오프셋 벡터 정의
            tf2::Vector3 v_camera_offset_in_base(0.05, 0.0, 0.581);

            // 2a. "v_camera_offset_in_base 이거만 회전시키라고"
            //      base_link 기준의 카메라 오프셋을 camera_link 기준으로 회전 변환
            tf2::Vector3 v_rotated_camera_offset = tf2::quatRotate(q_cam_to_base, v_camera_offset_in_base);

            // 2. 최종적인 "camera_link -> 마커" 자세 계산
            // 2a. 위치 계산: 요청하신 대로, 회전 없이 평행 이동(벡터 덧셈)만 적용합니다.
            //    (cam->base 벡터) + (base->marker 벡터) = (cam->marker 벡터)
            tf2::Vector3 final_pos_in_cam = v_cam_to_base + v_base_to_marker_assumed + v_rotated_camera_offset;

            // 2b. 방향 벡터 (rvecs) -> 쿼터니언 변환
            cv::Mat rot_mat;
            cv::Rodrigues(rvecs[i], rot_mat);
            tf2::Matrix3x3 R_marker_in_cam_cv(
                rot_mat.at<double>(0, 0), rot_mat.at<double>(0, 1), rot_mat.at<double>(0, 2),
                rot_mat.at<double>(1, 0), rot_mat.at<double>(1, 1), rot_mat.at<double>(1, 2),
                rot_mat.at<double>(2, 0), rot_mat.at<double>(2, 1), rot_mat.at<double>(2, 2));
            tf2::Quaternion q_base_to_marker_assumed;
            R_marker_in_cam_cv.getRotation(q_base_to_marker_assumed);

            // 3b. "각각 쿼터니언으로 바꾸고 곱해" 로직 수정
            // OpenCV->ROS 좌표계 변환 행렬을 쿼터니언으로 변환
            tf2::Matrix3x3 R_ros_from_cv;
            R_ros_from_cv.setValue(
                0, 1, 0,
                0, 0, -1,
                -1, 0, 0);
            tf2::Quaternion q_ros_from_cv;
            R_ros_from_cv.getRotation(q_ros_from_cv);

            // 두 쿼터니언을 곱하여 최종 상대 방향 계산
            tf2::Quaternion final_rot_in_cam = q_ros_from_cv * q_base_to_marker_assumed;

            // 2c. 계산된 위치와 방향을 합쳐 최종 상대 자세(Pose) 생성
            geometry_msgs::msg::Pose final_relative_pose;
            // 각 성분을 직접 대입합니다.
            final_relative_pose.position.x = final_pos_in_cam.x();
            final_relative_pose.position.y = final_pos_in_cam.y();
            final_relative_pose.position.z = final_pos_in_cam.z();
            final_relative_pose.orientation = tf2::toMsg(final_rot_in_cam);

            // 3. 최종 상대 자세를 map 기준 절대 자세로 변환
            geometry_msgs::msg::PoseStamped final_marker_in_camera_frame;
            final_marker_in_camera_frame.header.stamp = msg->header.stamp;
            final_marker_in_camera_frame.header.frame_id = "x500_gimbal_0/camera_link";
            final_marker_in_camera_frame.pose = final_relative_pose;

            geometry_msgs::msg::PoseStamped marker_pose_map = tf_buffer_->transform(
                final_marker_in_camera_frame, "map", tf2::durationFromSec(0.2));

            // ###################################################################################//

            // ########################### 카메라 위치 camera_link로 하는 버전 ########################//

            // // 1. 탐지된 마커의 위치(tvecs)를 ROS PointStamped 메시지로 생성
            // geometry_msgs::msg::PoseStamped marker_point_camera;
            // marker_point_camera.header.stamp = msg->header.stamp;
            // marker_point_camera.header.frame_id = "x500_gimbal_0/camera_link"; // "camera_link"

            // // 2. OpenCV 좌표계(X:오른쪽,Y:아래,Z:정면) -> ROS 카메라 좌표계(X:뒤,Y:오른쪽,Z:위) 변환
            // marker_point_camera.pose.position.x = -tvecs[i][2];
            // marker_point_camera.pose.position.y = tvecs[i][0];
            // marker_point_camera.pose.position.z = -tvecs[i][1];

            // geometry_msgs::msg::PoseStamped marker_pose_map = tf_buffer_->transform(
            //     marker_point_camera,
            //     "map",
            //     tf2::durationFromSec(0.1) // 0.1초 이내의 TF 정보를 사용
            // );

            // ###################################################################################//

            // // 4. 변환된 절대 좌표와 상대 좌표를 함께 로깅 (1초에 한 번)
            // RCLCPP_INFO_THROTTLE(this->get_logger(), *clock_, 1000,
            //                      "Marker ID: %d | Relative Pos (cam_link): [%.2f, %.2f, %.2f] | Absolute Pos (map): [%.2f, %.2f, %.2f]",
            //                      ids[i],
            //                      tvecs[i][0], tvecs[i][1], tvecs[i][2],
            //                      marker_pose_map.pose.position.x, marker_pose_map.pose.position.y, marker_pose_map.pose.position.z);

            // --- ✨ 2. SearchForMarker 단계의 마커 처리 로직 ✨ ---
            if (current_phase_ == MissionPhase::SearchForMarker)
            {
              // 아직 현재 탐색 액션이 끝나지 않았을 때만 처리
              if (process_image_)
              {
                // 지금은 어떤 마커든 찾으면 카운트
                RCLCPP_INFO(this->get_logger(), "Found marker! ID: %d, Abs Pos: [%.2f, %.2f, %.2f]",
                            ids[i], marker_pose_map.pose.position.x, marker_pose_map.pose.position.y, marker_pose_map.pose.position.z);

                // CSV 파일에 기록
                if (detection_log_file_.is_open())
                {
                  detection_log_file_ << ids[i] << ","
                                      << marker_pose_map.pose.position.x << ","
                                      << marker_pose_map.pose.position.y << ","
                                      << marker_pose_map.pose.position.z << std::endl;
                }
                // 찾은 마커의 ID를 벡터에 저장하고, 찾은 개수 증가
                found_marker_ids_this_action_.push_back(ids[i]);
                markers_found_this_action_++;

                // 목표 개수만큼 찾았는지 확인
                const auto &current_action = mission_route_[current_waypoint_index_].search_actions[current_search_action_index_];
                if (markers_found_this_action_ >= current_action.num_markers_to_find)
                {
                  // --- 로그 메시지 수정을 위해 찾은 ID 목록을 문자열로 만듦 ---
                  std::string found_ids_str = "[";
                  for (size_t j = 0; j < found_marker_ids_this_action_.size(); ++j)
                  {
                    found_ids_str += std::to_string(found_marker_ids_this_action_[j]);
                    if (j < found_marker_ids_this_action_.size() - 1)
                    {
                      found_ids_str += ", ";
                    }
                  }
                  found_ids_str += "]";

                  // --- 완료 로그 메시지 수정 (찾은 ID 목록 추가) ---
                  RCLCPP_INFO(this->get_logger(), "Completed search action #%zu. Found markers: %s",
                              current_search_action_index_, found_ids_str.c_str());
                  process_image_ = false;
                  markers_found_this_action_ = 0;
                  found_marker_ids_this_action_.clear(); // ID 저장 벡터 비우기
                  current_search_action_index_++;
                  current_search_step_ = SearchStep::SET_POSE;
                }
              }
            }

            // --- ✨ 2. Rendezvous 단계의 목표 위치 업데이트 로직 ✨ ---
            if (current_rendezvous_step_ == RendezvousStep::TRACK_AND_APPROACH)
            {
              // --- 로그 메시지에 절대좌표 추가 ---
              RCLCPP_INFO_THROTTLE(this->get_logger(), *clock_, 1000,
                                   "Marker detected during rendezvous. ID: %d, Abs Pos: [%.2f, %.2f, %.2f]",
                                   ids[i],
                                   marker_pose_map.pose.position.x,
                                   marker_pose_map.pose.position.y,
                                   marker_pose_map.pose.position.z);
              // ✨ handleRendezvous가 허락할 때만 마커 위치를 저장 ✨
              if (allow_marker_lock_)
              {

                // =========================== 랑데뷰에 상대좌표 사용 ============================//
                RCLCPP_INFO(this->get_logger(), "Marker detected. Saving its RELATIVE position.");

                // 1. OpenCV 좌표계(tvec) -> ROS 카메라 좌표계로 상대 위치 변환
                geometry_msgs::msg::Point relative_pos_in_cam_frame;
                relative_pos_in_cam_frame.x = -tvecs[i][2]; // ROS X (뒤) = -OpenCV Z (정면)
                relative_pos_in_cam_frame.y = tvecs[i][0];  // ROS Y (오른쪽) = OpenCV X (오른쪽)
                relative_pos_in_cam_frame.z = -tvecs[i][1]; // ROS Z (위) = -OpenCV Y (아래)

                // 2. 계산된 상대 위치를 멤버 변수에 저장
                locked_marker_position = relative_pos_in_cam_frame;

                // 2. 상대 방향(rvec)에서 Yaw 값만 추출하여 저장
                cv::Mat rot_mat;
                cv::Rodrigues(rvecs[i], rot_mat);
                tf2::Matrix3x3 tf2_rot_mat(
                    rot_mat.at<double>(0, 0), rot_mat.at<double>(0, 1), rot_mat.at<double>(0, 2),
                    rot_mat.at<double>(1, 0), rot_mat.at<double>(1, 1), rot_mat.at<double>(1, 2),
                    rot_mat.at<double>(2, 0), rot_mat.at<double>(2, 1), rot_mat.at<double>(2, 2));
                double marker_roll_cam, marker_pitch_cam, marker_yaw_cam;
                tf2_rot_mat.getRPY(marker_roll_cam, marker_pitch_cam, marker_yaw_cam);
                locked_marker_relative_yaw_ = marker_yaw_cam;

                // 마커를 한 번이라도 찾으면, 바로 탐색 허용 플래그를 꺼서
                // handleRendezvous가 이 값을 가지고 행동에 나설 수 있게 함
                allow_marker_lock_ = false;

                //============================================================================//
              }
            }
          }
          catch (const tf2::TransformException &ex)
          {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *clock_, 2000,
                                 "Could not transform marker pose: %s", ex.what());
          }
        }
      }

      // 1d. 처리된 이미지를 재발행
      processed_image_pub_->publish(*cv_ptr->toImageMsg());
    }
    else
    {
      // ===================================================================
      // === 패스스루 모드 (그 외 다른 모든 단계) ===
      // ===================================================================
      // 2. 받은 이미지를 변환 없이 그대로 재발행
      processed_image_pub_->publish(*msg);
    }
  }

  // 카메라 정보 수신 콜백
  void camera_info_callback(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
  {
    if (!camera_matrix_.empty())
    {
      return; // 이미 정보가 있으면 return
    }
    camera_matrix_ = cv::Mat(3, 3, CV_64F, const_cast<double *>(msg->k.data())).clone();
    dist_coeffs_ = cv::Mat(msg->d.size(), 1, CV_64F, const_cast<double *>(msg->d.data())).clone();
    RCLCPP_INFO(this->get_logger(), "Camera info received and calibrated.");
  }

  void send_gimbal_command(float gimbal_pitch_degree)
  {
    std_msgs::msg::Float32 msg;
    msg.data = gimbal_pitch_degree;
    gimbal_pitch_pub_->publish(msg);
    // RCLCPP_INFO(this->get_logger(), "Target gimbal pitch dgree: %f", gimbal_pitch_degree);
  }

  // 랑데부 시작 명령을 수신하는 콜백
  void rendezvous_command_callback(const std_msgs::msg::Bool::SharedPtr msg)
  {
    if (msg->data)
    {
      if (!rendezvous_command_received_)
      {
        RCLCPP_INFO(this->get_logger(), "Rendezvous command received and latched!");
        rendezvous_command_received_ = true;
      }
    }
  }

  void land_detected_callback(const px4_msgs::msg::VehicleLandDetected::SharedPtr msg)
  {
    // 기체가 착륙했고, 아직 disarm 신호를 보내지 않았다면
    if (msg->landed && !landed_ && current_phase_ == MissionPhase::Completed)
    {
      RCLCPP_INFO(this->get_logger(), "Landed detected. Sending disarm command...");

      // 상태 변수 업데이트
      landed_ = true;

      // Disarm 명령 전송
      send_disarm_command();
    }
  }

  // OffboardControl 클래스 멤버 함수로 추가
  void send_land_command()
  {
    auto msg = std_msgs::msg::Bool();
    msg.data = true;
    land_pub_->publish(msg);
  }

  void send_disarm_command()
  {
    auto msg = std_msgs::msg::Bool();
    msg.data = true;
    disarm_pub_->publish(msg);
  }
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<UAVController>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}