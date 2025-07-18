// uav_final.cpp
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/msg/pose.hpp>
#include <chrono>
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
#include <tf2_ros/transform_broadcaster.h> // <<< 이 헤더를 추가합니다.

class ArucoUavImageNode : public rclcpp::Node
{
public:
  ArucoUavImageNode()
      : Node("aruco_uav_image_node")
  {

    // 카메라 이미지 및 정보 수신 Subscriber
    auto sub_qos = rclcpp::QoS(1).best_effort();
    image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
        "/world/default/model/x500_gimbal_0/link/camera_link/sensor/camera/image", sub_qos,
        std::bind(&ArucoUavImageNode::image_callback, this, std::placeholders::_1));

    camera_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
        "/world/default/model/x500_gimbal_0/link/camera_link/sensor/camera/camera_info", sub_qos,
        std::bind(&ArucoUavImageNode::camera_info_callback, this, std::placeholders::_1));

    // 처리된 이미지 Publisher 생성 (Best Effort QoS 사용)
    processed_image_pub_ = this->create_publisher<sensor_msgs::msg::Image>(
        "/offboard_control/image_proc", rclcpp::QoS(1).best_effort());

    // ArUco 탐지기 초기화 (예: 6x6 크기, 250개 ID를 가진 딕셔너리 사용)
    aruco_dictionary_ = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);
    aruco_detector_params_ = cv::aruco::DetectorParameters::create();

    dynamic_tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    // ✨ 1. ROS 시뮬레이션 시간을 사용하는 시계를 명시적으로 생성합니다.
    clock_ = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);
    // ✨ 2. TF 버퍼가 이 시뮬레이션 시계를 사용하도록 함께 초기화합니다. (매우 중요)
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(clock_);
    tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);
  }

private:
  // TF 리스너
  tf2_ros::Buffer::SharedPtr tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;

  // 카메라 및 짐벌 제어 관련
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;
  cv::Mat camera_matrix_;
  cv::Mat dist_coeffs_;

  // 처리된 이미지를 발행할 Publisher
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr processed_image_pub_;
  // ArUco 마커 탐지기
  cv::Ptr<cv::aruco::Dictionary> aruco_dictionary_; // 이 라인으로 교체합니다.
  cv::Ptr<cv::aruco::DetectorParameters> aruco_detector_params_;

  // 마커의 실제 크기 (단위: 미터). Pose 추정을 위해 필수적입니다.
  float marker_size_ = 1.0f; // 예시: 10cm

  // 동적 TF를 발행할 broadcaster
  std::unique_ptr<tf2_ros::TransformBroadcaster> dynamic_tf_broadcaster_;
  // 우리가 만들 가상 프레임의 이름
  const std::string virtual_frame_id_ = "virtual_camera_at_base";

  rclcpp::Clock::SharedPtr clock_;

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

          // --- ✨ 사용자 요청을 반영한 최종 계산 로직 ✨ ---

          // 1. 필요한 TF 정보 가져오기 (이전과 동일)
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
          tf2::Vector3 v_camera_offset_in_base(0.00, 0.0, 0.0581);

          // 2a. "v_camera_offset_in_base 이거만 회전시키라고"
          //      base_link 기준의 카메라 오프셋을 camera_link 기준으로 회전 변환
          tf2::Vector3 v_rotated_camera_offset = tf2::quatRotate(q_cam_to_base, v_camera_offset_in_base);

          // 2. 최종적인 "camera_link -> 마커" 자세 계산
          // 2a. 위치 계산: 요청하신 대로, 회전 없이 평행 이동(벡터 덧셈)만 적용합니다.
          //    (cam->base 벡터) + (base->marker 벡터) = (cam->marker 벡터)
          tf2::Vector3 final_pos_in_cam = v_cam_to_base + v_base_to_marker_assumed + v_rotated_camera_offset; // + v_camera_offset_in_base

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

          tf2::Quaternion q_marker_abs;
          tf2::fromMsg(marker_pose_map.pose.orientation, q_marker_abs);

          // getRPY()는 라디안 단위의 롤, 피치, 요를 반환합니다.
          double marker_abs_roll_rad, marker_abs_pitch_rad, marker_abs_yaw_rad;
          tf2::Matrix3x3(q_marker_abs).getRPY(marker_abs_roll_rad, marker_abs_pitch_rad, marker_abs_yaw_rad);

          RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500,
                               "Marker ID: %d | Abs Pos(m): [%.2f, %.2f, %.2f]",
                               ids[i],
                               marker_pose_map.pose.position.x,
                               marker_pose_map.pose.position.y,
                               marker_pose_map.pose.position.z
          );
        }
        catch (const tf2::TransformException &ex)
        {
          RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                               "Could not transform marker pose: %s", ex.what());
        }
      }
    }

    // 1d. 처리된 이미지를 재발행
    processed_image_pub_->publish(*cv_ptr->toImageMsg());
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
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ArucoUavImageNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}