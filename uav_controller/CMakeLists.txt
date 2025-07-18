# ~/ros2_ws/src/uav_controller/CMakeLists.txt

cmake_minimum_required(VERSION 3.8)
project(uav_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# C++ 17 표준 사용
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 필요한 모든 패키지 찾기
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rcpputils REQUIRED)

# 노드 실행 파일 추가
add_executable(uav_controller src/uav_controller.cpp)

# C++ 코드에 소스 디렉토리 경로 전달
target_compile_definitions(uav_controller PRIVATE
  "PACKAGE_SOURCE_DIR=\"${CMAKE_CURRENT_SOURCE_DIR}\""
)

# ROS 2 패키지 의존성 연결
ament_target_dependencies(uav_controller
  rclcpp
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  px4_msgs
  sensor_msgs
  cv_bridge
  ament_index_cpp
  rcpputils
  OpenCV
)

# 생성된 실행 파일을 install 디렉토리에 설치
install(TARGETS
  uav_controller
  DESTINATION lib/${PROJECT_NAME}
)

# config 디렉토리(파라미터 YAML)를 install/share/.../config 로 복사
install(
  DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

  # launch 디렉토리(.py 런치 파일)를 install/share/.../launch 로 복사
install(
  DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
  FILES_MATCHING PATTERN "*.py"
)


add_executable(aruco_uav_image_node src/aruco_uav.cpp)
ament_target_dependencies(aruco_uav_image_node
  rclcpp
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  px4_msgs
  sensor_msgs
  cv_bridge
  ament_index_cpp
  rcpputils
  OpenCV
)

install(TARGETS aruco_uav_image_node
  DESTINATION lib/${PROJECT_NAME}
)

# ament 패키지로 선언
ament_package()