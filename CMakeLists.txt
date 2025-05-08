cmake_minimum_required(VERSION 3.8)
project(carla_behavior)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 빌드시 compile_commands.json 생성
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(behaviortree_ros2 REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(warning_mode_interfaces REQUIRED)
# find_package(rosidl_default_generators REQUIRED)
# find_package(unique_identifier_msgs REQUIRED)
# find_package(action_msgs REQUIRED)


# rosidl_generate_interfaces(${PROJECT_NAME}
#   "src/warning_mode_interfaces/action/WarningMode.action"
#   DEPENDENCIES builtin_interfaces unique_identifier_msgs action_msgs
# )



# 실행 파일 빌드 추가안하면 Include 에러 생김
add_executable(carla_behavior_node
  src/main.cpp
  src/checkRiskLevel.cpp
  src/checkRiskLevelWarning.cpp
  src/checkRiskLevelEmergency.cpp
  src/warningMode.cpp
  src/shoulderShift.cpp
  src/shoulderAlign.cpp
  src/handBrakePark.cpp
)

# include 디렉토리 추가
# target_include_directories(carla_behavior_node PUBLIC
#   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#   $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp>
#   $<INSTALL_INTERFACE:include>
# )

# 의존성 링크
ament_target_dependencies(carla_behavior_node
  rclcpp
  rclcpp_action
  std_msgs
  behaviortree_cpp
  behaviortree_ros2
  sensor_msgs
  ament_index_cpp
  action_msgs #이것도 필요?
  warning_mode_interfaces
) 

# # 생성된 인터페이스를 링크
# rosidl_target_interfaces(carla_behavior_node
#   ${PROJECT_NAME}
#   "rosidl_typesupport_cpp"
# )

# 실행 파일 설치
install(
  TARGETS carla_behavior_node
  DESTINATION lib/${PROJECT_NAME}
)

# behavior_trees 디렉토리 설치
install(
  DIRECTORY behavior_trees/
  DESTINATION share/${PROJECT_NAME}/behavior_trees
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_cpplint_FOUND TRUE)
  set(ament_cmake_cppcheck_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
