cmake_minimum_required(VERSION 3.8)
project(plansys_interface)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(plansys2_pddl_parser REQUIRED)
find_package(plansys2_msgs REQUIRED)
find_package(plansys2_domain_expert REQUIRED)
find_package(plansys2_planner REQUIRED)
find_package(plansys2_problem_expert REQUIRED)
find_package(plansys2_executor REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(image_transport REQUIRED)
find_package(action_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

add_executable(get_plan_and_execute src/getplan_and_execute.cpp)
add_executable(detect_id src/detect_id.cpp)
add_executable(change_state src/change_state.cpp)
add_executable(capture_first src/capture_first.cpp)
add_executable(capture_other src/capture_other.cpp)
add_executable(go_to_point_action src/go_to_point_action.cpp)

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/GoToPoint.action"
  DEPENDENCIES geometry_msgs action_msgs
)
rosidl_get_typesupport_target(cpp_typesupport_target "${PROJECT_NAME}" "rosidl_typesupport_cpp")
target_link_libraries(go_to_point_action "${cpp_typesupport_target}")
target_link_libraries(detect_id "${cpp_typesupport_target}")
target_link_libraries(capture_first "${cpp_typesupport_target}")  
target_link_libraries(capture_other "${cpp_typesupport_target}")

ament_target_dependencies(get_plan_and_execute
  rclcpp
  plansys2_pddl_parser
  plansys2_msgs
  plansys2_domain_expert
  plansys2_planner
  plansys2_problem_expert
  plansys2_executor
  rclcpp_action
)

ament_target_dependencies(detect_id
  rclcpp
  rclcpp_action
  plansys2_executor
  plansys2_msgs
  geometry_msgs
  nav2_msgs
  sensor_msgs 
  cv_bridge OpenCV
  image_transport
)

ament_target_dependencies(change_state
  rclcpp
  rclcpp_action
  plansys2_executor
  plansys2_msgs
  geometry_msgs
  nav2_msgs
  sensor_msgs 
  cv_bridge OpenCV
  image_transport
)

ament_target_dependencies(capture_first
  rclcpp
  rclcpp_action
  plansys2_executor
  plansys2_msgs
  geometry_msgs
  nav2_msgs
  sensor_msgs 
  cv_bridge OpenCV
  image_transport
)

ament_target_dependencies(capture_other
  rclcpp
  rclcpp_action
  plansys2_executor
  plansys2_msgs
  geometry_msgs
  nav2_msgs
  sensor_msgs 
  cv_bridge OpenCV
  image_transport
)


ament_target_dependencies(go_to_point_action
  rclcpp
  rclcpp_action
  plansys2_executor
  plansys2_msgs
  geometry_msgs
  nav2_msgs
  sensor_msgs 
  cv_bridge OpenCV
  image_transport
)

install(DIRECTORY
  domain
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

install(TARGETS
  get_plan_and_execute detect_id change_state capture_first capture_other go_to_point_action
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
