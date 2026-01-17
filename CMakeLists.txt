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

add_executable(get_plan src/getplan.cpp)
add_executable(get_plan_and_execute src/getplan_and_execute.cpp)
add_executable(move_action_node src/move_action_node.cpp)
add_executable(ask_charge_action_node src/ask_charge_action_node.cpp)
add_executable(charge_action_node src/charge_action_node.cpp)
add_executable(move_action_node_2 src/move_action_node_2.cpp)
add_executable(move_action_node_3 src/move_action_node_3.cpp)
add_executable(detect_id src/detect_id.cpp)
add_executable(carms src/carms.cpp)

ament_target_dependencies(get_plan
  rclcpp
  plansys2_pddl_parser
  plansys2_msgs
  plansys2_domain_expert
  plansys2_planner
  plansys2_problem_expert
)

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

ament_target_dependencies(move_action_node
  rclcpp
  rclcpp_action
  plansys2_executor
  plansys2_msgs
)

ament_target_dependencies(ask_charge_action_node
  rclcpp
  rclcpp_action
  plansys2_executor
  plansys2_msgs
)

ament_target_dependencies(charge_action_node
  rclcpp
  rclcpp_action
  plansys2_executor
  plansys2_msgs
)

ament_target_dependencies(move_action_node_2
  rclcpp
  rclcpp_action
  plansys2_executor
  plansys2_msgs
  geometry_msgs
)

ament_target_dependencies(move_action_node_3
  rclcpp
  rclcpp_action
  plansys2_executor
  plansys2_msgs
  geometry_msgs
  nav2_msgs
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

ament_target_dependencies(carms
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
  DESTINATION share/${PROJECT_NAME}
)

install(TARGETS
  get_plan get_plan_and_execute move_action_node ask_charge_action_node charge_action_node move_action_node_2 move_action_node_3 detect_id carms
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
