cmake_minimum_required(VERSION 3.8)
project(ur_mtc)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(control_msgs REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_task_constructor_core REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(pcl_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(mtc_test src/mtc_test.cpp)
ament_target_dependencies(mtc_test rclcpp moveit_core moveit_task_constructor_core moveit_ros_planning_interface pcl_ros pcl_conversions pcl_msgs sensor_msgs)

target_include_directories(mtc_test PUBLIC
$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
$<INSTALL_INTERFACE:include>)

target_compile_features(mtc_test PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

install(TARGETS mtc_test
		DESTINATION lib/${PROJECT_NAME})

add_executable(gripper_action src/gripper_action.cpp)
ament_target_dependencies(gripper_action rclcpp rclcpp_action control_msgs)

install(TARGETS gripper_action
		DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY 
        launch 
        config 
        DESTINATION share/${PROJECT_NAME})

include_directories(include ${PCL_INCLUDE_DIRS} /opt/ros/humble/include/pcl_msgs)

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

ament_export_include_directories(include)
ament_package()
