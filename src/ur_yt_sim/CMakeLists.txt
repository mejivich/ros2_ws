cmake_minimum_required(VERSION 3.8)
project(ur_yt_sim)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(xacro REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(gazebo_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(ament_cmake_python REQUIRED)



install(DIRECTORY launch config worlds urdf meshes models rviz srdf
  DESTINATION share/${PROJECT_NAME})



install(PROGRAMS
  scripts/hri_safety.py
  scripts/fake_human.py
  scripts/hri_bridge.py

  # add more scripts here
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# Explicitly install the initial positions file
install(FILES config/initial_positions.yaml
  DESTINATION share/${PROJECT_NAME}/config
)

ament_package()
