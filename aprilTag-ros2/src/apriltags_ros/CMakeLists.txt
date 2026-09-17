cmake_minimum_required(VERSION 3.8)
project(apriltags_ros)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)

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

# *Install files
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)
install(DIRECTORY include/
  DESTINATION share/${PROJECT_NAME}/include
)
install(DIRECTORY scripts/
  DESTINATION share/${PROJECT_NAME}/scripts)


# *Install python node
install(PROGRAMS
    scripts/base2camera_calibration.py
    scripts/end2camera_calibration.py
	  DESTINATION lib/${PROJECT_NAME}
	)

ament_package()
