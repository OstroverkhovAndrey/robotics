cmake_minimum_required(VERSION 3.8)
project(detect_aruco)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(OpenCV REQUIRED)

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

#include_directories( ${OpenCV_INCLUDE_DIRS} )
add_executable(image_publisher src/image_publisher.cpp)
add_executable(detect_aruco src/detect_aruco.cpp)

#target_link_libraries(image_publisher ${OpenCV_LIBS} )
#target_link_libraries(detect_aruco ${OpenCV_LIBS} )
#target_link_directories(image_publisher /usr/lib/x86_64-linux-gnu/ )
#target_link_directories(detect_aruco /usr/lib/x86_64-linux-gnu/ )

ament_target_dependencies(image_publisher rclcpp cv_bridge sensor_msgs OpenCV)
ament_target_dependencies(detect_aruco rclcpp cv_bridge sensor_msgs OpenCV)

install(TARGETS
    image_publisher
    detect_aruco
    DESTINATION lib/${PROJECT_NAME})

ament_package()
