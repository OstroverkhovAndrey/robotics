cmake_minimum_required(VERSION 3.8)
project(detect_aruco)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(msg_files
  "msg/MarkerInImage.msg"
  "msg/MarkersInImage.msg"
  "msg/PointInImage.msg"
)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(OpenCV 4.5.4 EXACT REQUIRED )
#find_package(OpenCV 4.5.4 EXACT REQUIRED PATHS /usr/lib/x86_64-linux-gnu/cmake/opencv4/OpenCVConfig.cmake)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

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

#include_directories( /usr/include/opencv4/ )
#link_directories( /usr/lib/x86_64-linux-gnu/ )

add_executable(image_publisher src/image_publisher.cpp)
add_executable(sub_images_and_detect_aruco src/detect_aruco.cpp)
add_executable(estimation_robot_position src/estimation_robot_position.cpp)

#target_link_directories(image_publisher PUBLIC /usr/lib/x86_64-linux-gnu/ )
#target_link_directories(detect_aruco PUBLIC /usr/lib/x86_64-linux-gnu/ )
#message(${OpenCV_VERSION})
#message(${OpenCV_INCLUDE_DIRECTORIES})
#message(${OpenCV_LIBRARIES})
#message(${OpenCV_LIBS})
#target_link_libraries(image_publisher ${OpenCV_LIBS} )
#target_link_libraries(detect_aruco ${OpenCV_LIBS} )

ament_target_dependencies(image_publisher rclcpp cv_bridge sensor_msgs OpenCV)
ament_target_dependencies(sub_images_and_detect_aruco rclcpp cv_bridge sensor_msgs OpenCV)
ament_target_dependencies(estimation_robot_position rclcpp cv_bridge sensor_msgs OpenCV geometry_msgs rclcpp tf2 tf2_ros)

install(TARGETS
    image_publisher
    sub_images_and_detect_aruco
    estimation_robot_position
    DESTINATION lib/${PROJECT_NAME})

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
)
ament_export_dependencies(rosidl_default_runtime)

rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} rosidl_typesupport_cpp)
target_link_libraries(sub_images_and_detect_aruco "${cpp_typesupport_target}")
target_link_libraries(estimation_robot_position "${cpp_typesupport_target}")

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
