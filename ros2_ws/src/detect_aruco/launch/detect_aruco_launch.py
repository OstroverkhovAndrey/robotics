from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="detect_aruco",
            executable="image_publisher",
            name="image_publisher",
            #output="screen",
            #emulate_tty=True,
        ),
        Node(
            package="detect_aruco",
            executable="sub_images_and_detect_aruco",
            name="sub_images_and_detect_aruco",
            #output="screen",
            #emulate_tty=True,
        ),
        Node(
            package="detect_aruco",
            executable="estimation_robot_position",
            name="estimation_robot_position",
            #output="screen",
            #emulate_tty=True,
        )
    ])
