from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    obstacle_detection = Node(
        package="final_project",
        executable="obstacle_detector",
    )

    roomba = Node(
        package="final_project",
        executable="bump_and_go",
    )

    drive_test = Node(
        package="final_project",
        executable="robot",
    )
    
    nodes = [
        # obstacle_detection, # uncomment to test lidar
        # roomba,   # uncomment to test lidar
        drive_test
    ]

    return LaunchDescription(nodes)
