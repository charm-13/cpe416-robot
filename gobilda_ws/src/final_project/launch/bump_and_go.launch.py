from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    
    return LaunchDescription([
        # Node(
        #     package='final_project',
        #     executable='obstacle_detector',
        # ),
        # Node(
        #     package='final_project',
        #     executable='bump_and_go',
        # )
        Node(
            package='final_project',
            executable='robot',
        )
    ])