from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='tb3_remote_control',
            executable='drive_forward_10cm',
            name='drive_forward_10cm',
            parameters=[{'distance_m': 0.10, 'speed_mps': 0.05}],
            output='screen'
        )
    ])
