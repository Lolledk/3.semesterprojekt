# launch/cartographer_ld08.launch.py
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    configuration_directory = LaunchConfiguration('configuration_directory')
    configuration_basename = LaunchConfiguration('configuration_basename')
    scan_topic = LaunchConfiguration('scan_topic', default='/scan')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('configuration_directory', default_value=os.path.join(
            os.getenv('PWD'), 'config')),
        DeclareLaunchArgument('configuration_basename', default_value='ld08_2d.lua'),
        DeclareLaunchArgument('scan_topic', default_value='/scan'),

        Node(
            package='cartographer_ros',
            executable='cartographer_node',
            name='cartographer_node',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
            arguments=[
                '-configuration_directory', configuration_directory,
                '-configuration_basename', configuration_basename,
            ],
            remappings=[('scan', scan_topic)]
        ),

        Node(
            package='cartographer_ros',
            executable='occupancy_grid_node',
            name='occupancy_grid_node',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time,
                         'resolution': 0.05,        # 5 cm grid
                         'publish_period_sec': 1.0}]
        ),
    ])