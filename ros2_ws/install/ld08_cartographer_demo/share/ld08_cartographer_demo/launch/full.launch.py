from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Expand '~' so users can pass ~/path
    default_cfg_dir = os.path.expanduser('~/Documents/3semester/cartographer/carto_cfg')
    default_cfg_base = 'ld08_2d.lua'

    use_sim_time = LaunchConfiguration('use_sim_time')
    configuration_directory = LaunchConfiguration('configuration_directory')
    configuration_basename = LaunchConfiguration('configuration_basename')
    scan_topic = LaunchConfiguration('scan_topic')
    start_rviz = LaunchConfiguration('start_rviz')

    # Try to include the LD08 driver's own launch if available
    ld08_driver_launch = os.path.join(
        get_package_share_directory('ld08_driver'),
        'launch', 'ld08.launch.py'
    )

    ld08_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(ld08_driver_launch)
    )

    # Static TF: base_link -> base_scan (matches the frame_id seen on your /scan)
    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='ld08_static_tf',
        arguments=[
            '--x','0','--y','0','--z','0',
            '--qx','0','--qy','0','--qz','0','--qw','1',
            '--frame-id','base_link','--child-frame-id','base_scan'
        ],
        output='screen'
    )

    cartographer = Node(
        package='cartographer_ros',
        executable='cartographer_node',
        name='cartographer_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=[
            '-configuration_directory', configuration_directory,
            '-configuration_basename', configuration_basename
        ],
        remappings=[('scan', scan_topic)]
    )

    occupancy_grid = Node(
        package='cartographer_ros',
        executable='cartographer_occupancy_grid_node',
        name='cartographer_occupancy_grid_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time,
                     'resolution': 0.05,
                     'publish_period_sec': 1.0}]
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        condition=None  # we keep it optional via start_rviz argument
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('configuration_directory', default_value=default_cfg_dir),
        DeclareLaunchArgument('configuration_basename', default_value=default_cfg_base),
        DeclareLaunchArgument('scan_topic', default_value='/scan'),
        DeclareLaunchArgument('start_rviz', default_value='true'),

        ld08_node,
        static_tf,
        cartographer,
        occupancy_grid,
        # If you want start_rviz gate: use IfCondition(start_rviz)
        # For simplicity we always start rviz; tune later if needed.
        rviz
    ])