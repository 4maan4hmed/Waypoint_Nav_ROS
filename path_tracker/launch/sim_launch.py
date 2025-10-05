import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    
    turtlebot3_gazebo_dir = get_package_share_directory('turtlebot3_gazebo')
    
    world = os.path.join(
        turtlebot3_gazebo_dir,
        'worlds',
        'empty_world.world'
    )
    
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(turtlebot3_gazebo_dir, 'launch', 'empty_world.launch.py')
        )
    )
    
    path_tracker_node = Node(
        package='path_tracker',
        executable='path_tracker_node',
        output='screen'
    )
    
    return LaunchDescription([
        gazebo_launch,
        path_tracker_node
    ])