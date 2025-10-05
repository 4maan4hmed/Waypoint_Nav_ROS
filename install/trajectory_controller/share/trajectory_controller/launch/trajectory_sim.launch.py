from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os

def generate_launch_description():
    os.environ['TURTLEBOT3_MODEL'] = 'burger'
    
    turtlebot3_gazebo_dir = FindPackageShare('turtlebot3_gazebo')
    
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                turtlebot3_gazebo_dir, 'launch', 'empty_world.launch.py'
            ])
        ])
    )
    
    trajectory_controller_node = Node(
        package='trajectory_controller',
        executable='trajectory_controller',
        name='trajectory_controller',
        output='screen'
    )
    
    return LaunchDescription([
        gazebo_launch,
        trajectory_controller_node,
    ])
