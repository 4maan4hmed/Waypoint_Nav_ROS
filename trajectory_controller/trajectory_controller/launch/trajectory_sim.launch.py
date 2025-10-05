

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, FindExecutable
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os


def generate_launch_description():
    """Generate launch description for trajectory controller simulation."""
    
    # Declare arguments with better descriptions
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    control_frequency = LaunchConfiguration('control_frequency', default='10.0')
    max_linear_vel = LaunchConfiguration('max_linear_vel', default='0.22')
    max_angular_vel = LaunchConfiguration('max_angular_vel', default='2.84')
    world_name = LaunchConfiguration('world_name', default='empty')
    launch_rviz = LaunchConfiguration('launch_rviz', default='true')
    
    # Set TurtleBot3 model
    turtlebot3_model = 'burger'
    os.environ['TURTLEBOT3_MODEL'] = turtlebot3_model
    
    # Find package directories with error handling
    try:
        trajectory_controller_dir = FindPackageShare('trajectory_controller').find('trajectory_controller')
        turtlebot3_gazebo_dir = FindPackageShare('turtlebot3_gazebo').find('turtlebot3_gazebo')
    except Exception as e:
        print(f"Error finding packages: {e}")
        # Fallback to common paths
        trajectory_controller_dir = os.path.expanduser('~/ros2_ws/src/trajectory_controller')
        turtlebot3_gazebo_dir = os.path.expanduser('~/ros2_ws/src/turtlebot3_simulations/turtlebot3_gazebo')
    
    # Include Gazebo launch file
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                turtlebot3_gazebo_dir,
                'launch',
                'turtlebot3_world.launch.py'
            ])
        ]),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'world_name': world_name,
        }.items()
    )
    
    # Spawn TurtleBot3 entity
    spawn_entity_node = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'turtlebot3_burger',
            '-x', '0.0',
            '-y', '0.0', 
            '-z', '0.01'
        ],
        output='screen'
    )
    
    # Trajectory controller node with better parameter organization
    trajectory_controller_node = Node(
        package='trajectory_controller',
        executable='trajectory_controller',
        name='trajectory_controller',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'control_frequency': control_frequency,
            'max_linear_vel': max_linear_vel,
            'max_angular_vel': max_angular_vel,
            'robot_model': turtlebot3_model,
            # Controller gains
            'kp_linear': 0.5,
            'kp_angular': 2.0,
            # Safety limits
            'min_linear_vel': 0.01,
            'min_angular_vel': 0.1,
        }],
        remappings=[
            ('/cmd_vel', '/turtlebot3_burger/cmd_vel'),
        ]
    )
    
    # RViz node with better configuration handling
    rviz_config_path = PathJoinSubstitution([
        FindPackageShare('trajectory_controller'),
        'rviz',
        'trajectory_view.rviz'
    ])
    
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_path],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=lambda context: LaunchConfiguration('launch_rviz').perform(context) == 'true'
    )
    
    # Create launch description
    ld = LaunchDescription([
        # Declare launch arguments
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'
        ),
        DeclareLaunchArgument(
            'control_frequency', 
            default_value='10.0',
            description='Controller frequency in Hz'
        ),
        DeclareLaunchArgument(
            'max_linear_vel',
            default_value='0.22',
            description='Maximum linear velocity in m/s'
        ),
        DeclareLaunchArgument(
            'max_angular_vel',
            default_value='2.84', 
            description='Maximum angular velocity in rad/s'
        ),
        DeclareLaunchArgument(
            'world_name',
            default_value='empty',
            description='Gazebo world name (empty, turtlebot3_world, etc.)'
        ),
        DeclareLaunchArgument(
            'launch_rviz',
            default_value='true',
            description='Whether to launch RViz2'
        ),
    ])
    
    # Add nodes in order
    ld.add_action(gazebo_launch)
    ld.add_action(spawn_entity_node)
    ld.add_action(trajectory_controller_node)
    ld.add_action(rviz_node)
    ld.add_action(waypoint_publisher_node)
    ld.add_action(record_waypoints_node)
    
    return ld