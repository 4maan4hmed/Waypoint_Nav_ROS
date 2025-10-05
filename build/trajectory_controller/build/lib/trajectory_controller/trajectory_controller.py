"""
Trajectory Controller for Differential Drive Robot
Implements path smoothing, trajectory generation, and tracking control.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry, Path
from visualization_msgs.msg import Marker, MarkerArray
import numpy as np
from scipy.interpolate import CubicSpline
import math

class PathSmoother:
    """
    Handles path smoothing operations using cubic spline interpolation.
    Separated for better modularity and testability.
    """
    
    @staticmethod
    def smooth_path(waypoints, num_points=100):
        """
        Generate smooth path from discrete waypoints using cubic spline.
        
        Args:
            waypoints: List of (x, y) tuples representing waypoints
            num_points: Number of points to sample on smooth path
            
        Returns:
            List of (x, y) tuples representing smoothed path
            
        Raises:
            ValueError: If waypoints list is invalid
        """
        if not waypoints or len(waypoints) < 2:
            raise ValueError("Need at least 2 waypoints for smoothing")
        
        try:
            waypoints = np.array(waypoints)
            x = waypoints[:, 0]
            y = waypoints[:, 1]
            
            # Parameterize by cumulative distance
            distances = [0]
            for i in range(1, len(waypoints)):
                dist = np.sqrt((x[i] - x[i-1])**2 + (y[i] - y[i-1])**2)
                distances.append(distances[-1] + dist)
            
            # Check for duplicate waypoints
            if distances[-1] < 1e-6:
                raise ValueError("Waypoints are too close together")
            
            # Create cubic splines
            if len(waypoints) > 3:
                cs_x = CubicSpline(distances, x)
                cs_y = CubicSpline(distances, y)
            else:
                # Fallback to linear for few points
                cs_x = lambda t: np.interp(t, distances, x)
                cs_y = lambda t: np.interp(t, distances, y)
            
            # Sample the smooth path
            t_smooth = np.linspace(0, distances[-1], num_points)
            x_smooth = cs_x(t_smooth)
            y_smooth = cs_y(t_smooth)
            
            return list(zip(x_smooth, y_smooth))
            
        except Exception as e:
            raise RuntimeError(f"Path smoothing failed: {str(e)}")


class TrajectoryGenerator:
    """
    Generates time-parameterized trajectories from smoothed paths.
    """
    
    @staticmethod
    def generate_trajectory(smooth_path, velocity=0.15):
        """
        Generate time-stamped trajectory with constant velocity profile.
        
        Args:
            smooth_path: List of (x, y) tuples
            velocity: Desired velocity in m/s
            
        Returns:
            List of (x, y, t) tuples
            
        Raises:
            ValueError: If inputs are invalid
        """
        if not smooth_path or len(smooth_path) < 2:
            raise ValueError("Need at least 2 points for trajectory")
        
        if velocity <= 0:
            raise ValueError("Velocity must be positive")
        
        try:
            trajectory = []
            t = 0.0
            
            trajectory.append((smooth_path[0][0], smooth_path[0][1], t))
            
            for i in range(1, len(smooth_path)):
                dx = smooth_path[i][0] - smooth_path[i-1][0]
                dy = smooth_path[i][1] - smooth_path[i-1][1]
                distance = np.sqrt(dx**2 + dy**2)
                dt = distance / velocity
                t += dt
                trajectory.append((smooth_path[i][0], smooth_path[i][1], t))
            
            return trajectory
            
        except Exception as e:
            raise RuntimeError(f"Trajectory generation failed: {str(e)}")


class TrajectoryTracker:
    """
    Pure pursuit-like trajectory tracking controller.
    """
    
    def __init__(self, kp_linear=0.5, kp_angular=2.0, 
                 max_linear_vel=0.22, max_angular_vel=2.84,
                 position_tolerance=0.1):
        """
        Initialize controller with gains and limits.
        
        Args:
            kp_linear: Proportional gain for linear velocity
            kp_angular: Proportional gain for angular velocity
            max_linear_vel: Maximum linear velocity (m/s)
            max_angular_vel: Maximum angular velocity (rad/s)
            position_tolerance: Distance threshold to consider target reached (m)
        """
        self.kp_linear = kp_linear
        self.kp_angular = kp_angular
        self.max_linear_vel = max_linear_vel
        self.max_angular_vel = max_angular_vel
        self.position_tolerance = position_tolerance
    
    def compute_control(self, current_x, current_y, current_theta,
                       target_x, target_y):
        """
        Compute velocity commands to reach target.
        
        Args:
            current_x, current_y: Current robot position
            current_theta: Current robot heading (rad)
            target_x, target_y: Target position
            
        Returns:
            tuple: (linear_vel, angular_vel, distance_to_target)
        """
        # Calculate distance and angle to target
        dx = target_x - current_x
        dy = target_y - current_y
        distance = np.sqrt(dx**2 + dy**2)
        
        # Calculate desired heading
        desired_theta = math.atan2(dy, dx)
        
        # Angular error (normalized to [-pi, pi])
        theta_error = desired_theta - current_theta
        theta_error = math.atan2(math.sin(theta_error), math.cos(theta_error))
        
        # Linear velocity (reduce when turning sharply)
        linear_vel = min(self.kp_linear * distance, self.max_linear_vel)
        if abs(theta_error) > 0.5:
            linear_vel *= 0.5
        
        # Angular velocity
        angular_vel = self.kp_angular * theta_error
        angular_vel = np.clip(angular_vel, -self.max_angular_vel, self.max_angular_vel)
        
        return linear_vel, angular_vel, distance


class TrajectoryController(Node):
    """
    Main ROS2 node for trajectory control.
    """
    
    def __init__(self):
        super().__init__('trajectory_controller')
        
        # Declare and get parameters
        self.declare_parameter('control_frequency', 10.0)
        self.declare_parameter('max_linear_vel', 0.22)
        self.declare_parameter('max_angular_vel', 2.84)
        
        control_freq = self.get_parameter('control_frequency').value
        max_lin = self.get_parameter('max_linear_vel').value
        max_ang = self.get_parameter('max_angular_vel').value
        
        # Publishers and Subscribers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.path_pub = self.create_publisher(Path, '/smooth_path', 10)
        self.waypoint_pub = self.create_publisher(MarkerArray, '/waypoints', 10)
        self.trajectory_pub = self.create_publisher(MarkerArray, '/trajectory_points', 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', 
                                                  self.odom_callback, 10)
        
        # Robot state
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_theta = 0.0
        self.odom_received = False
        
        # Define waypoints
        self.waypoints = [
            (0.0, 0.0), (1.0, 0.0), (2.0, 1.0), (3.0, 1.0), (4.0, 2.0),
            (4.0, 3.0), (3.0, 4.0), (2.0, 4.0), (1.0, 3.0), (0.0, 2.0)
        ]
        
        # Initialize components
        self.smoother = PathSmoother()
        self.traj_gen = TrajectoryGenerator()
        self.tracker = TrajectoryTracker(
            max_linear_vel=max_lin,
            max_angular_vel=max_ang
        )
        
        # Generate trajectory
        try:
            smooth_path = self.smoother.smooth_path(self.waypoints, num_points=200)
            self.trajectory = self.traj_gen.generate_trajectory(smooth_path, velocity=0.15)
            self.current_target_idx = 0
            self.trajectory_complete = False
            self.get_logger().info(f'Generated {len(self.trajectory)} trajectory points')
        except Exception as e:
            self.get_logger().error(f'Failed to generate trajectory: {str(e)}')
            raise
        
        # Timers
        control_period = 1.0 / control_freq
        self.timer = self.create_timer(control_period, self.control_loop)
        self.viz_timer = self.create_timer(1.0, self.publish_visualizations)
        
        self.get_logger().info('Trajectory Controller initialized successfully')
    
    def odom_callback(self, msg):
        """Update robot's current pose from odometry."""
        try:
            self.current_x = msg.pose.pose.position.x
            self.current_y = msg.pose.pose.position.y
            
            # Extract yaw from quaternion
            orientation = msg.pose.pose.orientation
            siny_cosp = 2 * (orientation.w * orientation.z + orientation.x * orientation.y)
            cosy_cosp = 1 - 2 * (orientation.y**2 + orientation.z**2)
            self.current_theta = math.atan2(siny_cosp, cosy_cosp)
            
            self.odom_received = True
            
        except Exception as e:
            self.get_logger().error(f'Error in odometry callback: {str(e)}')
    
    def control_loop(self):
        """Main control loop executing trajectory tracking."""
        # Wait for first odometry message
        if not self.odom_received:
            self.get_logger().warn('Waiting for odometry...', throttle_duration_sec=2.0)
            return
        
        # Check if trajectory is complete
        if self.trajectory_complete or self.current_target_idx >= len(self.trajectory):
            if not self.trajectory_complete:
                self.stop_robot()
                self.get_logger().info('Trajectory completed successfully!')
                self.trajectory_complete = True
            return
        
        try:
            # Get target point
            target_x, target_y, target_t = self.trajectory[self.current_target_idx]
            
            # Compute control
            linear_vel, angular_vel, distance = self.tracker.compute_control(
                self.current_x, self.current_y, self.current_theta,
                target_x, target_y
            )
            
            # Check if reached current target
            if distance < self.tracker.position_tolerance:
                self.current_target_idx += 1
                if self.current_target_idx < len(self.trajectory):
                    progress = (self.current_target_idx / len(self.trajectory)) * 100
                    self.get_logger().info(
                        f'Progress: {progress:.1f}% ({self.current_target_idx}/{len(self.trajectory)})',
                        throttle_duration_sec=1.0
                    )
                return
            
            # Publish velocity command
            twist = Twist()
            twist.linear.x = linear_vel
            twist.angular.z = angular_vel
            self.cmd_vel_pub.publish(twist)
            
        except Exception as e:
            self.get_logger().error(f'Error in control loop: {str(e)}')
            self.stop_robot()
    
    def stop_robot(self):
        """Send zero velocity command to stop the robot."""
        twist = Twist()
        self.cmd_vel_pub.publish(twist)
    
    def publish_visualizations(self):
        """Publish RViz visualizations for path and waypoints."""
        try:
            # Publish smooth path
            path_msg = Path()
            path_msg.header.frame_id = 'odom'
            path_msg.header.stamp = self.get_clock().now().to_msg()
            
            for x, y, t in self.trajectory:
                pose = PoseStamped()
                pose.header.frame_id = 'odom'
                pose.pose.position.x = x
                pose.pose.position.y = y
                pose.pose.position.z = 0.0
                path_msg.poses.append(pose)
            
            self.path_pub.publish(path_msg)
            
            # Publish waypoint markers
            self._publish_waypoint_markers()
            
            # Publish trajectory point markers
            self._publish_trajectory_markers()
            
        except Exception as e:
            self.get_logger().error(f'Error publishing visualizations: {str(e)}')
    
    def _publish_waypoint_markers(self):
        """Publish original waypoints as red spheres."""
        waypoint_markers = MarkerArray()
        
        for i, (x, y) in enumerate(self.waypoints):
            # Sphere marker
            marker = Marker()
            marker.header.frame_id = 'odom'
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = 'waypoints'
            marker.id = i
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position.x = x
            marker.pose.position.y = y
            marker.pose.position.z = 0.2
            marker.scale.x = 0.15
            marker.scale.y = 0.15
            marker.scale.z = 0.15
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
            marker.color.a = 1.0
            waypoint_markers.markers.append(marker)
            
            # Text label
            text_marker = Marker()
            text_marker.header.frame_id = 'odom'
            text_marker.header.stamp = self.get_clock().now().to_msg()
            text_marker.ns = 'waypoint_labels'
            text_marker.id = i + 1000
            text_marker.type = Marker.TEXT_VIEW_FACING
            text_marker.action = Marker.ADD
            text_marker.pose.position.x = x
            text_marker.pose.position.y = y
            text_marker.pose.position.z = 0.4
            text_marker.scale.z = 0.15
            text_marker.color.r = 1.0
            text_marker.color.g = 1.0
            text_marker.color.b = 1.0
            text_marker.color.a = 1.0
            text_marker.text = f'WP{i}'
            waypoint_markers.markers.append(text_marker)
        
        self.waypoint_pub.publish(waypoint_markers)
    
    def _publish_trajectory_markers(self):
        """Publish trajectory points as green spheres."""
        traj_markers = MarkerArray()
        
        for i in range(0, len(self.trajectory), 10):
            x, y, t = self.trajectory[i]
            marker = Marker()
            marker.header.frame_id = 'odom'
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = 'trajectory'
            marker.id = i
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position.x = x
            marker.pose.position.y = y
            marker.pose.position.z = 0.05
            marker.scale.x = 0.05
            marker.scale.y = 0.05
            marker.scale.z = 0.05
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 0.0
            marker.color.a = 0.7
            traj_markers.markers.append(marker)
        
        self.trajectory_pub.publish(traj_markers)


def main(args=None):
    rclpy.init(args=args)
    
    try:
        controller = TrajectoryController()
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'Error: {str(e)}')
    finally:
        try:
            controller.stop_robot()
            controller.destroy_node()
        except:
            pass
        rclpy.shutdown()


if __name__ == '__main__':
    main()