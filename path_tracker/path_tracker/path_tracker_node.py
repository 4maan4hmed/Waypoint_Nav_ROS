#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
import numpy as np
import math

from path_tracker.path_smoother import PathSmoother
from path_tracker.trajectory_controller import TrajectoryController

class PathTrackerNode(Node):
    def __init__(self):
        super().__init__('path_tracker_node')
        
        self.cmd_pub = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.path_pub = self.create_publisher(Path, '/smooth_path', 10)
        self.pose_sub = self.create_subscription(Pose, '/turtle1/pose', self.pose_callback, 10)
        
        self.marker_turtles = []
        self.path_drawn = False
        
        self.smoother = PathSmoother(
        num_points=200,
        smoothing_factor=0.1,
        max_curvature=2.0
)
        self.controller = TrajectoryController(k_linear=1.0, k_angular=4.0, lookahead_dist=0.5)
        
        # Turtlesim boundaries (with safety margin)
        self.min_x = 0.5
        self.max_x = 10.5
        self.min_y = 0.5
        self.max_y = 10.5
        
        self.current_pos = np.array([5.5, 5.5])
        self.current_yaw = 0.0
        self.current_idx = 0
        self.trajectory = None
        
        # Safe waypoints that avoid walls (staying within 0.5-10.5 boundaries)
        self.waypoints = [
            (2.0, 2.0),      # Start from bottom-left (safe from left wall)
            (8.0, 2.0),      # Move right (safe from bottom wall)
            (8.0, 8.0),      # Move up (safe from right wall)
            (2.0, 8.0),      # Move left (safe from top wall)
            (5.5, 8.0),      # Move to center-top
            (5.5, 5.5),      # Move down to center
            (2.0, 5.5),      # Move left to left-center
            (2.0, 2.0)       # Return to start (safe from left wall)
        ]
        
        # Alternative safer circular path:
        # self.waypoints = self.generate_circular_path(5.5, 5.5, 3.0, 16)
        
        self.timer = self.create_timer(0.05, self.control_loop)
        self.create_timer(1.0, self.draw_path)
        
        self.setup_trajectory()
        
        self.get_logger().info('Path Tracker Node Started for Turtlesim')
        self.get_logger().info(f'Waypoints: {self.waypoints}')
    
    def generate_circular_path(self, center_x, center_y, radius, num_points):
        """Generate a circular path that stays within boundaries"""
        waypoints = []
        for i in range(num_points + 1):
            angle = 2 * math.pi * i / num_points
            x = center_x + radius * math.cos(angle)
            y = center_y + radius * math.sin(angle)
            # Ensure the point is within boundaries
            x = max(self.min_x, min(self.max_x, x))
            y = max(self.min_y, min(self.max_y, y))
            waypoints.append((x, y))
        return waypoints
    
    def is_point_safe(self, x, y):
        """Check if a point is within safe boundaries"""
        return (self.min_x <= x <= self.max_x and 
                self.min_y <= y <= self.max_y)
    
    def setup_trajectory(self):
        # Convert waypoints to numpy arrays for the smoother
        waypoint_array = np.array(self.waypoints)
        
        smooth_path = self.smoother.smooth(waypoint_array)
        
        # Validate smoothed path points and convert to list of tuples for trajectory generation
        validated_path = []
        for point in smooth_path:
            if self.is_point_safe(point[0], point[1]):
                validated_path.append((point[0], point[1]))
            else:
                # Adjust point to be within boundaries
                safe_x = max(self.min_x, min(self.max_x, point[0]))
                safe_y = max(self.min_y, min(self.max_y, point[1]))
                validated_path.append((safe_x, safe_y))
                self.get_logger().warning(f'Adjusted path point ({point[0]:.2f}, {point[1]:.2f}) to ({safe_x:.2f}, {safe_y:.2f})')
        
        # Convert validated path back to numpy array for trajectory generation
        validated_array = np.array(validated_path)
        self.trajectory = self.smoother.generate_trajectory(validated_array, velocity=1.0)
        self.publish_path(validated_path)
        self.get_logger().info(f'Generated safe trajectory with {len(self.trajectory)} points')
    
    def publish_path(self, smooth_path):
        path_msg = Path()
        path_msg.header.frame_id = 'world'
        path_msg.header.stamp = self.get_clock().now().to_msg()
        
        for point in smooth_path:
            pose = PoseStamped()
            pose.header = path_msg.header
            pose.pose.position.x = float(point[0])
            pose.pose.position.y = float(point[1])
            path_msg.poses.append(pose)
        
        self.path_pub.publish(path_msg)
    
    def pose_callback(self, msg):
        self.current_pos[0] = msg.x
        self.current_pos[1] = msg.y
        self.current_yaw = msg.theta
        
        # Emergency stop if too close to walls
        if (msg.x < self.min_x + 0.2 or msg.x > self.max_x - 0.2 or
            msg.y < self.min_y + 0.2 or msg.y > self.max_y - 0.2):
            self.get_logger().warn(f'Too close to wall! Position: ({msg.x:.2f}, {msg.y:.2f})')
    
    def draw_path(self):
        if self.path_drawn or self.trajectory is None:
            return
        
        from turtlesim.srv import Spawn
        
        for i, waypoint in enumerate(self.waypoints):
            spawn_client = self.create_client(Spawn, '/spawn')
            if spawn_client.wait_for_service(timeout_sec=1.0):
                request = Spawn.Request()
                request.x = float(waypoint[0])
                request.y = float(waypoint[1])
                request.theta = 0.0
                request.name = f'waypoint_{i}'
                spawn_client.call_async(request)
        
        # Only draw every 30th point to avoid clutter
        for i in range(0, len(self.trajectory), 30):
            point = self.trajectory[i]
            if self.is_point_safe(point['x'], point['y']):
                spawn_client = self.create_client(Spawn, '/spawn')
                if spawn_client.wait_for_service(timeout_sec=1.0):
                    request = Spawn.Request()
                    request.x = float(point['x'])
                    request.y = float(point['y'])
                    request.theta = 0.0
                    request.name = f'path_{i}'
                    spawn_client.call_async(request)
        
        self.path_drawn = True
        self.get_logger().info('Safe path visualization spawned')
    
    def control_loop(self):
        if self.trajectory is None:
            return
        
        # Emergency stop if outside boundaries
        if not self.is_point_safe(self.current_pos[0], self.current_pos[1]):
            self.get_logger().error('Turtle outside safe boundaries! Emergency stop.')
            cmd = Twist()
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            self.cmd_pub.publish(cmd)
            return
        
        linear_vel, angular_vel, new_idx = self.controller.compute_control(
            self.current_pos, self.current_yaw, self.trajectory, self.current_idx
        )
        
        self.current_idx = new_idx
        
        # Reduce speed when approaching walls
        wall_margin = 1.0
        near_wall = (self.current_pos[0] < self.min_x + wall_margin or 
                     self.current_pos[0] > self.max_x - wall_margin or
                     self.current_pos[1] < self.min_y + wall_margin or 
                     self.current_pos[1] > self.max_y - wall_margin)
        
        if near_wall:
            linear_vel *= 0.5  # Reduce speed near walls
            angular_vel *= 0.8
        
        target = np.array([self.trajectory[self.current_idx]['x'], 
                          self.trajectory[self.current_idx]['y']])
        dist = np.linalg.norm(target - self.current_pos)
        
        self.get_logger().info(
            f'Pos: ({self.current_pos[0]:.2f}, {self.current_pos[1]:.2f}) | '
            f'Target[{self.current_idx}/{len(self.trajectory)-1}]: ({target[0]:.2f}, {target[1]:.2f}) | '
            f'Dist: {dist:.2f} | Vel: ({linear_vel:.2f}, {angular_vel:.2f})',
            throttle_duration_sec=0.5
        )
        
        cmd = Twist()
        
        if self.current_idx >= len(self.trajectory) - 1 and dist < 0.2:
            self.get_logger().info('=== Trajectory completed! ===')
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            self.cmd_pub.publish(cmd)
            self.timer.cancel()
        else:
            cmd.linear.x = linear_vel
            cmd.angular.z = angular_vel
            self.cmd_pub.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = PathTrackerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()