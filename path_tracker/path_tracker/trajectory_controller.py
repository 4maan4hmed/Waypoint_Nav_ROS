import numpy as np

class TrajectoryController:
    def __init__(self, k_linear=1.5, k_angular=6.0, lookahead_dist=0.8):
        self.k_linear = k_linear
        self.k_angular = k_angular
        self.lookahead_dist = lookahead_dist
        self.reached_threshold = 0.3
    
    def find_target_point(self, trajectory, current_pos, current_idx):
        for i in range(current_idx, len(trajectory)):
            point = np.array([trajectory[i]['x'], trajectory[i]['y']])
            dist = np.linalg.norm(point - current_pos)
            
            if dist < self.reached_threshold and i < len(trajectory) - 1:
                current_idx = i + 1
                continue
            
            if dist >= self.lookahead_dist or i == len(trajectory) - 1:
                return i
        
        return len(trajectory) - 1
    
    def compute_control(self, current_pos, current_yaw, trajectory, current_idx):
        if current_idx >= len(trajectory):
            return 0.0, 0.0, len(trajectory) - 1
        
        target_idx = self.find_target_point(trajectory, current_pos, current_idx)
        target = np.array([trajectory[target_idx]['x'], trajectory[target_idx]['y']])
        
        error = target - current_pos
        distance = np.linalg.norm(error)
        
        if distance < 0.01:
            return 0.0, 0.0, target_idx
        
        target_yaw = np.arctan2(error[1], error[0])
        yaw_error = np.arctan2(np.sin(target_yaw - current_yaw), 
                               np.cos(target_yaw - current_yaw))
        
        linear_vel = self.k_linear * distance
        angular_vel = self.k_angular * yaw_error
        
        linear_vel = np.clip(linear_vel, 0.0, 2.0)
        angular_vel = np.clip(angular_vel, -3.0, 3.0)
        
        return linear_vel, angular_vel, target_idx