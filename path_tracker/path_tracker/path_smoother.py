import numpy as np
from scipy.interpolate import CubicSpline

class PathSmoother:
    def __init__(self, num_points=100):
        self.num_points = num_points
    
    def smooth(self, waypoints):
        waypoints = np.array(waypoints)
        
        if len(waypoints) < 2:
            return waypoints
        
        distances = np.cumsum(np.sqrt(np.sum(np.diff(waypoints, axis=0)**2, axis=1)))
        distances = np.insert(distances, 0, 0)
        
        cs_x = CubicSpline(distances, waypoints[:, 0])
        cs_y = CubicSpline(distances, waypoints[:, 1])
        
        alpha = np.linspace(0, distances[-1], self.num_points)
        smooth_x = cs_x(alpha)
        smooth_y = cs_y(alpha)
        
        return np.column_stack([smooth_x, smooth_y])
    
    def generate_trajectory(self, smooth_path, velocity=0.2):
        trajectory = []
        total_time = 0.0
        
        for i in range(len(smooth_path)):
            if i > 0:
                dist = np.linalg.norm(smooth_path[i] - smooth_path[i-1])
                total_time += dist / velocity
            
            trajectory.append({
                'x': smooth_path[i][0],
                'y': smooth_path[i][1],
                't': total_time
            })
        
        return trajectory