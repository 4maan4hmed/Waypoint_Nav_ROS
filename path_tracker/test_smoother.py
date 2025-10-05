import numpy as np
import matplotlib.pyplot as plt
from path_tracker.path_smoother import PathSmoother

waypoints = [
    (0.0, 0.0),
    (1.0, 0.0),
    (1.5, 1.0),
    (1.0, 2.0),
    (0.0, 2.0)
]

smoother = PathSmoother(num_points=200)
smooth_path = smoother.smooth(waypoints)
trajectory = smoother.generate_trajectory(smooth_path, velocity=0.2)

waypoints_arr = np.array(waypoints)
plt.figure(figsize=(10, 6))
plt.plot(waypoints_arr[:, 0], waypoints_arr[:, 1], 'ro-', label='Waypoints', markersize=10)
plt.plot(smooth_path[:, 0], smooth_path[:, 1], 'b-', label='Smooth Path', linewidth=2)
plt.legend()
plt.grid(True)
plt.axis('equal')
plt.xlabel('X (m)')
plt.ylabel('Y (m)')
plt.title('Path Smoothing Result')
plt.savefig('path_smoothing.png')
print(f"Generated {len(trajectory)} trajectory points")
print(f"Total time: {trajectory[-1]['t']:.2f} seconds")