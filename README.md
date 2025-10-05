# Trajectory Controller for Differential Drive Robot

A ROS2-based path smoothing and trajectory tracking system for differential drive robots (tested on TurtleBot3).

## 📋 Table of Contents
- [Overview](#overview)
- [Features](#features)
- [Prerequisites](#prerequisites)
- [Installation](#installation)
- [Usage](#usage)
- [Testing](#testing)
- [Architecture & Design](#architecture--design)
- [Extending to Real Robot](#extending-to-real-robot)
- [AI Tools Used](#ai-tools-used)

---

## 🎯 Overview

This project implements a complete trajectory control system consisting of:
1. **Path Smoothing**: Converts discrete waypoints into smooth curves using cubic spline interpolation
2. **Trajectory Generation**: Creates time-parameterized trajectories with velocity profiles
3. **Trajectory Tracking**: Pure pursuit-like controller for accurate path following

## ✨ Features

- **Modular Architecture**: Separated concerns (PathSmoother, TrajectoryGenerator, TrajectoryTracker)
- **Error Handling**: Comprehensive exception handling and validation
- **RViz Visualization**: Real-time path and waypoint visualization
- **Configurable**: ROS2 parameters for easy tuning
- **Well-Tested**: Unit and integration tests included

---

## 📦 Prerequisites

### Required Software
- **Ubuntu 22.04** (or compatible)
- **ROS2 Humble** (or newer)
- **Python 3.10+**

### Required Python Packages
```bash
pip3 install numpy scipy
```

### Required ROS2 Packages
```bash
sudo apt install ros-humble-turtlebot3*
sudo apt install ros-humble-gazebo-ros-pkgs
```

---

## 🚀 Installation

### Step 1: Set up ROS2 Workspace
```bash
# Create workspace
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src

# Create package
ros2 pkg create trajectory_controller --build-type ament_python --dependencies rclpy geometry_msgs nav_msgs visualization_msgs
```

### Step 2: Add the Code
```bash
cd trajectory_controller/trajectory_controller

# Copy your trajectory_controller.py file here
# The file should be in: ~/ros2_ws/src/trajectory_controller/trajectory_controller/trajectory_controller.py
```

### Step 3: Update setup.py
Edit `~/ros2_ws/src/trajectory_controller/setup.py`:

```python
from setuptools import setup

package_name = 'trajectory_controller'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email@example.com',
    description='Trajectory controller for differential drive robots',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'trajectory_controller = trajectory_controller.trajectory_controller:main',
        ],
    },
)
```

### Step 4: Build the Package
```bash
cd ~/ros2_ws
colcon build --packages-select trajectory_controller
source install/setup.bash
```

---

## 🎮 Usage

### Terminal 1: Launch TurtleBot3 Simulation
```bash
# Set TurtleBot3 model
export TURTLEBOT3_MODEL=burger

# Launch empty world
ros2 launch turtlebot3_gazebo empty_world.launch.py
```

### Terminal 2: Launch RViz (for visualization)
```bash
# Source workspace
source ~/ros2_ws/install/setup.bash

# Launch RViz
rviz2
```

**In RViz, add these displays:**
1. Click "Add" → "Path" → Set topic to `/smooth_path`
2. Click "Add" → "MarkerArray" → Set topic to `/waypoints`
3. Click "Add" → "MarkerArray" → Set topic to `/trajectory_points`
4. Set Fixed Frame to `odom`

### Terminal 3: Run Trajectory Controller
```bash
# Source workspace
source ~/ros2_ws/install/setup.bash

# Run the controller
ros2 run trajectory_controller trajectory_controller
```

### Customizing Waypoints

Edit the waypoints in `trajectory_controller.py` (around line 193):

```python
self.waypoints = [
    (0.0, 0.0),
    (1.0, 0.0),
    (2.0, 1.0),
    # Add your waypoints here
]
```

Then rebuild:
```bash
cd ~/ros2_ws
colcon build --packages-select trajectory_controller
source install/setup.bash
```

### Adjusting Parameters

You can adjust controller parameters when launching:

```bash
ros2 run trajectory_controller trajectory_controller \
  --ros-args \
  -p control_frequency:=20.0 \
  -p max_linear_vel:=0.3 \
  -p max_angular_vel:=2.0
```

---

## 🧪 Testing

### Running Tests

Place `test_trajectory_controller.py` in the same directory as `trajectory_controller.py`:

```bash
cd ~/ros2_ws/src/trajectory_controller/trajectory_controller
python3 test_trajectory_controller.py
```

### Expected Output
```
============================================================
Trajectory Controller Test Suite
============================================================

=== Testing Path Smoother ===

Test 1: Straight line waypoints
✓ Generated 50 smooth points
✓ Length check passed

[... more tests ...]

✓ All tests passed!
```

### Manual Testing Checklist

1. **Visualization Test**
   - [ ] Red spheres appear at waypoints
   - [ ] Green line shows smooth path
   - [ ] Small green dots show trajectory points

2. **Movement Test**
   - [ ] Robot starts moving towards first waypoint
   - [ ] Robot follows the smooth path (not jerky)
   - [ ] Robot slows down at sharp turns
   - [ ] Robot stops at the end

3. **Console Output**
   - [ ] "Trajectory Controller initialized" message
   - [ ] Progress percentages displayed
   - [ ] "Trajectory completed" at the end
   - [ ] No error messages

---

## 🏗️ Architecture & Design

### Design Principles

1. **Separation of Concerns**: Each class handles one responsibility
   - `PathSmoother`: Only path smoothing logic
   - `TrajectoryGenerator`: Only trajectory generation
   - `TrajectoryTracker`: Only control algorithms
   - `TrajectoryController`: ROS2 integration and coordination

2. **Testability**: All core logic is in standalone classes that can be tested without ROS2

3. **Error Handling**: Comprehensive validation and exception handling at every level

4. **Maintainability**: Clear docstrings, type hints, and comments

### Algorithm Choices

#### 1. Path Smoothing - Cubic Spline Interpolation
**Why:** 
- Produces C² continuous paths (smooth acceleration)
- Natural-looking curves that pass through all waypoints
- Computationally efficient

**Alternative considered:** Bézier curves (less intuitive to control)

#### 2. Trajectory Generation - Constant Velocity Profile
**Why:**
- Simple and predictable
- Suitable for most navigation tasks
- Easy to tune and understand

**Improvement for production:** Trapezoidal velocity profile for better acceleration/deceleration

#### 3. Trajectory Tracking - Pure Pursuit Controller
**Why:**
- Simple proportional control
- Proven effective for differential drive robots
- Easy to tune with just 2 parameters

**Alternative considered:** Model Predictive Control (too complex for this application)

### Code Quality Features

- **Modular design**: Easy to replace/upgrade components
- **Type hints**: Better IDE support and error detection
- **Docstrings**: Every function documented
- **Error messages**: Clear, actionable error descriptions
- **Logging**: Informative console output with throttling
- **Constants as parameters**: Easy tuning without code changes

---

## 🤖 Extending to Real Robot

### Required Changes

#### 1. Hardware Setup
- Connect to real TurtleBot3
- Ensure odometry is publishing on `/odom`
- Verify cmd_vel commands work

#### 2. Sensor Integration
```python
# Add laser scan subscriber for obstacle detection
self.scan_sub = self.create_subscription(
    LaserScan, '/scan', self.scan_callback, 10
)
```

#### 3. Safety Features
```python
def check_obstacles(self):
    """Check if path is clear."""
    if self.min_laser_distance < 0.3:  # 30cm safety margin
        self.stop_robot()
        self.get_logger().warn('Obstacle detected!')
        return False
    return True
```

#### 4. Localization Enhancement
- Use **AMCL** (Adaptive Monte Carlo Localization) for better pose estimation
- Subscribe to `/amcl_pose` instead of raw odometry
- Implement pose covariance checking

#### 5. Real-World Tuning
```python
# Slower speeds for safety
self.max_linear_vel = 0.15  # Reduce from 0.22
self.max_angular_vel = 1.5  # Reduce from 2.84

# Larger tolerance for real-world uncertainty
self.position_tolerance = 0.15  # Increase from 0.1
```

#### 6. Recovery Behaviors
```python
def recovery_behavior(self):
    """Handle getting stuck."""
    # Back up
    self.move_backward(distance=0.2)
    # Rotate to find clear path
    self.rotate(angle=90)
    # Resume trajectory
```

### Testing on Real Robot

1. **Start with slow speeds**: Test at 0.1 m/s first
2. **Small test area**: Use 2-3 waypoints initially
3. **Manual override**: Keep a way to stop the robot (e.g., emergency stop button)
4. **Log everything**: Record odometry, commands, and sensor data
5. **Iterative tuning**: Adjust parameters based on performance

### Production Checklist

- [ ] Emergency stop functionality
- [ ] Battery level monitoring
- [ ] Obstacle detection and avoidance
- [ ] Robust error recovery
- [ ] Network reliability (if using WiFi)
- [ ] Logging and diagnostics
- [ ] Parameter validation
- [ ] Graceful degradation

---

## 🤖 AI Tools Used

This project was developed with assistance from AI tools:

### Claude AI (Anthropic)
- **Used for**: Code architecture review, bug fixing, documentation
- **Benefit**: Helped structure modular design and comprehensive error handling
- **Approach**: Iterative refinement of code based on best practices

### Development Process
1. Initial implementation of core algorithms
2. AI-assisted refactoring for modularity
3. AI-generated test cases
4. Documentation and README generation
5. Manual verification and testing

**Note**: All code was reviewed, tested, and validated manually. AI was used as a coding assistant, not a replacement for understanding.

---

## 📊 Expected Results

### Performance Metrics
- **Path Smoothness**: No abrupt direction changes
- **Tracking Accuracy**: Within 10cm of desired path
- **Completion Time**: ~30-40 seconds for default waypoints
- **Success Rate**: >95% in simulation

### Visual Results
- Smooth curved path connecting waypoints
- Robot follows green trajectory line
- Progress updates in console
- Smooth velocity changes (no jerking)

---

## 🐛 Troubleshooting

### Issue: Robot doesn't move
**Solution**: 
- Check if odometry is being received: `ros2 topic echo /odom`
- Verify cmd_vel is publishing: `ros2 topic echo /cmd_vel`
- Check for error messages in console

### Issue: Robot moves erratically
**Solution**:
- Reduce max velocities
- Increase control frequency
- Check waypoint spacing (not too close)

### Issue: "Waiting for odometry" message
**Solution**:
- Ensure Gazebo simulation is running
- Check that TurtleBot3 is spawned
- Verify topic names match

### Issue: Path not visible in RViz
**Solution**:
- Set Fixed Frame to `odom`
- Check topic names in RViz match publisher topics
- Ensure markers are added correctly

---

## 📝 Additional Notes

### Modifying the Trajectory
- Waypoints are defined in meters (Cartesian coordinates)
- Origin (0, 0) is where the robot starts
- Recommended spacing: 0.5-2.0 meters between waypoints

### Controller Tuning Guide
- `kp_linear`: Higher = faster approach (may overshoot)
- `kp_angular`: Higher = faster turning (may oscillate)
- `position_tolerance`: Smaller = more accurate (may not reach)

### Known Limitations
- No obstacle avoidance (as per requirements)
- Constant velocity profile (no acceleration optimization)
- No replanning if path becomes invalid

---

## 📧 Contact & Support

For issues or questions:
1. Check the troubleshooting section above
2. Review console error messages
3. Verify all prerequisites are installed
4. Test with simple waypoints first

---

## 📄 License

Apache License 2.0 (or as specified by your institution)
