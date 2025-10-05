#!/usr/bin/env python3
"""
Simple test cases for trajectory controller components.
Run with: python3 test_trajectory_controller.py
"""

import sys
import numpy as np

try:
    from trajectory_controller import PathSmoother, TrajectoryGenerator, TrajectoryTracker
except ImportError:
    print("Error: Could not import trajectory_controller module")
    print("Make sure trajectory_controller.py is in the same directory")
    sys.exit(1)


def test_path_smoother():
    """Test path smoothing functionality."""
    print("\n=== Testing Path Smoother ===")
    
    # Test 1: Normal case with straight line
    print("\nTest 1: Straight line waypoints")
    waypoints = [(0, 0), (1, 0), (2, 0)]
    try:
        smooth_path = PathSmoother.smooth_path(waypoints, num_points=50)
        print(f"Success: Generated {len(smooth_path)} smooth points")
        assert len(smooth_path) == 50, "Should generate requested number of points"
        print("Success: Length check passed")
    except Exception as e:
        print(f"Failed: {e}")
        return False
    
    # Test 2: L-shaped path
    print("\nTest 2: L-shaped waypoints")
    waypoints = [(0, 0), (1, 0), (1, 1)]
    try:
        smooth_path = PathSmoother.smooth_path(waypoints, num_points=30)
        print(f"Success: Generated {len(smooth_path)} smooth points")
    except Exception as e:
        print(f"Failed: {e}")
        return False
    
    # Test 3: Error handling - too few waypoints
    print("\nTest 3: Error handling (too few waypoints)")
    try:
        PathSmoother.smooth_path([(0, 0)], num_points=10)
        print("Failed: Should have raised ValueError")
        return False
    except ValueError as e:
        print(f"Success: Correctly raised ValueError: {e}")
    
    # Test 4: Error handling - empty list
    print("\nTest 4: Error handling (empty list)")
    try:
        PathSmoother.smooth_path([], num_points=10)
        print("Failed: Should have raised ValueError")
        return False
    except ValueError:
        print("Success: Correctly raised ValueError")
    
    print("\nSuccess: All Path Smoother tests passed!")
    return True


def test_trajectory_generator():
    """Test trajectory generation."""
    print("\n=== Testing Trajectory Generator ===")
    
    # Test 1: Normal trajectory generation
    print("\nTest 1: Generate trajectory with constant velocity")
    smooth_path = [(0, 0), (1, 0), (2, 0)]
    velocity = 0.5  # m/s
    
    try:
        trajectory = TrajectoryGenerator.generate_trajectory(smooth_path, velocity)
        print(f"Success: Generated {len(trajectory)} trajectory points")
        
        # Check format
        assert len(trajectory[0]) == 3, "Each point should have (x, y, t)"
        print("Success: Trajectory format is correct")
        
        # Check time stamps
        assert trajectory[0][2] == 0.0, "First time should be 0"
        assert trajectory[-1][2] > 0, "Last time should be positive"
        print(f"Success: Total trajectory time: {trajectory[-1][2]:.2f} seconds")
        
        # Check time is monotonically increasing
        times = [t for _, _, t in trajectory]
        assert all(times[i] <= times[i+1] for i in range(len(times)-1)), "Time should increase"
        print("Success: Time stamps are monotonically increasing")
        
    except Exception as e:
        print(f"Failed: {e}")
        return False
    
    # Test 2: Different velocity
    print("\nTest 2: Generate trajectory with different velocity")
    try:
        traj_slow = TrajectoryGenerator.generate_trajectory(smooth_path, velocity=0.1)
        traj_fast = TrajectoryGenerator.generate_trajectory(smooth_path, velocity=1.0)
        
        assert traj_slow[-1][2] > traj_fast[-1][2], "Slower velocity should take more time"
        print(f"Success: Slow trajectory: {traj_slow[-1][2]:.2f}s")
        print(f"Success: Fast trajectory: {traj_fast[-1][2]:.2f}s")
    except Exception as e:
        print(f"Failed: {e}")
        return False
    
    # Test 3: Error handling - invalid velocity
    print("\nTest 3: Error handling (invalid velocity)")
    try:
        TrajectoryGenerator.generate_trajectory(smooth_path, velocity=-0.5)
        print("Failed: Should have raised ValueError")
        return False
    except ValueError:
        print("Success: Correctly raised ValueError for negative velocity")
    
    print("\nSuccess: All Trajectory Generator tests passed!")
    return True


def test_trajectory_tracker():
    """Test trajectory tracking controller."""
    print("\n=== Testing Trajectory Tracker ===")
    
    # Test 1: Controller initialization
    print("\nTest 1: Initialize controller")
    try:
        tracker = TrajectoryTracker(
            kp_linear=0.5,
            kp_angular=2.0,
            max_linear_vel=0.5,
            max_angular_vel=2.0
        )
        print("Success: Controller initialized successfully")
    except Exception as e:
        print(f"Failed: {e}")
        return False
    
    # Test 2: Compute control to target ahead
    print("\nTest 2: Compute control (target ahead)")
    current_x, current_y, current_theta = 0.0, 0.0, 0.0
    target_x, target_y = 1.0, 0.0
    
    try:
        linear_vel, angular_vel, distance = tracker.compute_control(
            current_x, current_y, current_theta,
            target_x, target_y
        )
        print(f"Success: Linear velocity: {linear_vel:.3f} m/s")
        print(f"Success: Angular velocity: {angular_vel:.3f} rad/s")
        print(f"Success: Distance: {distance:.3f} m")
        
        assert linear_vel > 0, "Should move forward"
        assert abs(angular_vel) < 0.1, "Should go straight (low angular velocity)"
        assert abs(distance - 1.0) < 0.01, "Distance should be 1.0m"
        print("Success: Control values are reasonable")
    except Exception as e:
        print(f"Failed: {e}")
        return False
    
    # Test 3: Compute control to target on the side
    print("\nTest 3: Compute control (target to the left)")
    current_x, current_y, current_theta = 0.0, 0.0, 0.0
    target_x, target_y = 0.0, 1.0  # Target to the left
    
    try:
        linear_vel, angular_vel, distance = tracker.compute_control(
            current_x, current_y, current_theta,
            target_x, target_y
        )
        print(f"Success: Angular velocity: {angular_vel:.3f} rad/s")
        assert angular_vel > 0, "Should turn left (positive angular velocity)"
        print("Success: Correctly turning left")
    except Exception as e:
        print(f"Failed: {e}")
        return False
    
    # Test 4: Velocity limits
    print("\nTest 4: Velocity limits")
    tracker_limited = TrajectoryTracker(max_linear_vel=0.1, max_angular_vel=0.5)
    
    # Very far target should still respect limits
    linear_vel, angular_vel, distance = tracker_limited.compute_control(
        0, 0, 0, 100, 100
    )
    
    assert linear_vel <= 0.1, "Linear velocity should respect limit"
    assert abs(angular_vel) <= 0.5, "Angular velocity should respect limit"
    print(f"Success: Linear velocity limited to: {linear_vel:.3f} m/s")
    print(f"Success: Angular velocity limited to: {angular_vel:.3f} rad/s")
    
    print("\nSuccess: All Trajectory Tracker tests passed!")
    return True


def run_integration_test():
    """Simple integration test of the full pipeline."""
    print("\n=== Integration Test ===")
    
    # Define waypoints
    waypoints = [
        (0.0, 0.0),
        (1.0, 0.0),
        (2.0, 1.0),
        (2.0, 2.0)
    ]
    
    try:
        # Step 1: Smooth path
        print("\nStep 1: Smoothing path...")
        smoother = PathSmoother()
        smooth_path = smoother.smooth_path(waypoints, num_points=100)
        print(f"Success: Smoothed {len(waypoints)} waypoints into {len(smooth_path)} points")
        
        # Step 2: Generate trajectory
        print("\nStep 2: Generating trajectory...")
        traj_gen = TrajectoryGenerator()
        trajectory = traj_gen.generate_trajectory(smooth_path, velocity=0.2)
        print(f"Success: Generated trajectory with {len(trajectory)} points")
        print(f"Success: Total time: {trajectory[-1][2]:.2f} seconds")
        
        # Step 3: Test controller on first few points
        print("\nStep 3: Testing controller...")
        tracker = TrajectoryTracker()
        
        current_x, current_y, current_theta = trajectory[0][0], trajectory[0][1], 0.0
        
        for i in range(min(5, len(trajectory))):
            target_x, target_y, _ = trajectory[i]
            linear_vel, angular_vel, distance = tracker.compute_control(
                current_x, current_y, current_theta,
                target_x, target_y
            )
            print(f"  Point {i}: distance={distance:.3f}m, lin_vel={linear_vel:.3f}, ang_vel={angular_vel:.3f}")
        
        print("\nSuccess: Integration test passed!")
        return True
        
    except Exception as e:
        print(f"Failed: Integration test failed: {e}")
        return False


def main():
    """Run all tests."""
    print("=" * 60)
    print("Trajectory Controller Test Suite")
    print("=" * 60)
    
    results = []
    
    # Run individual component tests
    results.append(("Path Smoother", test_path_smoother()))
    results.append(("Trajectory Generator", test_trajectory_generator()))
    results.append(("Trajectory Tracker", test_trajectory_tracker()))
    results.append(("Integration", run_integration_test()))
    
    # Print summary
    print("\n" + "=" * 60)
    print("Test Summary")
    print("=" * 60)
    
    for name, passed in results:
        status = "PASSED" if passed else "FAILED"
        print(f"{name:.<40} {status}")
    
    total = len(results)
    passed = sum(1 for _, p in results if p)
    
    print(f"\nTotal: {passed}/{total} test suites passed")
    
    if passed == total:
        print("\nAll tests passed successfully!")
        return 0
    else:
        print("\nSome tests failed - please check the output above")
        return 1


if __name__ == '__main__':
    exit_code = main()
    sys.exit(exit_code)