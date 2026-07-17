#!/usr/bin/env python3
"""
Test IK solver with MoveIt servo node
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
import time

from servo_ik_solver import ServoIKSolver

def main():
    rclpy.init()
    node = Node('ik_test')
    
    # Create IK solver
    ik_solver = ServoIKSolver(node)
    
    if not ik_solver.available:
        print("❌ IK solver not available. Make sure MoveIt servo node is running.")
        return
    
    print("✓ IK solver available")
    
    # Test with a simple target pose
    pose = Pose()
    pose.position.x = 0.3
    pose.position.y = 0.0
    pose.position.z = 0.5
    pose.orientation.w = 1.0
    
    print(f"\nTesting IK for pose: ({pose.position.x}, {pose.position.y}, {pose.position.z})")
    
    result = ik_solver.solve_ik(pose)
    
    if result:
        print("\n✓ IK Success!")
        print("  Joint positions:")
        for name, pos in result.items():
            print(f"    {name}: {pos:.3f}")
    else:
        print("\n✗ IK Failed")
    
    rclpy.shutdown()

if __name__ == "__main__":
    main()