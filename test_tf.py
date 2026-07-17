#!/usr/bin/env python3
"""
Test script to check if TF transforms are available
"""
import rclpy
from rclpy.node import Node
import tf2_ros
from tf2_ros import TransformException

def main():
    rclpy.init()
    node = Node('tf_test')
    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer, node)
    
    print("Waiting for transforms...")
    print("Looking for: base_link -> Link1, Link2, Link3, etc.")
    
    links = ['base_link', 'Link1', 'Link2', 'Link3', 'Link4', 'Link5', 'Link6']
    
    import time
    for i in range(20):  # Try for 20 iterations
        print(f"\nIteration {i+1}:")
        for link in links:
            try:
                transform = tf_buffer.lookup_transform('base_link', link, rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=0.01))
                print(f"  ✓ {link}: position=({transform.transform.translation.x:.3f}, {transform.transform.translation.y:.3f}, {transform.transform.translation.z:.3f})")
            except TransformException as e:
                pass
        time.sleep(1.0)
    
    rclpy.shutdown()

if __name__ == "__main__":
    main()