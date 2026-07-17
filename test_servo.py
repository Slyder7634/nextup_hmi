# test_cartesian_direct.py
#!/usr/bin/env python3
"""
Test Cartesian twist commands directly - no planner, no GUI
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
import time

def main():
    rclpy.init()
    node = Node('test_twist_publisher')
    
    pub = node.create_publisher(
        TwistStamped,
        '/servo_node/delta_twist_cmds',
        10
    )
    
    print("Testing Cartesian twist commands...")
    print("This will move the robot in X direction slowly.")
    print("Press Ctrl+C to stop.")
    
    try:
        while rclpy.ok():
            # Move in X positive
            msg = TwistStamped()
            msg.header.stamp = node.get_clock().now().to_msg()
            msg.header.frame_id = 'base_link'
            msg.twist.linear.x = 0.1  # 0.1 m/s
            msg.twist.linear.y = 0.0
            msg.twist.linear.z = 0.0
            msg.twist.angular.x = 0.0
            msg.twist.angular.y = 0.0
            msg.twist.angular.z = 0.0
            
            pub.publish(msg)
            print("📤 Twist: X+ 0.1 m/s")
            time.sleep(2.0)
            
            # Stop
            msg.twist.linear.x = 0.0
            pub.publish(msg)
            print("📤 Stop")
            time.sleep(1.0)
            
            # Move in X negative
            msg.twist.linear.x = -0.1
            pub.publish(msg)
            print("📤 Twist: X- 0.1 m/s")
            time.sleep(2.0)
            
            # Stop
            msg.twist.linear.x = 0.0
            pub.publish(msg)
            print("📤 Stop")
            time.sleep(1.0)
            
    except KeyboardInterrupt:
        print("\nStopping...")
        # Send zero velocity
        msg = TwistStamped()
        msg.header.stamp = node.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.twist.linear.x = 0.0
        msg.twist.linear.y = 0.0
        msg.twist.linear.z = 0.0
        msg.twist.angular.x = 0.0
        msg.twist.angular.y = 0.0
        msg.twist.angular.z = 0.0
        pub.publish(msg)
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()