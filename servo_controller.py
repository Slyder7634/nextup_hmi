from geometry_msgs.msg import Twist, TwistStamped
from std_msgs.msg import Float64MultiArray, String
from control_msgs.msg import JointJog
from builtin_interfaces.msg import Time
import rclpy
from rclpy.node import Node

class ServoController:
    def __init__(self, node: Node):
        self.node = node
        # For joint velocity commands: /servo_node/delta_joint_cmds (JointJog format)
        self.joint_jog_pub = node.create_publisher(
            JointJog,
            '/servo_node/delta_joint_cmds',
            10
        )
        # For Cartesian velocity commands: /servo_node/delta_twist_cmds
        self.twist_pub = node.create_publisher(
            TwistStamped,
            '/servo_node/delta_twist_cmds',
            10
        )
        # Keep ui_command for backward compatibility / emergency stop
        self.ui_cmd_pub = node.create_publisher(
            String,
            '/ui_commands',
            10
        )
        self.joint_names = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']

    def publish_joint_jog(self, joint_names, velocities):
        """
        Send joint velocity command directly to servo node using JointJog format.
        
        Args:
            joint_names: list of joint names to command (e.g. ['joint1'])
            velocities: list of velocities corresponding to joint_names (e.g. [0.05])
        """
        msg = JointJog()
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.header.frame_id = 'end'
        msg.joint_names = joint_names
        msg.velocities = velocities
        msg.displacements = []  # Not used, keep empty
        msg.duration = 0.0  # Not used, keep 0
        self.joint_jog_pub.publish(msg)
        # Only print if velocity is significant
        if any(abs(v) > 0.001 for v in velocities):
            print(f"📤 JointJog: {dict(zip(joint_names, velocities))}")

    def publish_joint_velocity_by_index(self, joint_index, velocity):
        """
        Send joint velocity command to a single joint by index.
        
        Args:
            joint_index: 0-5 for joint1-joint6
            velocity: velocity in rad/s
        """
        if 0 <= joint_index < len(self.joint_names):
            self.publish_joint_jog(
                [self.joint_names[joint_index]],
                [velocity]
            )

    def publish_joint_velocity_all(self, velocities):
        """
        Send velocity commands to all joints.
        
        Args:
            velocities: list of 6 velocities (rad/s) for all joints
        """
        if len(velocities) == len(self.joint_names):
            self.publish_joint_jog(self.joint_names, velocities)

    def publish_joint_velocity(self, joint_index, velocity):
        """
        Legacy method - sends joint velocity command (rad/s) for a single joint.
        Now uses the direct JointJog publisher.
        """
        self.publish_joint_velocity_by_index(joint_index, velocity)

    def publish_twist(self, linear=(0,0,0), angular=(0,0,0)):
        """
        Send Cartesian velocity command directly to servo node.
        
        Args:
            linear: (x, y, z) in m/s
            angular: (roll, pitch, yaw) in rad/s
        
        Note: frame_id='end' means the twist is in the end-effector frame.
        """
        msg = TwistStamped()
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.header.frame_id = 'end'
        msg.twist.linear.x = linear[0]
        msg.twist.linear.y = linear[1]
        msg.twist.linear.z = linear[2]
        msg.twist.angular.x = angular[0]
        msg.twist.angular.y = angular[1]
        msg.twist.angular.z = angular[2]
        self.twist_pub.publish(msg)
        # Only print if velocity is significant
        if any(abs(v) > 0.001 for v in linear) or any(abs(v) > 0.001 for v in angular):
            print(f"📤 Twist (end frame): linear={linear}, angular={angular}")

    def publish_ui_command(self, sign, kind, axis):
        """
        Publish to /ui_commands for backward compatibility.
        This is kept for emergency stop and for the legacy node.
        
        Args:
            sign: '+', '-', or '0' (stop)
            kind: 'c' (cartesian) or 'j' (joint)
            axis: for cartesian: 'x','y','z','r','p','w'
                  for joint: '1'..'6'
        """
        msg = String()
        msg.data = f"{sign}{kind}{axis}"
        self.ui_cmd_pub.publish(msg)

    def publish_stop_all(self):
        """
        Emergency stop - send zero velocity to all joints and stop twist.
        """
        # Stop all joints via JointJog
        zero_vels = [0.0] * len(self.joint_names)
        self.publish_joint_jog(self.joint_names, zero_vels)
        
        # Stop twist
        self.publish_twist((0,0,0), (0,0,0))
        
        # Also send UI command for the legacy node
        for i in range(6):
            self.publish_ui_command('0', 'j', str(i + 1))
        for axis in ['x', 'y', 'z', 'r', 'p', 'w']:
            self.publish_ui_command('0', 'c', axis)