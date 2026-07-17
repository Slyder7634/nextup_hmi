from geometry_msgs.msg import TwistStamped
from control_msgs.msg import JointJog
import rclpy
from rclpy.node import Node

class ServoController:
    """Publishes jog commands to moveit_servo.

    Only talks to the two topics servo_node.cpp actually subscribes to:
    /servo_node/delta_joint_cmds (JointJog) and /servo_node/delta_twist_cmds
    (TwistStamped). There used to be a third publisher here for a
    '/ui_commands' string protocol ("+j1", "0cx", ...) kept around for a
    "legacy node" - but nothing in this codebase's servo_node.cpp (or
    anywhere else in the repo) subscribes to /ui_commands, so every jog
    command sent that way was going nowhere. Removed.
    """

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

    def publish_twist(self, linear=(0, 0, 0), angular=(0, 0, 0)):
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

    def publish_stop_all(self):
        """
        Emergency stop - send zero velocity to all joints and stop twist.
        """
        zero_vels = [0.0] * len(self.joint_names)
        self.publish_joint_jog(self.joint_names, zero_vels)
        self.publish_twist((0, 0, 0), (0, 0, 0))