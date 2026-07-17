# To centralize ROS2 availability
try:
    import rclpy
    from geometry_msgs.msg import Twist, TwistStamped
    from std_msgs.msg import Float64MultiArray
    import tf2_ros
    from moveit_msgs.srv import PlanKinematicPath, GetPositionFK, GetPlanningScene
    from moveit_msgs.msg import RobotState, DisplayTrajectory
    ROS2_AVAILABLE = True
except ImportError:
    ROS2_AVAILABLE = False