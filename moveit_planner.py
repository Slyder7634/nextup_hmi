"""
Simplified IK-based Planner for Nextup Cobot HMI
Uses only /compute_ik service for joint solution - no motion planning
"""
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from PyQt6.QtCore import QObject, pyqtSignal
from sensor_msgs.msg import JointState
import math
import time
import threading

# Set True to re-enable the per-plan step-by-step debug prints.
# They were firing 15-20 print()s (blocking stdout writes) for EVERY
# hover-triggered plan, which adds up fast if the pointer sweeps across
# several buttons. Off by default.
_VERBOSE = False


def _dbg(*args, **kwargs):
    if _VERBOSE:
        print(*args, **kwargs)


try:
    from moveit_msgs.srv import GetPositionFK, GetPositionIK
    from moveit_msgs.msg import RobotState, DisplayTrajectory, MotionPlanResponse, RobotTrajectory
    from geometry_msgs.msg import Pose, PoseStamped
    MOVEIT_AVAILABLE = True
except ImportError:
    MOVEIT_AVAILABLE = False
    print("⚠ MoveIt not available")


class MoveItPlanner(QObject):
    """
    Simplified planner that uses only FK and IK services.
    - Gets current EE pose via FK
    - Applies Cartesian offset
    - Solves IK for target joints
    - Generates interpolated path for visualization
    """
    planning_done = pyqtSignal(bool, str)
    trajectory_ready = pyqtSignal(object, str, str)
    progress_update = pyqtSignal(float)
    
    def __init__(self, node: Node = None, group='robot_manipulator', ee_link='link6'):
        super().__init__()
        self.node = node
        self.group = group
        self.ee_link = ee_link
        self.available = False
        self._current_plan_id = 0
        
        # Store last known joint state
        self.last_joint_state = None
        
        # Cartesian offset configuration
        self.translation_offset = 0.04  # 4 cm
        self.rotation_offset = 0.2     # rad
        
        if not MOVEIT_AVAILABLE:
            print("⚠ MoveIt planning unavailable")
            return
            
        if node is None:
            print("⚠ No ROS node provided")
            return
            
        _dbg(f"DEBUG: Initializing simplified IK planner with group='{group}', ee_link='{ee_link}'")
        self.available = True
        
        try:
            # Separate callback groups for FK and IK
            self.fk_group = MutuallyExclusiveCallbackGroup()
            self.ik_group = MutuallyExclusiveCallbackGroup()
            
            # Create service clients
            self.fk_client = node.create_client(
                GetPositionFK,
                '/compute_fk',
                callback_group=self.fk_group
            )
            self.ik_client = node.create_client(
                GetPositionIK,
                '/compute_ik',
                callback_group=self.ik_group
            )
            
            # Subscribe to joint states for current robot state
            self.joint_state_sub = node.create_subscription(
                JointState,
                '/joint_states',
                self._on_joint_state,
                10
            )
            print("  Subscribed to /joint_states for robot feedback")
            
            # Wait for services
            print(f"Waiting for IK/FK services...")
            services_ready = True
            
            if not self.fk_client.wait_for_service(timeout_sec=2.0):
                print("⚠ /compute_fk service not available")
                services_ready = False
            
            if not self.ik_client.wait_for_service(timeout_sec=2.0):
                print("⚠ /compute_ik service not available")
                services_ready = False
            
            if not services_ready:
                print("⚠ Some services not available.")
                self.available = False
                return
            
            print(f"✓ Simplified IK planner initialized")
        except Exception as e:
            print(f"⚠ Planner init error: {e}")
            self.available = False

    def _on_joint_state(self, msg):
        """Capture latest joint state from robot"""
        self.last_joint_state = msg

    def plan_cartesian_offset(self, direction, offset):
        """
        Plan Cartesian offset motion using IK.
        
        Args:
            direction: 'x', 'y', 'z', 'roll', 'pitch', 'yaw'
            offset: signed offset value (small, ~0.04m for translation, ~0.2 rad for rotation)
        """
        if not self.available:
            self.planning_done.emit(False, "IK planner not available")
            return
            
        self._current_plan_id += 1
        plan_id = self._current_plan_id
        
        threading.Thread(
            target=self._plan_cartesian_offset_thread,
            args=(direction, offset, plan_id),
            daemon=True
        ).start()

    def _plan_cartesian_offset_thread(self, direction, offset, plan_id):
        """Thread worker for Cartesian offset planning"""
        try:
            self.progress_update.emit(10.0)
            
            # Small delay to ensure latest joint_state is captured
            time.sleep(0.05)
            
            # Step 1: Get current robot state
            state = self._get_current_state(plan_id)
            if plan_id != self._current_plan_id:
                return
            if not state:
                self.planning_done.emit(False, "Failed to get current robot state")
                return
                
            self.progress_update.emit(20.0)
            
            # Step 2: Get current end-effector pose
            pose = self._get_current_pose(state, plan_id)
            if plan_id != self._current_plan_id:
                return
            if not pose:
                self.planning_done.emit(False, "Failed to get EE pose")
                return
                
            self.progress_update.emit(30.0)
            
            # Step 3: Apply Cartesian offset to pose
            target_pose = self._apply_offset(pose, direction, offset)
            _dbg(f"DEBUG: Target pose calculated")
            
            self.progress_update.emit(40.0)
            
            # Step 4: Solve IK for target pose
            target_joints = self._solve_ik(state, target_pose, plan_id)
            if plan_id != self._current_plan_id:
                return
            if not target_joints:
                self.planning_done.emit(False, f"IK failed for {direction} offset")
                return
                
            self.progress_update.emit(60.0)
            
            # Step 5: Create interpolated trajectory from current to target
            trajectory = self._create_interpolated_trajectory(
                state.joint_state,
                target_joints,
                num_points=20
            )
            if not trajectory:
                self.planning_done.emit(False, "Failed to create trajectory")
                return
                
            self.progress_update.emit(80.0)
            
            # Step 6: Emit trajectory for visualization
            display_traj = DisplayTrajectory()
            
            # Wrap JointTrajectory in RobotTrajectory
            robot_traj = RobotTrajectory()
            robot_traj.joint_trajectory = trajectory
            display_traj.trajectory.append(robot_traj)
            display_traj.trajectory_start = state
            
            self.trajectory_ready.emit(display_traj, direction, "plus" if offset > 0 else "minus")
            self.planning_done.emit(True, f"IK solved for {direction}")
            self.progress_update.emit(100.0)
            
        except Exception as e:
            print(f"Planning error: {e}")
            self.planning_done.emit(False, f"Error: {str(e)}")

    def plan_joint_offset(self, joint_name, offset):
        """
        Plan a single-joint offset motion for the ghost preview.

        Unlike plan_cartesian_offset, this needs no IK/FK service calls --
        the target joint state is just "current state with one joint
        nudged by offset", so it works purely from the /joint_states
        feedback we already subscribe to. This means the joint ghost
        preview keeps working even if the IK/FK services are down (i.e.
        even when self.available is False), as long as we have a node and
        at least one joint_states message.

        Args:
            joint_name: actual joint name, e.g. 'joint1' (NOT the display
                label like 'J1' -- caller is responsible for translating)
            offset: signed offset value in radians
        """
        if not MOVEIT_AVAILABLE or self.node is None:
            self.planning_done.emit(False, "Joint preview not available")
            return

        self._current_plan_id += 1
        plan_id = self._current_plan_id

        threading.Thread(
            target=self._plan_joint_offset_thread,
            args=(joint_name, offset, plan_id),
            daemon=True
        ).start()

    def _plan_joint_offset_thread(self, joint_name, offset, plan_id):
        """Thread worker for single-joint offset preview"""
        try:
            self.progress_update.emit(20.0)

            # Small delay to ensure latest joint_state is captured, same as
            # the Cartesian path.
            time.sleep(0.05)

            state = self._get_current_state(plan_id)
            if plan_id != self._current_plan_id:
                return
            if not state:
                self.planning_done.emit(False, "Failed to get current robot state")
                return

            self.progress_update.emit(50.0)

            current_joints = state.joint_state
            if joint_name not in current_joints.name:
                self.planning_done.emit(False, f"Unknown joint: {joint_name}")
                return

            target_joints = JointState()
            target_joints.name = list(current_joints.name)
            target_joints.position = list(current_joints.position)
            idx = target_joints.name.index(joint_name)
            target_joints.position[idx] += offset

            trajectory = self._create_interpolated_trajectory(
                current_joints,
                target_joints,
                num_points=20
            )
            if not trajectory:
                self.planning_done.emit(False, "Failed to create trajectory")
                return

            self.progress_update.emit(80.0)

            display_traj = DisplayTrajectory()
            robot_traj = RobotTrajectory()
            robot_traj.joint_trajectory = trajectory
            display_traj.trajectory.append(robot_traj)
            display_traj.trajectory_start = state

            # Reuse the same (axis, direction) signal shape as the
            # Cartesian path -- the HMI's on_trajectory_ready doesn't care
            # whether "axis" is a Cartesian axis or a joint name.
            self.trajectory_ready.emit(display_traj, joint_name, "plus" if offset > 0 else "minus")
            self.planning_done.emit(True, f"Preview ready for {joint_name}")
            self.progress_update.emit(100.0)
        except Exception as e:
            print(f"Joint offset planning error: {e}")
            self.planning_done.emit(False, f"Error: {str(e)}")

    def _get_current_state(self, plan_id):
        """Get current robot state from joint_states"""
        if not self.last_joint_state:
            _dbg("DEBUG: No joint state received yet")
            return None
        
        # Ensure joint state has all 6 joints in correct order
        js = self.last_joint_state
        
        # Expected joint names
        expected_joints = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
        
        # Check if we have all joints
        if len(js.name) < 6:
            print(f"⚠ Joint state incomplete: {len(js.name)} joints, expected 6")
            _dbg(f"  Joint names: {js.name}")
        
        _dbg(f"DEBUG: Current joint state:")
        _dbg(f"  Names: {js.name}")
        _dbg(f"  Positions: {js.position}")
        
        state = RobotState()
        state.joint_state = self.last_joint_state
        return state

    def _get_current_pose(self, state, plan_id):
        """Get current EE pose using FK service"""
        try:
            req = GetPositionFK.Request()
            req.header.frame_id = 'base_link'
            req.fk_link_names = [self.ee_link]
            req.robot_state = state
            
            if not self.fk_client.service_is_ready():
                print("ERROR: FK service not ready")
                return None
            
            future = self.fk_client.call_async(req)
            end_time = time.time() + 2.0
            while not future.done() and time.time() < end_time:
                if plan_id != self._current_plan_id:
                    return None
                time.sleep(0.01)
            
            if future.done():
                response = future.result()
                if response and response.pose_stamped:
                    pose = response.pose_stamped[0].pose
                    _dbg(f"DEBUG: Current EE pose: pos=({pose.position.x:.3f}, {pose.position.y:.3f}, {pose.position.z:.3f})")
                    return pose
            return None
        except Exception as e:
            print(f"FK error: {e}")
            return None

    def _apply_offset(self, pose, direction, offset):
        """
        Apply Cartesian offset to the EE pose.
        Works in end-effector frame (tool frame).
        """
        target = Pose()
        target.position.x = pose.position.x
        target.position.y = pose.position.y
        target.position.z = pose.position.z
        target.orientation = pose.orientation

        _dbg(f"DEBUG _apply_offset:")
        _dbg(f"  direction: {direction}, offset: {offset}")
        _dbg(f"  base pose: ({pose.position.x:.3f}, {pose.position.y:.3f}, {pose.position.z:.3f})")
        _dbg(f"  base orientation: ({pose.orientation.x:.3f}, {pose.orientation.y:.3f}, {pose.orientation.z:.3f}, {pose.orientation.w:.3f})")

        q = (pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w)

        if direction in ('x', 'y', 'z'):
            # Translation in end-effector frame
            local_vec = {
                'x': (offset, 0.0, 0.0),
                'y': (0.0, offset, 0.0),
                'z': (0.0, 0.0, offset),
            }[direction]
            dx, dy, dz = self._rotate_vector_by_quat(local_vec, q)
            target.position.x += dx
            target.position.y += dy
            target.position.z += dz
            
        elif direction in ('roll', 'pitch', 'yaw'):
            # Rotation in end-effector frame
            local_axis = {
                'roll': (1.0, 0.0, 0.0),
                'pitch': (0.0, 1.0, 0.0),
                'yaw': (0.0, 0.0, 1.0),
            }[direction]
            q_delta = self._axis_angle_to_quat(local_axis, offset)
            qx, qy, qz, qw = self._quat_multiply(q, q_delta)
            target.orientation.x = qx
            target.orientation.y = qy
            target.orientation.z = qz
            target.orientation.w = qw

        _dbg(f"  target pose: ({target.position.x:.3f}, {target.position.y:.3f}, {target.position.z:.3f})")
        return target

    def _solve_ik(self, state, target_pose, plan_id):
        """
        Solve IK for target pose using /compute_ik service.
        Returns joint state with solution, or None if IK fails.
        """
        try:
            req = GetPositionIK.Request()
            req.ik_request.group_name = self.group
            req.ik_request.robot_state = state
            
            # Set target pose
            req.ik_request.pose_stamped.header.frame_id = 'base_link'
            req.ik_request.pose_stamped.pose = target_pose
            
            # Set EE link
            req.ik_request.ik_link_name = self.ee_link
            
            # Set timeout for IK solver
            req.ik_request.timeout.sec = 1
            req.ik_request.timeout.nanosec = 0
            
            # Allow collision checking (set to False for preview mode if needed)
            req.ik_request.avoid_collisions = False
            
            if not self.ik_client.service_is_ready():
                print("ERROR: IK service not ready")
                return None
            
            future = self.ik_client.call_async(req)
            end_time = time.time() + 2.0
            while not future.done() and time.time() < end_time:
                if plan_id != self._current_plan_id:
                    return None
                time.sleep(0.01)
            
            if future.done():
                response = future.result()
                if response:
                    # Check error code
                    if response.error_code.val == 1:  # SUCCESS
                        if response.solution and response.solution.joint_state.position:
                            _dbg(f"DEBUG: IK solved successfully")
                            _dbg(f"  Solution: {response.solution.joint_state.position}")
                            return response.solution.joint_state
                    else:
                        error_code = response.error_code.val
                        _dbg(f"DEBUG: IK failed with error code {error_code}")
                        if error_code == -31:
                            print("  (No IK solution found for this pose)")
                        return None
                else:
                    _dbg(f"DEBUG: IK failed - no response")
                    return None
            return None
        except Exception as e:
            print(f"IK error: {e}")
            return None

    def _create_interpolated_trajectory(self, current_joints, target_joints, num_points=20):
        """
        Create a simple interpolated trajectory from current to target joints.
        Returns a JointTrajectory message suitable for DisplayTrajectory.
        """
        try:
            from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
            from builtin_interfaces.msg import Duration
            
            traj = JointTrajectory()
            traj.joint_names = current_joints.name
            
            _dbg(f"DEBUG _create_interpolated_trajectory:")
            _dbg(f"  Current joint names: {current_joints.name}")
            _dbg(f"  Current positions: {current_joints.position}")
            _dbg(f"  Target joint names: {target_joints.name}")
            _dbg(f"  Target positions: {target_joints.position}")
            
            # Reorder target positions to match current joint order
            current_pos = list(current_joints.position)
            target_pos_dict = dict(zip(target_joints.name, target_joints.position))
            
            # Reorder target positions based on current joint names
            target_pos = []
            for joint_name in current_joints.name:
                if joint_name in target_pos_dict:
                    target_pos.append(target_pos_dict[joint_name])
                else:
                    print(f"  ⚠ Joint '{joint_name}' not in IK solution!")
                    target_pos.append(0.0)  # Fallback
            
            _dbg(f"  Reordered target positions: {target_pos}")
            
            # Shortest-path delta per joint. Without this, a joint with a
            # wide/continuous range (joint5, joint6: -6.28..6.28) can have
            # an IK solution that's numerically far from the current angle
            # (e.g. +3.0 -> -3.0) but is actually only a tiny move the
            # "short way" around. Plain linear interpolation would instead
            # sweep the joint the LONG way through zero -- a ~6 rad swing
            # for what should be a fraction-of-a-radian jog -- which is
            # exactly what produced the wildly twisted wrist pose in the
            # ghost preview.
            deltas = []
            for j in range(len(current_pos)):
                delta = target_pos[j] - current_pos[j]
                # wrap delta into [-pi, pi]
                delta = (delta + math.pi) % (2 * math.pi) - math.pi
                deltas.append(delta)
            
            # Linear interpolation using the shortest-path delta
            for i in range(num_points + 1):
                t = i / num_points
                point = JointTrajectoryPoint()
                
                # Interpolate positions
                point.positions = [
                    current_pos[j] + deltas[j] * t
                    for j in range(len(current_pos))
                ]
                
                # Set time from start
                time_from_start = Duration()
                time_from_start.sec = int(t * 2.0)  # 2 second trajectory
                time_from_start.nanosec = int((t * 2.0 - time_from_start.sec) * 1e9)
                point.time_from_start = time_from_start
                
                traj.points.append(point)
            
            _dbg(f"DEBUG: Created interpolated trajectory with {len(traj.points)} points")
            return traj
        except Exception as e:
            print(f"Trajectory creation error: {e}")
            return None

    def _quat_multiply(self, a, b):
        """Hamilton product a*b for quaternions (x, y, z, w)"""
        ax, ay, az, aw = a
        bx, by, bz, bw = b
        x = aw*bx + ax*bw + ay*bz - az*by
        y = aw*by - ax*bz + ay*bw + az*bx
        z = aw*bz + ax*by - ay*bx + az*bw
        w = aw*bw - ax*bx - ay*by - az*bz
        return (x, y, z, w)

    def _quat_conjugate(self, q):
        """Conjugate of quaternion"""
        x, y, z, w = q
        return (-x, -y, -z, w)

    def _rotate_vector_by_quat(self, v, q):
        """Rotate vector v by quaternion q: v' = q * v * q^-1"""
        vq = (v[0], v[1], v[2], 0.0)
        rx, ry, rz, _ = self._quat_multiply(
            self._quat_multiply(q, vq),
            self._quat_conjugate(q)
        )
        return (rx, ry, rz)

    def _axis_angle_to_quat(self, axis, angle):
        """Build quaternion (x, y, z, w) for rotation of angle about axis"""
        ax, ay, az = axis
        half = angle / 2.0
        s = math.sin(half)
        return (ax * s, ay * s, az * s, math.cos(half))

    def set_planner(self, planner_id):
        """
        Placeholder for compatibility.
        This simplified planner doesn't use motion planning, so planner_id is ignored.
        """
        print(f"Note: Planner {planner_id} setting ignored (using IK-only mode)")
        return True