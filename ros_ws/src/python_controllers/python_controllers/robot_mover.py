import rclpy
from rclpy.node import Node

import numpy as np
from numpy.typing import NDArray

from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from python_controllers.InverseKinematicsRobot import ik_pose, usefull

# ---------------------------------------------------------------------------
# Types
# ---------------------------------------------------------------------------

# A waypoint is [x, y, z, orientation_or_None, gripper_value]
Waypoint = list[float | None]

# IK result is either a numpy array of joint angles, or a string error message
IKResult = NDArray[np.float64] | str

# Cache: maps (x, y, z) rounded position → (joint_angles, error)
_waypoint_cache: dict[tuple[float, ...], tuple[NDArray[np.float64], float]] = {}


# ---------------------------------------------------------------------------
# IK solving with caching
# ---------------------------------------------------------------------------

def solve_waypoint(pos: Waypoint, idx: int) -> tuple[IKResult, float]:
    """
    Solve inverse kinematics for a given waypoint position.

    Uses a position-keyed cache to avoid re-solving identical positions.
    On a successful solve, seeds the result back as an initial guess via usefull().
    """
    cache_key = tuple(np.round(pos[:3], 4))

    if cache_key in _waypoint_cache:
        return _waypoint_cache[cache_key]

    joint_angles, error = ik_pose(pos[:3], pos[3])

    if not isinstance(joint_angles, str):
        _waypoint_cache[cache_key] = (joint_angles, error)
        usefull(np.array(joint_angles))

    return joint_angles, error

class RobotMover(Node):
    """
    ROS2 node that moves a robot arm through a sequence of joint-space waypoints.

    Workflow:
      1. Pre-solve IK for all Cartesian waypoints.
      2. Move the robot to the first waypoint and wait for user confirmation.
      3. Execute the full trajectory using a fixed-rate timer.
    """

    def __init__(self, waypoints: list[Waypoint], duration: float = 1.0) -> None:
        super().__init__('robot_mover')

        self._publisher = self.create_publisher(JointTrajectory, 'joint_cmds', 10)

        self._joint_trajectory: list[NDArray[np.float64]] = self._solve_all_waypoints(waypoints)

        if not self._joint_trajectory:
            self.get_logger().error("No valid waypoints found!")
            return

        self._total_duration: float = duration
        self._num_segments: int = len(self._joint_trajectory) - 1
        self._time_per_segment: float = self._total_duration / self._num_segments

        self._run_with_confirmation()

    # ------------------------------------------------------------------
    # Setup helpers
    # ------------------------------------------------------------------

    def _solve_all_waypoints(self, waypoints: list[Waypoint]) -> list[NDArray[np.float64]]:
        """
        Solve IK for every waypoint and return a list of valid joint configurations.

        Waypoints that cannot be reached are skipped with a warning.
        """
        joint_trajectory: list[NDArray[np.float64]] = []

        for idx, waypoint in enumerate(waypoints):
            result, error = solve_waypoint(waypoint, idx)

            if isinstance(result, str):
                self.get_logger().warn(f"Waypoint {idx} unreachable. Skipping.")
                continue

            joint_angles = list(result)
            joint_angles[4] = np.pi / 2

            # end_effector_z_axis = forward_kinematics(result)[:3, 2]
            # print(end_effector_z_axis)

            gripper_value = float(waypoint[4])
            joint_trajectory.append(np.array(joint_angles + [gripper_value]))

        return joint_trajectory

    def _run_with_confirmation(self) -> None:
        """
        Move the robot to the start position, then wait for the user to confirm
        before executing the full trajectory.
        """
        self.get_logger().info("Moving to START position...")
        self._send_single_point(self._joint_trajectory[0])

        print("\n" + "=" * 40)
        user_input = input(" Robot at start. Press ENTER to begin trajectory: ")
        print("=" * 40 + "\n")

        if user_input.lower() in ['']:
            self._begin_trajectory()
        else:
            self.get_logger().info("Trajectory cancelled by user.")
            self.destroy_node()
            rclpy.shutdown()

    # ------------------------------------------------------------------
    # Trajectory execution
    # ------------------------------------------------------------------

    def _begin_trajectory(self) -> None:
        """Start the timer that drives the trajectory execution."""
        self._start_time = self.get_clock().now()
        self._timer = self.create_timer(0.04, self._timer_callback)
        self.get_logger().info("Executing trajectory...")

    def _timer_callback(self) -> None:
        """
        Interpolate between waypoints based on elapsed time and publish the
        current joint configuration at 25 Hz.

        Each segment uses a 95 % move / 5 % dwell profile:
          - First 95 % of segment time: linearly interpolate from start to end.
          - Last  5 % of segment time:  hold the end configuration.
        """
        now = self.get_clock().now()
        elapsed_seconds: float = (now - self._start_time).nanoseconds / 1e9
        segment_idx: int = int(elapsed_seconds // self._time_per_segment)

        if segment_idx >= self._num_segments:
            self._send_single_point(self._joint_trajectory[-1])
            self.get_logger().info("Trajectory complete.")
            self._timer.cancel()
            return

        q_start = self._joint_trajectory[segment_idx]
        q_end   = self._joint_trajectory[segment_idx + 1]

        # Normalised time within current segment [0, 1)
        t_normalised: float = (elapsed_seconds % self._time_per_segment) / self._time_per_segment

        move_fraction = 0.95
        if t_normalised < move_fraction:
            t_interp = t_normalised / move_fraction
            current_q = q_start + (q_end - q_start) * t_interp
        else:
            current_q = q_end

        self._send_single_point(current_q)

    def _send_single_point(self, joint_values: NDArray[np.float64]) -> None:
        """
        Publish a single joint configuration to the trajectory topic.
        """
        msg = JointTrajectory()
        point = JointTrajectoryPoint()
        point.positions = joint_values.tolist()
        msg.points = [point]
        self._publisher.publish(msg)