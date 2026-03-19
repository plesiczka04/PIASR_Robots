import rclpy
import numpy as np
import sys
from rclpy.node import Node
from scipy.optimize import minimize
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

np.set_printoptions(precision=4, suppress=True)

# ─────────────────────────────────────────────
#  Forward / Inverse Kinematics  (unchanged)
# ─────────────────────────────────────────────

def Rotation(angle, axis: int):
    c, s = np.cos(angle), np.sin(angle)
    if axis == 0:
        return np.array([[1,0,0],[0,c,-s],[0,s,c]])
    if axis == 1:
        return np.array([[c,0,s],[0,1,0],[-s,0,c]])
    if axis == 2:
        return np.array([[c,-s,0],[s,c,0],[0,0,1]])
    raise ValueError("axis must be 0, 1, or 2")

def Transformation(angle_vector: np.ndarray,
                   translation_vector: np.ndarray = np.zeros(3)):
    R = np.eye(4)
    R[0:3, 3] = translation_vector
    R[0:3, 0:3] = (
        Rotation(angle_vector[0], 0)
        @ Rotation(angle_vector[1], 1)
        @ Rotation(angle_vector[2], 2)
    )
    return R

T_WOB = Transformation(np.array([0, 0, np.pi]))
T_BS  = Transformation(np.zeros(3), np.array([0, -0.0452,  0.0165]))
T_SU  = Transformation(np.array([0, -np.pi/2, 0]), np.array([0, -0.0306, 0.1025]))
T_UL  = Transformation(np.zeros(3), np.array([0.11257, -0.028, 0]))
T_LW  = Transformation(np.array([0, 0, np.pi/2]), np.array([0.0052, -0.1349, 0]))
T_WRG = Transformation(np.array([0, -np.pi/2, 0]), np.array([-0.0601, 0, 0]))
T_GGC = Transformation(np.zeros(3), np.array([0, 0, 0.075]))

def fk_full(angles):
    θ_s, θ_l, θ_u, θ_w, θ_g = angles
    Ti = Transformation(np.array([0, 0, θ_s]))
    Tj = Transformation(np.array([0, 0, θ_l]))
    Tk = Transformation(np.array([0, 0, θ_u]))
    Tl = Transformation(np.array([0, 0, θ_w]))
    Tg = Transformation(np.array([0, 0, θ_g]))
    return T_WOB @ T_BS @ Ti @ T_SU @ Tj @ T_UL @ Tk @ T_LW @ Tl @ T_WRG @ Tg @ T_GGC

def rotation_error_vec(R_current, R_target):
    R_err = R_target @ R_current.T
    cos_angle = np.clip((np.trace(R_err) - 1) / 2, -1.0, 1.0)
    angle = np.arccos(cos_angle)
    if abs(angle) < 1e-8:
        return np.zeros(3)
    axis = np.array([
        R_err[2,1] - R_err[1,2],
        R_err[0,2] - R_err[2,0],
        R_err[1,0] - R_err[0,1]
    ]) / (2 * np.sin(angle))
    return axis * angle

def ik_pose(target, initial_guess=np.zeros(5), w_pos=1.0, w_rot=1.0):
    """Solve IK for target = [x, y, z, rx, ry, rz]."""
    target_pos = np.array(target[:3])
    target_rot = Transformation(np.array(target[3:]))[:3, :3]

    bounds = [
        (-1.9,  2.3),
        (-3.15, 0.8),
        (-1.59, 1.79),
        (-0.98, 2.60),
        (-np.pi, np.pi),
    ]

    def cost(angles):
        T = fk_full(angles)
        pos_err = np.linalg.norm(T[:3, 3] - target_pos)
        rot_err = np.linalg.norm(rotation_error_vec(T[:3, :3], target_rot))
        return w_pos * pos_err**2 + w_rot * rot_err**2

    result = minimize(cost, initial_guess, bounds=bounds, method='L-BFGS-B')
    return result.x, result.fun


# ─────────────────────────────────────────────
#  ROS 2 Node
# ─────────────────────────────────────────────

class MoveToPose(Node):
    """
    Accepts a 6-DOF target pose [x, y, z, rx, ry, rz],
    solves IK, then publishes the joint trajectory once
    (with a configurable move duration).
    """

    JOINT_NAMES = ['shoulder', 'lower_arm', 'upper_arm', 'wrist', 'gripper_rot']

    def __init__(self, target: list, move_duration: float = 3.0,
                 gripper_open: float = 0.0):
        super().__init__('move_to_pose')

        self._target = target
        self._move_duration = move_duration
        self._gripper_open = gripper_open   # gripper opening width (joint 6)

        self._publisher = self.create_publisher(
            JointTrajectory, 'joint_cmds', 10)

        # Solve IK before publishing
        self.get_logger().info(
            f'Solving IK for target: {target}')
        angles, cost = ik_pose(target)
        if cost > 1e-3:
            self.get_logger().warn(
                f'IK residual cost {cost:.6f} — solution may be inaccurate!')
            self.get_logger().warn(
                f'IK solved  →  angles (deg): {np.rad2deg(angles).round(2)}')
        else:
            self.get_logger().info(
                f'IK solved  →  angles (deg): {np.rad2deg(angles).round(2)}')

        self._angles = angles

        # Publish once after a short delay (lets the subscriber come up)
        self._timer = self.create_timer(0.5, self._publish_once)

    def _publish_once(self):
        self._timer.cancel()          # fire only once

        msg = JointTrajectory()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.joint_names = self.JOINT_NAMES

        point = JointTrajectoryPoint()

        # 5 arm joints from IK + gripper open/close as 6th joint
        point.positions = list(self._angles) + [self._gripper_open]

        # Tell the controller how long it has to reach the position
        secs = int(self._move_duration)
        nsecs = int((self._move_duration - secs) * 1e9)
        point.time_from_start.sec = secs
        point.time_from_start.nanosec = nsecs

        msg.points = [point]
        self._publisher.publish(msg)

        self.get_logger().info(
            f'Published trajectory — move duration: {self._move_duration} s')


# ─────────────────────────────────────────────
#  Entry point
# ─────────────────────────────────────────────

def main(args=None):
    # ── Parse target from CLI ────────────────────────────────
    # Usage:
    #   ros2 run <pkg> ik_move_to_pose  x y z rx ry rz  [duration]  [gripper]
    #
    # Example (position only, default orientation 0 0 0):
    #   ros2 run my_pkg ik_move_to_pose  0.2 0.2 0.2  0.0 1.57 0.65
    #
    # With custom move duration (5 s) and gripper fully open (0.8):
    #   ros2 run my_pkg ik_move_to_pose  0.2 0.2 0.2  0.0 1.57 0.65  5.0  0.8

    cli = sys.argv[1:]          # strip the script name

    if len(cli) < 6:
        print(
            "Usage: ik_move_to_pose  x y z rx ry rz  [move_duration=3.0]  [gripper=0.0]\n"
            "Example: ik_move_to_pose  0.2 0.2 0.2  0.0 1.57 0.65"
        )
        sys.exit(1)

    target = [float(v) for v in cli[:6]]
    move_duration = float(cli[6]) if len(cli) > 6 else 3.0
    gripper_open  = float(cli[7]) if len(cli) > 7 else 0.0

    rclpy.init(args=args)
    node = MoveToPose(target, move_duration, gripper_open)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
