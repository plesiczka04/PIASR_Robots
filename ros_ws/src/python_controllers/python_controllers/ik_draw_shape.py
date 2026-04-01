import rclpy
from rclpy.node import Node

import numpy as np
from numpy.typing import NDArray

from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from python_controllers.robot_mover import RobotMover


# ---------------------------------------------------------------------------
# Types
# ---------------------------------------------------------------------------

# A waypoint is [x, y, z, orientation_or_None, gripper_value]
Waypoint = list[float | None]

# IK result is either a numpy array of joint angles, or a string error message
IKResult = NDArray[np.float64] | str

# ---------------------------------------------------------------------------
# Path generation helpers
# ---------------------------------------------------------------------------

def build_approach_path(
    target: Waypoint,
    z_limit: float = 0.03,
    num_points: int = 3,
) -> list[Waypoint]:
    """
    Generate a vertical approach (or retreat) path toward `target`.

    For z-values above 0.10 m the orientation is set to None (free); below
    that threshold the full orientation from `target` is used so the gripper
    is correctly aligned before contact.
    """
    path: list[Waypoint] = []

    for z in np.linspace(target[2], z_limit, num_points):
        orientation = None if z > 0.10 else target[3]
        path.append([target[0], target[1], z, orientation, target[4]])

    return path

def build_figure8_path(
    center: tuple[float, float, float],
    A: float = 0.1,
    B: float = 0.1,
    num_points: int = 120,
    orientation=None,
    gripper: float = 0.0,
) -> list[Waypoint]:
    """
    Generate a smooth figure-8 (lemniscate) path in Cartesian space.
    """
    cx, cy, cz = center
    path: list[Waypoint] = []

    t_values = np.linspace(0, 2 * np.pi, num_points)

    for t in t_values:
        x = cx + A * np.sin(t)
        z = cz + B * np.sin(t) * np.cos(t)
        y = cy  # keep constant height

        path.append([x, y, z, orientation, gripper])

    return path


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main(args=None) -> None:
    rclpy.init(args=args)

    path = build_figure8_path([0.0,0.2,0.2])

    # --- Execute -------------------------------------------------------------
    mover = RobotMover(waypoints=path, duration=15.0)

    if rclpy.ok():
        rclpy.spin(mover)

    mover.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()