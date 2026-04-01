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

# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main(args=None) -> None:
    rclpy.init(args=args)

    # --- Constants -----------------------------------------------------------
    down_direction: list[int] = [0, 0, -1]

    gripper_open:      float = 0.4
    gripper_closed:    float = 0.2
    gripper_more_open: float = 0.55

    lift_height:  float = 0.05   # metres above the stack before descending
    feed_x:       float = 0.0
    feed_y:       float = 0.16
    stack_y:      float = 0.14

    total_blocks: int   = 9
    z_limit:      float = 0.035  # current top of the stack (grows each iteration)

    # --- Path construction ---------------------------------------------------
    path: list[Waypoint] = []
    block_index: int = 0

    while block_index < total_blocks:

        # ---- Pick from feed -------------------------------------------------
        if block_index < 2:
            pick_gripper   = gripper_open
            place_x        = 0.09
            place_z_factor = 1.0        # exact z_limit
            place_gripper  = gripper_open

        elif block_index < 4:
            pick_gripper   = gripper_more_open
            place_x        = 0.09
            place_z_factor = 1.1
            place_gripper  = gripper_more_open

        elif block_index < 8:
            pick_gripper   = gripper_more_open
            place_x        = 0.085
            place_z_factor = 1.1
            place_gripper  = gripper_more_open

        else:
            pick_gripper   = gripper_more_open
            place_x        = 0.085
            place_z_factor = 0.99
            place_gripper  = gripper_more_open

        # Approach feed location
        path += build_approach_path(
            [feed_x, feed_y, lift_height + z_limit, down_direction, pick_gripper],
            z_limit=0.03,
            num_points=2,
        )

        # Grasp at feed
        path += [[feed_x, feed_y, 0.03, down_direction, gripper_closed]]

        # Retreat from feed (only needed when the stack is already tall)
        if z_limit > 0.15:
            path += build_approach_path(
                [feed_x, feed_y, lift_height + z_limit, down_direction, gripper_closed],
                z_limit=0.03,
                num_points=2,
            )[::-1]

        # Approach stack location
        place_z = z_limit * place_z_factor
        path += build_approach_path(
            [place_x, stack_y, lift_height + z_limit, down_direction, gripper_closed],
            z_limit=z_limit,
            num_points=2,
        )

        # Place on stack
        path += [[place_x, stack_y, place_z, down_direction, place_gripper]]

        # Retreat from stack (skip on the very last block)
        if block_index != total_blocks - 1:
            path += build_approach_path(
                [place_x, stack_y, lift_height + place_z, down_direction, place_gripper],
                z_limit=z_limit,
            )[::-1]

        z_limit     += 0.02
        block_index += 1

    # --- Execute -------------------------------------------------------------
    mover = RobotMover(waypoints=path, duration=60.0)

    if rclpy.ok():
        rclpy.spin(mover)

    mover.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()