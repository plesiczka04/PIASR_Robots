import rclpy
from rclpy.node import Node

import numpy as np
from numpy.typing import NDArray

from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from python_controllers.robot_mover import RobotMover

# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main(args=None) -> None:
    rclpy.init(args=args)
    gripper = 0.0
    path = [
    [0.2,    0.2,    0.2,  [0.7961, 0.6052, 0.0008], gripper],
    [0.2,    0.1,    0.4,  [0.0, 0.0,1.0], gripper],
    [0.0,    0.0,    0.4,  [-0.0006 ,-0.7068,  0.7074], gripper],
    [0.0,    0.0,    0.07, [ 0.0,     -0.0006, -1.0   ], gripper],
    [0.0,    0.0452, 0.45, [-0.0004 ,-0.7068,  0.7074], gripper],
]

    # --- Execute -------------------------------------------------------------
    mover = RobotMover(waypoints=path, duration=15.0)

    if rclpy.ok():
        rclpy.spin(mover)

    mover.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()