import rclpy
from rclpy.node import Node
import numpy as np
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from python_controllers.robot_mover import RobotMover


def Approach (Point,zlim = 0.03,points = 3):
    path = []
    for i in np.linspace(Point[2],zlim,points):
        if i > 0.10:
            path += [[Point[0],Point[1],i] + [None,Point[4]]]
        else:
            path += [[Point[0],Point[1],i] + [Point[3],Point[4]]]
    return path
    
def main(args=None):
    rclpy.init(args=args)
    DOWN = [0,0,-1]

    path = []
    i = 0
    zlim = 0.02
    height = 0.05

    gripper_open = 0.6
    gripper_closed = 0.03

    x_left = -0.05
    y_left = 0.16

    x_right = 0.05
    y_right = 0.16

    # Move to left and grab
    path += [[0, 0.2, 0.2, None, gripper_open]]
    path += Approach([x_left, y_left, 0.03+zlim,  DOWN , gripper_open],zlim = 0.05,points=2) # Approach
    path += [[x_left, y_left, 0.03+zlim, DOWN, gripper_closed]]
    path += [[x_left, y_left, 0.1, DOWN, gripper_closed]]

    path += [[x_right, y_right, 0.1+zlim, None, gripper_closed]]
    path += [[x_right, y_right, 0.03+zlim, DOWN, gripper_closed]]
    path += [[x_right, y_right, 0.03+zlim, DOWN, gripper_open]]
    path += [[x_right, y_right, 0.1, None, gripper_open]]
    path += [[0, 0.2, 0.2, None, gripper_open]]

    mover = RobotMover(waypoints=path, duration=20.0)
    if rclpy.ok():
        rclpy.spin(mover)

    mover.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()