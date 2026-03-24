import numpy as np
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import time

# --- Rotation matrices ---
def Rx(theta):
    return np.array([[1,0,0],
                     [0,np.cos(theta),-np.sin(theta)],
                     [0,np.sin(theta), np.cos(theta)]])
def Ry(theta):
    return np.array([[ np.cos(theta),0,np.sin(theta)],
                     [0,1,0],
                     [-np.sin(theta),0,np.cos(theta)]])
def Rz(theta):
    return np.array([[np.cos(theta),-np.sin(theta),0],
                     [np.sin(theta), np.cos(theta),0],
                     [0,0,1]])

# --- Homogeneous transform ---
def homogeneous(R, p):
    T = np.eye(4)
    T[:3,:3] = R
    T[:3,3] = p
    return T

# --- Forward kinematics ---
def forward_kinematics(q):
    q1,q2,q3,q4,q5 = q
    T_0b = homogeneous(Rz(np.pi), np.array([0,0,0]))
    T_b1 = homogeneous(Rz(q1), np.array([0,-0.0452,0.0165]))
    T_12 = homogeneous(Rz(q2) @ Ry(-np.pi/2), np.array([0,-0.0306,0.1025]))
    T_23 = homogeneous(Rz(q3), np.array([0.11257,-0.028,0]))
    T_34 = homogeneous(Rz(q4) @ Rz(np.pi/2), np.array([0.0052,-0.1349,0]))
    T_45 = homogeneous(Rz(q5) @ Ry(-np.pi/2), np.array([-0.0601,0,0]))
    T_5E = homogeneous(np.eye(3), np.array([0,0,0.075]))
    T_01 = T_0b @ T_b1
    T_02 = T_01 @ T_12
    T_03 = T_02 @ T_23
    T_04 = T_03 @ T_34
    T_05 = T_04 @ T_45
    T_0E = T_05 @ T_5E
    return T_0E

class PickPlace(Node):
    def __init__(self):
        super().__init__('pick_place_single')

        # Single joint+gripper target position
        self._joint_sequence = [
            np.array([0.46326219556000003,
                    0.24390294402,
                    -0.7133010627,
                    -1.3222914323600001,
                    0.20555342452,
                    0.81914573652]),

            np.array([0.44638840698,
                    0.23930100168000001,
                    -0.6810874663200001,
                    -1.26400016272,
                    0.19788352062,
                    0.21110684904]),

            np.array([0.11504855850000001,
                    0.25924275182,
                    -0.12118448162,
                    -1.78862158948,
                    0.18714565516,
                    0.21110684904]),

            np.array([-0.29452430976,
                    0.28071848274,
                    -0.28532042508,
                    -1.64749535772,
                    0.20401944374,
                    0.21110684904]),

            np.array([-0.27458255962,
                    0.06596117354,
                    -0.5215534652,
                    -1.3529710479600001,
                    0.20555342452,
                    0.21110684904]),

            np.array([-0.3144660599,
                    0.058291269640000004,
                    -0.52308744598,
                    -1.36370891342,
                    0.20248546296,
                    0.9234564295600001]),

            np.array([-0.3221359638,
                    0.07209709666,
                    -0.09510680836,
                    -1.79015557026,
                    0.1917475975,
                    0.9234564295600001]),

            np.array([0.0,
                      0.0,
                      0.0,
                      0.0,
                      0.0,
                      0.0])
        ]
        self._step_index = 0
        self._publisher = self.create_publisher(JointTrajectory, 'joint_cmds', 10)
        self._timer = self.create_timer(0.04, self.timer_callback)  # 25 Hz

    def timer_callback(self):
        now = self.get_clock().now()

        # Initialize timing on first run
        if not hasattr(self, "_last_switch_time"):
            self._last_switch_time = now

        if self._step_index < len(self._joint_sequence):
            q_full = self._joint_sequence[self._step_index]

            # Split robot + gripper
            q_robot = q_full[:5]
            gripper = q_full[5]

            # Publish command
            msg = JointTrajectory()
            msg.header.stamp = now.to_msg()
            point = JointTrajectoryPoint()
            point.positions = q_robot.tolist() + [gripper]
            point.velocities = [0.0]*6
            msg.points = [point]
            self._publisher.publish(msg)

            # Debug print (only when switching)
            elapsed = (now - self._last_switch_time).nanoseconds * 1e-9

            if elapsed > 8:  # wait 1 second per waypoint
                self.get_logger().info(f"Moving to step {self._step_index+1}")
                self._step_index += 1
                self._last_switch_time = now
        else:
            self.get_logger().info("All commands executed.")

def main(args=None):
    rclpy.init(args=args)
    node = PickPlace()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()

# Close 0.21110684904