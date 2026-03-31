import rclpy
from rclpy.node import Node
import numpy as np
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

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

def homogeneous(R, p):
    T = np.eye(4)
    T[:3,:3] = R
    T[:3,3] = p
    return T

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

    T_list = [T_01, T_02, T_03, T_04, T_05]
    return T_0E, T_list

def compute_jacobian(q):
    T_0E, T_list = forward_kinematics(q)
    p_E = T_0E[:3,3]

    Jv, Jw = [], []
    T_frames = [np.eye(4)] + T_list
    for i in range(5):
        T = T_frames[i]
        z_i = T[:3,2]
        p_i = T[:3,3]
        Jv.append(np.cross(z_i, p_E - p_i))
        Jw.append(z_i)

    Jv = np.array(Jv).T
    Jw = np.array(Jw).T
    return np.vstack((Jv,Jw)), T_0E

def velocity_control(q, v_des, dof=3):
    J, _ = compute_jacobian(q) 
    if dof == 3:
        J_task = J[0:3, :]  # linear
        q_dot = J_task.T @ np.linalg.inv(J_task @ J_task.T) @ v_des
    else:
        J_task = J[0:6, :]  # linear + angular
        q_dot = J_task.T @ np.linalg.inv(J_task @ J_task.T) @ v_des
    return q_dot

class VelTraj(Node):
    def __init__(self, dof=6):
        super().__init__('vel_traj_node')

        # initial configuration
        self.q = np.array([0.0, 0.5, -0.2, -0.2, 0.0])
        self.dof = dof

        self._publisher = self.create_publisher(JointTrajectory, 'joint_cmds', 100)
        self.dt = 0.01
        self._timer = self.create_timer(self.dt, self.timer_callback)

        self.direction = 1.0
        self.v_mag = 0.005
        self.workspace_limit = 0.05
        self.offset = np.array([0.0, 0.15, 0.2])
        self.p_start = None

    def desired_ee_velocity(self, p):
        if self.p_start is None:
            self.p_start = p + self.offset

        p_rel = p - self.p_start
        self.direction = -1.0
        # if p_rel[0] > self.workspace_limit:
        #     self.direction = -1.0
        # elif p_rel[0] < -self.workspace_limit:
        #     self.direction = 1.0

        vz = self.direction * self.v_mag
        vy = 0.0
        vx = 0.0

        if self.dof == 3:
            return np.array([vx, vy, vz])
        else:
            # angular velocities if full 6 DoF control
            wx, wy, wz = 0.0, 0.0, 0.0
            return np.array([vx, vy, vz, wx, wy, wz])

    def timer_callback(self):
        msg = JointTrajectory()
        msg.header.stamp = self.get_clock().now().to_msg()
        point = JointTrajectoryPoint()

        T_0E, _ = forward_kinematics(self.q)
        p = T_0E[:3,3]

        v_des = self.desired_ee_velocity(p)
        q_dot = velocity_control(self.q, v_des, dof=self.dof)
        print(q_dot)
        self.q += q_dot * self.dt

        point.positions = self.q.tolist()
        point.velocities = q_dot.tolist()
        msg.points = [point]
        self._publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    # choose dof=3 for linear only, dof=6 for full 6 DoF
    node = VelTraj(dof=3)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()