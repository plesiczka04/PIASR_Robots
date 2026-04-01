import rclpy
from rclpy.node import Node
import numpy as np
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

# ---------------------------------------------------------------------------
# Rotation matrices
# ---------------------------------------------------------------------------

def Rx(theta: float) -> np.ndarray:
    """Rotation about X axis by theta radians."""
    return np.array([[1,0,0],
                     [0,np.cos(theta),-np.sin(theta)],
                     [0,np.sin(theta), np.cos(theta)]])

def Ry(theta: float) -> np.ndarray:
    """Rotation about Y axis by theta radians."""
    return np.array([[ np.cos(theta),0,np.sin(theta)],
                     [0,1,0],
                     [-np.sin(theta),0,np.cos(theta)]])

def Rz(theta: float) -> np.ndarray:
    """Rotation about Z axis by theta radians."""
    return np.array([[np.cos(theta),-np.sin(theta),0],
                     [np.sin(theta), np.cos(theta),0],
                     [0,0,1]])

# ---------------------------------------------------------------------------
# Homogeneous transformation
# ---------------------------------------------------------------------------

def homogeneous(R: np.ndarray, p: np.ndarray) -> np.ndarray:
    """
    Build a 4×4 homogeneous transformation matrix from rotation R and position p.
    """
    T = np.eye(4)
    T[:3,:3] = R
    T[:3,3] = p
    return T

# ---------------------------------------------------------------------------
# Forward kinematics
# ---------------------------------------------------------------------------

def forward_kinematics(q: np.ndarray) -> tuple[np.ndarray, list[np.ndarray]]:
    """
    Compute the end-effector pose and intermediate link transforms from joint angles.

    Returns:
        T_0E: 4x4 end-effector homogeneous transform
        T_list: list of 4x4 transforms for each intermediate link
    """
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

# ---------------------------------------------------------------------------
# Jacobian computation
# ---------------------------------------------------------------------------

def compute_jacobian(q: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """
    Compute the 6x5 Jacobian matrix for the manipulator.

    Returns:
        J: 6x5 Jacobian (top: linear, bottom: angular)
        T_0E: end-effector transform
    """
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

# ---------------------------------------------------------------------------
# Velocity control
# ---------------------------------------------------------------------------

def velocity_control(q: np.ndarray, v_des: np.ndarray, dof: int = 3) -> np.ndarray:
    """
    Compute joint velocities for a desired end-effector velocity using Jacobian.
    
    Args:
        q: current joint angles
        v_des: desired end-effector velocity (3D or 6D)
        dof: 3 for linear only, 6 for full DoF
    
    Returns:
        q_dot: computed joint velocities
    """
    J, _ = compute_jacobian(q) 
    if dof == 3:
        J_task = J[0:3, :]  # linear
        q_dot = J_task.T @ np.linalg.inv(J_task @ J_task.T) @ v_des
    else:
        J_task = J[0:6, :]  # linear + angular
        q_dot = J_task.T @ np.linalg.inv(J_task @ J_task.T) @ v_des
    return q_dot

# ---------------------------------------------------------------------------
# ROS2 Node for velocity trajectory control
# ---------------------------------------------------------------------------

class VelTraj(Node):
    """
    ROS2 node that publishes joint commands to follow a desired end-effector velocity.
    """
    def __init__(self, dof: int = 6):
        super().__init__('vel_traj_node')

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

    def desired_ee_velocity(self, p: np.ndarray) -> np.ndarray:
        """
        Compute a simple desired end-effector velocity along Z axis.
        """
        if self.p_start is None:
            self.p_start = p + self.offset

        p_rel = p - self.p_start
        self.direction = -1.0
        # Example for switching direction if workspace limits exceeded
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
            wx, wy, wz = 0.0, 0.0, 0.0
            return np.array([vx, vy, vz, wx, wy, wz])

    def timer_callback(self):
        """
        Timer callback to compute joint velocities, update joint angles, and publish commands.
        """
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

# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main(args=None):
    rclpy.init(args=args)
    node = VelTraj(dof=3)  # linear only (3 DoF)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()