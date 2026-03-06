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
    T_list = [T_01, T_02, T_03, T_04, T_05, T_0E]
    return T_0E, T_list

def compute_jacobian(q):
    T_0E, T_list = forward_kinematics(q)
    p_E = T_0E[:3,3]
    Jv, Jw = [], []
    T_frames = [np.eye(4)] + T_list[:-1]
    for i in range(5):
        T = T_frames[i]
        z_i = T[:3,2]
        p_i = T[:3,3]
        Jv.append(np.cross(z_i, p_E - p_i))
        Jw.append(z_i)
    Jv = np.array(Jv).T
    Jw = np.array(Jw).T
    return np.vstack((Jv,Jw))

def damped_pseudoinverse(J, lam=0.01):
    return J.T @ np.linalg.inv(J @ J.T + lam**2 * np.eye(J.shape[0]))

q_min = np.array([-1.9, -3.15, -1.59, -0.98, -np.pi])
q_max = np.array([ 2.3,  0.8,  1.79,  2.6,  np.pi])
def joint_limit_gradient(q):
    q_mid = (q_max + q_min)/2
    q_range = (q_max - q_min)/2
    grad = (q - q_mid) / (q_range**2)
    return -grad

def velocity_control(q, x_dot, lam=0.01, k_null=0.2):
    J = compute_jacobian(q)
    J_pinv = damped_pseudoinverse(J, lam)
    q_dot_task = J_pinv @ x_dot
    I = np.eye(len(q))
    N = I - J_pinv @ J
    q_dot_sec = joint_limit_gradient(q)
    q_dot = q_dot_task + k_null*(N @ q_dot_sec)
    q_dot_max = np.array([1.5]*5)
    q_dot = np.clip(q_dot, -q_dot_max, q_dot_max)
    return q_dot

class ExampleVelTraj(Node):
    def __init__(self):
        super().__init__('example_vel_traj_node')

        # Home position
        self.q_home = np.array([0, np.deg2rad(70), np.deg2rad(-40), np.deg2rad(-60), 0])
        self.q = np.zeros(5)  # start from zero
        self._publisher = self.create_publisher(JointTrajectory, 'joint_cmds', 100)
        self._timer = self.create_timer(0.01, self.timer_callback)

        # Flags and timing
        self._at_home = False
        self._beginning = self.get_clock().now()
        self._move_speed = 0.2  # rad/s

    def timer_callback(self):
        now = self.get_clock().now()
        dt = 0.04  # timer period

        msg = JointTrajectory()
        msg.header.stamp = now.to_msg()
        point = JointTrajectoryPoint()

        if not self._at_home:
            # Move to home smoothly
            q_diff = self.q_home - self.q
            step = np.clip(q_diff, -self._move_speed*dt, self._move_speed*dt)
            self.q += step
            if np.allclose(self.q, self.q_home, atol=1e-3):
                self.q = self.q_home.copy()
                self._at_home = True
                self.get_logger().info("Reached home, starting small periodic motion")
            point.positions = self.q.tolist()
            point.velocities = (step/dt).tolist()
        else:
            # Small periodic motion around home
            t = (now - self._beginning).nanoseconds * 1e-9
            period = 10  # seconds
            amp = 3     # rad
            vel = amp * 2*np.pi/period * np.cos(2*np.pi/period*t)
            q_dot = np.array([vel, -vel, vel/2, -vel/2, 0])
            self.q += q_dot*dt

            # Clamp to joint limits
            self.q = np.clip(self.q, q_min, q_max)
            point.positions = self.q.tolist()
            point.velocities = q_dot.tolist()

        msg.points = [point]
        self._publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = ExampleVelTraj()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()