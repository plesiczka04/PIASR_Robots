import rclpy
from rclpy.node import Node
import numpy as np
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from python_controllers.InverseKinematicsRobot import ik_pose

class RobotMover(Node):
    def __init__(self, waypoints, duration=1.0):
        super().__init__('robot_mover')
        self._publisher = self.create_publisher(JointTrajectory, 'joint_cmds', 10)
        
        # 1. Pre-calculate IK
        self.q_list = []
        for i, pos in enumerate(waypoints):
            res, err = ik_pose(pos[:3],pos[3])
            if isinstance(res, str):
                self.get_logger().warn(f"Waypoint {i} unreachable. Skipping.")
                continue
            
            joint_angles = list(res)
            joint_angles[4] = 1.57
            self.q_list.append(np.array(joint_angles + [float(pos[4])]))

        if not self.q_list:
            self.get_logger().error("No valid waypoints found!")
            return

        self.total_duration = float(duration)
        self.num_segments = len(self.q_list) - 1
        self.time_per_segment = self.total_duration / self.num_segments
        
        # --- MANUAL CONFIRMATION LOGIC ---
        self.get_logger().info("Moving to START position...")
        self.send_single_point(self.q_list[0])
        
        # This blocks the script until you interact with the terminal
        print("\n" + "="*40)
        user_input = input(" Robot at start. Press ENTER to begin trajectory: ")
        print("="*40 + "\n")

        if user_input.lower() in ['']:
            self.begin_trajectory()
        else:
            self.get_logger().info("Trajectory cancelled by user.")
            self.destroy_node()
            rclpy.shutdown()

    def send_single_point(self, q_values):
        msg = JointTrajectory()
        point = JointTrajectoryPoint()
        point.positions = q_values.tolist()
        msg.points = [point]
        self._publisher.publish(msg)

    def begin_trajectory(self):
        # We start the timer immediately now because the user confirmed
        self.start_time = self.get_clock().now()
        self.timer = self.create_timer(0.04, self.timer_callback)
        self.get_logger().info("Executing trajectory...")

    def timer_callback(self):
        now = self.get_clock().now()
        elapsed = (now - self.start_time).nanoseconds / 1e9
        segment_idx = int(elapsed // self.time_per_segment)

        if segment_idx >= self.num_segments:
            self.send_single_point(self.q_list[-1])
            self.get_logger().info("Trajectory complete.")
            self.timer.cancel()
        else:
            q_start = self.q_list[segment_idx]
            q_end = self.q_list[segment_idx + 1]
            t_raw = (elapsed % self.time_per_segment) / self.time_per_segment
            
            # 70% move, 30% wait logic
            if t_raw < 0.7:
                t_interp = t_raw / 0.7
                current_q = q_start + (q_end - q_start) * t_interp
            else:
                current_q = q_end

            self.send_single_point(current_q)

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
    rot = np.linspace(-1,1,10)
    orientations =  np.array(np.meshgrid(rot,rot,rot)).T.reshape(-1,3)
    path = []
    for i in orientations:
        path += [[0.2, 0.2, 0.2,list(i),0]] # Approach
    print(path[-1])
    mover = RobotMover(waypoints=path, duration=10.0)
    if rclpy.ok():
        rclpy.spin(mover)

    mover.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()