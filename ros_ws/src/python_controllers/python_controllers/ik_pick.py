import rclpy
import numpy as np
import sys

from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from python_controllers.InverseKinematicsRobot import ik_position


class PickAndPlace(Node):

    JOINT_NAMES = ['shoulder', 'lower_arm', 'upper_arm', 'wrist', 'gripper']

    def __init__(self, pick_pos, drop_pos):
        super().__init__('pick_and_place')

        self.pub = self.create_publisher(
            JointTrajectory, 'joint_cmds', 10)

        self.pick_pos = np.array(pick_pos, dtype=float)
        self.drop_pos = np.array(drop_pos, dtype=float)

        self.approach_height = 0.1
        self.grip_open = 0.8
        self.grip_closed = 0.1

        self.timer = self.create_timer(1.0, self.execute)

    # --------------------------------------------------
    # SAFE IK (with retry)
    # --------------------------------------------------
    def safe_ik(self, pos):

        for dz in [0.0, 0.02, 0.05, 0.1]:
            test_pos = pos + np.array([0, 0, dz])

            angles, _ = ik_position(test_pos)

            # ❌ string = IK failure
            if isinstance(angles, str):
                continue

            # ❌ NaN check
            if np.any(np.isnan(angles)):
                continue

            return np.asarray(angles, dtype=float)

        self.get_logger().error(f"IK FAILED after retries at {pos}")
        return None

    # --------------------------------------------------
    # CREATE TRAJECTORY POINT
    # --------------------------------------------------
    def make_point(self, angles, gripper, time_sec):

        p = JointTrajectoryPoint()

        # ✅ ensure Python floats
        clean_angles = [float(a) for a in angles]

        p.positions = clean_angles + [float(gripper)]

        p.time_from_start.sec = int(time_sec)
        p.time_from_start.nanosec = int((time_sec % 1) * 1e9)

        return p

    # --------------------------------------------------
    # MAIN EXECUTION PIPELINE
    # --------------------------------------------------
    def execute(self):
        self.timer.cancel()

        self.get_logger().info(f"Pick: {self.pick_pos}, Drop: {self.drop_pos}")

        traj = JointTrajectory()
        traj.joint_names = self.JOINT_NAMES

        points = []
        t = 0

        # -----------------------------
        # 1. APPROACH PICK
        # -----------------------------
        pos = self.pick_pos + [0, 0, self.approach_height]
        angles = self.safe_ik(pos)
        if angles is None: return

        t += 2
        points.append(self.make_point(angles, self.grip_open, t))

        # -----------------------------
        # 2. MOVE DOWN
        # -----------------------------
        angles = self.safe_ik(self.pick_pos)
        if angles is None: return

        t += 2
        points.append(self.make_point(angles, self.grip_open, t))

        # -----------------------------
        # 3. CLOSE GRIPPER
        # -----------------------------
        t += 1
        points.append(self.make_point(angles, 0.5, t))

        t += 1
        points.append(self.make_point(angles, self.grip_closed, t))

        # -----------------------------
        # 4. LIFT
        # -----------------------------
        pos = self.pick_pos + [0, 0, self.approach_height]
        angles = self.safe_ik(pos)
        if angles is None: return

        t += 2
        points.append(self.make_point(angles, self.grip_closed, t))

        # -----------------------------
        # 5. MOVE ABOVE DROP
        # -----------------------------
        pos = self.drop_pos + [0, 0, self.approach_height]
        angles = self.safe_ik(pos)
        if angles is None: return

        t += 3
        points.append(self.make_point(angles, self.grip_closed, t))

        # -----------------------------
        # 6. MOVE DOWN TO DROP
        # -----------------------------
        angles = self.safe_ik(self.drop_pos)
        if angles is None: return

        t += 2
        points.append(self.make_point(angles, self.grip_closed, t))

        # -----------------------------
        # 7. OPEN GRIPPER
        # -----------------------------
        t += 1
        points.append(self.make_point(angles, 0.5, t))

        t += 1
        points.append(self.make_point(angles, self.grip_open, t))

        # -----------------------------
        # 8. RETREAT
        # -----------------------------
        pos = self.drop_pos + [0, 0, self.approach_height]
        angles = self.safe_ik(pos)
        if angles is None: return

        t += 2
        points.append(self.make_point(angles, self.grip_open, t))

        traj.points = points

        self.pub.publish(traj)
        self.get_logger().info("Pick & Place trajectory sent successfully!")

# --------------------------------------------------
# CLI ENTRY
# --------------------------------------------------
def main(args=None):

    cli = sys.argv[1:]

    if len(cli) < 6:
        print("Usage: pick_and_place px py pz dx dy dz")
        sys.exit(1)

    pick = [float(v) for v in cli[:3]]
    drop = [float(v) for v in cli[3:6]]

    rclpy.init(args=args)

    node = PickAndPlace(pick, drop)
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()