#!/usr/bin/env python3
import sys
import termios
import tty
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
from sensor_msgs.msg import JointState

# Joints in the order from your YAML
JOINT_NAMES = [
    "ankleFR", "kneeFR",
    "ankleFL", "kneeFL",
    "ankleBR", "kneeBR",
    "ankleBL", "kneeBL"
]

# Joint limits
JOINT_LIMITS = {
    "ankleFR": (0.0, 0.872665),
    "kneeFR": (-1.5708, 0.0),
    "ankleFL": (0.0, 0.872665),
    "kneeFL": (-1.5708, 0.0),
    "ankleBR": (0.0, 0.872665),
    "kneeBR": (0.0, 1.5708),
    "ankleBL": (0.0, 0.872665),
    "kneeBL": (0.0, 1.5708),
}

STEP_SIZE = 0.05  # radians per keypress

def get_key():
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return ch


class LegTeleop(Node):
    def __init__(self):
        super().__init__("leg_teleop")

        # Action client for JointTrajectoryController
        self._action_client = ActionClient(self, FollowJointTrajectory, "/leg_controller/follow_joint_trajectory")

        # Subscription to joint states
        self.joint_states = {}
        self.create_subscription(JointState, "/joint_states", self.joint_state_cb, 10)

        # Timer to print state every second
        self.create_timer(1.0, self.print_joint_states)

        # Internal state
        self.joint_positions = [0.0] * len(JOINT_NAMES)
        self.active_leg = 0  # 0=FR, 1=FL, 2=BR, 3=BL

        self.get_logger().info(
            "Leg teleop started.\n"
            "Keys: q=quit | n=next leg | u/d=ankle | l/r=knee"
        )

    def joint_state_cb(self, msg: JointState):
        for name, pos in zip(msg.name, msg.position):
            self.joint_states[name] = pos

    def print_joint_states(self):
        if not self.joint_states:
            self.get_logger().warn("No joint states received yet...")
            return
        self.get_logger().info("------ Joint States ------")
        for j in JOINT_NAMES:
            val = self.joint_states.get(j, 0.0)
            self.get_logger().info(f"{j:8s}: {val:+.3f} rad")
        self.get_logger().info("--------------------------")

    def clamp(self, joint_name, value):
        lo, hi = JOINT_LIMITS[joint_name]
        return max(lo, min(hi, value))

    def send_trajectory(self):
        if not self._action_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().warn("JointTrajectory action server not available!")
            return

        traj = JointTrajectory()
        traj.joint_names = JOINT_NAMES

        point = JointTrajectoryPoint()
        point.positions = self.joint_positions
        point.velocities = [0.0] * len(JOINT_NAMES)
        point.time_from_start.sec = 1  # reach in 1 second

        traj.points.append(point)

        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory = traj

        self._action_client.send_goal_async(goal_msg)

    def update_joint(self, joint_index, delta):
        joint_name = JOINT_NAMES[joint_index]
        new_val = self.joint_positions[joint_index] + delta
        self.joint_positions[joint_index] = self.clamp(joint_name, new_val)

        self.get_logger().info(
            f"{joint_name} = {self.joint_positions[joint_index]:.3f} rad (commanded)"
        )
        self.send_trajectory()

    def run(self):
        while rclpy.ok():
            key = get_key()
            if key == "q":
                break
            elif key == "n":  # cycle active leg
                self.active_leg = (self.active_leg + 1) % 4
                self.get_logger().info(
                    f"Active leg: {['FR','FL','BR','BL'][self.active_leg]}"
                )
            elif key == "u":  # ankle up
                self.update_joint(self.active_leg * 2, STEP_SIZE)
            elif key == "d":  # ankle down
                self.update_joint(self.active_leg * 2, -STEP_SIZE)
            elif key == "r":  # knee +
                self.update_joint(self.active_leg * 2 + 1, STEP_SIZE)
            elif key == "l":  # knee -
                self.update_joint(self.active_leg * 2 + 1, -STEP_SIZE)


def main(args=None):
    rclpy.init(args=args)
    node = LegTeleop()
    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
