import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.action import FollowJointTrajectory
from sensor_msgs.msg import JointState
from cpp_pkg.srv import ComputeLegIK  

import time
import threading

class LegIKClient(Node):

    def __init__(self):
        super().__init__('leg_ik_client')

        self.ik_client = self.create_client(ComputeLegIK, '/compute_leg_ik')
        while not self.ik_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('IK Service not available, waiting...')
        self.request = ComputeLegIK.Request()

        self.traj_action_client = ActionClient(self, FollowJointTrajectory, "/leg_controller/follow_joint_trajectory")
        while not self.traj_action_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().info('Trajectory action server not available, waiting...')

        self._action_complete = False
        self._action_success = False
        self._action_lock = threading.Lock()

    def compute_ik_for_leg(self, leg_tip, target_foot_pos, curr_knee_joint, curr_ankle_joint):
        self.request.leg_tip = leg_tip
        self.request.target_foot_pos.x = target_foot_pos['x']
        self.request.target_foot_pos.y = target_foot_pos['y']
        self.request.target_foot_pos.z = target_foot_pos['z']
        self.request.curr_knee_joint = curr_knee_joint
        self.request.curr_ankle_joint = curr_ankle_joint

        future = self.ik_client.call_async(self.request)
        rclpy.spin_until_future_complete(self, future)
        response = future.result()
        if response is not None:
            return response.success_flag, response.tar_knee_joint, response.tar_ankle_joint 
        else:
            self.get_logger().error('Service call failed')
            return None
        
    def send_trajectory(self, moving_leg_names, computed_knee_joint, computed_ankle_joint, timeout=10.0):
        all_joints = [
            "kneeFR", "ankleFR",
            "kneeFL", "ankleFL",
            "kneeBR", "ankleBR",
            "kneeBL", "ankleBL"
        ]

        with self._action_lock:
            self._action_complete = False
            self._action_success = False

        if not hasattr(self, "current_positions"):
            self.current_positions = {j: 0.0 for j in all_joints}

        # Update only the two joints we want to move
        self.current_positions[moving_leg_names[0]] = computed_knee_joint
        self.current_positions[moving_leg_names[1]] = computed_ankle_joint

        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory = JointTrajectory()
        goal_msg.trajectory.joint_names = all_joints

        point = JointTrajectoryPoint()
        point.positions = [self.current_positions[j] for j in all_joints]
        point.time_from_start.sec = 2
        point.time_from_start.nanosec = 0
        goal_msg.trajectory.points = [point]

        self.get_logger().info(f'Sending trajectory for joints: {moving_leg_names}')
        self.get_logger().info(f'Target positions: knee={computed_knee_joint:.3f}, ankle={computed_ankle_joint:.3f}')

        self._send_goal_future = self.traj_action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
        self._send_goal_future.add_done_callback(self.trajectory_response_callback)

        start_time = time.time()
        while rclpy.ok() and (time.time() - start_time) < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)
            
            with self._action_lock:
                if self._action_complete:
                    success = self._action_success
                    self._action_complete = False  
                    return success
                    
        self.get_logger().error(f'Trajectory execution timed out after {timeout} seconds')
        return False
    
    def feedback_callback(self, feedback_msg):
        """Handle trajectory execution feedback"""
        feedback = feedback_msg.feedback

    def trajectory_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Trajectory goal rejected')
            with self._action_lock:
                self._action_complete = True
                self._action_success = False
            return
        self.get_logger().info('Trajectory goal accepted, waiting for execution...')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.trajectory_result_callback)

    def trajectory_result_callback(self, future):
        result = future.result().result
        with self._action_lock:
            self._action_complete = True
            if result.error_code == result.SUCCESSFUL:
                self.get_logger().info('Trajectory executed successfully')
                self._action_success = True
            else:
                self.get_logger().error(f'Trajectory failed with error code: {result.error_code}')
                if hasattr(result, 'error_string') and result.error_string:
                    self.get_logger().error(f'Error details: {result.error_string}')
                self._action_success = False

        
class Leg():
    def __init__(self, leg_tip, 
                knee_joint_name, ankle_joint_name, 
                knee_joint_limit, ankle_joint_limit, 
                last_foot_pos={'x': 0.0, 'y': 0.0, 'z': 0.0}, 
                tar_foot_pos={'x': 0.0, 'y': 0.0, 'z': 0.0},
                curr_knee_joint=0.0, curr_ankle_joint=0.0):
        self.leg_tip = leg_tip
        self.knee_joint_name = knee_joint_name
        self.ankle_joint_name = ankle_joint_name
        self.knee_joint_limit = knee_joint_limit
        self.ankle_joint_limit = ankle_joint_limit
        self.last_foot_pos = last_foot_pos
        self.tar_foot_pos = tar_foot_pos
        self.curr_knee_joint = curr_knee_joint
        self.curr_ankle_joint = curr_ankle_joint

def main(args=None):
    rclpy.init(args=args)
    ik_client = LegIKClient()
    rclpy.spin_once(ik_client, timeout_sec=1.0)

    legFR = Leg('leg_tip_1', 'kneeFR', 'ankleFR', 
                (-1.5708, 0.0), (0.0, 0.872665), 
                {'x': 0.32446, 'y': -0.072662, 'z': -0.047324}, 
                {'x': 0.185, 'y': -0.26, 'z': 0.05}, 
                0.0, 0.0)
    
    legBR = Leg('leg_tip_2', 'kneeBR', 'ankleBR',
                (0.0, 1.5708), (0.0, 0.872665), 
                {'x': -0.32446, 'y': -0.072662, 'z': -0.047324}, 
                {'x': -0.185, 'y': -0.26, 'z': 0.05}, 
                0.0, 0.0)
    
    legFL = Leg('leg_tip_3', 'kneeFL', 'ankleFL', 
                (-1.5708, 0.0), (0.0, 0.872665), 
                {'x': 0.32446, 'y': 0.072662, 'z': -0.047324}, 
                {'x': 0.185, 'y': 0.26, 'z': 0.05}, 
                0.0, 0.0)
    
    legBL = Leg('leg_tip_4', 'kneeBL', 'ankleBL',
                (0.0, 1.5708), (0.0, 0.872665), 
                {'x': -0.32446, 'y': 0.072662, 'z': -0.047324}, 
                {'x': -0.185, 'y': 0.26, 'z': 0.05}, 
                0.0, 0.0)
    
    legs = [legFR, legFL, legBR, legBL]
    ik_client.get_logger().info('Starting leg trajectory execution...')
    
    successful_legs = 0
    total_legs = len(legs)

    for i, leg in enumerate(legs):
        ik_client.get_logger().info(f'Processing leg {i+1}/{total_legs}: {leg.leg_tip}')

        result = ik_client.compute_ik_for_leg(leg.leg_tip, leg.tar_foot_pos, leg.curr_knee_joint, leg.curr_ankle_joint)

        if result and result[0]:
            leg.curr_knee_joint = result[1]
            leg.curr_ankle_joint = result[2]
            leg.last_foot_pos = leg.tar_foot_pos.copy()

            ik_client.get_logger().info(
                f'IK computed for {leg.leg_tip}: '
                f'knee={result[1]:.3f}, ankle={result[2]:.3f}'
            )

            success = ik_client.send_trajectory(
                [leg.knee_joint_name, leg.ankle_joint_name], 
                leg.curr_knee_joint, 
                leg.curr_ankle_joint,
                timeout=10.0
            )
            
            if success:
                successful_legs += 1
                ik_client.get_logger().info(f'Successfully moved {leg.leg_tip}')
            else:
                ik_client.get_logger().error(f'Failed to move {leg.leg_tip}')
        else:
            ik_client.get_logger().error(f'IK Failed for {leg.leg_tip}')

        time.sleep(0.5)
    
    ik_client.get_logger().info(
        f'Leg movement completed: {successful_legs}/{total_legs} legs moved successfully'
    )

    ik_client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()