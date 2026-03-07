#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action.client import ActionClient, ClientGoalHandle, GoalStatus
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from math import pi

column1_min = 0.1
column1_max = 0.7
column1_baseline = 0.4
column1_station_pick = 0.11


class TestMove(Node):
    def __init__(self):
        super().__init__("test_move")
        self.get_logger().info(f"test_move Started")
        
        self.joint_names: list[str] = ['column1_carriage1_joint', 'carriage1_shoulder_joint', 'shoulder_elbow_joint']
        
        self.station_checkpoints = {
            'station_uv': JointTrajectoryPoint(
                positions=[column1_station_pick, pi*0.43, -pi*0.60],
                time_from_start=Duration(sec=1)
            ),
            'station_spinner': JointTrajectoryPoint(
                positions=[column1_station_pick, 2.0, 0.8],
                time_from_start=Duration(sec=1)
            ),
            'station_dev': JointTrajectoryPoint(
                positions=[column1_station_pick, -2.0, -0.8],
                time_from_start=Duration(sec=1)
            ),
        }

        self.act_client = ActionClient(self, FollowJointTrajectory , "/joint_trajectory_controller/follow_joint_trajectory")
        self.start_test_movement()
    
    def start_test_movement(self):
        self.wait_for_act_server()
        goal = FollowJointTrajectory.Goal()
        # there was multi dof trajector that is interesting to look at 
        goal.trajectory.joint_names = self.joint_names
        
        goal.trajectory.points = [self.station_checkpoints['station_uv']]
        
        fut = self.act_client.send_goal_async(goal, self.feedback_callback)
        fut.add_done_callback(self.goal_response_callback)
        
    def feedback_callback(self, feedback_msg):
        feedback: FollowJointTrajectory.Feedback = feedback_msg.feedback
        self.get_logger().info(f'Received feedback: {feedback}')

    
    def goal_response_callback(self, fut):
        goal_handle: ClientGoalHandle = fut.result()
        
        if not goal_handle.accepted:
            self.get_logger().warning('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        status = future.result().status
        
        self.get_logger().info(f'Result: {result}')
        self.get_logger().info(f'Status: {status}')


    def wait_for_act_server(self):
        while not self.act_client.wait_for_server(2):
            self.get_logger().warning("Waiting for Action Client...")
        

def main(args=None):
    rclpy.init(args=args)
    node = TestMove()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()