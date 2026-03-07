#!/usr/bin/env python3
from enum import Enum, auto

import rclpy
from rclpy.node import Node
from rclpy.action.client import ActionClient, ClientGoalHandle, GoalStatus
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from math import pi

# Some constants
class States(Enum):
    IDLE = auto()
    MOVE_TO_STATION = auto()
    DROP = auto()
    PICK = auto()
    
class Station:
    joint_names: list[str] = ['column1_carriage1_joint', 'carriage1_shoulder_joint', 'shoulder_elbow_joint']
    movement_duration=1
    
    def __init__(self, name: str, baseline_height: float, pick_height: float, pos_shoulder: float, pos_elbow: float, running_time: float):
        # traj points expected ['shoudler', 'elbow']
        self.name = name
        self.baseline_height = baseline_height
        self.pick_height = pick_height
        self.pos_shoulder = pos_shoulder
        self.pos_elbow = pos_elbow
        self.running_time = running_time
        
        
    def get_traj_pick(self):
        return self._create_traj(self.pick_height)

    def get_traj_baseline(self):
        return self._create_traj(self.baseline_height)

    def _create_traj(self, height):
        joint_traj_action = FollowJointTrajectory.Goal()
        
        joint_traj_action.trajectory.joint_names = Station.joint_names
        
        joint_traj_action.trajectory.points = [JointTrajectoryPoint(
            positions=[height, self.pos_shoulder, self.pos_elbow],
            time_from_start=Duration(sec=Station.movement_duration)
        )]
        return joint_traj_action
        
class PerformerNode(Node):
    def __init__(self):
        super().__init__("performer_node")
        self.get_logger().info(f"performer_node Started")
        
        self.joint_names: list[str] = ['column1_carriage1_joint', 'carriage1_shoulder_joint', 'shoulder_elbow_joint']
        
        self.stations = [
            Station('uv', baseline_height=0.4, pick_height=0.11, pos_shoulder=pi*0.43, pos_elbow=-pi*0.60, running_time=5),
            Station('spinner', baseline_height=0.4, pick_height=0.11, pos_shoulder=2.0, pos_elbow=0.8, running_time=5),
            Station('dev', baseline_height=0.4, pick_height=0.11, pos_shoulder=-2.0, pos_elbow=-0.8, running_time=5),
        ]
        
        self.next_station_idx = 2

        self.act_client = ActionClient(self, FollowJointTrajectory , "/joint_trajectory_controller/follow_joint_trajectory")
        self.start_test_movement()
    
    
    def start_test_movement(self):
        self.wait_for_act_server()
        station = self.stations[self.next_station_idx]
        goal = station.get_traj_pick()
        
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
        
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info(f'SUCCESS - Result: {result}')
            
        else:
            self.get_logger().error(f"FAILed - Result: {result}, status: {status}")


    def create_station_timers(self):
        for key in self.station_keys:
            self.station_timers[key] = self.create_timer(self.station_timer)
        
    def wait_for_act_server(self):
        while not self.act_client.wait_for_server(2):
            self.get_logger().warning("Waiting for Action Client...")
            
    def increment_station_idx(self):
        self.next_station_idx = (self.next_station_idx + 1) % len(self.station_keys)
        

def main(args=None):
    rclpy.init(args=args)
    node = PerformerNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()