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
    GO_MID = auto()
    GO_GND = auto()
    DROP = auto()
    PICK = auto()
    
class Station:
    joint_names: list[str] = ['column1_carriage1_joint', 'carriage1_shoulder_joint', 'shoulder_elbow_joint']
    movement_duration=1
    
    def __init__(self, name: str, mid_height: float, pick_height: float, pos_shoulder: float, pos_elbow: float, running_time: float):
        # traj points expected ['shoudler', 'elbow']
        self.name = name
        self.mid_height = mid_height
        self.pick_height = pick_height
        self.pos_shoulder = pos_shoulder
        self.pos_elbow = pos_elbow
        self.running_time = running_time
        
        
    def get_traj_gnd_height(self):
        return self._create_traj(self.pick_height)

    def get_traj_mid_height(self):
        return self._create_traj(self.mid_height)

    def _create_traj(self, height):
        joint_traj_action = FollowJointTrajectory.Goal()
        
        joint_traj_action.trajectory.joint_names = Station.joint_names
        
        joint_traj_action.trajectory.points = [JointTrajectoryPoint(
            positions=[height, self.pos_shoulder, self.pos_elbow],
            time_from_start=Duration(sec=Station.movement_duration)
        )]
        return joint_traj_action
        
class BrainNode(Node):
    def __init__(self):
        super().__init__("brain_node")
        self.get_logger().info(f"brain_node Started")
        
        self.joint_names: list[str] = ['column1_carriage1_joint', 'carriage1_shoulder_joint', 'shoulder_elbow_joint']
        
        self.stations = [
            Station('uv', mid_height=0.4, pick_height=0.11, pos_shoulder=pi*0.43, pos_elbow=-pi*0.60, running_time=5),
            Station('spinner', mid_height=0.4, pick_height=0.11, pos_shoulder=2.0, pos_elbow=0.8, running_time=5),
            Station('dev', mid_height=0.4, pick_height=0.11, pos_shoulder=-2.0, pos_elbow=-0.8, running_time=5),
        ]
        
        self.curr_station_idx = 0

        self.act_client = ActionClient(self, FollowJointTrajectory , "/joint_trajectory_controller/follow_joint_trajectory")
        self.state = States.MOVE_TO_STATION
        self.is_holding_wafer = False
        self.execute_current_state()
        
    def execute_current_state(self):
        if self.state == States.MOVE_TO_STATION:
            self.move_to_station(gnd_height=False)
            
        elif self.state == States.PICK:
            self.get_logger().info("Sucking with vaccum")
            self.update_current_state()
            self.execute_current_state()
        elif self.state == States.DROP:
            self.get_logger().info("Letting go of wafer")
            self.update_current_state()
            self.execute_current_state()

        elif self.state == States.GO_GND:
            self.move_to_station(gnd_height=True)
        elif self.state == States.GO_MID:
            self.move_to_station(gnd_height=False)
        elif self.state == States.IDLE:
            self.get_logger().info("OK we should wait now.... still on teh todo process")
            self.update_current_state()
            self.execute_current_state()
            
    def update_current_state(self):
        self.get_logger().info(f"Updating Current state from {self.state}")
        if self.state == States.MOVE_TO_STATION:
            self.get_logger().info("Go Down first")
            self.state = States.GO_GND

        elif self.state == States.GO_GND:
            if self.is_holding_wafer:
                self.get_logger().info("Ok we want to drop the wafer!")
                self.state = States.DROP
            else:
                self.get_logger().info("Ok we want to pick the wafer!")
                self.state = States.PICK

        elif self.state == States.PICK:
            self.is_holding_wafer = True
            self.state = States.GO_MID
            self.get_logger().info(f"We are gonna move to station {self.curr_station_idx}...")

        elif self.state == States.DROP:
            self.is_holding_wafer = False
            self.state = States.GO_MID
            self.get_logger().info(f"We are gonna IDLE")
            
        elif self.state == States.GO_MID:
            if self.is_holding_wafer:
                self.get_logger().info("We have wafer, move to next machine")
                self.increment_station_idx()
                self.state = States.MOVE_TO_STATION
            else:
                self.get_logger().info("We have no wafer, meaning the machine is working")
                self.state = States.IDLE

        elif self.state == States.IDLE:
            self.get_logger().info("WE are now idling...")
            self.state = States.GO_GND # to start the picking sequence
            # todo create timer here according to the station...
            pass

    def move_to_station(self, gnd_height: bool):
        self.wait_for_act_server()
        station = self.stations[self.curr_station_idx]
        goal = station.get_traj_gnd_height() if gnd_height else station.get_traj_mid_height()
        
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
            self.update_current_state()
            self.execute_current_state()
            
        else:
            self.get_logger().error(f"FAILed - Result: {result}, status: {status}")

    def wait_for_act_server(self):
        while not self.act_client.wait_for_server(3):
            self.get_logger().warning("Waiting for Action Client...")

    def increment_station_idx(self):
        self.curr_station_idx = (self.curr_station_idx + 1) % len(self.stations)
        

def main(args=None):
    rclpy.init(args=args)
    node = BrainNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()