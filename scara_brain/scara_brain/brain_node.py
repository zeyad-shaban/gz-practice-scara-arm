#!/usr/bin/env python3
from enum import Enum, auto

import rclpy
from rclpy.node import Node
from rclpy.action.client import ActionClient, ClientGoalHandle, GoalStatus
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from math import pi
from scara_brain.modules.station import Station
from scara_brain.trees import pick_and_place


class BrainNode(Node):
    def __init__(self):
        super().__init__("brain_node")
        self.get_logger().info(f"brain_node Started")
        
        self.joint_names: list[str] = ['column1_carriage1_joint', 'carriage1_shoulder_joint', 'shoulder_elbow_joint']
        
        self.stations: list[Station] = [
            Station('wafer_stack',    mid_height=0.4, pick_height=0.11, pos_shoulder=0.848230, pos_elbow=0.4523890, running_time=0),
            Station('spinner',        mid_height=0.4, pick_height=0.11, pos_shoulder=2.261947, pos_elbow=0.0000000, running_time=5),
            Station('dlp_microscope', mid_height=0.4, pick_height=0.11, pos_shoulder=2.827433, pos_elbow=0.6785840, running_time=5),
            Station('dev',            mid_height=0.4, pick_height=0.11, pos_shoulder=-2.375044, pos_elbow=0.508938, running_time=5),
            Station('inspection',     mid_height=0.4, pick_height=0.11, pos_shoulder=-1.300619, pos_elbow=1.074425, running_time=5),
        ]
        
        self.curr_station_idx = 0

        self.act_client = ActionClient(self, FollowJointTrajectory , "/joint_trajectory_controller/follow_joint_trajectory")
        self._tree = pick_and_place.create_root(self.stations, self.act_client, self.get_logger())
        
        self._tree_timer = self.create_timer(0.1, self._tree_timer_cb)
        
    def _tree_timer_cb(self):
        self._tree.tick_once()
        

def main(args=None):
    rclpy.init(args=args)
    node = BrainNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()