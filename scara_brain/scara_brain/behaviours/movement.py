import py_trees
from py_trees.common import Status
from rclpy.action.client import ActionClient, GoalStatus, ClientGoalHandle
from rclpy.logging import RcutilsLogger
from scara_brain.modules.station import Station
from rclpy.node import Node
from geometry_msgs.msg import Vector3
import numpy as np
from scara_brain.utils.arm_movement import compute_joint_corrections, get_joint_pos
from sensor_msgs.msg import JointState

from control_msgs.action import FollowJointTrajectory
from control_msgs.msg import JointTolerance
from trajectory_msgs.msg import JointTrajectoryPoint
from builtin_interfaces.msg import Duration



# todo put those in better po
SHOULDER_JOINT_NAME =  'carriage1_shoulder_joint'
ELBOW_JOINT_NAME =  'shoulder_elbow_joint'
CARRAIGE_JOINT_NAME = 'column1_carriage1_joint'
MOVEMENT_DURATION = 1


class MoveToStation(py_trees.behaviour.Behaviour):
    def __init__(self, name: str, station: Station, is_mid: bool, act_client: ActionClient, logger: RcutilsLogger):
        super().__init__(name)
        self._result = None
        self.station = station
        self.is_mid = is_mid
        self.act_client = act_client
        self.logger = logger
    
    def initialise(self) -> None:
        self._wait_for_act_server()
        goal = self.station.get_traj_mid_height() if self.is_mid else self.station.get_traj_gnd_height()
        
        fut = self.act_client.send_goal_async(goal)
        fut.add_done_callback(self._goal_response_cb)
        
        self.logger.info(f"Starting {self.name}")
        
    def update(self) -> Status:
        if self._result is None:
            return Status.RUNNING
        
        if self._result.status == GoalStatus.STATUS_SUCCEEDED:
            return Status.SUCCESS
        else: # Aborted, or cancelled
            return Status.FAILURE
    
    def terminate(self, new_status: Status) -> None:
        # idk what am i suppsoed to do here
        return super().terminate(new_status)
        
    def _goal_response_cb(self, fut):
        goal_handle: ClientGoalHandle = fut.result()
        
        if not goal_handle.accepted:
            self.logger.info("Goal rejected...") # todo make this a failure
            return
        
        result_fut = goal_handle.get_result_async()
        result_fut.add_done_callback(self._result_cb)
        
    def _result_cb(self, fut):
        self._result = fut.result()
        
    
    def _wait_for_act_server(self):
        while not self.act_client.wait_for_server(1):
            self.logger.info("Waiting for action client...")
            
class GoDown(py_trees.behaviour.Behaviour):
    def __init__(self, name: str, node: Node, station: Station, act_client):
        super().__init__(name)
        
        self.node = node
        self._act_client = act_client
        self._station = station
        
        self._result = None
        self._goal_sent = False
    
    def initialise(self) -> None:
        self.node.get_logger().info(f"Starting {self.name}...")
        self.joint_sub = self.node.create_subscription(JointState, '/joint_states', self._joint_states_callback, 10)
    
    def update(self):
        if self._result is None:
            return Status.RUNNING
            
        return Status.SUCCESS
            
        
    def _joint_states_callback(self, msg: JointState):
        if self._goal_sent:
            return
        self._goal_sent = True
        
        shoulder_pos = get_joint_pos(msg,SHOULDER_JOINT_NAME)
        elbow_pos = get_joint_pos(msg, ELBOW_JOINT_NAME)
        
        self._wait_for_act_server()
        
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = [CARRAIGE_JOINT_NAME, SHOULDER_JOINT_NAME, ELBOW_JOINT_NAME]
        
        goal.trajectory.points = [JointTrajectoryPoint(
            positions=[self._station.pick_height, shoulder_pos, elbow_pos],
            time_from_start=Duration(sec=MOVEMENT_DURATION)
        )]
        
        goal.goal_tolerance = [JointTolerance(name=name, position=0.01) for name in Station.joint_names]
        # goal.goal_time_tolerance = Duration(sec=2)
        
        fut = self._act_client.send_goal_async(goal)
        fut.add_done_callback(self._goal_response_cb)
        
        self.node.get_logger().info(f"Starting alignment movement")
        
    def _goal_response_cb(self, fut):
        goal_handle: ClientGoalHandle = fut.result()
        
        if not goal_handle.accepted:
            self.node.get_logger().warning("Goal rejected...") # todo make this a failure
            return
        
        result_fut = goal_handle.get_result_async()
        result_fut.add_done_callback(self._result_cb)
        
    def _result_cb(self, fut):
        self._result = fut.result()
        
        
        
    def _wait_for_act_server(self):
        while not self._act_client.wait_for_server(1):
            self.logger.info("Waiting for action client...")
        
        
class GoUp(py_trees.behaviour.Behaviour):
    def __init__(self, name: str, node: Node, station: Station, act_client):
        super().__init__(name)
        
        self.node = node
        self._act_client = act_client
        self._station = station
        
        self._result = None
        self._goal_sent = False
    
    def initialise(self) -> None:
        self.node.get_logger().info(f"Starting {self.name}...")
        self.joint_sub = self.node.create_subscription(JointState, '/joint_states', self._joint_states_callback, 10)
    
    def update(self):
        if self._result is None:
            return Status.RUNNING
            
        return Status.SUCCESS
            
    def _joint_states_callback(self, msg: JointState):
        if self._goal_sent:
            return
        self._goal_sent = True
        
        shoulder_pos = get_joint_pos(msg,SHOULDER_JOINT_NAME)
        elbow_pos = get_joint_pos(msg, ELBOW_JOINT_NAME)
        
        self._wait_for_act_server()
        
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = [CARRAIGE_JOINT_NAME, SHOULDER_JOINT_NAME, ELBOW_JOINT_NAME]
        
        goal.trajectory.points = [JointTrajectoryPoint(
            positions=[self._station.mid_height, shoulder_pos, elbow_pos],
            time_from_start=Duration(sec=MOVEMENT_DURATION)
        )]
        
        goal.goal_tolerance = [JointTolerance(name=name, position=0.01) for name in Station.joint_names]
        # goal.goal_time_tolerance = Duration(sec=2)
        
        fut = self._act_client.send_goal_async(goal)
        fut.add_done_callback(self._goal_response_cb)
        
        self.node.get_logger().info(f"Starting alignment movement")
        
    def _goal_response_cb(self, fut):
        goal_handle: ClientGoalHandle = fut.result()
        
        if not goal_handle.accepted:
            self.node.get_logger().warning("Goal rejected...") # todo make this a failure
            return
        
        result_fut = goal_handle.get_result_async()
        result_fut.add_done_callback(self._result_cb)
        
    def _result_cb(self, fut):
        self._result = fut.result()
        
        
        
    def _wait_for_act_server(self):
        while not self._act_client.wait_for_server(1):
            self.logger.info("Waiting for action client...")
        
        
            
class AlignmentBehaviour(py_trees.behaviour.Behaviour):
    def __init__(self, name, node: Node, act_client, allowed_thresh_meters=0.01):
        super().__init__(name)
        
        self.node = node
        self._act_client = act_client
        self.align_sub = None
        self.joint_sub = None
        
        self._is_moving = False
        self._err = float('inf')
        self._allowed_thresh_meters = allowed_thresh_meters
        
        self._shoulder_pos = None
        self._elbow_pos = None
        self._dx = None
        self._dy = None
        self._z_pos = None
        
    def initialise(self) -> None:
        self.node.get_logger().info(f"Starting {self.name}...")
        self.align_sub = self.node.create_subscription(Vector3, 'wafer/align_vec', self._align_vec_callback, 10)
        self.joint_sub = self.node.create_subscription(JointState, '/joint_states', self._joint_states_callback, 10)
        
    def update(self) -> Status:
        if self._is_moving:
            return Status.RUNNING
        if self._err < self._allowed_thresh_meters:
            return Status.SUCCESS
            
        if None in [self._shoulder_pos, self._elbow_pos, self._dx, self._dy, self._z_pos]:
            return Status.RUNNING

        self._err = np.hypot(self._dx, self._dy) # type: ignore
        new_angles = compute_joint_corrections(self._shoulder_pos, self._elbow_pos, self._dx, self._dy)
        self._send_goal(new_angles)
        self._is_moving = True
        
        return Status.RUNNING
        
    def _joint_states_callback(self, msg: JointState):
        self._shoulder_pos = get_joint_pos(msg,SHOULDER_JOINT_NAME)
        self._elbow_pos = get_joint_pos(msg, ELBOW_JOINT_NAME)
        self._z_pos = get_joint_pos(msg, CARRAIGE_JOINT_NAME)
        
    def _align_vec_callback(self, msg: Vector3):
        self._dx, self._dy = msg.x, msg.y
        
    def _send_goal(self, d_angles: np.ndarray):
        d_shoulder, d_elbow = d_angles
        new_shoulder = self._shoulder_pos + d_shoulder
        new_elbow = self._elbow_pos + d_elbow
        
        self._wait_for_act_server()
        
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = [CARRAIGE_JOINT_NAME, SHOULDER_JOINT_NAME, ELBOW_JOINT_NAME]
        
        goal.trajectory.points = [JointTrajectoryPoint(
            positions=[self._z_pos, new_shoulder, new_elbow],
            time_from_start=Duration(sec=MOVEMENT_DURATION)
        )]
        
        goal.goal_tolerance = [JointTolerance(name=name, position=0.01) for name in Station.joint_names]
        # goal.goal_time_tolerance = Duration(sec=2)
        
        fut = self._act_client.send_goal_async(goal)
        fut.add_done_callback(self._goal_response_cb)
        
        self.node.get_logger().info(f"Starting alignment movement")
        
    def _goal_response_cb(self, fut):
        goal_handle: ClientGoalHandle = fut.result()
        
        if not goal_handle.accepted:
            self.node.get_logger().warning("Goal rejected...") # todo make this a failure
            return
        
        result_fut = goal_handle.get_result_async()
        result_fut.add_done_callback(self._result_cb)
        
    def _result_cb(self, fut):
        result, status = fut.result().result, fut.result().status
        
        if status == GoalStatus.STATUS_SUCCEEDED:
            self._is_moving = False
        
    def _wait_for_act_server(self):
        while not self._act_client.wait_for_server(1):
            self.logger.info("Waiting for action client...")