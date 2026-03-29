from litho_brain.constants import X_STAGE_NAME, Y_STAGE_NAME, STAGE_STABLE_TIMEOUT, STAGE_JOINT_NAMES, STAGE_Z_BOUNDS, Z_STAGE_NAME, N_MARKER_ROWS, N_MARKER_COLS, MARKER_HORIZ_GAP, MARKER_VERT_GAP
from rclpy.action.client import ActionClient, ClientGoalHandle, GoalStatus
from control_msgs.action import FollowJointTrajectory
from litho_brain.utils.movement_utils import build_goal
from py_trees.common import Status
from rclpy.node import Node, Subscription
import py_trees
from scipy.optimize import minimize_scalar
from geometry_msgs.msg import Vector3
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState
import numpy as np
import threading
import time

class GoToOriginBeh(py_trees.behaviour.Behaviour):
    def __init__(self, name: str, node: Node):
        super().__init__(name)
        
        self.node = node
        self._act_client = ActionClient(self.node, FollowJointTrajectory, '/joint_trajectory_controller/follow_joint_trajectory')
        
        self._orig_pos = [
            -0.020048,
            -0.020048,
            0.017362,
        ]
        
    def initialise(self) -> None:
        self._moving = True
        self._failed = False
        self._started = False
        self.node.get_logger().info(f"{self.name} initialized...")
        
    def update(self) -> Status:
        if not self._act_client.server_is_ready():
            self.node.get_logger().warning("Waiting for /joint_trajectory_controller/joint_trajectory action server...")
            return Status.RUNNING
        if not self._started:
            self._started = True
            self._go_to_origin()
            self.node.get_logger().info("WE STARTED TEH SEQUENCE")
            
        if self._failed:
            return Status.FAILURE
        if self._moving:
            return Status.RUNNING
            
        self.node.get_logger().info(f"returning success...")
        return Status.SUCCESS
        
    def _go_to_origin(self):
        goal = build_goal(STAGE_JOINT_NAMES, self._orig_pos, duration_sec=1, pos_tolerance=1e-6)
        res_fut = self._act_client.send_goal_async(goal)
        res_fut.add_done_callback(self._response_cb)
    
    def _response_cb(self, fut):
        goal_handle: ClientGoalHandle = fut.result()
        if not goal_handle.accepted:
            self.node.get_logger().error(f"{self.name} Goal rejected")
            self._moving = False
            self._failed = True
            return
            
        goal_handle.get_result_async().add_done_callback(self._result_cb)
        
    def _result_cb(self, fut):
        # result: FollowJointTrajectory.Result = fut.result().result
        status: GoalStatus = fut.result().status
        
        self.node.get_logger().info(f"{self.name} status: {status}")
        if status == GoalStatus.STATUS_SUCCEEDED:
            self._moving = False
            return
        else:
            self._moving = False
            self._failed = True
            
            

class GoToNextMarker(py_trees.behaviour.Behaviour):
    def __init__(self, name: str, node: Node, idx, precision_microns=1):
        super().__init__(name) 
        self.node = node
        self._act_client = ActionClient(self.node, FollowJointTrajectory, '/joint_trajectory_controller/follow_joint_trajectory')
        
        self._idx = idx
        self._tolerance_meters = precision_microns*1e-6
        
    def initialise(self) -> None:
        self._joints_sub = self.node.create_subscription(JointState, '/joint_states', self._pos_callback, 10)

        self._moving = False
        self._failed = False
        self._done = False
        
        self._curr_x: float|None = None
        self._curr_y: float|None = None
        self._curr_z: float|None = None
        
        if self._idx == N_MARKER_ROWS * N_MARKER_COLS - 1:
            self.node.get_logger().info(f"{self.name} Not moving as reached final marker, we are done...")
            self._done = True

    def update(self) -> Status:
        if self._failed:
            return Status.FAILURE
        if self._done:
            return Status.SUCCESS
        
        are_servers_ready = self._act_client.server_is_ready() and self._joints_sub.get_publisher_count() > 0 # todo dont' do this as we cna't tell which server is off
        if not are_servers_ready or self._curr_x is None or self._curr_y is None or self._curr_z is None:
            self.node.get_logger().warning(f"{self.name} waiting for autoalignment/correction_vec...")
            return Status.RUNNING

        if self._moving:
            return Status.RUNNING

        if not self._moving:
            self._start_moving()
            return Status.RUNNING
        
        return Status.RUNNING

    def terminate(self, new_status: Status) -> None:
        if self._joints_sub is not None:
            self.node.destroy_subscription(self._joints_sub)

    def _pos_callback(self, msg: JointState):
        idx_x = msg.name.index(X_STAGE_NAME) # type: ignore
        self._curr_x = msg.position[idx_x]
        
        idx_y = msg.name.index(Y_STAGE_NAME) # type: ignore
        self._curr_y = msg.position[idx_y]
        
        idx_z = msg.name.index(Z_STAGE_NAME) # type: ignore
        self._curr_z = msg.position[idx_z]
        
    def _start_moving(self):
        # decide if to move left or right
        horiz_offset = 0
        vert_offset = 0
        
        if (self._idx + 1) % N_MARKER_ROWS == 0:
            horiz_offset = -MARKER_HORIZ_GAP * (N_MARKER_ROWS - 1)
            vert_offset = MARKER_VERT_GAP
        else:
            vert_offset = 0
            horiz_offset = MARKER_HORIZ_GAP
        
        self.node.get_logger().info(f"{self.name} Starting moving - horiz {horiz_offset}, vert {vert_offset}")
        
        positions: list[float] = [self._curr_x + vert_offset, self._curr_y + horiz_offset, self._curr_z] # type: ignore
        
        self._moving = True
        goal = build_goal(STAGE_JOINT_NAMES, positions, duration_sec=1, pos_tolerance=self._tolerance_meters)
        
        self._act_client.send_goal_async(goal).add_done_callback(self._response_callback)
        
    def _response_callback(self, fut):
        response: ClientGoalHandle = fut.result()
        
        if not response.accepted:
            self.node.get_logger().error(f"{self.name} Goal Rejected...")
            self._failed = True
            
        response.get_result_async().add_done_callback(self._result_callback)
        
    def _result_callback(self, fut):
        status: GoalStatus = fut.result().status
        result: FollowJointTrajectory.Result = fut.result().result
        
        if status == GoalStatus.STATUS_SUCCEEDED:
            self._moving = False
            self._done = True
        else:
            self.node.get_logger().error(f"{self.name} Goal Failed...")
            self._failed = True
            self._moving = False
        
            
# todo the control system here fucken sucks, at least ad a feedback loop to update the values as we are going instead of overshooting this bdly
class AutoAlignmentBeh(py_trees.behaviour.Behaviour):
    def __init__(self, name: str, node: Node, min_thresh_micron=1):
        super().__init__(name) 
        self.node = node
        self._act_client = ActionClient(self.node, FollowJointTrajectory, '/joint_trajectory_controller/follow_joint_trajectory')
        
        self._min_thresh_meters = min_thresh_micron*1e-6
        
    def initialise(self) -> None:
        self._correction_vec_sub = self.node.create_subscription(Vector3, 'autoalignment/correction_vec', self._alignment_callback, 10)
        self._joints_sub = self.node.create_subscription(JointState, '/joint_states', self._pos_callback, 10)

        self._moving = False
        self._failed = False
        self._offset_x: float|None = None
        self._offset_y: float|None = None
        
        self._curr_x: float|None
        self._curr_y: float|None
        self._curr_z: float|None = None

    def update(self) -> Status:
        if self._failed:
            return Status.FAILURE
        
        are_servers_ready = self._correction_vec_sub.get_publisher_count() > 0 and self._act_client.server_is_ready() and self._joints_sub.get_publisher_count() > 0
        if not are_servers_ready or self._offset_x is None or self._offset_y is None or self._curr_x is None or self._curr_y is None or self._curr_z is None:
            self.node.get_logger().warning(f"{self.name} waiting for servers to start...") # todo acutlly tell the servers required
            return Status.RUNNING
        if self._moving:
            return Status.RUNNING
            
        if np.linalg.norm([self._offset_x, self._offset_y]) <= self._min_thresh_meters:
            return Status.SUCCESS

        if not self._moving:
            self._start_moving()
            return Status.RUNNING

        return Status.RUNNING
        
    def terminate(self, new_status: Status) -> None:
        if self._correction_vec_sub is not None:
            self.node.destroy_subscription(self._correction_vec_sub)
        if self._joints_sub is not None:
            self.node.destroy_subscription(self._joints_sub)
        
    def _pos_callback(self, msg: JointState):
        idx_x = msg.name.index(X_STAGE_NAME) # type: ignore
        self._curr_x = msg.position[idx_x]
        
        idx_y = msg.name.index(Y_STAGE_NAME) # type: ignore
        self._curr_y = msg.position[idx_y]
        
        idx_z = msg.name.index(Z_STAGE_NAME) # type: ignore
        self._curr_z = msg.position[idx_z]
        
    def _alignment_callback(self, msg: Vector3):
        self._offset_x = msg.x
        self._offset_y = msg.y
        
    def _start_moving(self):
        self.node.get_logger().info(f"{self.name} Starting moving x {self._offset_x:.6f} y {self._offset_y:.6f}")
        self._moving = True
        positions: list[float] = [self._curr_x + self._offset_x, self._curr_y + self._offset_y, self._curr_z] # type: ignore
        goal = build_goal(STAGE_JOINT_NAMES, positions, duration_sec=1, pos_tolerance=self._min_thresh_meters/2)
        
        self._act_client.send_goal_async(goal).add_done_callback(self._response_callback)
        
    def _response_callback(self, fut):
        response: ClientGoalHandle = fut.result()
        
        if not response.accepted:
            self.node.get_logger().error(f"{self.name} Goal Rejected...")
            self._failed = True
            
        response.get_result_async().add_done_callback(self._result_callback)
        
    def _result_callback(self, fut):
        status: GoalStatus = fut.result().status
        result: FollowJointTrajectory.Result = fut.result().result
        
        if status == GoalStatus.STATUS_SUCCEEDED:
            self._moving = False
        else:
            self.node.get_logger().error(f"{self.name} Goal Failed...")
            self._failed = True
            self._moving = False
        
        