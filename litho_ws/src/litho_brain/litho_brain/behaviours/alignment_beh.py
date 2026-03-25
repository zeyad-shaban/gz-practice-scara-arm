from litho_brain.constants import X_STAGE_NAME, Y_STAGE_NAME
from rclpy.action.client import ActionClient, ClientGoalHandle
from control_msgs.action import FollowJointTrajectory
from litho_brain.utils.movement_utils import build_goal
from py_trees.common import Status
import rclpy
from rclpy.node import Node
import py_trees
from scipy.optimize import minimize_scalar
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState
import threading
from litho_brain.constants import STAGE_JOINT_NAMES, STAGE_Z_BOUNDS
import time

class AutoFocusBeh(py_trees.behaviour.Behaviour):
    def __init__(self, name: str, node: Node, min_thresh_micron=100, _max_iter=60):
        super().__init__(name)
        self.node = node
        
        self._tolerance_meters = float(min_thresh_micron * 1e-6)
        self._max_iter = _max_iter
        
        self._sharpness_ready = threading.Event()
        self._goal_done = threading.Event()
        
        self._thread_started = False
        self._maximizer_done = False
        
        self._curr_pos_x: float = None  # type: ignore
        self._curr_pos_y: float = None  # type: ignore
        self._last_sharpness: float = None  # type: ignore
        
        self._maximizer_thread: threading.Thread = None # type: ignore
        self._sharpness_sub = None
        self._joint_sub = None
        self._act_client: ActionClient = None # type: ignore
        
    def initialise(self) -> None:
        # subscribers..
        self._sharpness_sub = self.node.create_subscription(Float64, 'autofocus/sharpness', self._sharpness_cb, 10)
        self._joint_sub = self.node.create_subscription(JointState, '/joint_states', self._joint_cb, 10)
        self._act_client = ActionClient(self.node, FollowJointTrajectory, '/joint_trajectory_controller/follow_joint_trajectory')
    
        # Maximizer thread
        self._maximizer_thread = threading.Thread(target=self._maximizer_worker)
        
        self.node.get_logger().info(f"{self.name} initalized...")

    def update(self) -> Status:
        # waiting for stuff to be ready
        if not self._act_client.is_ready:
            self.node.get_logger().warning("Waiting for /joint_trajectory_controller/joint_trajectory action server...")
            return Status.RUNNING
        if self._curr_pos_x is None or self._curr_pos_y is None:
            self.node.get_logger().warning("Waiting for /joint_states publisher...")
            return Status.RUNNING
        if self._last_sharpness is None:
            self.node.get_logger().warning("Waiting for autofocus/sharpness publisher...")
            return Status.RUNNING


        if not self._thread_started:
            self._maximizer_thread.start()
            self._thread_started = True
            return Status.RUNNING
        if not self._maximizer_done:
            return Status.RUNNING
        else:
            return Status.SUCCESS
            
    def terminate(self, new_status: Status) -> None:
        if self._sharpness_sub is not None:
            self.node.destroy_subscription(self._sharpness_sub)
            self._sharpness_sub = None
        if self._joint_sub is not None:
            self.node.destroy_subscription(self._joint_sub)
            self._joint_sub = None
        # if self._act_client is not None:
        #     self._act_client.destroy()

        self.node.get_logger().info(f"termianted {self.name} subscriptions")
        
    def _sharpness_cb(self, msg: Float64):
        self._last_sharpness = msg.data
        self._sharpness_ready.set() # why not just sit it always?
    
    def _joint_cb(self, msg: JointState):
        x_idx = msg.name.index(X_STAGE_NAME) # type: ignore
        self._curr_pos_x = msg.position[x_idx]
        
        y_idx = msg.name.index(Y_STAGE_NAME) # type: ignore
        self._curr_pos_y = msg.position[y_idx]
        
    def _maximizer_worker(self):
        result = minimize_scalar(
            self._sharpness_at_z,
            bounds=(-STAGE_Z_BOUNDS, STAGE_Z_BOUNDS),
            method='bounded',
            options={'maxiter': self._max_iter, 'xatol': self._tolerance_meters}
        )
        
        print(result)
        self._maximizer_done = True
        
    
    def _sharpness_at_z(self, z):
        self._goal_done.clear()
        positions = [self._curr_pos_x, self._curr_pos_y, z]
        goal = build_goal(STAGE_JOINT_NAMES, positions, duration_sec=1, pos_tolerance=self._tolerance_meters)
        
        # wait Blocking Till we reach
        fut = self._act_client.send_goal_async(goal)
        fut.add_done_callback(self._goal_response_cb)
        
        self._goal_done.wait(timeout=10.0)
        self._sharpness_ready.clear()
        time.sleep(2.0)
        self._sharpness_ready.wait(timeout=3.0)
        
        self.node.get_logger().info(f"{self.name} Brent's iter at z_pos: {z:.7f}, sharpness: {self._last_sharpness:.2f}")
        return -self._last_sharpness
        
    def _goal_response_cb(self, fut):
        goal_handle: ClientGoalHandle = fut.result()
        result_fut = goal_handle.get_result_async()
        result_fut.add_done_callback(self._goal_result_cb)
        
    def _goal_result_cb(self, fut):
        self._goal_done.set()


