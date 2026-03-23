from py_trees.common import Status
import rclpy
from rclpy.node import Node
import py_trees
from scipy.optimize import minimize_scalar
from std_msgs.msg import Float64
import threading

class AutoFocusBeh(py_trees.behaviour.Behaviour):
    def __init__(self, name: str, node: Node, min_thresh_micron=100, _max_iter=10):
        super().__init__(name)
        self.node = node
        
        self._min_thresh_micron = min_thresh_micron
        self._max_iter = _max_iter
        
        self._maximizer_done = False # i think i need to impelment some atomic protection here?
        self._last_sharpness = -1 # i think i need to implement atomic protection here??
        
        self._sharpness_sub = None
        
    def initialise(self) -> None:
        # subscribers..
        self._sharpness_sub = self.node.create_subscription(Float64, 'autofocus/sharpness', self._sharpness_cb, 10)
        
        while self._sharpness_sub.get_publisher_count() <= 0:
            self.node.get_logger().warning("Waiting for joint_states publisher...")
    
        # Maximizer thread
        _maximizer_thread = threading.Thread(target=self._maximizer_worker)
        _maximizer_thread.start()
        
        self.node.get_logger().info(f"{self.name} initalized...")

    def update(self) -> Status:
        if not self._maximizer_done:
            return Status.RUNNING
        else:
            return Status.SUCCESS
            
    def terminate(self, new_status: Status) -> None:
        # should we remove teh subs here or does ros2 handle it smoehow internally, or do we claenaup here or in the update function or where?
        self.node.get_logger().warning("TODO TERMINATION CLEANUP OF REMOVING THE SUBSCRIBERS NOT IMPELMENTED YET...")
        
    def _sharpness_cb(self, msg: Float64):
        self._last_sharpness = msg.data
    
    def _sharpness_at_z(self, z):
        # send trajectory to go to z
        # wait Blocking Till we reach
        return -self._last_sharpness

    def _maximizer_worker(self):
        result = minimize_scalar(
            self._sharpness_at_z,
            bounds=(-0.025, 0.025),
            method='bounded',
            options={'maxiter': self._max_iter, 'xatol': self._min_thresh_micron * 10^-6}
        )
        
        print(result)
        self._maximizer_done = True
        