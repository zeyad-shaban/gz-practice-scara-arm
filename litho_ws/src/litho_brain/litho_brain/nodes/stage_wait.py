import py_trees
from py_trees.common import Status
from sensor_msgs.msg import JointState
from rclpy.node import Node
import time
import numpy as np


class StageSettledBeh(py_trees.behaviour.Behaviour):
    """
    Waits until stage velocity ~ 0 for a duration.
    """

    def __init__(self, name: str, node: Node, timeout=0.1, vel_thresh=1e-6):
        super().__init__(name)
        self.node = node
        self.timeout = timeout
        self.vel_thresh = vel_thresh

    def initialise(self):
        self._last_positions = None
        self._last_time = None
        self._stable_start = None

        self._sub = self.node.create_subscription(
            JointState, "/joint_states", self._cb, 10
        )
        
        self.node.get_logger().info(f"{self.name} Initialized waiting for stabilization...")

    def update(self):
        if self._stable_start is None:
            return Status.RUNNING

        if time.time() - self._stable_start >= self.timeout:
            return Status.SUCCESS

        return Status.RUNNING

    def terminate(self, new_status):
        self.node.destroy_subscription(self._sub)

    def _cb(self, msg: JointState):
        now = time.time()
        positions = np.array(msg.position)

        if self._last_positions is None or self._last_time is None:
            self._last_positions = positions
            self._last_time = now
            return

        dt = now - self._last_time
        vel = np.linalg.norm((positions - self._last_positions) / max(dt, 1e-6))

        if vel < self.vel_thresh:
            if self._stable_start is None:
                self._stable_start = now
        else:
            self._stable_start = None

        self._last_positions = positions
        self._last_time = now