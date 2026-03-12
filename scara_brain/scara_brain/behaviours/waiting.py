from py_trees.common import Status
import rclpy
import py_trees
from rclpy.logging import RcutilsLogger
import time

class WaitingBehaviour(py_trees.behaviour.Behaviour):
    def __init__(self, name: str, duration_secs: float, logger: RcutilsLogger):
        super().__init__(name)
        
        self.duration_secs = duration_secs
        self.start_time = None
        self.logger = logger
        
    def initialise(self) -> None:
        self.start_time = time.time()
        self.logger.info(f"Starting {self.name}")
        
    def update(self) -> Status:
        curr_time = time.time()
        if self.start_time is not None and curr_time - self.start_time >= self.duration_secs:
            return Status.SUCCESS
        else:    
            return Status.RUNNING