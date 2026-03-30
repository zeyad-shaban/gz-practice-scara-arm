from litho_brain.behaviours.movement_beh import GoToOriginBeh, AutoAlignmentBeh, GoToNextMarker
from litho_brain.behaviours.focus_beh import AutoFocusBeh
from litho_brain.nodes.stage_wait import StageSettledBeh
import py_trees
from py_trees.composites import Sequence, Selector, Parallel
import rclpy
from rclpy.node import Node


def get_root(node: Node):
    root = Sequence(name='lol', memory=True)
    root.add_child(GoToOriginBeh('GoToOrigin', node))
    
    root.add_child(AutoFocusBeh('Auto focus', node))
    root.add_child(StageSettledBeh('WaitFocusStable', node))
    dies = [{}]*25
    
    for i, die in enumerate(dies):
        root.add_child(AutoAlignmentBeh('Auto Alignment', node))
        root.add_child(StageSettledBeh('WaitMovementStable', node))
        # root.add_child(DLPExpose)
        root.add_child(GoToNextMarker(f'GoToMarker {i}', node, i))
    return root