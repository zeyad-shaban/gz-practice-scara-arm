import py_trees
from rclpy.action.client import ActionClient
from rclpy.logging import RcutilsLogger
from scara_brain.modules.station import Station
from py_trees.composites import Sequence, Selector
from scara_brain.behaviours.movement import AlignmentBehaviour, GoDown, GoUp, MoveToStation
from scara_brain.behaviours.waiting import WaitingBehaviour
from rclpy.node import Node
from scara_brain.behaviours.manipulation import VacuumOn, VacuumOff

def create_root(stations: list[Station], act_client: ActionClient, node: Node):
    logger = node.get_logger()
    root = Sequence(name="Pick And Place", memory=True)
    
    for i in range(1, len(stations)):
        prev_station = stations[i-1]
        station = stations[i]
        
        root.add_child(MoveToStation(f"Go Prev {station.name} mid To Pick", prev_station, True, act_client, logger))
        root.add_child(AlignmentBehaviour(f"Align in {station.name}", node, act_client))
        root.add_child(GoDown(f"Go Prev {station.name} Gnd To Pick", node, prev_station, act_client))
        
        root.add_child(VacuumOn(f"Vacuum On At {station.name}", node))
        root.add_child(GoUp(f"Go Prev {station.name} Mid Picked", node, prev_station, act_client))
        
        
        root.add_child(MoveToStation(f"Go This {station.name} Mid To Drop", station, True, act_client, logger))
        root.add_child(MoveToStation(f"Go This {station.name} Gnd To Drop", station, False, act_client, logger))
        root.add_child(VacuumOff(f"Vacuum Off At {station.name}", node))
        
        root.add_child(MoveToStation(f"Go This {station.name} Mid Dropped", station, True, act_client, logger))
        root.add_child(WaitingBehaviour(f"Waiting {station.name} [{station.running_time} secs]", station.running_time, logger))
        
    return root