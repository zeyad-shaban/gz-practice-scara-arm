import py_trees
from rclpy.action.client import ActionClient
from rclpy.logging import RcutilsLogger
from scara_brain.modules.station import Station
from py_trees.composites import Sequence, Selector
from scara_brain.behaviours.movement import MoveToStation
from scara_brain.behaviours.waiting import WaitingBehaviour

def create_root(stations: list[Station], act_client: ActionClient, logger: RcutilsLogger):
    root = Sequence(name="Pick And Place", memory=True)
    
    for i in range(1, len(stations)):
        prev_station = stations[i-1]
        station = stations[i]
        
        root.add_child(MoveToStation(f"Go Prev {station.name} mid To Pick", prev_station, True, act_client, logger))
        # later on: ai here to auto align
        
        root.add_child(MoveToStation(f"Go Prev {station.name} Gnd To Pick", prev_station, False, act_client, logger))
        # later on: suck
        
        root.add_child(MoveToStation(f"Go Prev {station.name} Mid Picked", prev_station, True, act_client, logger))
        
        root.add_child(MoveToStation(f"Go This {station.name} Mid To Drop", station, True, act_client, logger))
        root.add_child(MoveToStation(f"Go This {station.name} Gnd To Drop", station, False, act_client, logger))
        # later on: drop
        
        root.add_child(MoveToStation(f"Go This {station.name} Mid Dropped", station, True, act_client, logger))
        root.add_child(WaitingBehaviour(f"Waiting {station.name} [{station.running_time} secs]", station.running_time, logger))
        
    return root