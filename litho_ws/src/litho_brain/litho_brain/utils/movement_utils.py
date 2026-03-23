from control_msgs.action import FollowJointTrajectory
from control_msgs.msg import JointTolerance
from builtin_interfaces.msg import Duration
from trajectory_msgs.msg import JointTrajectoryPoint


def build_goal(joint_names: list[str], positions: list[float], duration_sec: int, pos_tolerance=0.01, time_tolerance=None):
    goal = FollowJointTrajectory.Goal()
    goal.trajectory.joint_names = joint_names

    goal.trajectory.points = [JointTrajectoryPoint(
        positions=positions,
        time_from_start=Duration(sec=duration_sec)
    )]

    if pos_tolerance is not None:
        goal.goal_tolerance = [JointTolerance(name=name, position=pos_tolerance) for name in joint_names]
    if time_tolerance is not None:
        goal.goal_time_tolerance = Duration(sec=time_tolerance)

    return goal
    