import argparse
import os
import sys

import matplotlib.pyplot as plt
import numpy as np

import constants
from utils.field import Field
from utils.robot import Robot
from utils.quikplan import QuikPlan, StoppedPoseConstraint, PoseConstraint
from utils.helpers import write_to_csv


def plan(quiet):
    # Create the field
    field = Field()

    # Create the robot model
    robot = Robot()

    # Configure the optimizer
    start_pose = constants.BACK_LEFT_APPROACH_POSE
    qp = QuikPlan(
        field,
        robot,
        start_pose,
        [StoppedPoseConstraint(start_pose)],
        apply_boundaries=False,
        use_simplified_constraints=False,
    )

    waypoint2 = constants.BACK_LEFT_TO_SOURCE_INTERMEDIATE
    qp.add_waypoint(waypoint2, 10, end_constraints=[PoseConstraint(waypoint2)])

    waypoint3 = constants.TOP_RIGHT_SOURCE
    qp.add_waypoint(waypoint3, 50, end_constraints=[StoppedPoseConstraint(waypoint3)])

    waypoint4 = constants.FRONT_LEFT_RETURN_POSE
    qp.add_waypoint(waypoint4, 10, end_constraints=[PoseConstraint(waypoint4)])

    # Plan the trajectory
    traj = qp.plan()
    path_name = os.path.basename(__file__)
    file_name = os.path.splitext(path_name)[0][4:]
    write_to_csv(traj, f"{file_name}")

    # Plot
    field.plot_traj(robot, traj, f"{file_name}.png", save=True, quiet=quiet)

    if not quiet:
        # Animate
        field.anim_traj(robot, traj, f"{file_name}.gif", save_gif=False)


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--quiet", action="store_true")
    args = parser.parse_args(sys.argv[1:])
    plan(args.quiet)
