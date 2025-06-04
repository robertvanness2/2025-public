import argparse
import sys

import matplotlib.pyplot as plt
import numpy as np

from constants import FIELD_CENTER_START_POSE, BACK_CENTER_APPROACH_POSE
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
    start_pose = FIELD_CENTER_START_POSE
    qp = QuikPlan(
        field,
        robot,
        start_pose,
        [StoppedPoseConstraint(start_pose)],
        apply_boundaries=False,
        use_simplified_constraints=False,
    )

    waypoint2 = BACK_CENTER_APPROACH_POSE
    qp.add_waypoint(waypoint2, 50, end_constraints=[StoppedPoseConstraint(waypoint2)])

    # Plan the trajectory
    traj = qp.plan()
    write_to_csv(traj, "center_1_piece")

    # Plot
    field.plot_traj(robot, traj, "center_1_piece.png", save=True, quiet=quiet)

    if not quiet:
        # Animate
        field.anim_traj(robot, traj, "center_1_piece.gif", save_gif=False)


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--quiet", action="store_true")
    args = parser.parse_args(sys.argv[1:])
    plan(args.quiet)
