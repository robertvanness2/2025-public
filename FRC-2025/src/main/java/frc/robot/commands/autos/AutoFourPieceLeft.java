package frc.robot.commands.autos;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.quixlib.swerve.QuikPlanSwervePartialTrajectoryReader;
import frc.robot.AlignmentState;
import frc.robot.AlignmentState.ReefLevel;
import frc.robot.AlignmentState.ReefStackChoice;
import frc.robot.Constants;
import frc.robot.commands.FollowQuikplan;
import frc.robot.commands.SafelyStowAndIntakeCommand;
import frc.robot.commands.ScoreCoralCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.GripperSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import java.util.ArrayList;
import java.util.Arrays;

public class AutoFourPieceLeft implements AutoCommand {
  private final QuikPlanSwervePartialTrajectoryReader topStartBackLeft =
      new QuikPlanSwervePartialTrajectoryReader("left_start_to_back.csv");
  private final QuikPlanSwervePartialTrajectoryReader backLeftToSourceRetFL =
      new QuikPlanSwervePartialTrajectoryReader("BL_S_RET_FL.csv");
  private final QuikPlanSwervePartialTrajectoryReader frontLeftToSourceRetFL =
      new QuikPlanSwervePartialTrajectoryReader("FL_S_RET_FL.csv");
  private final QuikPlanSwervePartialTrajectoryReader frontLeftToSourceRetBL =
      new QuikPlanSwervePartialTrajectoryReader("FL_S_RET_BL.csv");

  private final Command m_command;

  public AutoFourPieceLeft(
      GripperSubsystem gripper,
      ElevatorSubsystem elevator,
      ArmSubsystem arm,
      SwerveSubsystem swerve) {

    m_command =
        new SequentialCommandGroup(
            // new FollowQuikplan(topStartBackLeft, swerve),
            new InstantCommand(
                () -> {
                  AlignmentState.getInstance().setScoringLevel(ReefLevel.FOUR);
                  elevator.setHeight(Constants.Elevator.stowHeight, false);
                  arm.setAngle(Constants.Arm.stowAngle, true);
                  gripper.setWristAngle(Constants.Gripper.stowAngle);
                }),
            new ScoreCoralCommand(ReefStackChoice.LEFT, swerve, arm, elevator, gripper, 20),
            new ParallelDeadlineGroup(
                new FollowQuikplan(backLeftToSourceRetFL, swerve),
                new SafelyStowAndIntakeCommand(
                    elevator, arm, gripper, () -> swerve.getSingleTagPose())),
            new ScoreCoralCommand(ReefStackChoice.RIGHT, swerve, arm, elevator, gripper, 19),
            new ParallelDeadlineGroup(
                new FollowQuikplan(frontLeftToSourceRetFL, swerve),
                new SafelyStowAndIntakeCommand(
                    elevator, arm, gripper, () -> swerve.getSingleTagPose())),
            new ScoreCoralCommand(ReefStackChoice.LEFT, swerve, arm, elevator, gripper, 19),
            new ParallelDeadlineGroup(
                new FollowQuikplan(frontLeftToSourceRetBL, swerve),
                new SafelyStowAndIntakeCommand(
                    elevator, arm, gripper, () -> swerve.getSingleTagPose())),
            new ScoreCoralCommand(ReefStackChoice.RIGHT, swerve, arm, elevator, gripper, 20));
  }

  public Command getCommand() {
    return m_command;
  }

  public Pose2d getInitialPose() {
    return topStartBackLeft.getInitialPose();
  }

  public ArrayList<QuikPlanSwervePartialTrajectoryReader> getPartialTrajectories() {
    return new ArrayList<>(
        Arrays.asList(
            topStartBackLeft,
            backLeftToSourceRetFL,
            frontLeftToSourceRetFL,
            frontLeftToSourceRetBL));
  }
}
