package frc.robot.commands.autos;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.quixlib.swerve.QuikPlanSwervePartialTrajectoryReader;
import frc.robot.AlignmentState;
import frc.robot.AlignmentState.ReefLevel;
import frc.robot.AlignmentState.ReefStackChoice;
import frc.robot.Constants;
import frc.robot.commands.ScoreCoralCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.GripperSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import java.util.ArrayList;
import java.util.Arrays;

public class AutoCenter implements AutoCommand {
  private final QuikPlanSwervePartialTrajectoryReader centerStart =
      new QuikPlanSwervePartialTrajectoryReader("center_1_piece.csv", true);

  private final Command m_command;

  public AutoCenter(
      GripperSubsystem gripper,
      ElevatorSubsystem elevator,
      ArmSubsystem arm,
      SwerveSubsystem swerve) {

    m_command =
        new SequentialCommandGroup(
            // new FollowQuikplan(centerStart, swerve),
            new InstantCommand(
                () -> {
                  AlignmentState.getInstance().setScoringLevel(ReefLevel.FOUR);
                  elevator.setHeight(Constants.Elevator.stowHeight, false);
                  arm.setAngle(Constants.Arm.stowAngle, true);
                  gripper.setWristAngle(Constants.Gripper.stowAngle);
                }),
            new ScoreCoralCommand(ReefStackChoice.RIGHT, swerve, arm, elevator, gripper, 21));
  }

  public Command getCommand() {
    return m_command;
  }

  public Pose2d getInitialPose() {
    return centerStart.getInitialPose();
  }

  public ArrayList<QuikPlanSwervePartialTrajectoryReader> getPartialTrajectories() {
    return new ArrayList<>(Arrays.asList(centerStart));
  }
}
