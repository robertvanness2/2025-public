package frc.robot.commands.autos;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.quixlib.swerve.QuikPlanSwervePartialTrajectoryReader;
import frc.robot.commands.FollowQuikplan;
import frc.robot.subsystems.SwerveSubsystem;
import java.util.ArrayList;
import java.util.Arrays;

public class AutoSimpleTest implements AutoCommand {
  private final QuikPlanSwervePartialTrajectoryReader m_traj =
      new QuikPlanSwervePartialTrajectoryReader("simple_test.csv");

  private final Command m_command;

  public AutoSimpleTest(final SwerveSubsystem swerve) {
    m_command = new FollowQuikplan(m_traj, swerve, 1.0);
  }

  public Command getCommand() {
    return m_command;
  }

  public Pose2d getInitialPose() {
    return m_traj.getInitialPose();
  }

  public ArrayList<QuikPlanSwervePartialTrajectoryReader> getPartialTrajectories() {
    return new ArrayList<>(Arrays.asList(m_traj));
  }
}
