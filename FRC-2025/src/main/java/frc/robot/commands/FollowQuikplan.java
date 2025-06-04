package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.quixlib.swerve.QuikPlanSwervePartialTrajectoryReader;
import frc.quixlib.swerve.QuikPlanSwervePartialTrajectoryReader.QuikPlanAction;
import frc.quixlib.swerve.QuikPlanSwervePartialTrajectoryReader.QuikplanTrajectoryState;
import frc.robot.Constants;
import frc.robot.subsystems.SwerveSubsystem;
import java.util.ArrayList;

public class FollowQuikplan extends Command {
  private final boolean kDebugPrint = false;

  private final QuikPlanSwervePartialTrajectoryReader m_reader;
  private final SwerveSubsystem m_swerve;

  private final Timer m_timer = new Timer();
  private final double m_endTimeExtension;

  private ArrayList<Double> m_times = new ArrayList<>();
  private ArrayList<Double> m_xErrors = new ArrayList<>();
  private ArrayList<Double> m_yErrors = new ArrayList<>();
  private ArrayList<Double> m_thetaErrors = new ArrayList<>();

  public FollowQuikplan(QuikPlanSwervePartialTrajectoryReader reader, SwerveSubsystem swerve) {
    this(reader, swerve, 0.0);
  }

  public FollowQuikplan(
      QuikPlanSwervePartialTrajectoryReader reader,
      SwerveSubsystem swerve,
      double endTimeExtension) {
    m_reader = reader;
    m_swerve = swerve;
    m_endTimeExtension = endTimeExtension;

    addRequirements(swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_timer.start();
    m_timer.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    final double curTime = m_timer.get();
    final QuikplanTrajectoryState targetState = m_reader.getState(curTime);
    final var actionEntry = m_reader.getAction(curTime);
    final QuikPlanAction action = actionEntry == null ? null : actionEntry.getValue();

    if (actionEntry != null) {
      switch (action.actionType) {
          // TODO: Update for 2025
        default:
          break;
      }
    }

    final Pose2d poseError =
        m_swerve.driveToPose(
            targetState.pose,
            targetState.xVel,
            targetState.yVel,
            targetState.thetaVel,
            Constants.Swerve.autoScrubLimit);
    if (kDebugPrint) {
      m_times.add(m_timer.get());
      m_xErrors.add(poseError.getX());
      m_yErrors.add(poseError.getY());
      m_thetaErrors.add(poseError.getRotation().getRadians());
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_timer.stop();
    m_swerve.stop();
    if (kDebugPrint) {
      for (int i = 0; i < m_times.size(); i++) {
        System.out.println(
            m_times.get(i)
                + ","
                + m_xErrors.get(i)
                + ","
                + m_yErrors.get(i)
                + ","
                + m_thetaErrors.get(i));
      }
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_timer.hasElapsed(m_reader.getTotalTime() + m_endTimeExtension);
  }
}
