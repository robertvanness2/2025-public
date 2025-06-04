// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.quixlib.planning.QuixTrapezoidProfile.Constraints;
import frc.quixlib.planning.SimpleSwerveTrajectory;
import frc.quixlib.planning.SwerveTrajectoryState;
import frc.robot.AlignmentUtilities;
import frc.robot.Constants;
import frc.robot.Fiducials;
import frc.robot.MapleSimUtils;
import frc.robot.Robot;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.GripperSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import java.util.ArrayList;
import java.util.List;
import org.littletonrobotics.junction.Logger;

public class IntakeOrScoreAlgaeCommand extends Command {
  private final SwerveSubsystem m_swerve;
  private final ArmSubsystem m_arm;
  private final ElevatorSubsystem m_elevator;
  private final GripperSubsystem m_gripper;

  private ScoringState m_currentState;

  private Pose2d m_targetPose;
  private SimpleSwerveTrajectory m_trajectory;
  private final Timer m_trajectoryTimer = new Timer();
  private final Timer m_onTargetTimer = new Timer();
  private boolean m_isHigh;
  private double m_driveTime;
  boolean isNet;
  private boolean m_isBlue = false;

  private enum ScoringState {
    DRIVE_TO_REEF,
    RAISE_FOR_ALGAE,
    INTAKE_ALGAE,
    BACK_UP_WITH_ALGAE,
    ALIGN_TO_NET,
    START_SCORE_IN_NET,
    SCORE_IN_NET,
    START_SCORE_IN_PROCESSOR,
    SCORE_IN_PROCESSOR,
    FINISHED,
  }

  private static final List<Integer> highTagIDs = new ArrayList<>(List.of(9, 7, 11, 18, 20, 22));

  private enum DriveMode {
    TO_POSE,
    TO_X_VALUE,
    BACKUP,
    FORWARD,
    STOP,
  }

  public IntakeOrScoreAlgaeCommand(
      SwerveSubsystem swerve,
      ArmSubsystem arm,
      ElevatorSubsystem elevator,
      GripperSubsystem gripper,
      boolean net) {
    m_swerve = swerve;
    m_arm = arm;
    m_elevator = elevator;
    m_gripper = gripper;
    isNet = net;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(swerve, arm, elevator, gripper);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_isBlue =
        DriverStation.getAlliance().isPresent()
            && DriverStation.getAlliance().get() == Alliance.Blue;

    if (!m_gripper.hasAlgae) {
      final Pose2d futurePose =
          m_swerve.getSingleTagPose().transformBy(m_swerve.getVelocity().times(0.15));
      final int closestTagID = AlignmentUtilities.determineClosestTagID(futurePose, m_isBlue);
      Pose2d closestTagPose = Fiducials.aprilTagFiducials[closestTagID - 1].getPose().toPose2d();
      m_isHigh = highTagIDs.contains(closestTagID);
      if (m_isHigh
          && m_arm.isArmAtAngle(Constants.Arm.levelFourAngle, Units.degreesToRadians(3.0))) {
        m_driveTime = 0.4;
        m_targetPose =
            closestTagPose.transformBy(
                new Transform2d(
                    Constants.Field.robotReefWallL4ScoringOffsetDistance, 0, Rotation2d.k180deg));
      } else {
        m_driveTime = 0.6;
        m_targetPose =
            closestTagPose.transformBy(
                new Transform2d(
                    Constants.Field.robotAlgaeIntakeOffsetDistance, 0, Rotation2d.k180deg));
      }
      // Create target trajectory
      m_trajectory =
          new SimpleSwerveTrajectory(
              m_swerve.getSingleTagPose(),
              m_swerve.getFieldSpeeds(),
              m_targetPose,
              new Constraints(
                  Constants.Swerve.maxDriveSpeed, 0.3 * Constants.Swerve.maxDriveAcceleration),
              new Constraints(
                  Constants.Swerve.maxAngularVelocity,
                  0.7 * Constants.Swerve.maxAngularAcceleration));
      m_currentState = ScoringState.DRIVE_TO_REEF;
      // Set trajectory timer
      m_trajectoryTimer.start();
      m_onTargetTimer.start();
    } else {
      if (isNet) {
        boolean oppSide =
            m_isBlue
                ? m_swerve.getPose().getX() > (Constants.Field.fieldLength / 2.0)
                : m_swerve.getPose().getX() < (Constants.Field.fieldLength / 2.0);
        // Select target tag based on what side of the field we're on
        final int tagIndex = m_isBlue ? (oppSide ? 3 : 13) : (oppSide ? 14 : 4);
        Pose2d tag = Fiducials.aprilTagFiducials[tagIndex].getPose().toPose2d();
        m_targetPose =
            tag.transformBy(
                new Transform2d(
                    Constants.Field.robotReefWallL4ScoringOffsetDistance, 0, Rotation2d.kZero));

        m_trajectory =
            new SimpleSwerveTrajectory(
                m_swerve.getSingleTagPose(),
                m_swerve.getFieldSpeeds(),
                m_targetPose,
                new Constraints(
                    Constants.Swerve.maxDriveSpeed, 0.3 * Constants.Swerve.maxDriveAcceleration),
                new Constraints(
                    Constants.Swerve.maxAngularVelocity,
                    0.7 * Constants.Swerve.maxAngularAcceleration));

        m_trajectoryTimer.start();
        m_onTargetTimer.start();

        m_currentState = ScoringState.ALIGN_TO_NET;
      } else {
        m_currentState = ScoringState.START_SCORE_IN_PROCESSOR;
      }
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    DriveMode driveMode = DriveMode.STOP;

    final Pose2d robotPose = m_swerve.getSingleTagPose();
    switch (m_currentState) {
      case DRIVE_TO_REEF:
        {
          // Preraise when close
          if (isWithinPoseTolerance(
                  robotPose,
                  m_targetPose,
                  Units.inchesToMeters(48.0),
                  Units.inchesToMeters(48.0),
                  Math.toRadians(80.0))
              && AlignmentUtilities.isClearOfReef(robotPose, m_isBlue)) {
            if (m_isHigh) {
              m_elevator.setHeight(Constants.Elevator.highAlgaeHeight, false);
              m_arm.setAngle(Constants.Arm.highAlgaeAngle, false);
              m_gripper.setWristAngle(Constants.Gripper.highAlgaeWristAngle);
            } else {
              m_elevator.setHeight(Constants.Elevator.lowAlgaeHeight, false);
              m_arm.setAngle(Constants.Arm.lowAlgaeAngle, false);
              m_gripper.setWristAngle(Constants.Gripper.lowAlgaeWristAngle);
            }
          }

          driveMode = DriveMode.TO_POSE;
          if (m_trajectoryTimer.get() < m_trajectory.totalTime()
              || !isWithinPoseTolerance(
                  robotPose,
                  m_targetPose,
                  Units.inchesToMeters(6.0),
                  Units.inchesToMeters(8.0),
                  Math.toRadians(5.0))) {

            m_onTargetTimer.reset();
          }
          if (m_onTargetTimer.get() > 0.1) {
            m_currentState = ScoringState.RAISE_FOR_ALGAE;
          }
          break;
        }
      case RAISE_FOR_ALGAE:
        {
          double targetElevatorHeight;
          double targetArmAngle;
          double targetWristAngle;
          driveMode = DriveMode.TO_POSE;

          if (m_isHigh) {
            targetElevatorHeight = Constants.Elevator.highAlgaeHeight;
            targetArmAngle = Constants.Arm.highAlgaeAngle;
            targetWristAngle = Constants.Gripper.highAlgaeWristAngle;
          } else {
            targetElevatorHeight = Constants.Elevator.lowAlgaeHeight;
            targetArmAngle = Constants.Arm.lowAlgaeAngle;
            targetWristAngle = Constants.Gripper.lowAlgaeWristAngle;
          }
          m_elevator.setHeight(targetElevatorHeight, false);
          m_arm.setAngle(targetArmAngle, false);
          m_gripper.setWristAngle(targetWristAngle);
          if (m_gripper.isWristAtAngle(targetWristAngle, Units.degreesToRadians(2.0))
              && m_elevator.isElevatorAtHeight(targetElevatorHeight, Units.inchesToMeters(2.0))
              && m_arm.isArmAtAngle(targetArmAngle, Units.degreesToRadians(3.0))) {
            m_currentState = ScoringState.INTAKE_ALGAE;
            m_onTargetTimer.reset();
          }
          break;
        }
      case INTAKE_ALGAE:
        {
          driveMode = DriveMode.FORWARD;
          m_gripper.holdAlgae();

          final double targetArmAngle =
              m_isHigh ? Constants.Arm.levelFourAngle : Constants.Arm.lowAlgaeAngle;

          if (m_onTargetTimer.get() > m_driveTime) {
            m_currentState = ScoringState.BACK_UP_WITH_ALGAE;
            m_arm.setAngle(targetArmAngle + Units.degreesToRadians(5.0), false);
            m_onTargetTimer.reset();
            m_gripper.hasAlgae = true;
            if (Robot.isSimulation()) {
              m_gripper.setSimHasAlgae(true);
            }
          }
          break;
        }
      case BACK_UP_WITH_ALGAE:
        {
          m_gripper.holdAlgae();
          driveMode = DriveMode.BACKUP;
          if (m_onTargetTimer.get() >= 0.5) {
            m_currentState = ScoringState.FINISHED;
          }
          break;
        }
      case ALIGN_TO_NET:
        {
          m_gripper.holdAlgae();
          driveMode = DriveMode.TO_X_VALUE;
          if (Math.abs(robotPose.getX() - m_targetPose.getX()) > Units.inchesToMeters(6.0)
              || Math.abs(robotPose.getRotation().minus(m_targetPose.getRotation()).getRadians())
                  > Math.toRadians(15.0)) {
            m_onTargetTimer.reset();
          }
          if (m_onTargetTimer.get() > 0.1) {
            m_currentState = ScoringState.START_SCORE_IN_NET;
          }
          break;
        }
      case START_SCORE_IN_NET:
        {
          driveMode = DriveMode.STOP;
          m_elevator.setHeight(Constants.Elevator.netHeight, false);
          m_gripper.setWristAngle(Constants.Gripper.netWristAngle);
          m_gripper.holdAlgae();

          if (m_elevator.isElevatorAtHeight(Constants.Elevator.netHeight, Units.inchesToMeters(2.0))
              && m_gripper.isWristAtAngle(Constants.Gripper.netWristAngle, Math.toRadians(3.0))) {
            m_currentState = ScoringState.SCORE_IN_NET;
          }
          break;
        }
      case SCORE_IN_NET:
        {
          driveMode = DriveMode.STOP;
          m_arm.setAngle(Constants.Arm.netAngle, true);

          if (m_arm.getAngle() > Math.toRadians(25.0)) {
            m_gripper.scoreAlgae();
            if (Robot.isSimulation() && m_gripper.simHasAlgae()) {
              MapleSimUtils.scoreAlgaeInNet(m_swerve, m_elevator, m_arm, m_gripper);
              m_gripper.setSimHasAlgae(false);
            }
          } else {
            m_gripper.holdAlgae();
          }

          if (m_arm.isArmAtAngle(Constants.Arm.netAngle, Math.toRadians(3.0))) {
            m_currentState = ScoringState.FINISHED;
            m_gripper.hasAlgae = false;
          }
          break;
        }
      case START_SCORE_IN_PROCESSOR:
        {
          m_elevator.setHeight(Constants.Elevator.unstowAlgaeHeight, true);
          m_arm.setAngle(Constants.Arm.stowAngle, true);
          m_gripper.setWristAngle(Constants.Gripper.stowAngle);
          m_gripper.holdAlgae();

          if (m_elevator.isElevatorAtHeight(
                  Constants.Elevator.unstowAlgaeHeight, Units.inchesToMeters(2.0))
              && m_gripper.isWristAtAngle(Constants.Gripper.stowAngle, Math.toRadians(3.0))) {
            m_currentState = ScoringState.SCORE_IN_PROCESSOR;
          }
          break;
        }
      case SCORE_IN_PROCESSOR:
        {
          m_elevator.setHeight(Constants.Elevator.scoreAlgaeHeight, true);
          m_arm.setAngle(Constants.Arm.processorAngle, true);
          m_gripper.setWristAngle(Constants.Gripper.processorWristAngle);
          if (m_elevator.isElevatorAtHeight(
                  Constants.Elevator.scoreAlgaeHeight, Units.inchesToMeters(1.0))
              && m_arm.isArmAtAngle(Constants.Arm.processorAngle, Math.toRadians(3.0))) {
            m_gripper.scoreAlgae();
            if (Robot.isSimulation() && m_gripper.simHasAlgae()) {
              MapleSimUtils.scoreAlgae(m_swerve, m_elevator, m_arm, m_gripper);
              m_gripper.setSimHasAlgae(false);
            }
          } else {
            m_gripper.holdAlgae();
            m_onTargetTimer.restart();
          }
          if (m_onTargetTimer.get() > 1.0) {
            m_currentState = ScoringState.FINISHED;
            m_gripper.hasAlgae = false;
          }
          break;
        }
      case FINISHED:
        {
          break;
        }
    }
    switch (driveMode) {
      case TO_POSE:
        {
          final double t = m_trajectoryTimer.get();
          final SwerveTrajectoryState trajectoryState = m_trajectory.getState(t);
          m_swerve.driveToPose(
              trajectoryState.pose,
              trajectoryState.vx,
              trajectoryState.vy,
              trajectoryState.vTheta,
              Constants.Swerve.teleopScrubLimit,
              true);
          break;
        }
      case TO_X_VALUE:
        {
          final double t = m_trajectoryTimer.get();
          final SwerveTrajectoryState trajectoryState = m_trajectory.getState(t);
          final Pose2d pose =
              new Pose2d(
                  trajectoryState.pose.getX(),
                  robotPose.getY(),
                  trajectoryState.pose.getRotation());
          m_swerve.driveToPose(
              pose,
              trajectoryState.vx,
              0.0,
              trajectoryState.vTheta,
              Constants.Swerve.teleopScrubLimit,
              true);
          break;
        }
      case BACKUP:
        {
          m_swerve.driveClosedLoop(-0.6, 0.0, 0.0, false, Constants.Swerve.teleopScrubLimit);
          break;
        }
      case FORWARD:
        {
          m_swerve.driveClosedLoop(0.6, 0.0, 0.0, false, Constants.Swerve.teleopScrubLimit);
          break;
        }
      case STOP:
        {
          m_swerve.driveOpenLoop(0.0, 0.0, 0.0, true, Constants.Swerve.teleopScrubLimit);
          break;
        }
    }
    Logger.recordOutput("ScoreAlgaeCommand/State", m_currentState);
    Logger.recordOutput("ScoreAlgaeCommand/Drive Mode", driveMode);
    Logger.recordOutput("ScoreAlgaeCommand/On Target Timer", m_onTargetTimer.get());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_swerve.trackAllTags();
    m_trajectoryTimer.stop();
    m_onTargetTimer.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_currentState == ScoringState.FINISHED;
  }

  private boolean isWithinPoseTolerance(
      Pose2d currentPose,
      Pose2d targetPose,
      double translationalTolX,
      double translationalTolY,
      double rotationalTol) {

    Pose2d relativePose = currentPose.relativeTo(targetPose);

    return (Math.abs(relativePose.getX()) < translationalTolX)
        && (Math.abs(relativePose.getY()) < translationalTolY)
        && Math.abs(targetPose.getRotation().minus(currentPose.getRotation()).getRadians())
            < rotationalTol;
  }
}
