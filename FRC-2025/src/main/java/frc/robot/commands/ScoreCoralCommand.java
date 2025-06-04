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
import frc.robot.AlignmentState;
import frc.robot.AlignmentState.ReefLevel;
import frc.robot.AlignmentState.ReefStackChoice;
import frc.robot.AlignmentUtilities;
import frc.robot.Constants;
import frc.robot.Fiducials;
import frc.robot.MapleSimUtils;
import frc.robot.Robot;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.GripperSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

public class ScoreCoralCommand extends Command {
  private final SwerveSubsystem m_swerve;
  private final ArmSubsystem m_arm;
  private final ElevatorSubsystem m_elevator;
  private final GripperSubsystem m_gripper;
  private final AlignmentState m_aState = AlignmentState.getInstance();
  private final ReefStackChoice m_stackChoice;
  private final int m_forceTagID;
  private boolean m_isBlue = false;

  private final LoggedNetworkNumber m_depthTrim =
      new LoggedNetworkNumber("Trim/coralDepthTrim", 0.0);
  private final LoggedNetworkNumber m_lateralTrim =
      new LoggedNetworkNumber("Trim/coralLateralTrim", 0.0);

  private ScoringState m_currentState;

  private Pose2d m_reefTipPose;
  private Pose2d m_targetPrescorePose;
  private Pose2d m_targetScoringPose;
  private Pose2d m_prevPoseError;

  private SimpleSwerveTrajectory m_trajectory;
  private final Timer m_trajectoryTimer = new Timer();

  private final Timer m_waitForPieceTimer = new Timer();
  private final Timer m_onTargetTimer = new Timer();

  private enum ScoringState {
    PLAN_DRIVE_TO_PRESCORE,
    DRIVE_TO_PRESCORE_AND_PRESCORE,
    PLAN_DRIVE_TO_REEF,
    DRIVE_TO_REEF,
    LEVEL_ALIGN,
    SCORE,
    PRESTOW,
    FINISHED,
  }

  public ScoreCoralCommand(
      ReefStackChoice choice,
      SwerveSubsystem swerve,
      ArmSubsystem arm,
      ElevatorSubsystem elevator,
      GripperSubsystem gripper) {
    this(choice, swerve, arm, elevator, gripper, 0);
  }

  public ScoreCoralCommand(
      ReefStackChoice choice,
      SwerveSubsystem swerve,
      ArmSubsystem arm,
      ElevatorSubsystem elevator,
      GripperSubsystem gripper,
      int forceTagID) {
    m_stackChoice = choice;
    m_swerve = swerve;
    m_arm = arm;
    m_elevator = elevator;
    m_gripper = gripper;
    m_forceTagID = forceTagID;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(swerve, arm, elevator, gripper);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_isBlue =
        DriverStation.getAlliance().isPresent()
            && DriverStation.getAlliance().get() == Alliance.Blue;

    m_aState.setStackChoice(m_stackChoice);

    final int selectedTagID =
        m_forceTagID != 0
            ? Constants.Field.getReefTagForAlliance(m_forceTagID, m_isBlue)
            : AlignmentUtilities.determineClosestTagID(
                m_swerve.getSingleTagPose().transformBy(m_swerve.getVelocity().times(0.15)),
                m_isBlue);
    if (DriverStation.isEnabled()) {
      m_swerve.setTagsToTrack(new int[] {selectedTagID});
    }
    m_currentState =
        m_aState.getScoringLevel() == AlignmentState.ReefLevel.FOUR
            ? ScoringState.PLAN_DRIVE_TO_PRESCORE
            : ScoringState.PLAN_DRIVE_TO_REEF;
    m_waitForPieceTimer.start();
    m_waitForPieceTimer.reset();
    m_onTargetTimer.start();
    m_onTargetTimer.reset();
    m_trajectoryTimer.start();
    m_trajectoryTimer.reset();
    m_prevPoseError = new Pose2d();

    final Pose2d selectedTagPose =
        Fiducials.aprilTagFiducials[selectedTagID - 1].getPose().toPose2d();
    m_reefTipPose =
        AlignmentUtilities.determineTipPoseFromTag(selectedTagPose, m_aState.getStackChoice());
    Transform2d scoringTransform =
        new Transform2d(Constants.Field.robotReefWallScoringOffsetDistance, 0, Rotation2d.k180deg);
    if (m_aState.getScoringLevel() == AlignmentState.ReefLevel.ONE) {
      final double sign =
          m_aState.getStackChoice() == AlignmentState.ReefStackChoice.LEFT ? 1.0 : -1.0;
      scoringTransform =
          new Transform2d(
              Constants.Field.robotReefWallL1ScoringOffsetDistance,
              sign * Units.inchesToMeters(2),
              Rotation2d.k180deg.plus(Constants.Field.L1AngleOffset.times(sign)));
    } else if (m_aState.getScoringLevel() == AlignmentState.ReefLevel.FOUR) {
      scoringTransform =
          new Transform2d(
              Constants.Field.robotReefWallL4ScoringOffsetDistance, 0, Rotation2d.k180deg);
    }

    m_targetScoringPose =
        m_reefTipPose
            .transformBy(scoringTransform)
            .transformBy(
                new Transform2d(m_depthTrim.get(), -m_lateralTrim.get(), Rotation2d.kZero));
    m_targetPrescorePose =
        m_reefTipPose.transformBy(
            new Transform2d(
                Constants.Field.robotReefWallPrescoreOffsetDistance, 0, Rotation2d.k180deg));

    m_trajectory =
        new SimpleSwerveTrajectory(
            m_swerve.getSingleTagPose(),
            m_swerve.getFieldSpeeds(),
            m_swerve.getSingleTagPose(),
            new Constraints(
                Constants.Swerve.maxDriveSpeed, 0.3 * Constants.Swerve.maxDriveAcceleration),
            new Constraints(
                Constants.Swerve.maxAngularVelocity,
                0.7 * Constants.Swerve.maxAngularAcceleration));
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    final Pose2d robotPose = m_swerve.getSingleTagPose();
    Pose2d targetPose;

    switch (m_currentState) {
      case PLAN_DRIVE_TO_PRESCORE:
        {
          targetPose = m_targetPrescorePose;
          m_trajectory =
              new SimpleSwerveTrajectory(
                  m_swerve.getSingleTagPose(),
                  m_swerve.getFieldSpeeds(),
                  targetPose,
                  new Constraints(
                      Constants.Swerve.maxDriveSpeed, 0.3 * Constants.Swerve.maxDriveAcceleration),
                  new Constraints(
                      Constants.Swerve.maxAngularVelocity,
                      0.7 * Constants.Swerve.maxAngularAcceleration));
          m_trajectoryTimer.reset();
          m_currentState = ScoringState.DRIVE_TO_PRESCORE_AND_PRESCORE;
          break;
        }
      case DRIVE_TO_PRESCORE_AND_PRESCORE:
        {
          targetPose = m_targetPrescorePose;
          if (isWithinPoseTolerance(
                  robotPose,
                  targetPose,
                  Units.inchesToMeters(84.0),
                  Units.inchesToMeters(84.0),
                  Math.toRadians(80.0))
              && AlignmentUtilities.isClearOfReef(robotPose, m_isBlue)) {
            if (m_gripper.hasPiece()) {
              m_arm.setAngle(Constants.Arm.levelFourPrescoreAngle, false);
            }
          } else {
            m_waitForPieceTimer.reset();
          }

          if (m_gripper.hasPiece()) {
            m_gripper.stopRollers();
          } else {
            m_gripper.intake();
          }

          if (m_arm.getAngle() > Math.toRadians(30.0)) {
            m_currentState = ScoringState.PLAN_DRIVE_TO_REEF;
          } else if (m_forceTagID != 0 && m_waitForPieceTimer.get() > 1.45) {
            m_currentState = ScoringState.FINISHED;
          }
          break;
        }
      case PLAN_DRIVE_TO_REEF:
        {
          targetPose = m_targetScoringPose;
          m_trajectory =
              new SimpleSwerveTrajectory(
                  m_swerve.getSingleTagPose(),
                  m_swerve.getFieldSpeeds(),
                  targetPose,
                  new Constraints(
                      Constants.Swerve.maxDriveSpeed, 0.3 * Constants.Swerve.maxDriveAcceleration),
                  new Constraints(
                      Constants.Swerve.maxAngularVelocity,
                      0.7 * Constants.Swerve.maxAngularAcceleration));
          m_trajectoryTimer.reset();
          m_currentState = ScoringState.DRIVE_TO_REEF;
          break;
        }
      case DRIVE_TO_REEF:
        {
          targetPose = m_targetScoringPose;
          final double distanceToTarget =
              robotPose.getTranslation().getDistance(targetPose.getTranslation());
          if (m_gripper.hasPiece() && distanceToTarget <= Units.inchesToMeters(48.0)) {
            if (m_aState.getScoringLevel() == ReefLevel.FOUR) {
              // This shouldn't be necessary, as this is taken care of in
              // DRIVE_TO_PRESCORE_AND_PRESCORE.
              m_arm.setAngle(Constants.Arm.levelFourPrescoreAngle, false);
            } else {
              m_arm.setAngle(Constants.Arm.funnelWallClearanceAngle, false);
            }
            m_gripper.setWristAngle(Constants.Gripper.stowAngle);
          }

          if (m_gripper.hasPiece()) {
            m_gripper.stopRollers();
          } else {
            m_gripper.intake();
          }

          if (!isWithinPoseTolerance(
              robotPose,
              targetPose,
              Units.inchesToMeters(48.0),
              Units.inchesToMeters(48.0),
              Math.toRadians(30.0))) {
            m_onTargetTimer.reset();
          }

          final boolean isArmClear =
              m_aState.getScoringLevel() == ReefLevel.FOUR
                  ? m_arm.getAngle() > Math.toRadians(10.0)
                  : m_arm.isArmAtAngle(Constants.Arm.funnelWallClearanceAngle, Math.toRadians(3.0));
          if (isArmClear && m_onTargetTimer.get() > 0.02) {
            m_currentState = ScoringState.LEVEL_ALIGN;
          }

          break;
        }
      case LEVEL_ALIGN:
        {
          targetPose = m_targetScoringPose;
          double targetHeight;
          double targetArmAngle;
          double targetWristAngle;
          switch (m_aState.getScoringLevel()) {
            default:
            case ONE:
              targetHeight = Constants.Elevator.levelOneHeight;
              targetArmAngle = Constants.Arm.levelOneAngle;
              targetWristAngle = Constants.Gripper.levelOneWristAngle;
              break;
            case TWO:
              targetHeight = Constants.Elevator.levelTwoHeight;
              targetArmAngle = Constants.Arm.levelTwoAngle;
              targetWristAngle = Constants.Gripper.levelTwoWristAngle;
              break;
            case THREE:
              targetHeight = Constants.Elevator.levelThreeHeight;
              targetArmAngle = Constants.Arm.levelThreeAngle;
              targetWristAngle = Constants.Gripper.levelThreeWristAngle;
              break;
            case FOUR:
              targetHeight = Constants.Elevator.levelFourHeight;
              targetArmAngle = Constants.Arm.levelFourAngle;
              targetWristAngle = Constants.Gripper.levelFourWristAngle;
              break;
          }
          m_elevator.setHeight(targetHeight, false);
          m_arm.setAngle(targetArmAngle, m_aState.getScoringLevel() == ReefLevel.FOUR);
          m_gripper.setWristAngle(targetWristAngle);
          if (!isWithinPoseTolerance(
                  robotPose,
                  targetPose,
                  Units.inchesToMeters(1.5),
                  Units.inchesToMeters(1.0),
                  Math.toRadians(1.0))
              || !m_elevator.isElevatorAtHeight(targetHeight, Units.inchesToMeters(0.25))
              || !m_arm.isArmAtAngle(targetArmAngle, Math.toRadians(3.0))
              || !m_gripper.isWristAtAngle(targetWristAngle, Math.toRadians(3.0))) {
            m_onTargetTimer.reset();
          }
          if (m_onTargetTimer.get() > 0.05) {
            m_currentState = ScoringState.SCORE;
            m_onTargetTimer.reset();
          }
          break;
        }
      case SCORE:
        targetPose = m_targetScoringPose;
        {
          if (m_aState.getScoringLevel() == AlignmentState.ReefLevel.ONE) {
            if (m_aState.getStackChoice() == AlignmentState.ReefStackChoice.LEFT) {
              m_gripper.scoreLeft();
            } else {
              m_gripper.scoreRight();
            }
          } else if (m_aState.getScoringLevel() == AlignmentState.ReefLevel.FOUR) {
            m_gripper.scoreL4();
          } else {
            m_gripper.scoreL3L2();
          }

          if (Robot.isSimulation()) {
            if (m_gripper.simHasCoral()) {
              MapleSimUtils.scoreCoral(
                  m_swerve,
                  m_elevator,
                  m_arm,
                  m_gripper,
                  m_aState.getScoringLevel() == AlignmentState.ReefLevel.FOUR);
            }
            m_gripper.setSimHasCoral(false);
          }

          if (!m_gripper.hasPiece() || m_onTargetTimer.get() > 0.5) {
            m_currentState = ScoringState.PRESTOW;
          }
          break;
        }
      case PRESTOW:
        targetPose = m_targetScoringPose;
        {
          if (m_aState.getScoringLevel() == ReefLevel.FOUR) {
            m_elevator.setHeight(
                m_arm.getAngle() > Constants.Arm.levelFourReefClearanceAngle
                    ? Constants.Elevator.stowHeight
                    : Constants.Elevator.levelFourHeight,
                false);
            m_arm.setAngle(Constants.Arm.levelFourPrestowAngle, false);
            m_gripper.setWristAngle(Constants.Gripper.stowAngle);
          } else {
            if (m_aState.getScoringLevel() != ReefLevel.ONE) {
              m_elevator.setHeight(Constants.Elevator.stowHeight, false);
            }
            m_arm.setAngle(Constants.Arm.stowAngle, false);
            m_gripper.setWristAngle(Constants.Gripper.stowAngle);
          }

          m_currentState = ScoringState.FINISHED;
          break;
        }
      case FINISHED:
      default:
        targetPose = robotPose;
        {
          break;
        }
    }

    // Do control in targetPose's frame.
    final SwerveTrajectoryState nextTrajectoryState =
        m_trajectory.getState(m_trajectoryTimer.get());
    final Pose2d transformedTarget = nextTrajectoryState.pose.relativeTo(targetPose);
    final Pose2d transformedRobot = robotPose.relativeTo(targetPose);
    final Pose2d poseError = transformedTarget.relativeTo(transformedRobot);

    Logger.recordOutput("ScoreCoralCommand/actualX", transformedRobot.getX());
    Logger.recordOutput("ScoreCoralCommand/targetX", transformedTarget.getX());
    Logger.recordOutput("ScoreCoralCommand/actualY", transformedRobot.getY());
    Logger.recordOutput("ScoreCoralCommand/targetY", transformedTarget.getY());
    Logger.recordOutput(
        "ScoreCoralCommand/actualTheta", transformedRobot.getRotation().getRadians());
    Logger.recordOutput(
        "ScoreCoralCommand/targetTheta", transformedTarget.getRotation().getRadians());
    Logger.recordOutput("ScoreCoralCommand/xError", poseError.getX());
    Logger.recordOutput("ScoreCoralCommand/yError", poseError.getY());
    Logger.recordOutput("ScoreCoralCommand/thetaError", poseError.getRotation().getRadians());

    // Scale y and theta gains based on x distance to target.
    final double kXTol = 0.015;
    final double kYTol = 0.015;
    final double kThetaTol = Math.toRadians(1.0);

    final Pose2d dError = poseError.relativeTo(m_prevPoseError);
    final double xVel =
        Math.abs(poseError.getX()) < kXTol ? 0.0 : 5.0 * poseError.getX() + 2.0 * dError.getX();
    final double yVel =
        Math.abs(poseError.getY()) < kYTol ? 0.0 : 5.0 * poseError.getY() + 2.0 * dError.getY();
    final double thetaVel =
        Math.abs(poseError.getRotation().getRadians()) < kThetaTol
            ? 0.0
            : 6.0 * poseError.getRotation().getRadians() + 2.0 * dError.getRotation().getRadians();
    m_prevPoseError = poseError;

    // Transform back into field frame velocities.
    final Rotation2d inverseRotation = targetPose.getRotation().unaryMinus();
    final double fieldXVel = xVel * inverseRotation.getCos() + yVel * inverseRotation.getSin();
    final double fieldYVel = -xVel * inverseRotation.getSin() + yVel * inverseRotation.getCos();

    m_swerve.driveClosedLoop(
        nextTrajectoryState.vx + fieldXVel,
        nextTrajectoryState.vy + fieldYVel,
        nextTrajectoryState.vTheta + thetaVel,
        /*fieldRelative=*/ true,
        Constants.Swerve.teleopScrubLimit);

    Logger.recordOutput("DriveToReef/Target Pose", targetPose);
    Logger.recordOutput("DriveToReef/Trajectory Pose", nextTrajectoryState.pose);

    Logger.recordOutput("ScoreCoralCommand/State", m_currentState);
    Logger.recordOutput("ScoreCoralCommand/On Target Timer", m_onTargetTimer.get());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_swerve.trackAllTags();
    m_waitForPieceTimer.stop();
    m_onTargetTimer.stop();
    m_trajectoryTimer.stop();
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
