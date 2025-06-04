package frc.robot;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.ScoringKinematics.ScoringKinematicsOutput;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.GripperSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.seasonspecific.reefscape2025.ReefscapeAlgaeOnFly;
import org.ironmaple.simulation.seasonspecific.reefscape2025.ReefscapeCoralOnFly;

public class MapleSimUtils {
  /** Simulates ejecting the coral from the gripper */
  public static void scoreCoral(
      SwerveSubsystem swerve,
      ElevatorSubsystem elevator,
      ArmSubsystem arm,
      GripperSubsystem gripper,
      boolean isL4) {
    final ScoringKinematicsOutput kinematicsOutput =
        ScoringKinematics.computeForwardKinematics(
            elevator.getHeight(), arm.getAngle(), gripper.getWristAngle());
    final Pose3d coralPose = kinematicsOutput.getCoralPose();
    SimulatedArena.getInstance()
        .addGamePieceProjectile(
            new ReefscapeCoralOnFly(
                // Obtain robot position from drive simulation
                swerve.getPose().getTranslation(),
                // The scoring mechanism releases the coral at this position on the
                // robot
                coralPose
                    .toPose2d()
                    .getTranslation()
                    .plus(isL4 ? new Translation2d(0.1, 0) : Translation2d.kZero),
                // Obtain robot speed from drive simulation
                swerve.getFieldRelativeChassisSpeeds(),
                // Obtain robot facing from drive simulation
                swerve.getPose().getRotation(),
                // The height at which the coral is ejected
                coralPose
                    .transformBy(new Transform3d(0.0, 0.0, 0.1, Rotation3d.kZero))
                    .getMeasureZ(),
                // The initial speed of the coral
                MetersPerSecond.of(3),
                // Angle coral is ejected at
                coralPose
                    .getRotation()
                    .plus(new Rotation3d(0, Math.toRadians(isL4 ? 45.0 : 10.0), 0))
                    .getMeasureY()
                    .times(-1.0)));
  }

  public static void scoreAlgae(
      SwerveSubsystem swerve,
      ElevatorSubsystem elevator,
      ArmSubsystem arm,
      GripperSubsystem gripper) {
    final ScoringKinematicsOutput kinematicsOutput =
        ScoringKinematics.computeForwardKinematics(
            elevator.getHeight(), arm.getAngle(), gripper.getWristAngle());
    final Pose3d algaePose = kinematicsOutput.getAlgaePose();
    final boolean isFlipped = algaePose.getRotation().getX() > 0.5 * Math.PI;
    SimulatedArena.getInstance()
        .addGamePieceProjectile(
            new ReefscapeAlgaeOnFly(
                // Obtain robot position from drive simulation
                swerve.getPose().getTranslation(),
                // The scoring mechanism releases the algae at this position on the
                // robot
                algaePose.toPose2d().getTranslation(),
                // Obtain robot speed from drive simulation
                swerve.getFieldRelativeChassisSpeeds(),
                // Obtain robot facing from drive simulation
                swerve
                    .getPose()
                    .getRotation()
                    .plus(isFlipped ? Rotation2d.k180deg : Rotation2d.kZero),
                // The height at which the algae is ejected
                algaePose.getMeasureZ(),
                // The initial speed of the algae
                MetersPerSecond.of(3),
                // Angle algae is ejected at
                algaePose.getRotation().getMeasureY().times(-1.0)));
  }

  public static void scoreAlgaeInNet(
      SwerveSubsystem swerve,
      ElevatorSubsystem elevator,
      ArmSubsystem arm,
      GripperSubsystem gripper) {
    final ScoringKinematicsOutput kinematicsOutput =
        ScoringKinematics.computeForwardKinematics(
            elevator.getHeight(), arm.getAngle(), gripper.getWristAngle());
    final Pose3d algaePose = kinematicsOutput.getAlgaePose();
    SimulatedArena.getInstance()
        .addGamePieceProjectile(
            new ReefscapeAlgaeOnFly(
                // Obtain robot position from drive simulation
                swerve.getPose().getTranslation(),
                // The scoring mechanism releases the algae at this position on the
                // robot
                algaePose.toPose2d().rotateBy(Rotation2d.k180deg).getTranslation(),
                // Obtain robot speed from drive simulation
                swerve.getFieldRelativeChassisSpeeds(),
                // Obtain robot facing from drive simulation
                swerve.getPose().getRotation().plus(Rotation2d.k180deg),
                // The height at which the algae is ejected
                algaePose.getMeasureZ(),
                // The initial speed of the algae
                MetersPerSecond.of(5),
                // Angle algae is ejected at
                new Rotation3d(0, 0.25 * Math.PI, 0).getMeasureY()));
  }
}
