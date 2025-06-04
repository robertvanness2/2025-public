package frc.robot;

import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import frc.quixlib.devices.CANDeviceID;
import frc.quixlib.motorcontrol.MechanismRatio;
import frc.quixlib.motorcontrol.PIDConfig;
import frc.quixlib.swerve.QuixSwerveController;
import frc.quixlib.vision.Fiducial;
import frc.quixlib.vision.PipelineConfig;
import java.util.Map;

public class Constants {
  public static final boolean isReplay = false;
  public static final boolean resimWithTiming = false;
  public static final boolean simLocalization = false;

  public static final String kCanivoreName = "canivore";

  public static final class IMU {
    public static final CANDeviceID pigeonID = new CANDeviceID(1, kCanivoreName);
    public static final double gyroTrimZ = -0.05418;
  }

  public static final class Auto {
    // Push auto
    // Blue Alliance poses
    public static final Pose2d pushStartPoseBlue =
        new Pose2d(8.0, 6.7, new Rotation2d(1.25 * Math.PI));
    public static final Pose2d pushOutPoseBlue =
        new Pose2d(7.3, 6.7, new Rotation2d(1.25 * Math.PI));
    public static final Pose2d pushRetractPoseBlue =
        new Pose2d(7.1, 5.4, new Rotation2d(Units.degreesToRadians(240)));
    public static final Pose2d avoidPushedRobotsPoseBlue =
        new Pose2d(7.3, 5.8, new Rotation2d(Units.degreesToRadians(240)));

    // Transform to convert blue alliance poses to red alliance poses
    private static Pose2d transformBlueToRed(Pose2d bluePose) {
      final double redX = Constants.Field.fieldLength - bluePose.getX();
      final double redY = Constants.Field.fieldWidth - bluePose.getY();
      double redHeading = bluePose.getRotation().getRadians() + Math.PI;
      while (redHeading > Math.PI) redHeading -= 2 * Math.PI;
      while (redHeading < -Math.PI) redHeading += 2 * Math.PI;

      return new Pose2d(redX, redY, new Rotation2d(redHeading));
    }

    // Red Alliance poses
    public static final Pose2d pushStartPoseRed = transformBlueToRed(pushStartPoseBlue);
    public static final Pose2d pushOutPoseRed = transformBlueToRed(pushOutPoseBlue);
    public static final Pose2d pushRetractPoseRed = transformBlueToRed(pushRetractPoseBlue);
    public static final Pose2d avoidPushedRobotsPoseRed =
        transformBlueToRed(avoidPushedRobotsPoseBlue);
  }

  public static final class Field {
    public static final double fieldLength = Units.inchesToMeters(12 * 57 + 6.875); // From sim
    public static final double fieldWidth = Units.inchesToMeters(12 * 26 + 5); // From sim

    // Reef measurements
    public static final double robotReefWallPrescoreClearanceDistance = Units.inchesToMeters(40.0);
    public static final double robotReefWallPrescoreOffsetDistance = Units.inchesToMeters(44.0);
    public static final double robotReefWallScoringOffsetDistance = Units.inchesToMeters(30.0);
    public static final double robotReefWallL1ScoringOffsetDistance = Units.inchesToMeters(28.0);
    public static final Rotation2d L1AngleOffset = Rotation2d.fromDegrees(0.0);
    public static final double robotAlgaeIntakeOffsetDistance = Units.inchesToMeters(36.0);
    public static final double robotReefWallL4ScoringOffsetDistance = Units.inchesToMeters(26.0);
    public static final double netScoringOffsetDistance = Units.inchesToMeters(12.0); // TBD
    public static final double processorScoringOffsetDistance = Units.inchesToMeters(24.0); // TBD
    public static final Transform2d tagToLeftReefTipTransform =
        new Transform2d(Units.inchesToMeters(-2.0), Units.inchesToMeters(-6.5), Rotation2d.kZero);
    public static final Transform2d tagToRightReefTipTransform =
        new Transform2d(Units.inchesToMeters(-2.0), Units.inchesToMeters(6.5), Rotation2d.kZero);
    public static final Pose2d reefCenterBlue =
        new Pose2d(
            Fiducials.aprilTagFiducials[20]
                .getPose()
                .toPose2d()
                .getTranslation()
                .plus(Fiducials.aprilTagFiducials[17].getPose().toPose2d().getTranslation())
                .div(2.0),
            Rotation2d.kZero);
    public static final Pose2d reefCenterRed =
        new Pose2d(
            Fiducials.aprilTagFiducials[9]
                .getPose()
                .toPose2d()
                .getTranslation()
                .plus(Fiducials.aprilTagFiducials[6].getPose().toPose2d().getTranslation())
                .div(2.0),
            Rotation2d.kZero);

    // Tag shennaniganery
    private static final Map<Integer, Integer> blueToRedMap =
        Map.ofEntries(
            Map.entry(20, 11),
            Map.entry(19, 6),
            Map.entry(18, 7),
            Map.entry(17, 8),
            Map.entry(22, 9),
            Map.entry(21, 10));

    public static int getReefTagForAlliance(int tagID, boolean isBlue) {
      if (isBlue) {
        if (blueToRedMap.containsValue(tagID)) {
          for (Map.Entry<Integer, Integer> entry : blueToRedMap.entrySet()) {
            if (entry.getValue() == tagID) {
              return entry.getKey();
            }
          }
        }
        return tagID;
      } else {
        if (blueToRedMap.containsKey(tagID)) {
          return blueToRedMap.get(tagID);
        }

        return tagID;
      }
    }
  }

  public static final class Cameras {
    public static final class LeftCam {
      public static final Transform3d robotToCameraT =
          new Transform3d(
              new Translation3d(
                  Units.inchesToMeters(6.448335840566671),
                  Units.inchesToMeters(12.433084524656367),
                  Units.inchesToMeters(8.940524334006923)),
              new Rotation3d(
                  Units.degreesToRadians(-0.6787726091457186),
                  Units.degreesToRadians(-15.779523524036104),
                  Units.degreesToRadians(-15.791622361429742)));
      public static final PipelineConfig[] pipelineConfigs =
          new PipelineConfig[] {
            new PipelineConfig(
                Fiducial.Type.APRILTAG,
                1280,
                800,
                MatBuilder.fill(
                    Nat.N3(),
                    Nat.N3(),
                    916.965230021908,
                    0.0,
                    661.6928938560056,
                    0.0,
                    916.8276406094255,
                    435.5533504564346,
                    0.0,
                    0.0,
                    1.0),
                MatBuilder.fill(
                    Nat.N8(),
                    Nat.N1(),
                    0.05009856981900392,
                    -0.07369910749297018,
                    -1.0660525417228317E-5,
                    1.3422933851637837E-4,
                    0.009667561013012865,
                    0.0,
                    0.0,
                    0.0)),
          };
    }

    public static final class RightCam {
      public static final Transform3d robotToCameraT =
          new Transform3d(
              new Translation3d(
                  Units.inchesToMeters(6.532495676417899),
                  Units.inchesToMeters(-12.67579593665854),
                  Units.inchesToMeters(8.637531861540134)),
              new Rotation3d(
                  Units.degreesToRadians(-0.5133124038290454),
                  Units.degreesToRadians(-16.560452484503056),
                  Units.degreesToRadians(15.538407387175491)));
      public static final PipelineConfig[] pipelineConfigs =
          new PipelineConfig[] {
            new PipelineConfig(
                Fiducial.Type.APRILTAG,
                1280,
                800,
                MatBuilder.fill(
                    Nat.N3(),
                    Nat.N3(),
                    903.7449893954214,
                    0.0,
                    654.6877054577706,
                    0.0,
                    903.9577323243168,
                    369.13545590011745,
                    0.0,
                    0.0,
                    1.0),
                MatBuilder.fill(
                    Nat.N8(),
                    Nat.N1(),
                    0.05318440103944059,
                    -0.07489968261371575,
                    -5.477461690531807E-4,
                    -7.032312933604217E-4,
                    0.009722692505020142,
                    0.0,
                    0.0,
                    0.0)),
          };
    }
  }

  public static final class Swerve {
    public static final double maxDriveSpeed = 4.5; // m/s
    public static final double maxDriveAcceleration = 8.0; // m/s/s
    public static final double maxAngularVelocity = Math.PI * 2.0; // rad/s
    public static final double maxAngularAcceleration = Math.PI * 4.0; // rad/s/s
    public static final double trackWidth = Units.inchesToMeters(24.00);
    public static final double wheelBase = Units.inchesToMeters(24.00);
    public static final double wheelDiameter = Units.inchesToMeters(3.91);
    public static final double wheelCircumference = wheelDiameter * Math.PI;
    public static final MechanismRatio driveRatio =
        new MechanismRatio(1.0, (54.0 / 11.0) * (16.0 / 40.0) * (45.0 / 15.0), wheelCircumference);
    public static final MechanismRatio steeringRatio =
        new MechanismRatio(1.0, (22.0 / 10.0) * (88.0 / 16.0));
    public static final double steerDriveCouplingRatio = 54.0 / 11.0;
    public static final PIDConfig driveOpenLoopPIDConfig =
        new PIDConfig(0.0, 0.0, 0.0, 0.1, 0.125, 0.0, 0.0);
    public static final PIDConfig driveClosedLoopPIDConfig =
        new PIDConfig(0.3, 0.0, 0.0, 0.1, 0.125, 0.005, 0.0);
    public static final PIDConfig steeringPIDConfig =
        Robot.isSimulation() ? new PIDConfig(1.5, 0.0, 0.0) : new PIDConfig(5.0, 0.0, 0.01);

    /* Swerve Teleop Slew Rate Limits */
    public static final double linearSlewRate = 60.0; // m/s/s
    public static final double angularSlewRate = 120.0; // rad/s/s
    public static final double stickDeadband = 0.05;

    /* Module Slew Rate Limits */
    public static final double maxModuleAcceleration = 60.0; // m/s/s
    public static final double maxModuleSteeringRate = 4.0 * Math.PI; // rad/s/s

    /* Allowable scrub */
    public static final double autoScrubLimit = 0.25; // m/s
    public static final double teleopScrubLimit = 0.25; // m/s

    public static final QuixSwerveController driveController =
        new QuixSwerveController(
            new PIDController(5.0, 0.0, 0.0),
            new PIDController(5.0, 0.0, 0.0),
            new PIDController(4.0, 0.0, 0.0));

    /* Front Left Module - Module 0 */
    public static final class FrontLeft {
      public static final CANDeviceID driveMotorID = new CANDeviceID(1, kCanivoreName);
      public static final CANDeviceID steeringMotorID = new CANDeviceID(2, kCanivoreName);
      public static final CANDeviceID canCoderID = new CANDeviceID(1, kCanivoreName);
      public static final Translation2d modulePosition =
          new Translation2d(wheelBase / 2.0, trackWidth / 2.0);
      public static final double absEncoderOffsetRad = -0.212891 * 2.0 * Math.PI;
    }

    /* Rear Left Module - Module 1 */
    public static final class RearLeft {
      public static final CANDeviceID driveMotorID = new CANDeviceID(3, kCanivoreName);
      public static final CANDeviceID steeringMotorID = new CANDeviceID(4, kCanivoreName);
      public static final CANDeviceID canCoderID = new CANDeviceID(2, kCanivoreName);
      public static final Translation2d modulePosition =
          new Translation2d(-wheelBase / 2.0, trackWidth / 2.0);
      public static final double absEncoderOffsetRad = 0.169678 * 2.0 * Math.PI;
    }

    /* Rear Right Module - Module 2 */
    public static final class RearRight {
      public static final CANDeviceID driveMotorID = new CANDeviceID(5, kCanivoreName);
      public static final CANDeviceID steeringMotorID = new CANDeviceID(6, kCanivoreName);
      public static final CANDeviceID canCoderID = new CANDeviceID(3, kCanivoreName);
      public static final Translation2d modulePosition =
          new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0);
      public static final double absEncoderOffsetRad = 0.085938 * 2.0 * Math.PI;
    }

    /* Front Right Module - Module 3 */
    public static final class FrontRight {
      public static final CANDeviceID driveMotorID = new CANDeviceID(7, kCanivoreName);
      public static final CANDeviceID steeringMotorID = new CANDeviceID(8, kCanivoreName);
      public static final CANDeviceID canCoderID = new CANDeviceID(4, kCanivoreName);
      public static final Translation2d modulePosition =
          new Translation2d(wheelBase / 2.0, -trackWidth / 2.0);
      public static final double absEncoderOffsetRad = 0.462158 * 2.0 * Math.PI;
    }
  }

  public static final class Elevator {
    public static final CANDeviceID leftMotorID = new CANDeviceID(11, kCanivoreName);
    public static final CANDeviceID rightMotorID = new CANDeviceID(12, kCanivoreName);
    public static final double sprocketPitchDiameter = Units.inchesToMeters(1.273); // 16T #25
    public static final MechanismRatio motorRatio =
        new MechanismRatio(10, 32, Math.PI * sprocketPitchDiameter); // From CAD, not sure if final
    public static final boolean leftMotorInvert = true;
    public static final boolean rightMotorInvert = false;

    public static final int positionSlot = 0;
    public static final PIDConfig elevatorPIDConfig =
        new PIDConfig(2.0, 0, 0.1, 0, 0.115, 0.003, 0.39);
    public static final double maxVelocity = 2.0; // m/s
    public static final double maxGentleVelocity = 1.0; // rad/s
    public static final double maxAcceleration = 12.0; // m/s
    public static final double maxJerk = 0.0; // m/s/s
    public static final double maxGentleAcceleration = 3.0; // m/s

    public static final double rotorBootOffset = -0.125;
    public static final double minHeight = Units.inchesToMeters(0);
    public static final double maxHeight = Units.inchesToMeters(27);
    public static final double startingHeight = minHeight;
    public static final double stowHeight = Units.inchesToMeters(2.0);
    public static final double stowAlgaeHeight = Units.inchesToMeters(11.0);
    public static final double unstowAlgaeHeight = Units.inchesToMeters(13.0);
    public static final double scoreAlgaeHeight = Units.inchesToMeters(5.0);

    public static final double levelOneHeight = Units.inchesToMeters(4.0);
    public static final double levelTwoHeight = Units.inchesToMeters(11.0);
    public static final double levelThreeHeight = Units.inchesToMeters(26.0);
    public static final double levelFourHeight = Units.inchesToMeters(27.0);

    public static final double netHeight = maxHeight;
    public static final double lowAlgaeHeight = Units.inchesToMeters(10.0);
    public static final double highAlgaeHeight = Units.inchesToMeters(7.0);

    // For simulatilon.
    public static final double simCarriageMass = Units.lbsToKilograms(30.0);
  }

  public static final class Gripper {
    public static final CANDeviceID leftRollerMotorID = new CANDeviceID(16, kCanivoreName);
    public static final boolean leftRollerMotorInvert = true;
    public static final int leftRollerVelocitySlot = 0;

    public static final CANDeviceID rightRollerMotorID = new CANDeviceID(17, kCanivoreName);
    public static final boolean rightRollerMotorInvert = false;
    public static final int rightRollerVelocitySlot = 0;

    public static final int beamBreakPort = 0;

    // Roller motors have same gearing
    public static final MechanismRatio rollerMotorRatio = new MechanismRatio(12, 36);
    public static final PIDConfig rollerVelocityPIDConfig = new PIDConfig(0.5, 0, 0, 0, 0.12, 0, 0);

    public static final CANDeviceID wristMotorID = new CANDeviceID(15, kCanivoreName);
    public static final MechanismRatio wristMotorRatio =
        new MechanismRatio(1, (30.0 / 8.0) * (52.0 / 20.0) * (36.0 / 12.0));
    public static final boolean wristMotorInvert = false;
    public static final int wristPositionSlot = 0;
    public static final PIDConfig wristPositionPIDConfig =
        new PIDConfig(2.0, 0, 0.1, 0, 0.105, 0.001, 0);
    public static final double wristMaxVelocity = 10.0; // rad/s
    public static final double wristMaxAcceleration = 70.0; // rad/s^2
    public static final double wristMaxJerk = 0.0; // rad/s^3

    public static final double rotorBootOffset = 0.452637;
    public static final double minAngle = Units.degreesToRadians(-110.0); // rads
    public static final double maxAngle = Units.degreesToRadians(90.0); // rads
    public static final double startingAngle = Units.degreesToRadians(82.5);

    public static final double stowAngle = Units.degreesToRadians(72.0);
    public static final double stowAlgaeAngle = Units.degreesToRadians(0.0);
    public static final double untangleAngle = 0.0;

    // Positive velocity should mean rollers are spinning outwards
    public static final double scoreCoralVelocityL4 = 100.0; // rad/sec
    public static final double scoreCoralVelocityL3L2 = 60.0; // rad/sec
    public static final double scoreCoralVelocityL1Straight = 60.0; // rad/sec
    public static final double scoreCoralVelocityL1 = 50.0; // rad/sec
    public static final double intakeVelocity = 40.0; // rad/sec

    public static final double levelOneWristAngle = Units.degreesToRadians(55.0);
    public static final double levelTwoWristAngle = Units.degreesToRadians(30.0);
    public static final double levelThreeWristAngle = Units.degreesToRadians(30.0);
    public static final double levelFourWristAngle = Units.degreesToRadians(-80.0);

    public static final double netWristAngle = Units.degreesToRadians(60.0);
    public static final double processorWristAngle = Units.degreesToRadians(75.0);
    public static final double lowAlgaeWristAngle = Units.degreesToRadians(10.0);
    public static final double highAlgaeWristAngle = Units.degreesToRadians(-40.0);

    // For simulation.
    public static final double simWristMOI = 0.1; // kgMetersSquared
    public static final double simWristCGLength = Units.inchesToMeters(3.0); // m
  }

  public static final class Arm {
    public static final CANDeviceID rightMotorID = new CANDeviceID(14, kCanivoreName);
    public static final MechanismRatio armRatio = new MechanismRatio(8 * 20 * 16, 30 * 52 * 84);
    public static final boolean rightMotorInvert = false;

    // replace the placeholders in this chunk
    // kG is chosen to balance the arm at +/- 45 deg because we don't want to deal with the TalonFX
    // "arm cosine" gravity type.
    public static final PIDConfig PIDConfig = new PIDConfig(6.0, 0, 0.1, 0, 0.11, 0.003, 0.8);
    public static final int positionSlot = 0;
    public static final double maxVelocity = 10.0; // rad/s
    public static final double maxAcceleration = 25.0; // rad/s^2
    public static final double maxJerk = 200.0; // rad/s^3
    public static final double maxGentleAcceleration = 10.0; // rad/s^2

    public static final double rotorBootOffset = -0.085938;
    public static final double minAngle = Units.degreesToRadians(-82.5); // rads
    public static final double untangleAngle = Units.degreesToRadians(70); // rads
    public static final double maxAngle = Units.degreesToRadians(90.0); // rads
    public static final double startingAngle = Units.degreesToRadians(-82.5);
    public static final double stowAngle = Units.degreesToRadians(-82.5);
    public static final double stowAlgaeAngle = Units.degreesToRadians(-75);

    public static final double funnelWallClearanceAngle = Units.degreesToRadians(-60.0);
    public static final double levelOneAngle = Units.degreesToRadians(-45.0);
    public static final double levelTwoAngle = Units.degreesToRadians(-40.0);
    public static final double levelThreeAngle = Units.degreesToRadians(-40.0);
    public static final double levelFourPrescoreAngle = Units.degreesToRadians(40.0);
    public static final double levelFourPrestowAngle = Units.degreesToRadians(80.0);
    public static final double levelFourAngle = Units.degreesToRadians(40.0);
    public static final double levelFourReefClearanceAngle = Units.degreesToRadians(55.0);
    public static final double processorAngle = Units.degreesToRadians(-70.0);

    public static final double netAngle = maxAngle;
    public static final double lowAlgaeAngle = Units.degreesToRadians(-10.0);
    public static final double highAlgaeAngle = Units.degreesToRadians(40.0);

    // For simulation.
    public static final double simArmMOI = 1.5; // kgMetersSquared
    public static final double simArmCGLength = Units.inchesToMeters(15.0); // m
  }

  public static final class Viz3d {
    public static final Transform3d robotToElevatorCariage =
        new Transform3d(
            Units.inchesToMeters(4.0), 0.0, Units.inchesToMeters(5.75), Rotation3d.kZero);
    public static final Transform3d elevatorCarriageToArmPivot =
        new Transform3d(0, 0, Units.inchesToMeters(26.0), Rotation3d.kZero);
    public static final Transform3d armToGripperPivot =
        new Transform3d(Units.inchesToMeters(20.0), 0, 0, Rotation3d.kZero);
    public static final Transform3d gripperToCoral =
        new Transform3d(Units.inchesToMeters(-3.0), 0, Units.inchesToMeters(3.5), Rotation3d.kZero);
    public static final Transform3d gripperToAlgae =
        new Transform3d(Units.inchesToMeters(10.5), 0, 0, Rotation3d.kZero);
  }

  public static final class Funnel {
    public static final CANDeviceID funnelMotorID = new CANDeviceID(18, kCanivoreName);
    public static final MechanismRatio funnelMotorRatio = new MechanismRatio(12, 36);
    public static final boolean funnelMotorInvert = true;

    public static final CANDeviceID topRollerMotorID = new CANDeviceID(20, kCanivoreName);
    public static final MechanismRatio topRollerMotorRatio = new MechanismRatio(12, 30);
    public static final boolean topRollerMotorInvert = true;
  }

  // Placeholder values
  public static final class Example {
    public static final CANDeviceID motorID = new CANDeviceID(62, kCanivoreName);
    public static final MechanismRatio motorRatio = new MechanismRatio(0, 0, 0);
    public static final boolean motorInvert = false;
  }

  public static final class Climber {
    public static final CANDeviceID motorID = new CANDeviceID(19, kCanivoreName);
    public static final MechanismRatio motorRatio = new MechanismRatio(1.0, 1.0);
    public static final boolean motorInvert = false;
  }
}
