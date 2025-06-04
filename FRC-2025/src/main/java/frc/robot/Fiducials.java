package frc.robot;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;
import frc.quixlib.vision.Fiducial;

public class Fiducials {
  // https://firstfrc.blob.core.windows.net/frc2025/FieldAssets/2025FieldDrawings-FieldLayoutAndMarking.pdf

  private static final double aprilTagSize = Units.inchesToMeters(6.5); // m

  public static final Fiducial[] aprilTagFiducials =
      new Fiducial[] {
        new Fiducial(
            Fiducial.Type.APRILTAG,
            1,
            new Pose3d(
                Units.inchesToMeters(657.37),
                Units.inchesToMeters(25.80),
                Units.inchesToMeters(58.50),
                new Rotation3d(0.0, 0.0, Math.toRadians(126.0))),
            aprilTagSize),
        new Fiducial(
            Fiducial.Type.APRILTAG,
            2,
            new Pose3d(
                Units.inchesToMeters(657.37),
                Units.inchesToMeters(291.20),
                Units.inchesToMeters(58.50),
                new Rotation3d(0.0, 0.0, Math.toRadians(234.0))),
            aprilTagSize),
        new Fiducial(
            Fiducial.Type.APRILTAG,
            3,
            new Pose3d(
                Units.inchesToMeters(455.15),
                Units.inchesToMeters(317.15),
                Units.inchesToMeters(51.25),
                new Rotation3d(0.0, 0.0, Math.toRadians(270.0))),
            aprilTagSize),
        new Fiducial(
            Fiducial.Type.APRILTAG,
            4,
            new Pose3d(
                Units.inchesToMeters(365.20),
                Units.inchesToMeters(241.64),
                Units.inchesToMeters(73.54),
                new Rotation3d(0.0, Math.toRadians(30.0), Math.toRadians(0.0))),
            aprilTagSize),
        new Fiducial(
            Fiducial.Type.APRILTAG,
            5,
            new Pose3d(
                Units.inchesToMeters(365.20),
                Units.inchesToMeters(75.39),
                Units.inchesToMeters(73.54),
                new Rotation3d(0.0, Math.toRadians(30.0), Math.toRadians(0.0))),
            aprilTagSize),
        new Fiducial(
            Fiducial.Type.APRILTAG,
            6,
            new Pose3d(
                Units.inchesToMeters(530.49),
                Units.inchesToMeters(130.17),
                Units.inchesToMeters(12.13),
                new Rotation3d(0.0, 0.0, Math.toRadians(300.0))),
            aprilTagSize),
        new Fiducial(
            Fiducial.Type.APRILTAG,
            7,
            new Pose3d(
                Units.inchesToMeters(546.87),
                Units.inchesToMeters(158.50),
                Units.inchesToMeters(12.13),
                new Rotation3d(0.0, 0.0, Math.toRadians(0.0))),
            aprilTagSize),
        new Fiducial(
            Fiducial.Type.APRILTAG,
            8,
            new Pose3d(
                Units.inchesToMeters(530.49),
                Units.inchesToMeters(186.83),
                Units.inchesToMeters(12.13),
                new Rotation3d(0.0, 0.0, Math.toRadians(60.0))),
            aprilTagSize),
        new Fiducial(
            Fiducial.Type.APRILTAG,
            9,
            new Pose3d(
                Units.inchesToMeters(497.77),
                Units.inchesToMeters(186.83),
                Units.inchesToMeters(12.13),
                new Rotation3d(0.0, 0.0, Math.toRadians(120.0))),
            aprilTagSize),
        new Fiducial(
            Fiducial.Type.APRILTAG,
            10,
            new Pose3d(
                Units.inchesToMeters(481.39),
                Units.inchesToMeters(158.50),
                Units.inchesToMeters(12.13),
                new Rotation3d(0.0, 0.0, Math.toRadians(180.0))),
            aprilTagSize),
        new Fiducial(
            Fiducial.Type.APRILTAG,
            11,
            new Pose3d(
                Units.inchesToMeters(497.77),
                Units.inchesToMeters(130.17),
                Units.inchesToMeters(12.13),
                new Rotation3d(0.0, 0.0, Math.toRadians(240.0))),
            aprilTagSize),
        new Fiducial(
            Fiducial.Type.APRILTAG,
            12,
            new Pose3d(
                Units.inchesToMeters(33.51),
                Units.inchesToMeters(25.80),
                Units.inchesToMeters(58.50),
                new Rotation3d(0.0, 0.0, Math.toRadians(54.0))),
            aprilTagSize),
        new Fiducial(
            Fiducial.Type.APRILTAG,
            13,
            new Pose3d(
                Units.inchesToMeters(33.51),
                Units.inchesToMeters(291.20),
                Units.inchesToMeters(58.50),
                new Rotation3d(0.0, 0.0, Math.toRadians(306.0))),
            aprilTagSize),
        new Fiducial(
            Fiducial.Type.APRILTAG,
            14,
            new Pose3d(
                Units.inchesToMeters(325.68),
                Units.inchesToMeters(241.64),
                Units.inchesToMeters(73.54),
                new Rotation3d(0.0, Math.toRadians(30.0), Math.toRadians(180.0))),
            aprilTagSize),
        new Fiducial(
            Fiducial.Type.APRILTAG,
            15,
            new Pose3d(
                Units.inchesToMeters(325.68),
                Units.inchesToMeters(75.39),
                Units.inchesToMeters(73.54),
                new Rotation3d(0.0, Math.toRadians(30.0), Math.toRadians(180.0))),
            aprilTagSize),
        new Fiducial(
            Fiducial.Type.APRILTAG,
            16,
            new Pose3d(
                Units.inchesToMeters(235.73),
                Units.inchesToMeters(-0.15),
                Units.inchesToMeters(51.25),
                new Rotation3d(0.0, 0.0, Math.toRadians(90.0))),
            aprilTagSize),
        new Fiducial(
            Fiducial.Type.APRILTAG,
            17,
            new Pose3d(
                Units.inchesToMeters(160.39),
                Units.inchesToMeters(130.17),
                Units.inchesToMeters(12.13),
                new Rotation3d(0.0, 0.0, Math.toRadians(240.0))),
            aprilTagSize),
        new Fiducial(
            Fiducial.Type.APRILTAG,
            18,
            new Pose3d(
                Units.inchesToMeters(144.00),
                Units.inchesToMeters(158.50),
                Units.inchesToMeters(12.13),
                new Rotation3d(0.0, 0.0, Math.toRadians(180.0))),
            aprilTagSize),
        new Fiducial(
            Fiducial.Type.APRILTAG,
            19,
            new Pose3d(
                Units.inchesToMeters(160.39),
                Units.inchesToMeters(186.83),
                Units.inchesToMeters(12.13),
                new Rotation3d(0.0, 0.0, Math.toRadians(120.0))),
            aprilTagSize),
        new Fiducial(
            Fiducial.Type.APRILTAG,
            20,
            new Pose3d(
                Units.inchesToMeters(193.10),
                Units.inchesToMeters(186.83),
                Units.inchesToMeters(12.13),
                new Rotation3d(0.0, 0.0, Math.toRadians(60.0))),
            aprilTagSize),
        new Fiducial(
            Fiducial.Type.APRILTAG,
            21,
            new Pose3d(
                Units.inchesToMeters(209.49),
                Units.inchesToMeters(158.50),
                Units.inchesToMeters(12.13),
                new Rotation3d(0.0, 0.0, Math.toRadians(0.0))),
            aprilTagSize),
        new Fiducial(
            Fiducial.Type.APRILTAG,
            22,
            new Pose3d(
                Units.inchesToMeters(193.10),
                Units.inchesToMeters(130.17),
                Units.inchesToMeters(12.13),
                new Rotation3d(0.0, 0.0, Math.toRadians(300.0))),
            aprilTagSize)
      };

  public static final Fiducial[] blueCoralStationTags = {
    aprilTagFiducials[11], aprilTagFiducials[12]
  };
  public static final Fiducial[] redCoralStationTags = {aprilTagFiducials[0], aprilTagFiducials[1]};
}
