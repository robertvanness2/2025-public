package frc.robot;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;

public class ScoringKinematics {
  public static class ScoringKinematicsOutput {
    public final Transform3d elevatorT;
    public final Transform3d armT;
    public final Transform3d gripperT;
    public final Transform3d coralT;
    public final Transform3d algaeT;

    public ScoringKinematicsOutput(
        Transform3d elevatorT,
        Transform3d armT,
        Transform3d gripperT,
        Transform3d coralT,
        Transform3d algaeT) {
      this.elevatorT = elevatorT;
      this.armT = armT;
      this.gripperT = gripperT;
      this.coralT = coralT;
      this.algaeT = algaeT;
    }

    public Pose3d getElevatorPose() {
      return Pose3d.kZero.transformBy(elevatorT);
    }

    public Pose3d getArmPose() {
      return Pose3d.kZero.transformBy(armT);
    }

    public Pose3d getGripperPose() {
      return Pose3d.kZero.transformBy(gripperT);
    }

    public Pose3d getCoralPose() {
      return Pose3d.kZero.transformBy(coralT);
    }

    public Pose3d getAlgaePose() {
      return Pose3d.kZero.transformBy(algaeT);
    }
  }

  public static final ScoringKinematicsOutput computeForwardKinematics(
      double elevatorHeight, double armAngle, double wristAngle) {
    final Transform3d robotToElevatorTransform =
        Constants.Viz3d.robotToElevatorCariage.plus(
            new Transform3d(0, 0, elevatorHeight, Rotation3d.kZero));
    final Transform3d robotToArmTransform =
        robotToElevatorTransform
            .plus(Constants.Viz3d.elevatorCarriageToArmPivot)
            .plus(new Transform3d(Translation3d.kZero, new Rotation3d(0, -armAngle, 0)));
    final Transform3d robotToGripperTransform =
        robotToArmTransform
            .plus(Constants.Viz3d.armToGripperPivot)
            .plus(new Transform3d(Translation3d.kZero, new Rotation3d(0, -wristAngle, 0)));
    final Transform3d robotToCoralTransform =
        robotToGripperTransform.plus(Constants.Viz3d.gripperToCoral);
    final Transform3d robotToAlgaeTransform =
        robotToGripperTransform.plus(Constants.Viz3d.gripperToAlgae);
    return new ScoringKinematicsOutput(
        robotToElevatorTransform,
        robotToArmTransform,
        robotToGripperTransform,
        robotToCoralTransform,
        robotToAlgaeTransform);
  }
}
