package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import frc.robot.AlignmentState.ReefStackChoice;
import org.apache.commons.math3.geometry.euclidean.twod.Line;
import org.apache.commons.math3.geometry.euclidean.twod.Vector2D;

public class AlignmentUtilities {
  public static int determineClosestTagID(Pose2d robotPose, Boolean isBlue) {
    // Define tag indices
    final int[] tagIndices =
        isBlue ? new int[] {16, 17, 18, 19, 20, 21} : new int[] {5, 6, 7, 8, 9, 10};

    // Evaluate each canidate
    int closestTagIndex = 0; // Is this the right initial value?
    double closestTagDist = Double.POSITIVE_INFINITY;
    for (int tagIndex : tagIndices) {
      Pose2d tag = Fiducials.aprilTagFiducials[tagIndex].getPose().toPose2d();
      double tagDist = tag.getTranslation().getDistance(robotPose.getTranslation());

      if (tagDist < closestTagDist) {
        closestTagIndex = tagIndex;
        closestTagDist = tagDist;
      }
    }
    return closestTagIndex + 1;
  }

  private static Line[] createReefLines(Pose2d centerPose) {
    Line[] lines = new Line[3];
    final Vector2D center = new Vector2D(centerPose.getX(), centerPose.getY());

    for (int i = 0; i < 3; i++) {
      double angle = Math.toRadians(30 + 60 * i);
      lines[i] = new Line(center, angle, 1e-6);
    }

    return lines;
  }

  static final Line[] reefBlueLines = createReefLines(Constants.Field.reefCenterBlue);
  static final Line[] reefRedLines = createReefLines(Constants.Field.reefCenterRed);

  public static boolean rumbleCondition(Pose2d robotPose, boolean isBlue) {
    final Line[] reefLines = isBlue ? reefBlueLines : reefRedLines;

    final Vector2D robotPoint = new Vector2D(robotPose.getX(), robotPose.getY());

    double minDistance = Double.POSITIVE_INFINITY;
    for (Line line : reefLines) {
      minDistance = Math.min(minDistance, line.distance(robotPoint));
    }

    return minDistance < 0.2;
  }

  public static Pose2d determineTipPoseFromTag(Pose2d tagPose, ReefStackChoice stackChoice) {
    final Transform2d tagToTop =
        stackChoice == ReefStackChoice.RIGHT
            ? Constants.Field.tagToRightReefTipTransform
            : Constants.Field.tagToLeftReefTipTransform;
    return tagPose.transformBy(tagToTop);
  }

  public static boolean isClearOfReef(Pose2d robotPose, boolean isBlue) {
    final int closestTagID = determineClosestTagID(robotPose, isBlue);
    final Pose2d closestTagPose =
        Fiducials.aprilTagFiducials[closestTagID - 1].getPose().toPose2d();
    final double distToClosestReefWall =
        robotPose.relativeTo(closestTagPose).getTranslation().getX();
    return distToClosestReefWall > Constants.Field.robotReefWallPrescoreClearanceDistance;
  }
}
