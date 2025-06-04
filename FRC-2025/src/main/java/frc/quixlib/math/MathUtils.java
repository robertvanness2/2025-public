package frc.quixlib.math;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;

/** A utility class that provides various mathematical functions and operations. */
public class MathUtils {

  /** A small constant used for floating-point comparisons. */
  private static final double kEps = 1E-9;

  /**
   * Compares two double values for equality within a small epsilon.
   *
   * @param a the first value
   * @param b the second value
   * @return true if the values are equal within the epsilon, false otherwise
   */
  public static boolean epsilonEquals(double a, double b) {
    return epsilonEquals(a, b, kEps);
  }

  /**
   * Compares two double values for equality within a specified epsilon.
   *
   * @param a the first value
   * @param b the second value
   * @param epsilon the epsilon value
   * @return true if the values are equal within the epsilon, false otherwise
   */
  public static boolean epsilonEquals(double a, double b, double epsilon) {
    return (a - epsilon <= b) && (a + epsilon >= b);
  }

  /**
   * Converts Cartesian coordinates to polar coordinates.
   *
   * @param x the x-coordinate
   * @param y the y-coordinate
   * @return a Pair containing the radius and angle in radians
   */
  public static Pair<Double, Double> cart2pol(final double x, final double y) {
    final double r = Math.sqrt(x * x + y * y);
    final double theta = Math.atan2(y, x);
    return new Pair<>(r, theta);
  }

  /**
   * Converts polar coordinates to Cartesian coordinates.
   *
   * @param r the radius
   * @param theta the angle in radians
   * @return a Pair containing the x and y coordinates
   */
  public static Pair<Double, Double> pol2cart(final double r, final double theta) {
    final double x = r * Math.cos(theta);
    final double y = r * Math.sin(theta);
    return new Pair<>(x, y);
  }

  /**
   * Constrains an angle to be within the range [-pi, pi).
   *
   * @param angle the angle to constrain
   * @return the constrained angle
   */
  public static double constrainAngleNegPiToPi(final double angle) {
    double x = (angle + Math.PI) % (2.0 * Math.PI);
    if (x < 0.0) {
      x += 2.0 * Math.PI;
    }
    return x - Math.PI;
  }

  /**
   * Returns the angle placed within [-pi, pi) of the reference angle.
   *
   * @param angle the angle to place in scope
   * @param referenceAngle the reference angle
   * @return the angle placed within the scope of the reference angle
   */
  public static double placeInScope(final double angle, final double referenceAngle) {
    return referenceAngle + constrainAngleNegPiToPi(angle - referenceAngle);
  }

  /**
   * Clamps a value between a minimum and maximum value.
   *
   * @param val the value to clamp
   * @param min the minimum value
   * @param max the maximum value
   * @return the clamped value
   */
  public static double clamp(final double val, final double min, final double max) {
    return Math.max(Math.min(val, max), min);
  }

  /**
   * Obtains a new Pose2d from a constant curvature velocity. See:
   * https://github.com/strasdat/Sophus/blob/master/sophus/se2.hpp. Borrowed from 254:
   * https://github.com/Team254/FRC-2022-Public/blob/main/src/main/java/com/team254/lib/geometry/Pose2d.java
   *
   * @param delta the Twist2d representing the change in pose
   * @return the new Pose2d
   */
  public static Pose2d exp(final Twist2d delta) {
    final double sin_theta = Math.sin(delta.dtheta);
    final double cos_theta = Math.cos(delta.dtheta);
    final double s =
        Math.abs(delta.dtheta) < kEps
            ? 1.0 - 1.0 / 6.0 * delta.dtheta * delta.dtheta
            : sin_theta / delta.dtheta;
    final double c =
        Math.abs(delta.dtheta) < kEps ? 0.5 * delta.dtheta : (1.0 - cos_theta) / delta.dtheta;
    return new Pose2d(
        new Translation2d(delta.dx * s - delta.dy * c, delta.dx * c + delta.dy * s),
        new Rotation2d(cos_theta, sin_theta));
  }
}
