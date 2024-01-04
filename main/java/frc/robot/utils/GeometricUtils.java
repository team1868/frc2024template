package frc.robot.utils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;

public class GeometricUtils {
  public static boolean inRange(
      Pose2d curPose,
      Rotation2d yaw,
      Pose2d targetPose,
      double xyToleranceM,
      double thetaToleranceDeg
  ) {
    double thetaDiff = targetPose.getRotation().getDegrees() - yaw.getDegrees();
    return Math.abs(targetPose.getX() - curPose.getX()) <= xyToleranceM
        && Math.abs(targetPose.getY() - curPose.getY()) <= xyToleranceM
        && Math.abs(thetaDiff > 180 ? thetaDiff - 360 : thetaDiff) <= thetaToleranceDeg;
  }

  public static void poseToArray(Pose2d pose, double[] arr) {
    arr[0] = pose.getX();
    arr[1] = pose.getY();
    arr[2] = pose.getRotation().getDegrees();
  }

  /**
   * @param scopeReference Current Angle
   * @param newAngle Target Angle
   * @return Closest angle within -180 to 180 scope
   */
  public static double placeInClosestScopeDeg(double currentAngle, double desiredAngle) {
    double delta = (desiredAngle - currentAngle) % 360;
    if (delta > 180.0) {
      delta += -360.0;
    } else if (delta < -180.0) {
      delta += 360.0;
    }
    return currentAngle + delta;
  }

  public static double placeInClosestScopeRad(double scopeReferenceRad, double newAngleRad) {
    return Units.degreesToRadians(placeInClosestScopeDeg(
        Units.radiansToDegrees(scopeReferenceRad), Units.radiansToDegrees(newAngleRad)
    ));
  }
}
