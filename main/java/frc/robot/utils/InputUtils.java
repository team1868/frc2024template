package frc.robot.utils;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.constants.Constants.Control;

public class InputUtils {
  public static double scaleJoystickXMPS(double rawXAxis, double xyNet, double maxSpeedMPS) {
    // double scaledXY = xyNet < Control.STICK_NET_DEADBAND ? 0 : xyNet;
    // double scaledXY =
    //     xyNet < Control.STICK_NET_DEADBAND ? 0 : Math.signum(rawXAxis) * Math.pow(xyNet, 2);
    if (xyNet == 0)
      return 0;
    double scaledXY = xyNet < Control.STICK_NET_DEADBAND ? 0 : Math.pow(xyNet, 3);
    double scaledX = rawXAxis * scaledXY / xyNet;
    return scaledX * maxSpeedMPS;
  }

  public static double scaleJoystickYMPS(double rawYAxis, double xyNet, double maxSpeedMPS) {
    // double scaledXY = xyNet < Control.STICK_NET_DEADBAND ? 0 : xyNet;
    // double scaledXY =
    //     xyNet < Control.STICK_NET_DEADBAND ? 0 : Math.signum(rawYAxis) * Math.pow(xyNet, 2);
    if (xyNet == 0)
      return 0;
    double scaledXY = Math.abs(xyNet) < Control.STICK_NET_DEADBAND ? 0 : Math.pow(xyNet, 3);
    double scaledY = rawYAxis * scaledXY / xyNet;
    return scaledY * maxSpeedMPS;
  }

  public static double ScaleJoystickThetaRadPS(double rawAxis, Rotation2d maxAngularSpeed) {
    return ScaleJoystickThetaRadPS(rawAxis, maxAngularSpeed.getRadians());
  }

  public static double ScaleJoystickThetaRadPS(double rawAxis, double maxAngularSpeedRadPS) {
    // double angular = Math.abs(rawAxis) < Control.STICK_DEADBAND ? 0 : rawAxis;
    double angular = Math.abs(rawAxis) < Control.STICK_DEADBAND
        ? 0.0
        : Math.signum(rawAxis) * Math.pow(rawAxis, 2);
    // double angular = Math.abs(rawAxis) < Control.STICK_DEADBAND ? 0 : Math.pow(rawAxis, 3);
    return angular * maxAngularSpeedRadPS;
  }
}
