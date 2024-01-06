package frc.robot.parsers.utils;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

public class TrapezoidalLimitsJson {
  public String unit;
  public double velocity;
  public double acceleration;

  public TrapezoidProfile.Constraints getLimitsM() {
    if (unit.equalsIgnoreCase("inch")) {
      return new TrapezoidProfile.Constraints(
          Units.inchesToMeters(velocity), Units.inchesToMeters(acceleration)
      );
    } else if (unit.equalsIgnoreCase("meter")) {
      return new TrapezoidProfile.Constraints(velocity, acceleration);
    } else {
      System.err.println("Incompatible types: expected length but got " + unit);
      System.exit(1);
    }
    return null;
  }
  public TrapezoidProfile.Constraints getLimitsRad() {
    if (unit.equalsIgnoreCase("degree") || unit.equalsIgnoreCase("deg")) {
      return new TrapezoidProfile.Constraints(
          Units.degreesToRadians(velocity), Units.degreesToRadians(acceleration)
      );
    } else if (unit.equalsIgnoreCase("radian") || unit.equalsIgnoreCase("rad")) {
      return new TrapezoidProfile.Constraints(velocity, acceleration);
    } else {
      System.err.println("Incompatible types: expected length but got " + unit);
      System.exit(1);
    }
    return null;
  }
}
