package frc.robot.parsers.utils;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;

public class UnitJson {
  public String unit;
  public double value;

  public double getLengthM() {
    if (unit.equalsIgnoreCase("inch") || unit.equalsIgnoreCase("in")) {
      return Units.inchesToMeters(value);
    } else if (unit.equalsIgnoreCase("meter") || unit.equalsIgnoreCase("m")) {
      return value;
    } else {
      System.err.println("Incompatible types: expected length but got " + unit);
      System.exit(1);
    }
    return -1;
  }

  public double getVeloM() {
    if (unit.equalsIgnoreCase("meterps") || unit.equalsIgnoreCase("mps")) {
      return value;
    } else {
      System.err.println("Incompatible types: expected length but got " + unit);
      System.exit(1);
    }
    return -1;
  }

  public double getAccelM() {
    if (unit.equalsIgnoreCase("meterps2") || unit.equalsIgnoreCase("mps2")) {
      return value;
    } else {
      System.err.println("Incompatible types: expected length but got " + unit);
      System.exit(1);
    }
    return -1;
  }

  public Rotation2d getDistRotation() {
    if (unit.equalsIgnoreCase("degree") || unit.equalsIgnoreCase("deg")) {
      return Rotation2d.fromDegrees(value);
    } else if (unit.equalsIgnoreCase("radian") || unit.equalsIgnoreCase("rad")) {
      return Rotation2d.fromRadians(value);
    } else if (unit.equalsIgnoreCase("rotation") || unit.equalsIgnoreCase("rot")) {
      return Rotation2d.fromRotations(value);
    } else {
      System.err.println("Incompatible types: expected rotation but got " + unit);
      System.exit(1);
    }
    return new Rotation2d();
  }

  public Rotation2d getVeloRotation() {
    if (unit.equalsIgnoreCase("DPS") || unit.equalsIgnoreCase("DegPS")) {
      return Rotation2d.fromDegrees(value);
    } else if (unit.equalsIgnoreCase("RadPS")) {
      return Rotation2d.fromRadians(value);
    } else if (unit.equalsIgnoreCase("RPS") || unit.equalsIgnoreCase("RotPS")) {
      return Rotation2d.fromRotations(value);
    } else {
      System.err.println("Incompatible types: expected rotation but got " + unit);
      System.exit(1);
    }
    return new Rotation2d();
  }

  public Rotation2d getAccelRotation() {
    if (unit.equalsIgnoreCase("degps2")) {
      return Rotation2d.fromDegrees(value);
    } else if (unit.equalsIgnoreCase("radps2")) {
      return Rotation2d.fromRadians(value);
    } else if (unit.equalsIgnoreCase("rotps2")) {
      return Rotation2d.fromRotations(value);
    } else {
      System.err.println("Incompatible types: expected rotation but got " + unit);
      System.exit(1);
    }
    return new Rotation2d();
  }
}
