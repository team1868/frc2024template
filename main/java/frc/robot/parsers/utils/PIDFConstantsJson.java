package frc.robot.parsers.utils;

import edu.wpi.first.math.util.Units;
import frc.robot.utils.PIDFConstants;

public class PIDFConstantsJson {
  public double p;
  public double i;
  public double d;
  public double f;

  public PIDFConstants getPIDFConstants() {
    return new PIDFConstants(p, i, d, f);
  }
}
