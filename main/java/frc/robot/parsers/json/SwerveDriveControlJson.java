package frc.robot.parsers.json;

import edu.wpi.first.math.filter.SlewRateLimiter;
import frc.robot.parsers.utils.PIDFConstantsJson;
import frc.robot.parsers.utils.UnitJson;
import frc.robot.parsers.utils.swerve.SwerveTrapezoidalLimitsJson;

public class SwerveDriveControlJson {
  public SwerveTrapezoidalLimitsJson defaultLimits;
  public SwerveTrapezoidalLimitsJson defaultTrapezoidalLimits;

  public UnitJson translationSlewingRate;
  public UnitJson angularSlewingRate;

  public UnitJson translationalTolerance;
  public UnitJson rotationalTolerance;

  public PIDFConstantsJson translationalControl;
  public PIDFConstantsJson rotationalControl;
  public PIDFConstantsJson autoTranslationalControl;
  public PIDFConstantsJson autoRotationalControl;

  public SlewRateLimiter getTranslationalSlewRateLimiter() {
    return new SlewRateLimiter(translationSlewingRate.getAccelM());
  }

  public SlewRateLimiter getAngularSlewRateLimiter() {
    return new SlewRateLimiter(angularSlewingRate.getAccelRotation().getRotations());
  }
}
