package frc.robot.utils;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

public class KinematicLimits extends VelocityLimits2d {
  public double maxTranslationalAccelerationMPS2;
  public Rotation2d maxAngularAccelerationPS2;

  public TrapezoidProfile.Constraints getTranslationalTrapezoidalContraints() {
    return new TrapezoidProfile.Constraints(
        maxTranslationalVelocityMPS, maxTranslationalAccelerationMPS2
    );
  }

  public TrapezoidProfile.Constraints getAngularTrapezoidalContraints() {
    return new TrapezoidProfile.Constraints(
        maxAngularVelocityPS.getRadians(), maxAngularAccelerationPS2.getRadians()
    );
  }

  public TrapezoidProfile.Constraints getAngularTrapezoidalContraintsDeg() {
    return new TrapezoidProfile.Constraints(
        maxAngularVelocityPS.getDegrees(), maxAngularAccelerationPS2.getDegrees()
    );
  }

  public SlewRateLimiter getTranslationalSlewRateLimiter() {
    return new SlewRateLimiter(
        maxTranslationalAccelerationMPS2, -maxTranslationalAccelerationMPS2, 0.0
    );
  }

  public SlewRateLimiter getAngularSlewRateLimiter() {
    return new SlewRateLimiter(
        maxAngularAccelerationPS2.getRadians(), -maxAngularAccelerationPS2.getRadians(), 0.0
    );
  }

  public SlewRateLimiter getAngularSlewRateLimiterDeg() {
    return new SlewRateLimiter(
        maxAngularAccelerationPS2.getDegrees(), -maxAngularAccelerationPS2.getDegrees(), 0.0
    );
  }

  public KinematicLimits(
      double maxTranslationalVelocityMPS,
      double maxTranslationalAccelerationMPS2,
      Rotation2d maxAngularVelocityPS,
      Rotation2d maxAngularAccelerationPS2
  ) {
    super(maxTranslationalVelocityMPS, maxAngularVelocityPS);
    this.maxTranslationalAccelerationMPS2 = maxTranslationalAccelerationMPS2;
    this.maxAngularAccelerationPS2 = maxAngularAccelerationPS2;
  }

  public static KinematicLimits fromDegrees(
      double maxTranslationalVelocityMPS,
      double maxTranslationalAccelerationMPS2,
      double maxAngularVelocityDEGPS,
      double maxAngularAccelerationDEGPS2
  ) {
    return new KinematicLimits(
        maxTranslationalVelocityMPS,
        maxTranslationalAccelerationMPS2,
        Rotation2d.fromDegrees(maxAngularVelocityDEGPS),
        Rotation2d.fromDegrees(maxAngularAccelerationDEGPS2)
    );
  }

  public static KinematicLimits fromRadians(
      double maxTranslationalVelocityMPS,
      double maxTranslationalAccelerationMPS2,
      double maxAngularVelocityRadPS,
      double maxAngularAccelerationPS2
  ) {
    return new KinematicLimits(
        maxTranslationalVelocityMPS,
        maxTranslationalAccelerationMPS2,
        Rotation2d.fromRadians(maxAngularVelocityRadPS),
        Rotation2d.fromRadians(maxAngularAccelerationPS2)
    );
  }
}
