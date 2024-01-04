package frc.robot.utils;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.robot.constants.Constants;

public class PIDConstants {
  public final double p, i, d;
  public PIDConstants(double p, double i, double d) {
    this.p = p;
    this.i = i;
    this.d = d;
  }

  public static PIDConstants fromDegrees(double p, double i, double d) {
    return new PIDConstants(p, i, d);
  }

  // public com.pathplanner.lib.util.PIDConstants getPathPlannerTheta() {
  //   return new com.pathplanner.lib.util.PIDConstants(p, i, d, Constants.LOOP_PERIOD_S);
  // }

  // public com.pathplanner.lib.util.PIDConstants getPathPlannerTranslation() {
  //   return new com.pathplanner.lib.util.PIDConstants(p, i, d, Constants.LOOP_PERIOD_S);
  // }

  public ProfiledPIDController getProfiledController(TrapezoidProfile.Constraints constraints) {
    return new ProfiledPIDController(p, i, d, constraints, Constants.LOOP_PERIOD_S);
  }
}
