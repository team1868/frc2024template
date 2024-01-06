package frc.robot.constants;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.constants.enums.FieldVersions;

public class Constants {
  public static final int NUM_SCORING_LOCATIONS = 9;

  public static final RobotVersions CRobot = RobotVersions.SWERVE_BASE;
  public static final FieldVersions CField = FieldVersions.THEORETICAL_FIELD;

  // if we do our configurations right, this shouldn't be necessary
  public static final boolean isCompBot = false; // CRobot == RobotVersions.COMP_BOT;
  public static final boolean isPracticeBot = false; // CRobot == RobotVersions.PRACTICE_BOT;
  public static final boolean isSwerveBase = false; // CRobot == RobotVersions.SWERVE_BASE;

  public static final boolean isNASAField = CField == FieldVersions.NASA_FIELD;
  // public static final boolean isSDSMK4 = CRobot.drive.type == ModuleModels.ModuleTypes.SDS_MK4;
  // public static final boolean isSDSMK4I = CRobot.drive.type == ModuleModels.ModuleTypes.SDS_MK4I;
  public static final boolean isCANEncoder = CRobot == RobotVersions.SWERVE_BASE;

  // These are high level robot configurations that fundamentally change robot
  // behavior and control schemes, this also allows for manual overrides. I don't like this,
  // would love to see alternatives... These should really become part of the top level
  // RobotVersions

  // TODO move
  /* ========== ROBOT OFFSET DIMENSIONS ========== */
  public static final double LOOP_PERIOD_MS = 20.0;
  public static final double LOOP_PERIOD_S = Units.millisecondsToSeconds(LOOP_PERIOD_MS);

  public static final class CTREConstants {
    /* ============= Falcon Constants2 ============= */
    // ticks per motor rotation
    public static final double FALCON_ENCODER_TICKS = 2048.0;
    // TODO: measure this free speed on blocks
    // 6380 +/- 10%
    // TODO FOC?
    public static final double MAX_FALCON_RPM = 5800.0;

    public static final double CANCODER_ENCODER_TICKS = 4096.0;

    // multiply to convert Constants2
    public static final double FALCON_TO_RPS = 10.0 / FALCON_ENCODER_TICKS;
    public static final double FALCON_TO_RPM = 60.0 * FALCON_TO_RPS;
    public static final double CANCODER_TO_RPS = 10.0 / CANCODER_ENCODER_TICKS;
    public static final double CANCODER_TO_RPM = 60.0 * CANCODER_TO_RPS;
    public static final double RPS_TO_FALCON = 1.0 / FALCON_TO_RPS;
    public static final double RPM_TO_FALCON = 1.0 / FALCON_TO_RPM;
    public static final double RPS_TO_CANCODER = 1.0 / CANCODER_TO_RPS;
    public static final double RPM_TO_CANCODER = 1.0 / CANCODER_TO_RPM;
    public static final double FALCON_TO_DEG = 360.0 / FALCON_ENCODER_TICKS;
    public static final double DEG_TO_FALCON = FALCON_ENCODER_TICKS / 360.0;
    public static final double DEG_TO_CANCODER = CANCODER_ENCODER_TICKS / 360.0;
    public static final double FALCON_TO_RAD = FALCON_TO_DEG * Math.PI / 180.0;
    public static final double RAD_TO_FALCON = 180.0 / (FALCON_TO_DEG * Math.PI);
    public static final double FALCON_TICKS_TO_ROT = 1.0 / FALCON_ENCODER_TICKS;
    public static final double ROT_TO_FALCON_TICKS = FALCON_ENCODER_TICKS;
  }

  public static final class Sensors {
    public static final boolean INVERT_GYRO = false;

    public static final Rotation2d GYRO_ZERO_BLUE = Rotation2d.fromDegrees(0);
    public static final Rotation2d GYRO_ZERO_RED = GYRO_ZERO_BLUE.plus(Rotation2d.fromDegrees(180));

    public static final double POV_ZERO_BLUE_DEG = 0;
    public static final double POV_ZERO_RED_DEG = POV_ZERO_BLUE_DEG + 180;
  }

  /* For Drivetrain Auto */
  public static final double TRANSLATION_MAX_TRIM_SPEED_MPS = 1;
  public static final Rotation2d ANGLE_MAX_TRIM_SPEED_DPS = Rotation2d.fromDegrees(90.0);

  public static final class Control {
    public static final double STICK_DEADBAND = 0.1; // TODO: tune
    public static final double STICK_NET_DEADBAND = 0.125; // TODO: tune
    public static final boolean IS_OPEN_LOOP = false; // swerve
  }
}
