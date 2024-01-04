package frc.robot.constants.enums;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.robot.constants.RobotAltModes;
import frc.robot.utils.PIDFConstants;

public enum ModuleControl {
  FALCON_MK4_FALCON_L2(
      new PIDFConstants(0.20019550342130987, 0.0, 0.0, 0.0),
      new PIDFConstants(1.2011730205278592, 0.0, 0.024023460410557185, 0.0),
      new PIDFConstants(0.20019550342130987, 0.0, 0.0, 0.0),
      new PIDFConstants(0.40039100684261975, 0.0, 0.0, 0.0)
  ),
  FALCON_MK4I_FALCON_L2(
      new PIDFConstants(0.20019550342130987, 0.0, 0.0, 0.0),
      new PIDFConstants(1.2011730205278592 * 2.0, 0.0, 0.024023460410557185 * 2.0, 0.0),
      new PIDFConstants(0.10, 0.0, 0.0, 0.0),
      new PIDFConstants(0.4, 0.0, 0.0, 0.0)
  ),
  COLSON_FALCON_MK4_FALCON_L2(
      new PIDFConstants(0.20019550342130987, 0.0, 0.0, 0.0),
      new PIDFConstants(1.2011730205278592, 0.0, 0.024023460410557185, 0.0),
      new PIDFConstants(0.20019550342130987, 0.0, 0.0, 0.0),
      new PIDFConstants(0.40039100684261975, 0.0, 0.0, 0.0)
  ),
  COLSON_FALCON_MK4I_FALCON_L2(
      new PIDFConstants(0.20019550342130987, 0.0, 0.0, 0.0),
      new PIDFConstants(1.2011730205278592 * 2.0, 0.0, 0.024023460410557185 * 2.0, 0.0),
      new PIDFConstants(0.10, 0.0, 0.0, 0.0),
      new PIDFConstants(0.4, 0.0, 0.0, 0.0)
  );
  // https://github.com/FRCTeam2910/2021CompetitionRobot/blob/5fabbff6814a8fa71ef614f691342847ad885bf5/src/main/java/org/frcteam2910/c2020/subsystems/DrivetrainSubsystem.java

  // TODO: Move and factor into an enum or class
  /* Drive Motor Closed Loop PID Values */
  public final PIDFConstants drive;
  /* Steer Motor Closed Loop PID Values */
  public final PIDFConstants steer;
  public final PIDFConstants simDrive;
  public final PIDFConstants simSteer;
  public final TalonFXConfiguration driveConf;
  public final TalonFXConfiguration steerConf;

  ModuleControl(PIDFConstants drive, PIDFConstants steer) {
    this(drive, steer, drive, steer);
  }

  ModuleControl(
      PIDFConstants drive, PIDFConstants steer, PIDFConstants simDrive, PIDFConstants simSteer
  ) {
    this.drive = drive;
    this.steer = steer;
    this.simDrive = simDrive;
    this.simSteer = simSteer;
    driveConf = initDriveFalcon();
    steerConf = initSteerFalcon();
  }

  private TalonFXConfiguration initDriveFalcon() {
    TalonFXConfiguration driveConfiguration = new TalonFXConfiguration();

    // TODO: Move and factor into an enum or class
    /* Swerve Drive Motor Configuration */

    driveConfiguration.Slot0 = RobotAltModes.isSim ? simDrive.toCTRESlot0Configuration()
                                                   : drive.toCTRESlot0Configuration();
    driveConfiguration.CurrentLimits = ElectricalConf.DRIVE_CURRENT_LIMITS_CONFIGS;

    // TODO: Avoid circular reference
    return driveConfiguration;
  }

  private TalonFXConfiguration initSteerFalcon() {
    TalonFXConfiguration steerConfiguration = new TalonFXConfiguration();

    /* Swerve Angle Motor Configurations */
    steerConfiguration.Slot0 = RobotAltModes.isSim ? simSteer.toCTRESlot0Configuration()
                                                   : steer.toCTRESlot0Configuration();
    steerConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Coast;

    steerConfiguration.CurrentLimits = ElectricalConf.ANGLE_CURRENT_LIMITS_CONFIGS;

    return steerConfiguration;
  }

  /* Neutral Modes */
  public static final NeutralModeValue STEER_NEUTRAL_MODE = NeutralModeValue.Coast;
  public static final NeutralModeValue DRIVE_NEUTRAL_MODE = NeutralModeValue.Brake;

  // TODO: Move and factor into an enum or class
  public static final class ElectricalConf {
    /* Drive Motor Characterization Values */
    // divide by 12 to convert from volts to percent output for CTRE (not sure if
    // this is needed)
    public static final double DRIVE_KS_VOLT = 0.667 / 12;
    public static final double DRIVE_KV_VOLTPMPS = 2.44 / 12;
    public static final double DRIVE_KA_VOLTPMPS_SQ = 0.27 / 12;
    // /* Drive Motor Characterization Values */
    // public static final SimpleMotorFeedforward driveFeedforward = new
    // SimpleMotorFeedforward(0.2, 2.2201, 0.16343);
    // ???

    /* Swerve Current Limiting */
    public static final double ANGLE_CONTINUOUS_CURRENT_LIMIT = 30;
    public static final double ANGLE_PEAK_SUPPLY_CURRENT_LIMIT = 40;
    public static final double ANGLE_PEAK_STATOR_CURRENT_LIMIT = 40;

    public static final double ANGLE_PEAK_CURRENT_DURATION = 0.1;
    public static final boolean ANGLE_ENABLE_STATOR_CURRENT_LIMIT = true;
    public static final boolean ANGLE_ENABLE_SUPPLY_CURRENT_LIMIT = true;

    public static final double DRIVE_CONTINUOUS_SUPPLY_CURRENT_LIMIT = 30;
    public static final double DRIVE_PEAK_SUPPLY_CURRENT_LIMIT = 40;

    public static final double DRIVE_CONTINUOUS_STATOR_CURRENT_LIMIT = 80;
    public static final double DRIVE_PEAK_STATOR_CURRENT_LIMIT = 90;

    public static final double AUTON_DRIVE_CONTINUOUS_STATOR_CURRENT_LIMIT = 60;
    public static final double AUTON_DRIVE_PEAK_STATOR_CURRENT_LIMIT = 80;

    public static final double DRIVE_PEAK_CURRENT_DURATION = 0.1;
    public static final boolean DRIVE_ENABLE_STATOR_CURRENT_LIMIT = true;
    public static final boolean DRIVE_ENABLE_SUPPLY_CURRENT_LIMIT = true;

    public static final double DRIVE_OPEN_LOOP_RAMP = 0.25;
    public static final double DRIVE_CLOSED_LOOP_RAMP = 0.0;

    public static final double AUTONOMOUS_MOTOR_VOLTAGE_COMPENSATION_SCALE = 12.0;

    // TODO: Move and factor into an enum or class
    public static final CurrentLimitsConfigs DRIVE_CURRENT_LIMITS_CONFIGS =
        new CurrentLimitsConfigs()
            .withStatorCurrentLimitEnable(DRIVE_ENABLE_STATOR_CURRENT_LIMIT)
            .withStatorCurrentLimit(DRIVE_PEAK_STATOR_CURRENT_LIMIT)
            .withSupplyCurrentLimitEnable(DRIVE_ENABLE_SUPPLY_CURRENT_LIMIT)
            .withSupplyCurrentLimit(DRIVE_PEAK_SUPPLY_CURRENT_LIMIT)
            .withSupplyCurrentThreshold(DRIVE_CONTINUOUS_SUPPLY_CURRENT_LIMIT)
            .withSupplyTimeThreshold(DRIVE_PEAK_CURRENT_DURATION);

    public static final CurrentLimitsConfigs AUTO_DRIVE_CURRENT_LIMITS_CONFIGS =
        new CurrentLimitsConfigs()
            .withStatorCurrentLimitEnable(DRIVE_ENABLE_STATOR_CURRENT_LIMIT)
            .withStatorCurrentLimit(DRIVE_PEAK_STATOR_CURRENT_LIMIT)
            .withSupplyCurrentLimitEnable(DRIVE_ENABLE_SUPPLY_CURRENT_LIMIT)
            .withSupplyCurrentLimit(DRIVE_PEAK_SUPPLY_CURRENT_LIMIT)
            .withSupplyCurrentThreshold(DRIVE_CONTINUOUS_SUPPLY_CURRENT_LIMIT)
            .withSupplyTimeThreshold(DRIVE_PEAK_CURRENT_DURATION);

    public static final CurrentLimitsConfigs ANGLE_CURRENT_LIMITS_CONFIGS =
        new CurrentLimitsConfigs()
            .withStatorCurrentLimitEnable(ANGLE_ENABLE_STATOR_CURRENT_LIMIT)
            .withStatorCurrentLimit(ANGLE_PEAK_STATOR_CURRENT_LIMIT)
            .withSupplyCurrentLimitEnable(ANGLE_ENABLE_SUPPLY_CURRENT_LIMIT)
            .withSupplyCurrentLimit(ANGLE_PEAK_SUPPLY_CURRENT_LIMIT)
            .withSupplyCurrentThreshold(ANGLE_CONTINUOUS_CURRENT_LIMIT)
            .withSupplyTimeThreshold(ANGLE_PEAK_CURRENT_DURATION);
  }
}
