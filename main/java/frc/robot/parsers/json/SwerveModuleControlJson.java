package frc.robot.parsers.json;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.robot.constants.RobotAltModes;
import frc.robot.parsers.SwerveParser;
import frc.robot.parsers.utils.PIDFConstantsJson;

public class SwerveModuleControlJson {
  public String moduleType;
  public PIDFConstantsJson driveControl;
  public PIDFConstantsJson steerControl;
  public PIDFConstantsJson simDriveControl;
  public PIDFConstantsJson simSteerControl;

  public TalonFXConfiguration getDriveConfiguration() {
    TalonFXConfiguration driveConfiguration = new TalonFXConfiguration();

    // TODO: Move and factor into an enum or class
    /* Swerve Drive Motor Configuration */

    driveConfiguration.Slot0 = RobotAltModes.isSim
        ? simDriveControl.getPIDFConstants().toCTRESlot0Configuration()
        : driveControl.getPIDFConstants().toCTRESlot0Configuration();
    driveConfiguration.CurrentLimits = SwerveParser.ElectricalConf.DRIVE_CURRENT_LIMITS_CONFIGS;

    // TODO: Avoid circular reference
    return driveConfiguration;
  }

  public TalonFXConfiguration getSteerConfiguration() {
    TalonFXConfiguration steerConfiguration = new TalonFXConfiguration();

    /* Swerve Steer Motor Configurations */
    steerConfiguration.Slot0 = RobotAltModes.isSim
        ? simSteerControl.getPIDFConstants().toCTRESlot0Configuration()
        : steerControl.getPIDFConstants().toCTRESlot0Configuration();
    steerConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Coast;

    steerConfiguration.CurrentLimits = SwerveParser.ElectricalConf.STEER_CURRENT_LIMITS_CONFIGS;

    return steerConfiguration;
  }
}
