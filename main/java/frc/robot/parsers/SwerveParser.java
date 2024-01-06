package frc.robot.parsers;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.fasterxml.jackson.databind.ObjectMapper;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.parsers.json.SwerveDriveJson;
import frc.robot.parsers.json.SwerveModuleControlJson;
import frc.robot.parsers.utils.DeviceJson;
import frc.robot.parsers.utils.swerve.SwerveModuleJson;
import frc.robot.parsers.utils.swerve.SwerveModuleTypeConfJson;
import frc.robot.subsystems.SwerveModule;
import frc.robot.utils.FileUtils;
import frc.robot.utils.ModuleIO;
import java.io.File;
import java.io.IOException;
/**
 * Helper class used to parse the JSON directory with specified configuration options.
 */
public class SwerveParser {
  // Parsed files
  public SwerveDriveJson swerveConf;
  public SwerveModuleControlJson moduleControlConf;
  public SwerveModuleTypeConfJson moduleTypeConf;
  public SwerveModuleJson[] moduleConfs;

  // Parsed or derived values
  public double DRIVE_MOTOR_FREE_RPM = 6080.0;

  public int _numModules;
  public double _theoreticalMaxWheelSpeedMPS;
  public double _theoreticalMaxTranslationSpeedMPS;
  public double _theoreticalMaxRotationalSpeedDPS;

  // local
  private SwerveModule[] _modules = null;
  private Translation2d[] _swerveModuleLocations = null;

  /**
   * Construct a swerve parser. Will throw an error if there is a missing file.
   *
   * @param directory Directory with swerve configurations.
   * @throws IOException if a file doesn't exist.
   */
  public SwerveParser(File dir, String filename) {
    try {
      checkDirectoryStructure(dir);

      File drivetrainFile = new File(dir, filename);
      // Get the module files (and other associated config files for specified drivetrain)
      FileUtils.checkForFile(drivetrainFile);
      swerveConf = new ObjectMapper().readValue(drivetrainFile, SwerveDriveJson.class);

      File moduleControlFile = new File(dir, "modules/" + swerveConf.moduleControl);
      FileUtils.checkForFile(moduleControlFile);
      moduleControlConf =
          new ObjectMapper().readValue(moduleControlFile, SwerveModuleControlJson.class);

      File moduleTypeFile = new File(dir, "module_properties/" + moduleControlConf.moduleType);
      FileUtils.checkForFile(moduleTypeFile);
      moduleTypeConf =
          new ObjectMapper().readValue(moduleTypeFile, SwerveModuleTypeConfJson.class).resolve();

      _numModules = swerveConf.moduleFiles.length;
      moduleConfs = new SwerveModuleJson[swerveConf.moduleFiles.length];
      boolean[] idUsage = new boolean[swerveConf.moduleFiles.length];
      for (int i = 0; i < swerveConf.moduleFiles.length; i++) {
        File moduleFile = new File(dir, "modules/" + swerveConf.moduleFiles[i]);
        FileUtils.checkForFile(moduleFile);
        var module = new ObjectMapper().readValue(moduleFile, SwerveModuleJson.class);

        // Ensure module id uniqueness while sorting the modules by id number
        assert (!idUsage[module.id]);
        idUsage[module.id] = true;
        moduleConfs[module.id] = module;
      }

      _theoreticalMaxWheelSpeedMPS =
          DRIVE_MOTOR_FREE_RPM * moduleTypeConf._motorFreeRPMToSurfaceSpeed;

    } catch (Exception err) {
      System.err.println(err);
      System.exit(1);
    }
  }

  /**
   * Check directory structure.
   *
   * @param directory JSON Configuration Directory
   */
  private void checkDirectoryStructure(File directory) {
    File modules = new File(directory, "modules");
    assert modules.exists() && modules.isDirectory();

    File properties = new File(directory, "modules_properties");
    assert properties.exists() && properties.isDirectory();
  }

  public SwerveModule[] getSwerveModules() {
    if (_modules == null) {
      _modules = new SwerveModule[moduleConfs.length];
      for (int i = 0; i < moduleConfs.length; i++) {
        _modules[i] = new SwerveModule(
            i,
            moduleConfs[i],
            moduleTypeConf,
            moduleControlConf,
            _theoreticalMaxWheelSpeedMPS,
            new ModuleIO() {}
        );
      }
    }
    return _modules;
  }
  public Translation2d[] getSwerveModuleLocations() {
    if (_swerveModuleLocations == null) {
      for (int i = 0; i < moduleConfs.length; i++) {
        _swerveModuleLocations[i] = moduleConfs[i].getModuleLocation();
      }
    }
    return _swerveModuleLocations;
  }

  /* Neutral Modes */
  public static final NeutralModeValue STEER_NEUTRAL_MODE = NeutralModeValue.Coast;
  public static final NeutralModeValue DRIVE_NEUTRAL_MODE = NeutralModeValue.Brake;

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
    public static final double STEER_CONTINUOUS_CURRENT_LIMIT = 30;
    public static final double STEER_PEAK_SUPPLY_CURRENT_LIMIT = 40;
    public static final double STEER_PEAK_STATOR_CURRENT_LIMIT = 40;

    public static final double STEER_PEAK_CURRENT_DURATION = 0.1;
    public static final boolean STEER_ENABLE_STATOR_CURRENT_LIMIT = true;
    public static final boolean STEER_ENABLE_SUPPLY_CURRENT_LIMIT = true;

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

    public static final CurrentLimitsConfigs STEER_CURRENT_LIMITS_CONFIGS =
        new CurrentLimitsConfigs()
            .withStatorCurrentLimitEnable(STEER_ENABLE_STATOR_CURRENT_LIMIT)
            .withStatorCurrentLimit(STEER_PEAK_STATOR_CURRENT_LIMIT)
            .withSupplyCurrentLimitEnable(STEER_ENABLE_SUPPLY_CURRENT_LIMIT)
            .withSupplyCurrentLimit(STEER_PEAK_SUPPLY_CURRENT_LIMIT)
            .withSupplyCurrentThreshold(STEER_CONTINUOUS_CURRENT_LIMIT)
            .withSupplyTimeThreshold(STEER_PEAK_CURRENT_DURATION);
  }
}
