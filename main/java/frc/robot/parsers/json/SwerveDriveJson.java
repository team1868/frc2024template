package frc.robot.parsers.json;

import edu.wpi.first.math.filter.SlewRateLimiter;
import frc.robot.parsers.utils.DeviceJson;
import frc.robot.parsers.utils.DrivetrainDimensionsJson;

public class SwerveDriveJson {
  public String[] moduleFiles;
  public String moduleControl;
  public DrivetrainDimensionsJson dimensions;
  public SwerveDriveControlJson drivetrainControl;
  public DeviceJson IMU;
}
