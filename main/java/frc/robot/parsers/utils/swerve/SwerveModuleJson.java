package frc.robot.parsers.utils.swerve;

import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.parsers.utils.DeviceJson;

public class SwerveModuleJson {
  /**
   * Module id number
   */
  public int id;
  /**
   * Drive motor.
   */
  public DeviceJson drive;
  /**
   * Steer motor.
   */
  public DeviceJson steer;
  /**
   * Absolute encoder.
   */
  public DeviceJson encoder;
  /**
   * Absolute encoder.
   */
  public double halfTrackWidthIn;
  /**
   * Absolute encoder.
   */
  public double halfTrackLengthIn;

  /**
   * Encoder offset
   */
  public double encoderOffset;

  public double getEncoderOffset() {
    // Do unit stuff here
    return encoderOffset;
  }

  public Translation2d getModuleLocation() {
    // Do unit stuff here
    return new Translation2d(halfTrackLengthIn, halfTrackWidthIn);
  }
}
