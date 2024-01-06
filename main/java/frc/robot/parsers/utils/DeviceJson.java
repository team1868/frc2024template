package frc.robot.parsers.utils;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.hardware.TalonFX;

/**
 * Device JSON parsed class. Used to access the JSON data.
 */
public class DeviceJson {
  /**
   * The device type, e.g. pigeon/pigeon2/sparkmax/talonfx/navx
   */
  public String type;
  /**
   * The CAN ID or pin ID of the device.
   */
  public int id;
  /**
   * The CAN bus name which the device resides on if using CAN.
   */
  public String canbus = "";

  public Pigeon2 getPigeon2() {
    assert (type.equalsIgnoreCase("pigeon2"));
    return new Pigeon2(id, canbus);
  }

  public TalonFX getTalonFX() {
    assert (type.equalsIgnoreCase("falcon") || type.equalsIgnoreCase("kraken"));
    return new TalonFX(id, canbus);
  }

  public CANcoder getCancoder() {
    assert (type.equalsIgnoreCase("cancoder"));
    return new CANcoder(id, canbus);
  }
}
