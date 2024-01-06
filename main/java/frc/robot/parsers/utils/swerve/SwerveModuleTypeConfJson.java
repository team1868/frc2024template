package frc.robot.parsers.utils.swerve;

import frc.robot.parsers.utils.UnitJson;

public class SwerveModuleTypeConfJson {
  public UnitJson wheelDiameter;

  public boolean invertDrive;
  public boolean invertSteer;

  public int[][] steerRatio;
  public int[][] driveRatio;

  // Calculated or derived values
  public double _calculatedSteerRatio;
  public double _calculatedDriveRatio;
  public double _wheelCircumferenceM;
  public double _motorFreeRPMToSurfaceSpeed;

  public SwerveModuleTypeConfJson resolve() {
    _calculatedSteerRatio = calculateRatio(steerRatio);
    _calculatedDriveRatio = calculateRatio(driveRatio);
    _wheelCircumferenceM = Math.PI * wheelDiameter.getLengthM();
    _motorFreeRPMToSurfaceSpeed = (1.0 / 60.0) * _wheelCircumferenceM / _calculatedDriveRatio;
    return this;
  }

  private double calculateRatio(int[][] gearPairs) {
    double ratio = 1.0;
    for (int i = 0; i < gearPairs.length; i++) {
      assert (gearPairs[i].length == 2);
      ratio = ratio * gearPairs[i][0] / gearPairs[i][1];
    }
    return ratio;
  }
}
