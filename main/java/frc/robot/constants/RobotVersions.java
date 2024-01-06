package frc.robot.constants;

public enum RobotVersions {
  SWERVE_BASE("swervebaseDrive.json");

  public final String _drive;
  public final boolean _hasDrive;

  RobotVersions(String driveConfFile) {
    _drive = driveConfFile;
    _hasDrive = driveConfFile != null;
  }
}
