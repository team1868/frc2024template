package frc.robot.constants;

public enum RobotVersions {
  COMP_BOT(DrivetrainConfs.COMP_BOT_CONFS
           // should just be a file path per subsystem
  ),
  PRACTICE_BOT(DrivetrainConfs.PRACTICE_BOT_CONFS),
  SWERVE_BASE(DrivetrainConfs.SWERVE_BASE_CONFS),
  LEY(DrivetrainConfs.COMP_BOT_CONFS),
  TEST_BOARD(null);

  public final DrivetrainConfs drive;

  RobotVersions(DrivetrainConfs drive) {
    this.drive = drive;
  }
}
