package frc.robot.constants.enums;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.utils.AlliancePose2d;

public enum StaticTargets {
  EXAMPLE_TARGET_NAME(null);

  AlliancePose2d target;

  StaticTargets(Pose2d blue, Pose2d red) {
    this.target = new AlliancePose2d(blue, red);
  }

  StaticTargets(AlliancePose2d target) {
    this.target = target;
  }
}
