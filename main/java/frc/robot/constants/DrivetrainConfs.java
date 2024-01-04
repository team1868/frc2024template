package frc.robot.constants;

import frc.robot.Ports.DrivetrainPorts;
import frc.robot.constants.enums.*;

public enum DrivetrainConfs {
  COMP_BOT_CONFS(
      DrivetrainDimensions.COMP_BOT_DIMENSIONS,
      DrivetrainPorts.COMP_PORTS,
      DrivetrainControl.COMP_CONTROL,
      4,
      ModuleModels.SDS_MK4I_L2,
      ModuleControl.COLSON_FALCON_MK4I_FALCON_L2,
      new double[] {
          (360.0 - 186.38) / 360.0,
          (360.0 - 91.41) / 360.0,
          (360.0 - 199.27) / 360.0,
          (360.0 - 52.54) / 360.0}
  ),
  PRACTICE_BOT_CONFS(
      DrivetrainDimensions.PRACTICE_BOT_DIMENSIONS,
      DrivetrainPorts.PRACTICE_PORTS,
      DrivetrainControl.PRACTICE_CONTROL,
      4,
      ModuleModels.SDS_MK4I_L2,
      ModuleControl.FALCON_MK4I_FALCON_L2,
      new double[] {
          359.64 / 360.0,
          208.98 / 360.0,
          291.9 / 360.0,
          77.42 / 360.0,
      }
  ),
  SWERVE_BASE_CONFS(
      DrivetrainDimensions.SWERVE_BASE_DIMENSIONS,
      DrivetrainPorts.SWERVE_BASE_PORTS,
      DrivetrainControl.SWERVE_BASE_CONTROL,
      4,
      ModuleModels.SDS_MK4_L2,
      ModuleControl.FALCON_MK4_FALCON_L2,
      // new double[] {(1-0.202637) * 360.0, (1 - 0.456787) * 360.0, (1 - 0.381836) * 360.0,(1 -
      // 0.561523) * 360.0} new double[] {(1-0.202637) * 360.0, (1 - 0.456787) * 360.0, (1 -
      // 0.381836) * 360.0,(1 - 0.561523) * 360.0}
      new double[] {//   (1 - 0.8483611111) * 360.0,
                    //   (1 - 0.9777830556) * 360.0,
                    //   (1 - 0.9438472222) * 360.0,
                    //   (1 - 0.0292966667) * 360.0

                    (1.0 - 0.8483611111),
                    (1.0 - 0.9777830556),
                    (1.0 - 0.9438472222 + 0.01),
                    (1.0 - 0.0292966667 - 0.005)}
  );

  // Drivetrain Configurations
  public final DrivetrainDimensions dims;
  public final DrivetrainPorts ports;
  public final DrivetrainControl control;

  /* Module Configurations */
  public final ModuleModels model;
  public final ModuleControl moduleControl;
  public final int numModules;
  public final double[] moduleOffsetsRotation;
  public final ModuleModels.ModuleTypes type;
  public final ModuleModels.ModuleDriveRatios ratio;
  public final double alignmentToleranceDeg;
  public final double theoreticalMaxTranslationSpeed;
  public final double theoreticalMaxRotationalSpeed;

  DrivetrainConfs(
      DrivetrainDimensions dims,
      DrivetrainPorts ports,
      DrivetrainControl control,
      int numModules,
      ModuleModels model,
      ModuleControl moduleControl,
      double[] offsets
  ) {
    this(dims, ports, control, numModules, model, moduleControl, offsets, 5.0);
  }

  DrivetrainConfs(
      DrivetrainDimensions dims,
      DrivetrainPorts ports,
      DrivetrainControl control,
      int numModules,
      ModuleModels model,
      ModuleControl moduleControl,
      double[] offsets,
      double alignmentToleranceDeg
  ) {
    assert (numModules == ports.drive.length);
    assert (numModules == ports.steer.length);
    assert (numModules == ports.encoder.length);

    this.dims = dims;
    this.ports = ports;
    this.control = control;
    this.numModules = numModules;
    this.model = model;
    this.moduleControl = moduleControl;
    this.type = model.type;
    this.ratio = model.ratio;
    this.alignmentToleranceDeg = alignmentToleranceDeg;

    assert (offsets.length == numModules);
    moduleOffsetsRotation = offsets;

    this.theoreticalMaxTranslationSpeed = model.theoreticalMaxWheelSpeed;

    double location_m = Math.hypot(dims.halfTrackLength_M, dims.halfTrackWidth_M);
    this.theoreticalMaxRotationalSpeed = 2 * Math.PI * location_m / model.theoreticalMaxWheelSpeed;
  };
}
