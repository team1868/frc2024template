// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleArrayLogEntry;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.commands.base.GoToPoseCommand;
import frc.robot.commands.base.TeleopSwerveCommand;
import frc.robot.commands.complex.ResetAllCommand;
import frc.robot.constants.Constants;
import frc.robot.constants.RobotAltModes;
import frc.robot.constants.RobotVersions;
import frc.robot.constants.enums.AutonomousAction;
import frc.robot.constants.enums.AutonomousRoutines;
import frc.robot.constants.enums.DriveModes;
import frc.robot.constants.enums.DrivetrainControl;
import frc.robot.constants.enums.LedColors;
import frc.robot.subsystems.*;
import frc.robot.utils.GeometricUtils;
import frc.robot.utils.LoopTimer;
import frc.robot.utils.PIDConstants;
import java.util.Map;

public class RobotContainer {
  /* --- Shared Resources --- */
  private Field2d _field = new Field2d();

  /* --- Subsystems --- */
  public Controlboard _controlboard = new Controlboard(_field);
  public Drivetrain _drivetrain = new Drivetrain(_field);

  /* --- Commands --- */
  // Synchronization commands
  private Command _resetAllCommand = new ResetAllCommand(_drivetrain);

  // Driver controller feedback
  private InstantCommand _xboxRumbleCommand = _controlboard.driverRumbleCommand();
  private InstantCommand _xboxResetRumbleCommand = _controlboard.driverResetRumbleCommand();

  // Drivetrain
  private TeleopSwerveCommand _teleopSwerveCommand =
      new TeleopSwerveCommand(_drivetrain, _controlboard, DriveModes.FIELD_RELATIVE);
  private InstantCommand _zeroGyroCommand = _drivetrain.zeroGyroCommand();
  private InstantCommand _zeroPoseCommand = _drivetrain.zeroPoseCommand();
  private Command _forceChargingWheelDirection = _drivetrain.forceChargingWheelDirectionCommand();

  // Go to pose testing
  private GoToPoseCommand _goToPoseCommandPos1 =
      new GoToPoseCommand(_drivetrain, new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(180.0)));
  private GoToPoseCommand _goToPoseCommandPos2 =
      new GoToPoseCommand(_drivetrain, new Pose2d(-2.0, -2.0, Rotation2d.fromDegrees(0.0)));
  private InstantCommand _forceSetInitialPose = _drivetrain.forceSetInitalPoseCommand();

  /* Shuffleboard */
  // Auto and Path Planner
  private Alliance _alliance = Alliance.Blue;
  private SendableChooser<Integer> _autoChooser = new SendableChooser<Integer>();

  public AutonomousRoutines _curAutoSelected = AutonomousRoutines.DEFAULT_AUTO;

  private Map<String, Command> _eventMap;
  private AutonomousRoutines[] _autonModes;

  // Verbose mode
  // private double[] _yawPitchRoll = new double[3];
  private double[] _curPoses = new double[4];
  private double[] _visionPoses = new double[4];

  private DataLog _log = DataLogManager.getLog();
  private DoubleArrayLogEntry _poseLog = new DoubleArrayLogEntry(_log, "/Swerve/RobotPose");
  private DoubleArrayLogEntry _visionPoseLog = new DoubleArrayLogEntry(_log, "/Swerve/VisionPose");
  private DoubleArrayLogEntry _gyroLog = new DoubleArrayLogEntry(_log, "/Swerve/Gyro");
  private DoubleArrayLogEntry _swerveSetpoints = new DoubleArrayLogEntry(_log, "/Swerve/Setpoints");
  private DoubleArrayLogEntry _swerveOutputs = new DoubleArrayLogEntry(_log, "/Swerve/RealOutputs");

  private DoubleLogEntry _posXLog = new DoubleLogEntry(_log, "/Drive/X");
  private DoubleLogEntry _posYLog = new DoubleLogEntry(_log, "/Drive/Y");
  private DoubleLogEntry _posThetaLog = new DoubleLogEntry(_log, "/Drive/Theta");

  public RobotContainer() {
    var setupTab = Shuffleboard.getTab("Setup");

    // Auto Chooser
    setupTab.add("Auto Chooser", _autoChooser).withSize(3, 2);
    _autoChooser.setDefaultOption("NO AUTONOMOUS", AutonomousRoutines.DEFAULT_AUTO.ordinal());

    // Default commands
    _drivetrain.setDefaultCommand(_teleopSwerveCommand);

    registerAutonomousCommands();
    registerAutonomousRoutines();

    configureBindings();
    configShuffleboard();

    LoopTimer.markCompletion("\n Robot Initialized: ");
  }

  private void registerAutonomousCommands() {
    /* --- Print marker commands --- */
    // Use these to log data

    /* --- Wait Commands --- */
    // Use these for parallel deadline stop points to stop the drivetrain from moving

    /* --- LED Commands --- */

    _eventMap = AutonomousAction.getEventMap();
  }

  private void registerAutonomousRoutines() {
    if (_autonModes == null) {
      _autonModes = AutonomousRoutines.values();
      for (AutonomousRoutines routine : _autonModes) {
        if (routine.showInDashboard) {
          routine.build(_drivetrain, _controlboard);
          _autoChooser.addOption(routine.shuffleboardName, routine.ordinal());
        }
      }
    }
  }

  public AutonomousRoutines getAutonomousRoutineSelection() {
    var result = _autoChooser.getSelected();
    return _autonModes[result == null ? 0 : result.intValue()];
  }

  public void configFMSData() {
    _alliance = DriverStation.getAlliance().orElse(_alliance);
    _drivetrain.updateAlliance(_alliance);
  }

  public void configShuffleboard() {
    /* Put commands for pit testing here */
    ShuffleboardTab tuningTab = Shuffleboard.getTab("Tuning tab");
  }

  public void robotInit() {
    configFMSData();
    DataLogManager.start();
    DataLogManager.logNetworkTables(false);
    if (RobotAltModes.isVerboseMode) {
      DriverStation.startDataLog(_log);
    }
  }
  public void autonomousInit() {}

  public void teleopInit() {
    _teleopSwerveCommand.teleopInit();
    resetAll();
  }

  public void simulationInit() {
    _drivetrain.simulationInit();
  }

  public void log() {
    if (RobotAltModes.isVerboseMode) {
      Pose2d curPose = _drivetrain.getPose();
      GeometricUtils.poseToArray(curPose, _curPoses);
      _poseLog.append(_curPoses);

      _posXLog.append(curPose.getX());
      _posYLog.append(curPose.getY());
      _posThetaLog.append(curPose.getRotation().getDegrees());

      // GeometricUtils.poseToArray(_drivetrain.getVisionPose(), _visionPoses);
      // _visionPoseLog.append(_visionPoses);

      // Module Config:
      // Swerve chassis FL, BL, BR, FR
      // Practice bot FL, FR, BL, BR
      _swerveOutputs.append(_drivetrain.swerveMeasuredIO());
      _swerveSetpoints.append(_drivetrain.swerveSetpointsIO());
    }
  }

  public void periodic() {
    _controlboard.updateShuffleboard();
  }

  public void disabledPeriodic() {
    AutonomousRoutines prev = _curAutoSelected;
    _curAutoSelected = getAutonomousRoutineSelection();

    Alliance prevAlliance = _alliance;
    _alliance = DriverStation.getAlliance().get() == null ? DriverStation.Alliance.Blue
                                                          : DriverStation.getAlliance().get();

    // For initial pose aligning
    if (prev != _curAutoSelected || prevAlliance != _alliance) {
      _drivetrain.updateAlliance(_alliance);

      // Pose lighting here
    }

    // Disabled periodic for subsystem
  }

  public void onEnable() {}

  public void onDisable() {
    resetAll();
    _controlboard.driverResetRumble();
  }

  public void resetAll() {
    // Upon disabling, open all control loops
    _resetAllCommand.schedule();
  }

  private void configureBindings() {
    // DRIVER
    _controlboard._xboxDrive.x().onTrue(_zeroGyroCommand);
    _controlboard._xboxDrive.start().onTrue(_zeroPoseCommand);
  }
}
