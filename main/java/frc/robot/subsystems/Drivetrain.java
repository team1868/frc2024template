package frc.robot.subsystems;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest.SwerveControlRequestParameters;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.*;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.GenericPublisher;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Threads;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Ports.DrivetrainPorts;
import frc.robot.Robot;
import frc.robot.constants.Constants;
import frc.robot.constants.Constants.Control;
import frc.robot.constants.RobotAltModes;
import frc.robot.constants.enums.*;
import frc.robot.parsers.SwerveParser;
import frc.robot.parsers.json.SwerveDriveControlJson;
import frc.robot.parsers.json.SwerveDriveJson;
import frc.robot.parsers.utils.DrivetrainDimensionsJson;
import frc.robot.utils.GyroIO;
import frc.robot.utils.GyroIOInputsAutoLogged;
import frc.robot.utils.InputUtils;
import frc.robot.utils.LoopTimer;
import frc.robot.utils.ModuleIO;
import frc.robot.utils.PIDFConstants;
import java.util.concurrent.locks.ReadWriteLock;
import java.util.concurrent.locks.ReentrantReadWriteLock;
import java.util.function.Consumer;
import org.littletonrobotics.junction.Logger;

public class Drivetrain extends SubsystemBase {
  // /* --- Constant configuration shortcuts --- */
  // private final DrivetrainConfs DRIVE_CONSTS = Constants.CRobot.drive;
  // private boolean disabled = DRIVE_CONSTS == null;
  // private final DrivetrainDimensions DRIVE_DIMS = disabled ? null : DRIVE_CONSTS.dims;
  // private final DrivetrainControl DRIVE_CONTROL = disabled ? null : DRIVE_CONSTS.control;
  // private final DrivetrainPorts DRIVE_PORTS = disabled ? null : DRIVE_CONSTS.ports;

  private boolean wantOrientationMode = false;
  private final boolean USE_POSE_ESTIMATION_ANGLE = false; // change if true?

  /* --- Sensors, motors, and hardware --- */
  private Pigeon2 _gyro;
  private final GyroIO _gyroIO;
  private final GyroIOInputsAutoLogged _gyroInputs = new GyroIOInputsAutoLogged();
  private Pigeon2Configuration _gyroConfigs = new Pigeon2Configuration();
  // private final StatusSignal<Double> _yaw = _gyro.getYaw();
  // private final Queue<Double> _yawPositionQueue;
  // private final StatusSignal<Double> _yawVelocity = _gyro.getAngularVelocityZ();

  // Thread odometry resources
  private final OdometryThread _odometryThread;
  protected final ReadWriteLock _odometryLock = new ReentrantReadWriteLock();
  private double _averageOdometryLoopTime = 0.0;

  // 604 Scrub utils and odometry
  private Pose2d m_targetPose;
  private double m_xVelRef = 0.0;
  private double m_yVelRef = 0.0;
  private double m_thetaVelRef = 0.0;
  private double m_allowedScrub = 0.0;

  private SwerveModule[] _modules;

  /* --- State and Physical Property variables --- */
  private ChassisSpeeds _chassisSpeeds = new ChassisSpeeds();
  // Initialize with real values regardless of canbus state to ensure good launch to pose
  // estimator
  private SwerveModulePosition[] _modulePositions;

  /* --- Odometry Utils --- */
  // TODO: move to configurations
  private Translation2d[] _swerveModuleLocations;
  private edu.wpi.first.math.kinematics.SwerveDriveKinematics _swerveKinematics =
      new edu.wpi.first.math.kinematics.SwerveDriveKinematics(_swerveModuleLocations);

  SwerveDrivePoseEstimator _robotPoseEstimator = new SwerveDrivePoseEstimator(
      _swerveKinematics, Rotation2d.fromDegrees(0.0), _modulePositions, new Pose2d()
  );
  SwerveDriveOdometry _odom =
      new SwerveDriveOdometry(_swerveKinematics, getYaw(), _modulePositions, new Pose2d());

  // weird stuff for threaded implementation
  protected SwerveControlRequestParameters _requestParameters =
      new SwerveControlRequestParameters();
  protected Rotation2d _fieldRelativeOffset;
  protected SwerveRequest _requestToApply = new SwerveRequest.Idle();

  /**
   * Plain-Old-Data class holding the state of the swerve drivetrain.
   * This encapsulates most data that is relevant for telemetry or
   * decision-making from the Swerve Drive.
   */
  public class SwerveDriveState {
    public int SuccessfulDaqs;
    public int FailedDaqs;
    public Pose2d Pose;
    public Pose2d EstimatedPose;
    public SwerveModuleState[] ModuleStates;
    public double OdometryPeriod;
  }

  protected Consumer<SwerveDriveState> _telemetryFunction = null;
  protected SwerveDriveState _cachedState = new SwerveDriveState();
  protected final StatusSignal<Double> _yawGetter = _gyro.getYaw().clone();
  protected final StatusSignal<Double> _angularZGetter = _gyro.getAngularVelocityZ().clone();

  /* --- Game State variables --- */
  private Field2d _field;
  private Alliance _alliance;
  private boolean _isAuto = false;
  private boolean _autoPrepScore = false;

  /* --- Control Utils --- */
  private ProfiledPIDController _xController;
  private ProfiledPIDController _yController;
  private ProfiledPIDController _angleController;
  private ProfiledPIDController _visionCenterOffsetController;
  private SlewRateLimiter _xSlewRateFilter;
  private SlewRateLimiter _ySlewRateFilter;
  private SlewRateLimiter _angleSlewRateFilter;

  /* --- Simulation resources and variables --- */
  private Pose2d _simPose = new Pose2d();

  /* --- Logging variables --- */
  private double[] _desiredSwerveStates;
  private double[] _currentSwerveStates;

  /* --- Shuffleboard entries --- */
  private SendableChooser<StaticTargets> _staticTargetChooser =
      new SendableChooser<StaticTargets>();
  private GenericEntry _desiredSpeed, _actualSpeed;
  private GenericEntry _actualSpeedX, _actualSpeedY, _actualSpeedTheta;
  private GenericEntry _desiredRobotTheta, _actualRobotTheta, _errorRobotTheta;

  private GenericPublisher _xPoseError, _yPoseError, _thetaPoseError;
  private GenericEntry _usingPoseTuning;

  private GenericEntry _anglePFac, _angleIFac, _angleDFac;
  private GenericEntry _xPFac, _xIFac, _xDFac;
  private GenericEntry _yPFac, _yIFac, _yDFac;

  private GenericEntry _steerTargetAngle, _steerErrorAngle;
  private GenericEntry _steerAnglePFac, _steerAngleIFac, _steerAngleDFac;
  private GenericEntry _steerPFac, _steerIFac, _steerDFac;

  private Pose2d TARGET_RELATIVE_POSE = new Pose2d(0.0, 1.0, Rotation2d.fromDegrees(0.0));
  public SwerveParser _conf;

  private PIDFConstants xy;
  private PIDFConstants theta;

  public Drivetrain(SwerveParser conf, Field2d field, GyroIO gyroIO) {
    _field = field;
    _gyroIO = gyroIO;
    _conf = conf;

    _gyro = conf.swerveConf.IMU.getPigeon2();
    _modules = conf.getSwerveModules();
    _modulePositions = new SwerveModulePosition[] {
        _modules[0].getPosition(),
        _modules[1].getPosition(),
        _modules[2].getPosition(),
        _modules[3].getPosition()};
    _swerveModuleLocations = conf.getSwerveModuleLocations();
    _desiredSwerveStates = new double[2 * _conf._numModules];
    _currentSwerveStates = new double[2 * _conf._numModules];

    xy = conf.swerveConf.drivetrainControl.translationalControl.getPIDFConstants();
    theta = conf.swerveConf.drivetrainControl.rotationalControl.getPIDFConstants();

    /* --- Control Utils --- */
    _xController = xy.getProfiledController(
        _conf.swerveConf.drivetrainControl.defaultTrapezoidalLimits.translation.getLimitsM()
    );
    _yController = xy.getProfiledController(
        _conf.swerveConf.drivetrainControl.defaultTrapezoidalLimits.translation.getLimitsM()
    );
    _angleController = theta.getProfiledController(
        _conf.swerveConf.drivetrainControl.defaultTrapezoidalLimits.angular.getLimitsRad()
    );
    // TODO: set limits

    // _visionCenterOffsetController =
    //     _conf.swerveConf.getVisionProfiledPIDController(getProfileConstraints());

    _xSlewRateFilter = _conf.swerveConf.drivetrainControl.getTranslationalSlewRateLimiter();
    _ySlewRateFilter = _conf.swerveConf.drivetrainControl.getTranslationalSlewRateLimiter();
    _angleSlewRateFilter = _conf.swerveConf.drivetrainControl.getAngularSlewRateLimiter();

    resetDefaultTolerance();
    _angleController.enableContinuousInput(0.0, Units.degreesToRadians(360.0));
    // because ryan introduced a bug, need to reintro to the configs
    _angleController.setTolerance(1.0 * 2.0 * Math.PI / 360.0);
    if (Robot.isReal()) {
      int counter = 0;
      while (!checkInitStatus()) {
        System.out.println("DRIVETRAIN Check Init Status : " + counter++);
      }
    } else {
    }

    _gyro.getConfigurator().apply(_gyroConfigs, 0);
    _gyro.getConfigurator().setYaw(0.0, 0);
    // _yaw.setUpdateFrequency(250.0);
    // _yawVelocity.setUpdateFrequency(250.0);
    // _gyro.optimizeBusUtilization();
    // _yawPositionQueue = PhoenixOdometryThread.getInstance().registerSignal(_gyro,
    // _gyro.getYaw());

    zeroGyro();
    configShuffleboard();
    // initialize all cameras
    // _multiCameraController = new MultiCameraController(_field, _robotPoseEstimator);

    if (RobotAltModes.isPoseTuning) {
      // _xPoseError;
      // _yPoseEntry;
      // _thetaPoseError;
      // _usingPoseTuning;
    }

    if (RobotAltModes.isPIDTuningMode) {
      //  _anglePFac;
      //  _angleIFac;
      //  _angleDFac;
      //  _xPFac;
      //  _xIFac;
      //  _xDFac;
      //  _yPFac;
      //  _yIFac;
      //  _yDFac;
      //  _steerTargetAngle
      //  _steerAnglePFac;
      //  _steerAngleIFac;
      //  _steerAngleDFac;
      //  _steerPFac;
      //  _steerIFac;
      //  _steerDFac;
    }

    _fieldRelativeOffset = new Rotation2d();
    m_targetPose = getPose();
    _odometryThread = new OdometryThread(_modules);
    if (RobotAltModes.kEnableThreadedOdometry) {
      _odometryThread.start();
    }
    LoopTimer.markEvent(" Drivetrain Initialization Complete: ");
  }

  @Override
  public void periodic() {
    LoopTimer.markLoopStart();

    if (RobotAltModes.kEnableThreadedOdometry) {
      double averageOdometryLoopTime = _cachedState.OdometryPeriod;
      // System.out.println("Average odometry loop time: " + averageOdometryLoopTime);
    } else {
      // get module positions ensures the order of data beign collected
      _robotPoseEstimator.updateWithTime(Timer.getFPGATimestamp(), getYaw(), getModulePositions());
    }

    LoopTimer.markEvent(" PoseEstimator ");

    try {
      _odometryLock.writeLock().lock();
      // _multiCameraController.updateOdom(_isAuto);
    } finally {
      _odometryLock.writeLock().unlock();
    }

    LoopTimer.markEvent(" Vision ");

    updateShuffleboard();

    if (!RobotAltModes.kEnableThreadedOdometry) {
      // TODO: call an update function and pre-construct the array
      _chassisSpeeds = _swerveKinematics.toChassisSpeeds(
          _modules[0].getState(),
          _modules[1].getState(),
          _modules[2].getState(),
          _modules[3].getState()
      );
    } else {
      System.out.println(getPose());
    }

    // Logging
    _gyroIO.updateInputs(_gyroInputs);
    Logger.processInputs("Drive/Gyro", _gyroInputs);

    LoopTimer.markCompletion(" Drivetrain Shuffleboard ", "\n Total Drivetrain ");
  }

  public void updateModulePositions() {
    for (int i = 0; i < _conf._numModules; i++) {
      _modulePositions[i] = _modules[i].getPosition();
    }
  }

  public SwerveModulePosition[] getModulePositions() {
    updateModulePositions();
    return _modulePositions;
  }

  public void setAutoMode(boolean enable) {
    _isAuto = enable;
  }

  public void configShuffleboard() {
    ShuffleboardTab tab = Shuffleboard.getTab("SwerveModules");
    _steerTargetAngle = tab.add("target angle", 0.0).getEntry();

    ShuffleboardTab drivetrainTab = Shuffleboard.getTab("Drivetrain");

    if (RobotAltModes.isPoseTuning) {
      ShuffleboardLayout poseLayout = drivetrainTab.getLayout("Go To Pose", BuiltInLayouts.kGrid)
                                          .withSize(1, 2)
                                          .withPosition(7, 0);

      _usingPoseTuning = poseLayout.add("using pose tuning", false).withPosition(0, 0).getEntry();
      _xPoseError = poseLayout.add("x Pose Error", 0.0).withPosition(0, 1).getEntry();
      _yPoseError = poseLayout.add("y Pose Error", 0.0).withPosition(0, 2).getEntry();
      _thetaPoseError = poseLayout.add("theta Pose Error", 0.0).withPosition(0, 3).getEntry();
    }

    if (RobotAltModes.isPIDTuningMode) {
      int PIDCOL = 3;
      int PIDROW = 1;

      _errorRobotTheta = drivetrainTab.add("Error Angle", 0.0).getEntry();
      ShuffleboardLayout anglePIDLayout = drivetrainTab.getLayout("Angle PID", BuiltInLayouts.kGrid)
                                              .withSize(1, 2)
                                              .withPosition(PIDCOL, PIDROW);
      _anglePFac = anglePIDLayout.add("P", theta.p).withPosition(0, 0).getEntry();
      _angleIFac = anglePIDLayout.add("I", theta.i).withPosition(0, 1).getEntry();
      _angleDFac = anglePIDLayout.add("D", theta.d).withPosition(0, 2).getEntry();

      ShuffleboardLayout xLayout = drivetrainTab.getLayout("X PID", BuiltInLayouts.kGrid)
                                       .withSize(1, 2)
                                       .withPosition(PIDCOL + 1, PIDROW);
      _xPFac = xLayout.add("P", xy.p).withPosition(0, 0).getEntry();
      _xIFac = xLayout.add("I", xy.i).withPosition(0, 1).getEntry();
      _xDFac = xLayout.add("D", xy.d).withPosition(0, 2).getEntry();

      ShuffleboardLayout yLayout = drivetrainTab.getLayout("Y PID", BuiltInLayouts.kGrid)
                                       .withSize(1, 2)
                                       .withPosition(PIDCOL + 2, PIDROW);
      _yPFac = yLayout.add("P", xy.p).withPosition(0, 0).getEntry();
      _yIFac = yLayout.add("I", xy.i).withPosition(0, 1).getEntry();
      _yDFac = yLayout.add("D", xy.d).withPosition(0, 2).getEntry();
    }

    ShuffleboardTab controlboardTab = Shuffleboard.getTab("Competition HUD");
    controlboardTab.add("Field", _field).withSize(11, 5).withPosition(1, 1);
  }

  public void updateShuffleboard() {
    for (SwerveModule module : _modules) {
      module.updateShuffleboard();
    }

    // _actualAngle->SetDouble(ToAbsoluteAngle(GetYaw()));

    // _actualSpeedX->SetDouble(_chassisSpeeds.vx.value());
    // _actualSpeedY->SetDouble(_chassisSpeeds.vy.value());
    // _actualAngularVelocity->SetDouble(_chassisSpeeds.omega.value());

    if (RobotAltModes.isVisionMode) {
      // _multiCameraController.updateShuffleboard();
    }

    var pose = getPose();
    _field.setRobotPose(pose.getX(), pose.getY(), pose.getRotation());

    if (RobotAltModes.isPIDTuningMode) {
      _errorRobotTheta.setDouble(Units.degreesToRadians(_angleController.getPositionError()));
    }
  }

  public void simulationInit() {
    if (RobotAltModes.isSim) {
      for (SwerveModule module : _modules) module.simulationInit();
    }
  }

  public void simulationPeriodic() {
    if (RobotAltModes.isSim) {
      for (SwerveModule module : _modules) module.simulationPeriodic();

      _simPose = _simPose.transformBy(new Transform2d(
          new Translation2d(
              _chassisSpeeds.vxMetersPerSecond * Constants.LOOP_PERIOD_S,
              _chassisSpeeds.vyMetersPerSecond * Constants.LOOP_PERIOD_S
          ),
          Rotation2d.fromRadians(_chassisSpeeds.omegaRadiansPerSecond * Constants.LOOP_PERIOD_S)
      ));

      _gyro.getSimState().setRawYaw(_simPose.getRotation().getDegrees());
    }
  }

  public boolean isRedAlliance() {
    return _alliance == Alliance.Red;
  }

  public Rotation2d getYaw() {
    return Rotation2d.fromDegrees(_gyro.getYaw().getValueAsDouble());
  }

  public Rotation2d getPitch() {
    return Rotation2d.fromDegrees(_gyro.getPitch().getValueAsDouble());
  }

  public Rotation2d getRoll() {
    return Rotation2d.fromDegrees(_gyro.getRoll().getValueAsDouble());
  }

  public Pose2d getPose() {
    if (RobotAltModes.kEnableThreadedOdometry) {
      try {
        _odometryLock.readLock().lock();
        return RobotAltModes.kPoseEstimator ? _cachedState.EstimatedPose : _cachedState.Pose;
      } finally {
        _odometryLock.readLock().unlock();
      }
    } else {
      return _robotPoseEstimator.getEstimatedPosition();
    }
  }

  public double[] swerveMeasuredIO() {
    for (int i = 0; i < _conf._numModules; i++) {
      SwerveModuleState moduleState = _modules[i].getState();
      _currentSwerveStates[i * 2] = moduleState.angle.getDegrees();
      _currentSwerveStates[(i * 2) + 1] = moduleState.speedMetersPerSecond;
    }
    return _currentSwerveStates;
  }

  public double[] swerveSetpointsIO() {
    return _desiredSwerveStates;
  }

  public void setDesiredSwerveState(SwerveModuleState[] goalModuleStates) {
    for (int i = 0; i < _conf._numModules; i++) {
      SwerveModuleState state = goalModuleStates[i];
      _desiredSwerveStates[i * 2] = state.angle.getDegrees();
      _desiredSwerveStates[(i * 2) + 1] = state.speedMetersPerSecond;
    }
  }

  public edu.wpi.first.math.kinematics.SwerveDriveKinematics getSwerveKinematics() {
    return _swerveKinematics;
  }

  public boolean areWheelsAligned(SwerveModuleState[] goalStates) {
    for (int i = 0; i < _conf._numModules; i++) {
      if (!_modules[i].isAlignedTo(goalStates[i]))
        return false;
    }
    return true;
  }

  public boolean areWheelsAligned(SwerveModuleState goalState) {
    for (int i = 0; i < _conf._numModules; i++) {
      if (!_modules[i].isAlignedTo(goalState))
        return false;
    }
    return true;
  }

  public void resetModulesToAbsolute() {
    for (SwerveModule module : _modules) {
      module.resetToAbsolute();
    }
  }

  public void alignModulesToZero() {
    for (SwerveModule module : _modules) {
      module.alignToZero();
    }
  }

  public void updateAlliance(Alliance alliance) {
    _alliance = alliance;
  }

  public double POVZeroOffsetDeg() {
    return _alliance == Alliance.Red ? Constants.Sensors.POV_ZERO_RED_DEG
                                     : Constants.Sensors.POV_ZERO_BLUE_DEG;
  }

  public Rotation2d allianceGyroAngle() {
    return _alliance == Alliance.Red ? Constants.Sensors.GYRO_ZERO_RED
                                     : Constants.Sensors.GYRO_ZERO_BLUE;
  }

  public void zeroGyro() {
    Pose2d pose = getPose();
    setPose(pose == null ? new Pose2d() : pose, allianceGyroAngle());
  }

  public void zeroPose() {
    setPose(new Pose2d());
  }

  public void autoZeroGyro() {
    setPose(getPose(), getYaw().plus(Rotation2d.fromDegrees(POVZeroOffsetDeg())));
  }

  public void setPose(Pose2d pose, Rotation2d yaw) {
    _gyro.setYaw(yaw.getDegrees());
    setOdometryPose(pose, yaw);
  }

  public void setPose(Pose2d pose) {
    setOdometryPose(pose, getYaw());
  }

  private void setOdometryPose(Pose2d pose, Rotation2d yaw) {
    try {
      _odometryLock.writeLock().lock();
      _robotPoseEstimator.resetPosition(
          yaw, _modulePositions, new Pose2d(pose.getTranslation(), yaw)
      );
      _odom.resetPosition(yaw, _modulePositions, new Pose2d(pose.getTranslation(), yaw));
    } finally {
      _odometryLock.writeLock().unlock();
    }
  }

  public boolean motorResetConfig() {
    for (SwerveModule module : _modules) {
      if (module.motorResetConfig())
        return true;
    }
    return false;
  }

  public void setAutoPrepScore(boolean enable) {
    _autoPrepScore = enable;
  }

  public boolean getAutoPrepScore() {
    return _autoPrepScore;
  }

  /* Used by SwerveControllerCommand in Auto */
  public void setModuleStates(SwerveModuleState[] desiredStates, double maxSpeed) {
    // Desaturate based of max theoretical or functional rather than current max
    edu.wpi.first.math.kinematics.SwerveDriveKinematics.desaturateWheelSpeeds(
        desiredStates, _conf._theoreticalMaxWheelSpeedMPS
    );

    for (SwerveModule module : _modules) {
      module.setDesiredState(desiredStates[module._moduleNumber], Constants.Control.IS_OPEN_LOOP);
    }

    setDesiredSwerveState(desiredStates);
  }

  public void setModuleStates(SwerveModuleState[] desiredStates) {
    setModuleStates(
        desiredStates,
        _conf.swerveConf.drivetrainControl.defaultLimits.translation.getLimitsM().maxVelocity
    );
  }

  public void drive(
      double joystickX,
      double joystickY,
      double compositeXY,
      double joystickTheta,
      DriveModes mode,
      double maxSpeedMPS,
      Rotation2d maxAngularSpeed
  ) {
    switch (mode) {
      case ROBOT_CENTRIC:
        robotCentricDrive(
            InputUtils.scaleJoystickXMPS(joystickX, compositeXY, maxSpeedMPS),
            InputUtils.scaleJoystickYMPS(joystickY, compositeXY, maxSpeedMPS),
            InputUtils.ScaleJoystickThetaRadPS(joystickTheta, maxAngularSpeed)
        );
        break;
      case FIELD_RELATIVE:
        fieldRelativeDrive(
            InputUtils.scaleJoystickXMPS(joystickX, compositeXY, maxSpeedMPS),
            InputUtils.scaleJoystickYMPS(joystickY, compositeXY, maxSpeedMPS),
            InputUtils.ScaleJoystickThetaRadPS(joystickTheta, maxAngularSpeed)
        );
        break;
      case FIELD_RELATIVE_ROTATION_COMPENSATION_SCALING:
        rotationCompensationFieldRelativeDrive(
            InputUtils.scaleJoystickXMPS(joystickX, compositeXY, 1.0),
            InputUtils.scaleJoystickYMPS(joystickY, compositeXY, 1.0),
            InputUtils.ScaleJoystickThetaRadPS(joystickTheta, 1.0)
        );
      case SNAP_TO_ANGLE:
        snapToAngleDrive(
            InputUtils.scaleJoystickXMPS(joystickX, compositeXY, maxSpeedMPS),
            InputUtils.scaleJoystickYMPS(joystickY, compositeXY, maxSpeedMPS)
        );
        break;
      case SNAKE:
        snakeDrive(joystickX, joystickY, compositeXY, maxSpeedMPS);
        break;
      case TARGET_RELATIVE:
        targetCentricDrive(
            InputUtils.scaleJoystickXMPS(joystickX, compositeXY, maxSpeedMPS),
            InputUtils.scaleJoystickYMPS(joystickY, compositeXY, maxSpeedMPS),
            InputUtils.ScaleJoystickThetaRadPS(joystickTheta, maxAngularSpeed),
            new Translation2d(Constants.CField.dims.halfX_M, Constants.CField.dims.halfY_M),
            maxSpeedMPS
        );
        break;
      case CHASE_STATIC_TARGET:
        chaseStaticTargetDrive(maxSpeedMPS);
        break;
      case SLEWING_FIELD_RELATIVE:
        slewingFieldRelativeDrive(
            InputUtils.scaleJoystickXMPS(joystickX, compositeXY, maxSpeedMPS),
            InputUtils.scaleJoystickYMPS(joystickY, compositeXY, maxSpeedMPS),
            InputUtils.ScaleJoystickThetaRadPS(joystickTheta, maxAngularSpeed)
        );
        break;
      case FIELD_RELATIVE_SKEW_COMPENSATION:
        fieldRelativeDrive(
            InputUtils.scaleJoystickXMPS(joystickX, compositeXY, maxSpeedMPS),
            InputUtils.scaleJoystickYMPS(joystickY, compositeXY, maxSpeedMPS),
            InputUtils.ScaleJoystickThetaRadPS(joystickTheta, maxAngularSpeed)
        );
        break;
      case CHASE_DYNAMIC_TARGET:
        // TODO update this function to use the AprilTag layout
        // ChaseDynamicTargetDrive(FIELD_TO_TARGET, TARGET_RELATIVE_POSE, true, maxSpeed);
        // ChaseDynamicTargetDrive(FIELD_TO_TARGET, TARGET_RELATIVE_POSE, _hasTarget);
        break;
      default:
        // ERROR
        // throw, error message, or default behavior
        // return;
        break;
    }
  }

  public void drive(double joystickX, double joystickY, double joystickTheta) {
    drive(
        joystickX,
        joystickY,
        Math.hypot(joystickX, joystickY),
        joystickTheta,
        DriveModes.FIELD_RELATIVE
    );
  }

  public void drive(double joystickX, double joystickY, double joystickTheta, DriveModes mode) {
    drive(joystickX, joystickY, Math.hypot(joystickX, joystickY), joystickTheta, mode);
  }

  public void drive(double joystickX, double joystickY, double compositeXY, double joystickTheta) {
    drive(joystickX, joystickY, compositeXY, joystickTheta, DriveModes.FIELD_RELATIVE);
  }

  public void drive(
      double joystickX, double joystickY, double compositeXY, double joystickTheta, DriveModes mode
  ) {
    drive(
        joystickX,
        joystickY,
        compositeXY,
        joystickTheta,
        mode,
        _conf.swerveConf.drivetrainControl.defaultLimits.translation.getLimitsM().maxVelocity
    );
  }

  public void drive(
      double joystickX,
      double joystickY,
      double compositeXY,
      double joystickTheta,
      DriveModes mode,
      double maxSpeedMPS
  ) {
    drive(
        joystickX,
        joystickY,
        compositeXY,
        joystickTheta,
        mode,
        maxSpeedMPS,
        new Rotation2d(
            _conf.swerveConf.drivetrainControl.defaultLimits.angular.getLimitsRad().maxVelocity
        )
    );
  }

  // individual drive functions
  public void robotCentricDrive(
      double translationXMPS, double translationYMPS, double rotationRadPS, double maxSpeed
  ) {
    SwerveModuleState[] goalModuleStates = _swerveKinematics.toSwerveModuleStates(
        new ChassisSpeeds(translationXMPS, translationYMPS, rotationRadPS)
    );

    // Desaturate based of max theoretical or functional rather than current max
    edu.wpi.first.math.kinematics.SwerveDriveKinematics.desaturateWheelSpeeds(
        goalModuleStates, _conf._theoreticalMaxWheelSpeedMPS
    );

    for (SwerveModule module : _modules) {
      module.setDesiredState(
          goalModuleStates[module._moduleNumber], Constants.Control.IS_OPEN_LOOP
      );
    }
    setDesiredSwerveState(goalModuleStates);
  }

  public void robotCentricDrive(
      double translationXMPS, double translationYMPS, double rotationRadPS
  ) {
    robotCentricDrive(
        translationXMPS,
        translationYMPS,
        rotationRadPS,
        _conf.swerveConf.drivetrainControl.defaultLimits.translation.getLimitsM().maxVelocity
    );
  }

  public void robotCentricDrive(ChassisSpeeds speeds) {
    robotCentricDrive(
        speeds.vxMetersPerSecond,
        speeds.vyMetersPerSecond,
        speeds.omegaRadiansPerSecond,
        _conf.swerveConf.drivetrainControl.defaultLimits.translation.getLimitsM().maxVelocity
    );
  }

  public void fieldRelativeDrive(ChassisSpeeds speeds) {
    fieldRelativeDrive(
        speeds.vxMetersPerSecond, speeds.vyMetersPerSecond, speeds.omegaRadiansPerSecond
    );
  }

  public boolean isOversaturated(SwerveModuleState[] states) {
    for (SwerveModuleState state : states) {
      if (state.speedMetersPerSecond > _conf._theoreticalMaxWheelSpeedMPS)
        return true;
    }
    return false;
  }

  public void fieldRelativeDrive(
      double translationXMPS, double translationYMPS, double rotationRadPS
  ) {
    SwerveModuleState[] goalModuleStates =
        _swerveKinematics.toSwerveModuleStates(ChassisSpeeds.fromFieldRelativeSpeeds(
            translationXMPS,
            translationYMPS,
            rotationRadPS,
            USE_POSE_ESTIMATION_ANGLE ? getPose().getRotation() : getYaw()
        ));

    // Desaturate based of max theoretical or functional rather than current max
    edu.wpi.first.math.kinematics.SwerveDriveKinematics.desaturateWheelSpeeds(
        goalModuleStates, _conf._theoreticalMaxWheelSpeedMPS
    );

    for (SwerveModule module : _modules) {
      module.setDesiredState(
          goalModuleStates[module._moduleNumber], Constants.Control.IS_OPEN_LOOP
      );
    }
    //    _desiredSpeed.setDouble(goalModuleStates[0].speedMetersPerSecond);
    setDesiredSwerveState(goalModuleStates);
  }

  public void rotationCompensationFieldRelativeDrive(
      double percentX, double percentY, double percentR
  ) {
    // args
    double configuredMaxTranslationalSpeed = 0;
    double configuredMaxRotationalSpeed = 0;

    // consts
    // these need to be robot relative if guaranteeing the angle
    ChassisSpeeds robotRelative =
        ChassisSpeeds.fromFieldRelativeSpeeds(percentX, percentY, percentR, getYaw());
    // robotRelative.
    double scaledX = robotRelative.vxMetersPerSecond * configuredMaxTranslationalSpeed;
    double scaledY = robotRelative.vyMetersPerSecond * configuredMaxTranslationalSpeed;
    double percentXY = Math.hypot(percentX, percentY);
    double scaledXY = percentXY * configuredMaxTranslationalSpeed;
    double absScaledX = Math.abs(scaledX);
    double absScaledY = Math.abs(scaledY);
    double scaledR = robotRelative.omegaRadiansPerSecond * configuredMaxRotationalSpeed;
    double rWheel =
        scaledR * configuredMaxTranslationalSpeed / _conf._theoreticalMaxRotationalSpeedDPS;
    double projected =
        (Math.min(absScaledX, absScaledY) * 2 + Math.abs(absScaledX - absScaledY)) / Math.sqrt(2.0);

    // We should rescale
    if (projected + rWheel > _conf._theoreticalMaxTranslationSpeedMPS) {
      // // projected oversaturation values (using projected vector value)
      // // no because this is dependent on the direction of translation
      // // which will change with constant input
      // double maxOverSaturation =
      //     (-1 + (configuredMaxRotationalSpeed / DRIVE_CONSTS.theoreticalMaxRotationalSpeed)
      //      + (configuredMaxTranslationalSpeed /
      //      DRIVE_CONSTS.theoreticalMaxTranslationSpeed));
      // double overSaturation =
      //     -1 + (projected + rWheel) / DRIVE_CONSTS.theoreticalMaxTranslationSpeed;
      // double scalingCoefficient = overSaturation / maxOverSaturation;

      // unprojected value
      double scalingCoefficient = -1 + (scaledXY / _conf._theoreticalMaxTranslationSpeedMPS)
          + (scaledR / _conf._theoreticalMaxRotationalSpeedDPS);
      double newScaledXY = scaledXY
          + (_conf._theoreticalMaxTranslationSpeedMPS - configuredMaxTranslationalSpeed) * percentXY
              * scalingCoefficient;
      double newScaledR =
          scaledR + (_conf._theoreticalMaxRotationalSpeedDPS - configuredMaxRotationalSpeed);
      scaledX = scaledX * newScaledXY / scaledXY;
      scaledY = scaledY * newScaledXY / scaledXY;
      scaledR = newScaledR;
    }

    fieldRelativeDrive(scaledX, scaledY, scaledR);
  }

  public void slewingFieldRelativeDrive(
      double translationXMPS, double translationYMPS, double rotationRadPS
  ) {
    fieldRelativeDrive(
        _xSlewRateFilter.calculate(translationXMPS),
        _ySlewRateFilter.calculate(translationYMPS),
        _angleSlewRateFilter.calculate(rotationRadPS)
    );
  }

  public void snapToAngleDrive(double translationXMPS, double translationYMPS, double maxSpeedMPS) {
    fieldRelativeDrive(
        translationXMPS,
        translationYMPS,
        // calculate snap to angle rotation here
        // TODO: reconfigure deadband logic to limit jittering
        _angleController.atGoal() ? 0.0 : _angleController.calculate(getYaw().getRadians())
    );
  }

  public void snapToAngleDrive(double translationXMPS, double translationYMPS) {
    snapToAngleDrive(
        translationXMPS,
        translationYMPS,
        _conf.swerveConf.drivetrainControl.defaultLimits.translation.getLimitsM()
            .maxVelocity // TODO apply units
    );
  }

  public void snakeDrive(
      double controllerX, double controllerY, double compositeXY, double maxSpeedMPS
  ) {
    if (compositeXY > Control.STICK_NET_DEADBAND) {
      _angleController.reset(
          getPose().getRotation().getRadians(), _chassisSpeeds.omegaRadiansPerSecond
      );
      if (controllerX == 0.0) {
        _angleController.setGoal(
            controllerY > 0.0 ? Units.degreesToRadians(90.0) : Units.degreesToRadians(270.0)
        );
      } else {
        _angleController.setGoal(Math.atan2(controllerY, controllerX));
      }
    }

    snapToAngleDrive(
        InputUtils.scaleJoystickXMPS(controllerX, compositeXY, maxSpeedMPS),
        InputUtils.scaleJoystickYMPS(controllerY, compositeXY, maxSpeedMPS),
        maxSpeedMPS
        // InputUtils.scaleJoystickYMPS(controllerY, compositeXY, maxSpeedMPS),
        // DRIVE_CONTROL.defaultLimits.maxTranslationalVelocityMPS
    );
  }

  public void snakeDrive(double controllerX, double controllerY) {
    snakeDrive(controllerX, controllerY, Math.hypot(controllerX, controllerY));
  }

  public void snakeDrive(double translationXMPS, double translationYMPS, double compositeXY) {
    snakeDrive(
        translationXMPS,
        translationYMPS,
        compositeXY,
        _conf.swerveConf.drivetrainControl.defaultLimits.translation.getLimitsM()
            .maxVelocity // TODO apply units
    );
  }

  public void fieldRelativeDriveCompensation(
      double translationXMPS, double translationYMPS, double rotationRadPS, double maxSpeedMPS
  ) {
    double x_meter = translationXMPS * Constants.LOOP_PERIOD_S;
    double y_meter = translationYMPS * Constants.LOOP_PERIOD_S;
    double theta_deg = rotationRadPS * Constants.LOOP_PERIOD_S;
    Pose2d goal = new Pose2d(x_meter, y_meter, Rotation2d.fromDegrees(theta_deg));
    Twist2d twistTranslation = new Pose2d().log(goal);
    fieldRelativeDrive(
        twistTranslation.dx / Constants.LOOP_PERIOD_S,
        twistTranslation.dy / Constants.LOOP_PERIOD_S,
        twistTranslation.dtheta / Constants.LOOP_PERIOD_S
    );
  }

  public void fieldRelativeDriveCompensation(
      double translationXMPS, double translationYMPS, double rotationRadPS
  ) {
    fieldRelativeDriveCompensation(
        translationXMPS,
        translationYMPS,
        rotationRadPS,
        _conf.swerveConf.drivetrainControl.defaultLimits.translation.getLimitsM().maxVelocity
    );
  }

  public void setTolerance(double xTolerance, double yTolerance, Rotation2d thetaTolerance) {
    _xController.setTolerance(xTolerance);
    _yController.setTolerance(yTolerance);
    _angleController.setTolerance(thetaTolerance.getRadians());
  }

  public void resetDefaultTolerance() {
    setTolerance(
        _conf.swerveConf.drivetrainControl.translationalTolerance.getLengthM(),
        _conf.swerveConf.drivetrainControl.translationalTolerance.getLengthM(),
        _conf.swerveConf.drivetrainControl.rotationalTolerance.getDistRotation()
    );
  }

  /*
   * @param translationX, translationY apply to entire robot
   * @param rotation measured from center of rotation
   * @param target point of rotation
   * @param maxSpeed
   */
  public void targetCentricDrive(
      double translationX,
      double translationY,
      double rotation,
      Translation2d target,
      double maxSpeed
  ) {
    SwerveModuleState[] goalModuleStates = _swerveKinematics.toSwerveModuleStates(
        new ChassisSpeeds(translationX, translationY, rotation), target
    );

    // Desaturate based of max theoretical or functional rather than current max
    edu.wpi.first.math.kinematics.SwerveDriveKinematics.desaturateWheelSpeeds(
        goalModuleStates, _conf._theoreticalMaxWheelSpeedMPS
    );

    for (SwerveModule module : _modules) {
      module.setDesiredState(
          goalModuleStates[module._moduleNumber], Constants.Control.IS_OPEN_LOOP
      );
    }
    setDesiredSwerveState(goalModuleStates);
  }

  public void chaseStaticTargetDrive(double maxSpeedMPS) {
    Pose2d currentPose = getPose();

    if (RobotAltModes.isUnprofiledPIDMode) {
      // TODO Test/replace with slewing drive (?)
      fieldRelativeDrive(
          _xController.atSetpoint() ? 0.0 : _xController.calculate(currentPose.getX()),
          _yController.atSetpoint() ? 0.0 : _yController.calculate(currentPose.getY()),
          _angleController.atSetpoint() ? 0.0 : _angleController.calculate(getYaw().getRadians())
      );
    } else {
      // Profiled controllers naturally filter/slew
      if (RobotAltModes.isPoseTuning) {
        _xPoseError.setDouble(_xController.getPositionError());
        _yPoseError.setDouble(_yController.getPositionError());
        _thetaPoseError.setDouble(Units.radiansToDegrees(_angleController.getPositionError()));
      }

      fieldRelativeDrive(
          _xController.calculate(currentPose.getX()),
          _yController.calculate(currentPose.getY()),
          _angleController.calculate(getYaw().getRadians())
      );
    }
  }

  public void chaseStaticTargetDrive() {
    chaseStaticTargetDrive(
        _conf.swerveConf.drivetrainControl.defaultLimits.translation.getLimitsM().maxVelocity
    );
  }

  public void forceWheelChaseStaticTargetDrive(double maxSpeedMPS) {
    Pose2d currentPose = getPose();

    if (RobotAltModes.isUnprofiledPIDMode) {
      // TODO Test/replace with slewing drive (?)
      forceWheelDirection(
          _xController.atSetpoint() ? 0 : _xController.calculate(currentPose.getX()),
          _yController.atSetpoint() ? 0 : _yController.calculate(currentPose.getY()),
          _angleController.atSetpoint() ? 0 : _angleController.calculate(getYaw().getRadians()),
          maxSpeedMPS
      );
    } else {
      // Profiled controllers naturally filter/slew
      forceWheelDirection(
          _xController.calculate(currentPose.getX()),
          _yController.calculate(currentPose.getY()),
          _angleController.calculate(getYaw().getRadians()),
          maxSpeedMPS
      );
    }
  }

  public void forceWheelChaseStaticTargetDrive() {
    forceWheelChaseStaticTargetDrive(
        _conf.swerveConf.drivetrainControl.defaultLimits.translation.getLimitsM().maxVelocity
    );
  }

  public void chaseDynamicTargetDrive(
      Pose2d visionTarget, Pose2d relativeGoal, boolean targetMoved, double maxSpeedMPS
  ) {
    if (targetMoved) {
      Transform2d goalPose = visionTarget.minus(relativeGoal);

      if (RobotAltModes.isUnprofiledPIDMode) {
        _xController.reset(goalPose.getX());
        _yController.reset(goalPose.getY());
        _angleController.reset(goalPose.getRotation().getRadians());
      } else {
        Pose2d currentPose = getPose();
        _xController.reset(currentPose.getX(), _chassisSpeeds.vxMetersPerSecond);
        _yController.reset(currentPose.getY(), _chassisSpeeds.vyMetersPerSecond);
        _angleController.reset(
            currentPose.getRotation().getRadians(), _chassisSpeeds.omegaRadiansPerSecond
        );
        _xController.setGoal(goalPose.getX());
        _yController.setGoal(goalPose.getY());
        _angleController.setGoal(goalPose.getRotation().getRadians());
      }
    }
    chaseStaticTargetDrive(maxSpeedMPS);
  }

  public void chaseDynamicTargetDrive(
      Pose2d visionTarget, Pose2d relativeGoal, boolean targetMoved
  ) {
    chaseDynamicTargetDrive(
        visionTarget, relativeGoal, targetMoved, _conf._theoreticalMaxWheelSpeedMPS
    );
  }

  public void forceWheelDirection(
      double translationXMPS, double translationYMPS, double rotationRadPS, double maxSpeedMPS
  ) {
    SwerveModuleState[] goalModuleStates =
        _swerveKinematics.toSwerveModuleStates(ChassisSpeeds.fromFieldRelativeSpeeds(
            translationXMPS, translationYMPS, rotationRadPS, getYaw()
        ));

    for (int i = 0; i < _conf._numModules; i++) _modules[i].setLastAngle(goalModuleStates[i].angle);

    if (areWheelsAligned(goalModuleStates)) {
      // Desaturate based of max theoretical or functional rather than current max
      edu.wpi.first.math.kinematics.SwerveDriveKinematics.desaturateWheelSpeeds(
          goalModuleStates, _conf._theoreticalMaxWheelSpeedMPS
      );

      for (SwerveModule module : _modules) {
        module.setDesiredState(
            goalModuleStates[module._moduleNumber], Constants.Control.IS_OPEN_LOOP
        );
      }

      // _desiredSpeed->SetDouble(goalModuleStates[0].speed());
      setDesiredSwerveState(goalModuleStates);
    } else {
      fieldRelativeDrive(0.0, 0.0, 0.0);
      // _desiredSpeed->SetDouble(0.0);
    }
  }

  public void forceWheelDirection(
      double translationXMPS, double translationYMPS, double rotationRadPS
  ) {
    forceWheelDirection(
        translationXMPS, translationYMPS, rotationRadPS, _conf._theoreticalMaxWheelSpeedMPS
    );
  }

  public void forceWheelDirectionDrive(double moduleAngleDeg, double speedMPS, double maxSpeedMPS) {
    speedMPS = Math.min(speedMPS, maxSpeedMPS);
    Rotation2d moduleAngle = Rotation2d.fromDegrees(moduleAngleDeg);
    SwerveModuleState goalModuleState = new SwerveModuleState(speedMPS, moduleAngle);

    for (SwerveModule module : _modules) module.setLastAngle(moduleAngle);

    if (areWheelsAligned(goalModuleState)) {
      for (SwerveModule module : _modules)
        module.setDesiredState(goalModuleState, Constants.Control.IS_OPEN_LOOP);

      // _desiredSpeed->SetDouble(goalModuleState.speed());
    } else {
      fieldRelativeDrive(0.0, 0.0, 0.0);
      // _desiredSpeed->SetDouble(0.0);
    }
  }

  public void forceWheelDirectionDrive(double moduleAngleDeg, double speedMPS) {
    forceWheelDirectionDrive(
        moduleAngleDeg,
        speedMPS,
        _conf.swerveConf.drivetrainControl.defaultLimits.translation.getLimitsM().maxVelocity
    );
  }

  public void setSnapAngleDeg(double angleDeg) {
    setSnapAngleRad(Units.degreesToRadians(angleDeg));
  }

  public void setSnapAngle(Rotation2d angle) {
    setSnapAngleRad(angle.getRadians());
  }

  public void setSnapScoringAngle() {
    // setSnapAngleRad(allianceScoreAngle().getRadians());
  }

  public void setSnapAngleRad(double angleRad) {
    _angleController.reset(
        getPose().getRotation().getRadians(), _chassisSpeeds.omegaRadiansPerSecond
    );
    _angleController.setGoal(angleRad);
  }

  public void setStaticTarget(Pose2d goalPose) {
    Pose2d currentPose = getPose();
    _xController.reset(currentPose.getX(), _chassisSpeeds.vxMetersPerSecond);
    _yController.reset(currentPose.getY(), _chassisSpeeds.vyMetersPerSecond);
    _angleController.reset(
        currentPose.getRotation().getRadians(), _chassisSpeeds.omegaRadiansPerSecond
    );
    _xController.setGoal(goalPose.getX());
    _yController.setGoal(goalPose.getY());
    _angleController.setGoal(goalPose.getRotation().getRadians());
  }

  public Rotation2d toAbsoluteAngle(Rotation2d angle) {
    return toAbsoluteAngle(angle);
  }

  public double toAbsoluteAngleRad(double angleRad) {
    return Units.degreesToRadians(toAbsoluteAngleDeg(Units.radiansToDegrees(angleRad)));
  }

  public double toAbsoluteAngleDeg(double angleDeg) {
    double scaled = (angleDeg % 360.0);
    return scaled + (scaled < 0 ? 360.0 : 0.0);
  }

  public void testSteer(double angleDeg) {
    for (var module : _modules) module.setLastAngleDeg(angleDeg);
  }

  public void forceScoreWheelDirection() {
    var angle = isRedAlliance() ? 0.0 : 180.0;
    for (var module : _modules) module.setLastAngleDeg(angle);
  }

  public void forceScoreWheelDirection(double angleDeg) {}

  public void forceChargingWheelDirection() {
    var goalModuleStates = _swerveKinematics.toSwerveModuleStates(
        ChassisSpeeds.fromFieldRelativeSpeeds(0, 0, 90, getYaw())
    );
    for (int i = 0; i < _conf._numModules; i++) {
      goalModuleStates[i].angle = goalModuleStates[i].angle.plus(Rotation2d.fromDegrees(90.0));
      _modules[i].setLastAngle(goalModuleStates[i]);
    }
  }

  public void autonomousDriveMode(boolean enable) {
    for (SwerveModule module : _modules) {
      module.autonomousDriveMode(enable);
    }
  }

  public boolean inRange() {
    return (_xController.atGoal() && _yController.atGoal() && _angleController.atGoal());
  }

  public boolean thetaInRange() {
    return _angleController.atGoal();
  }

  public Pose2d getAlliancePose(Pose2d red, Pose2d blue) {
    return _alliance == Alliance.Red ? red : blue;
  }

  public void updateTuneScoringShuffleboard(double xError, double yError, double thetaError) {
    if (RobotAltModes.isPoseTuning) {
      _xPoseError.setDouble(xError);
      _yPoseError.setDouble(yError);
      _thetaPoseError.setDouble(thetaError);
    }
  }

  public void updateTuneScoringStatus(boolean tuneScoring) {
    if (RobotAltModes.isPoseTuning) {
      _usingPoseTuning.setBoolean(tuneScoring);
    }
  }

  public InstantCommand zeroGyroCommand() {
    return new InstantCommand(() -> zeroGyro());
  }

  public InstantCommand zeroPoseCommand() {
    return new InstantCommand(() -> zeroPose());
  }

  public InstantCommand forceChargingWheelDirectionCommand() {
    return new InstantCommand(() -> forceChargingWheelDirection());
  }

  private boolean checkInitStatus() {
    StatusCode initStatus = _gyro.getConfigurator().apply(_gyroConfigs);
    return (initStatus == StatusCode.OK);
  }

  private void resetSimPose(Pose2d pose) {
    _simPose = pose;
  }

  // // Gets vision pose of the primary camera
  // public Pose2d getVisionPose() {
  //   return _multiCameraController.getVisionPose();
  // }

  // public double getVisionCenterOffsetController(double _centerOffsetX) {
  //   return _visionCenterOffsetController.calculate(_centerOffsetX);
  // }

  // public double getCenterOffsetX(ScoringLocations scoringLocations) {
  //   return _multiCameraController.getCenterOffsetX(scoringLocations, isRedAlliance());
  // }

  // // Gets vision pose for the camera with the passed id
  // public Pose2d getVisionPose(int id) {
  //   return _multiCameraController.getVisionPose(id);
  // }

  // public void setDesiredCameras(CameraSets camera) {
  //   _multiCameraController.setDesiredCameras(camera);
  // }

  // public CameraSets getDesiredCameras() {
  //   return _multiCameraController.getDesiredCameras();
  // }

  public InstantCommand forceSetInitalPoseCommand() {
    return new InstantCommand(() -> {
      setPose(new Pose2d(
          new Pose2d().getTranslation().plus(new Translation2d(-1.0, 0.0)),
          Rotation2d.fromDegrees(180.0)
      ));
    });
  }

  public Command forceAllianceBasedFieldRelativeMovementCommand(
      double blueXMPS, double blueYMPS, double timeout_s
  ) {
    return forceFieldRelativeMovementCommand(blueXMPS, blueYMPS, -blueXMPS, -blueYMPS, timeout_s);
  }

  public Command forceFieldRelativeMovementCommand(double xMPS, double yMPS, double timeout_s) {
    return forceFieldRelativeMovementCommand(xMPS, yMPS, xMPS, yMPS, timeout_s);
  }

  public Command forceFieldRelativeMovementCommand(
      double blueXMPS, double blueYMPS, double redXMPS, double redyMPS, double timeout_s
  ) {
    // run rather than Commands.run so this implicity requires the current subsystem
    // (drivertrain)
    return run(() -> {
             fieldRelativeDrive(
                 isRedAlliance() ? redXMPS : blueXMPS, isRedAlliance() ? redyMPS : blueYMPS, 0.0
             );
           }
    ).withTimeout(timeout_s);
  }

  // Threaded odometry
  public class OdometryThread extends Thread {
    private static int kThreadPriority = 1;
    private BaseStatusSignal[] _allSignals;
    public int SuccessfulDaqs = 0;
    public int FailedDaqs = 0;

    // Track thread timing
    private final MedianFilter _peakRemover = new MedianFilter(3);
    private final LinearFilter _lowPass = LinearFilter.movingAverage(50);
    private double _lastTime = 0.0;

    public OdometryThread(final SwerveModule[] modules) {
      super();
      setDaemon(true);

      // 4 signals for each module + 2 for Pigeon2.
      _allSignals = new StatusSignal[(modules.length * 4) + 2];
      for (int i = 0; i < modules.length; i++) {
        BaseStatusSignal[] signals = modules[i].getSignals();
        int baseIndex = i * 4;
        for (int j = 0; j < 4; j++) _allSignals[baseIndex + j] = signals[j];
      }

      _allSignals[_allSignals.length - 2] = _gyro.getYaw().clone();
      _allSignals[_allSignals.length - 1] = _gyro.getAngularVelocityZ().clone();

      /* Make sure all signals update at around 250hz */
      while (
          BaseStatusSignal.setUpdateFrequencyForAll(RobotAltModes.kOdometryFrequency, _allSignals)
          != StatusCode.OK
      ) {
        System.out.println("setUpdateFrequencyForAll not ok");
      }
      Threads.setCurrentThreadPriority(true, 1);
    }

    @Override
    public void run() {
      Threads.setCurrentThreadPriority(true, kThreadPriority);

      // Run as fast as possible, the blocking for the signals will control the timing.
      while (true) {
        StatusCode status;

        if (RobotAltModes.kEnableLatencyCompensation) {
          // Synchronously wait for all signals, up to twice the period of the update frequency.
          status = StatusSignal.waitForAll(2.0 / RobotAltModes.kOdometryFrequency, _allSignals);
        }

        if (status != StatusCode.OK) {
          System.err.println("!!");
        } else {
          _odometryLock.writeLock().lock();

          // Compute loop stats
          final double currentTime = Timer.getFPGATimestamp();
          _averageOdometryLoopTime =
              _lowPass.calculate(_peakRemover.calculate(currentTime - _lastTime));
          _lastTime = currentTime;

          /* Get status of first element */
          if (status.isOK()) {
            SuccessfulDaqs++;
          } else {
            FailedDaqs++;
          }

          /* Now update odometry */
          /* Keep track of the change in azimuth rotations */
          for (int i = 0; i < _modules.length; i++) {
            _modulePositions[i] = _modules[i].getPosition(false);
          }
          // Assume Pigeon2 is flat-and-level so latency compensation can be performed
          // TODO FIX THREAD SAFETY
          double yawDegrees =
              BaseStatusSignal.getLatencyCompensatedValue(_gyro.getYaw(), _angularZGetter);

          /* Keep track of previous and current pose to account for the carpet vector */
          _robotPoseEstimator.updateWithTime(
              currentTime, Rotation2d.fromDegrees(yawDegrees), _modulePositions
          );
          _odom.update(Rotation2d.fromDegrees(yawDegrees), _modulePositions);

          // TODO
          // /* And now that we've got the new odometry, update the controls */
          // _requestParameters.currentPose =
          // _robotPoseEstimator.getEstimatedPosition().relativeTo(
          //     new Pose2d(0, 0, _fieldRelativeOffset)
          // );
          // _requestParameters.kinematics = _swerveKinematics;
          // _requestParameters.swervePositions = _swerveModuleLocations;
          // _requestParameters.timestamp = currentTime;
          // _requestParameters.updatePeriod = 1.0 / RobotAltModes.kOdometryFrequency;

          // setModuleStates(_requestParameters)
          // _requestToApply.apply(_requestParameters, _modules);

          /* Update our cached state with the newly updated data */
          _cachedState.FailedDaqs = FailedDaqs;
          _cachedState.SuccessfulDaqs = SuccessfulDaqs;
          _cachedState.ModuleStates = new SwerveModuleState[_modules.length];
          for (int i = 0; i < _modules.length; ++i) {
            _cachedState.ModuleStates[i] = _modules[i].getState();
          }
          _cachedState.EstimatedPose = _robotPoseEstimator.getEstimatedPosition();
          _cachedState.Pose = _odom.getPoseMeters();

          // TODO: call an update function and pre-construct the array
          _chassisSpeeds = _swerveKinematics.toChassisSpeeds(_cachedState.ModuleStates);

          _cachedState.OdometryPeriod = _averageOdometryLoopTime;

          if (_telemetryFunction != null) {
            /* Log our state */
            _telemetryFunction.accept(_cachedState);
          }

          // TODO:604 Output stats
          // // Only update localizer when enabled
          // if (DriverStation.isEnabled()) {
          //   final double yaw = Constants.kEnableLatencyCompensation
          //       ? StatusSignal.getLatencyCompensatedValue(
          //           _gyro.continuousYawSignal(), _gyro.yawRateSignal()
          //       )
          //       : _gyro.getContinuousYaw();
          //   final var odometryMeasurement =
          //       new Pose2d(new Rotation2d(yaw), getModulePositionStates(false));
          //   m_localizer.update(odometryMeasurement, new ArrayList<>());
          // }

          // // Apply closed-loop controls
          // // TODO: this is a temp hack for testing
          // if (Constants.kEnableSynchronousOutput && DriverStation.isAutonomousEnabled()) {
          //   final ChassisSpeeds fieldRelativeChassisSpeeds = _driveController.calculate(
          //       RobotAltModes.getPose(), m_targetPose, m_xVelRef, m_yVelRef, m_thetaVelRef
          //   );
          //   driveOpenLoop(
          //       fieldRelativeChassisSpeeds.vxMetersPerSecond,
          //       fieldRelativeChassisSpeeds.vyMetersPerSecond,
          //       fieldRelativeChassisSpeeds.omegaRadiansPerSecond,
          //       /*fieldRelative=*/false,
          //       m_allowedScrub,
          //       1.0 / RobotAltModes.kOdometryFrequency
          //   );
          // }

          _odometryLock.writeLock().unlock();
        }
      }
    }

    public boolean odometryIsValid() {
      return SuccessfulDaqs > 2; // Wait at least 3 daqs before saying the odometry is valid
    }

    /**
     * Sets the DAQ thread priority to a real time priority under the specified priority level
     *
     * @param priority Priority level to set the DAQ thread to.
     *                 This is a value between 0 and 99, with 99 indicating higher priority and
     * 0 indicating lower priority.
     */
    public void setThreadPriority(int priority) {
      kThreadPriority = priority;
    }
  }

  public ChassisSpeeds getRobotRelativeSpeeds() {
    try {
      _odometryLock.readLock().lock();
      return _chassisSpeeds;
    } finally {
      _odometryLock.readLock().unlock();
    }
  }

  public ChassisSpeeds getFieldRelativeSpeeds() {
    return ChassisSpeeds.fromRobotRelativeSpeeds(_chassisSpeeds, getYaw());
  }
}
