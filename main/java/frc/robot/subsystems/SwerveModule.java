package frc.robot.subsystems;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.Robot;
import frc.robot.constants.Constants;
import frc.robot.constants.DrivetrainConfs;
import frc.robot.constants.RobotAltModes;
import frc.robot.constants.enums.ModuleControl;

public class SwerveModule {
  public static final DrivetrainConfs DRIVE_CONSTS = Constants.CRobot.drive;

  /* --- Module Identifiers --- */
  public final int _moduleNumber;
  public final String _moduleNumberStr;

  /* --- Sensors, motors, and hardware --- */
  private final TalonFX _angleMotor;
  private final TalonFX _driveMotor;

  private AnalogInput _analogInput;
  private AnalogPotentiometer _angleEncoder;
  private CANcoder _angleEncoderCAN;

  /* --- State and Physical Property variables --- */
  private SwerveModuleState _desiredState;
  private SwerveModuleState _curState = new SwerveModuleState();
  private SwerveModulePosition _curPosition = new SwerveModulePosition();
  private final double _angleOffsetRotation;
  private double _lastAngleDeg;
  private double _percentOutput = 0.0;
  private double _velocity = 0.0;
  private double _angleDeg = 0.0;
  private double _absolutePosition;
  private final String _canBusName;

  /* --- Control Utils --- */
  private static CANcoderConfiguration _swerveCanCoderConfig;
  private static MotorOutputConfigs _driveStatorLimit;
  private static MotorOutputConfigs _autonomousDriveStatorLimit;

  // TODO: Move constants but not class into separate enum or configuration
  private final SimpleMotorFeedforward _feedforward = new SimpleMotorFeedforward(
      ModuleControl.ElectricalConf.DRIVE_KS_VOLT,
      ModuleControl.ElectricalConf.DRIVE_KV_VOLTPMPS,
      ModuleControl.ElectricalConf.DRIVE_KA_VOLTPMPS_SQ
  );

  /* --- Simulation resources and variables --- */
  private TalonFXSimState _driveMotorSim;
  private TalonFXSimState _angleMotorSim;

  private FlywheelSim _driveWheelSim;

  private SingleJointedArmSim _moduleAngleSim;

  /* --- Shuffleboard entries --- */
  public GenericEntry _steerAnglePFac, _steerAngleIFac, _steerAngleDFac;
  public GenericEntry _steerPFac, _steerIFac, _steerDFac;

  // telemetry widgets
  public GenericEntry _shuffleboardModuleCANcoder;
  public GenericEntry _shuffleboardModuleAngle;
  public GenericEntry _shuffleboardModuleSpeed;

  /* Threaded signal utils */
  private final StatusSignal<Double> _drivePosition;
  private final StatusSignal<Double> _driveVelocity;
  private final StatusSignal<Double> _driveAppliedVolts;
  private final StatusSignal<Double> _driveCurrent;

  private StatusSignal<Double> _steerAbsolutePosition;
  private final StatusSignal<Double> _steerPosition;
  private final StatusSignal<Double> _steerVelocity;
  private final StatusSignal<Double> _steerAppliedVolts;
  private final StatusSignal<Double> _steerCurrent;
  private final BaseStatusSignal[] _signals;

  public SwerveModule(int moduleNumber, String canBusName) {
    _moduleNumber = moduleNumber;
    _canBusName = canBusName;
    _moduleNumberStr = "M" + Integer.toString(_moduleNumber);
    _angleOffsetRotation = DRIVE_CONSTS.moduleOffsetsRotation[_moduleNumber];
    initCanCoderConfigs();

    _angleMotor = new TalonFX(DRIVE_CONSTS.ports.steer[_moduleNumber], canBusName);
    _driveMotor = new TalonFX(DRIVE_CONSTS.ports.drive[_moduleNumber], canBusName);

    waitForCAN();

    // Must be done before the angle motor is configured
    configAngleEncoder();

    configAngleMotor();

    configDriveMotor();

    configShuffleboard();

    // probably redundant
    resetToAbsolute();

    simulationInit();

    _drivePosition = _driveMotor.getPosition().clone();
    _driveVelocity = _driveMotor.getVelocity().clone();
    _driveAppliedVolts = _driveMotor.getMotorVoltage();
    _driveCurrent = _driveMotor.getStatorCurrent();

    _steerPosition = _angleMotor.getPosition().clone();
    _steerVelocity = _angleMotor.getVelocity().clone();
    _steerAppliedVolts = _angleMotor.getMotorVoltage();
    _steerCurrent = _angleMotor.getStatorCurrent();

    _signals = new BaseStatusSignal[4];
    _signals[0] = _drivePosition;
    _signals[1] = _driveVelocity;
    _signals[2] = _steerPosition;
    _signals[3] = _steerVelocity;
  }

  public BaseStatusSignal[] getSignals() {
    return _signals;
  }

  public SwerveModule(int moduleNumber) {
    this(moduleNumber, "");
  }

  public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop) {
    if (_angleMotor.hasResetOccurred()) {
      resetToAbsolute();
    }

    // TODO cleanup
    _desiredState = optimize(desiredState, getState().angle);

    if (isOpenLoop) {
      _percentOutput = _desiredState.speedMetersPerSecond
          / Constants.CRobot.drive.model.theoreticalMaxWheelSpeed;
      _driveMotor.set(_percentOutput);
    } else {
      _velocity = MPSToRotorRPS(
          _desiredState.speedMetersPerSecond,
          DRIVE_CONSTS.type.wheelCircumferenceM,
          DRIVE_CONSTS.model.driveRatio
      );
      _driveMotor.setControl(new VelocityVoltage(
          _velocity,
          0,
          true,
          _feedforward.calculate(_desiredState.speedMetersPerSecond),
          0,
          false,
          false,
          false
      ));
    }

    _angleDeg = Math.abs(_desiredState.speedMetersPerSecond)
            <= (DRIVE_CONSTS.model.theoreticalMaxWheelSpeed * 0.01)
        ? _lastAngleDeg
        : _desiredState.angle.getDegrees();
    _angleMotor.setControl(new PositionVoltage(_angleDeg * DRIVE_CONSTS.model.steerRatio / 360.0));
    _lastAngleDeg = _angleDeg;
  }

  public double getDegreeAngleEncoder() {
    return getRotationAngleEncoder() * 360.0;
  }

  public double getRotationAngleEncoder() {
    if (Constants.isCANEncoder) {
      return _angleEncoderCAN.getAbsolutePosition().getValue();
    }
    return _angleEncoder.get() + _angleOffsetRotation;
  }

  public static double MPSToRotorRPS(double velocityMPS, double circumferenceM, double gearRatio) {
    return velocityMPS * gearRatio / circumferenceM;
  }
  public static double RotorRPSToMPS(double rotorRPS, double circumferenceM, double gearRatio) {
    return rotorRPS * circumferenceM / gearRatio;
  }

  public SwerveModuleState getState() {
    _curState.speedMetersPerSecond = RotorRPSToMPS(
        _driveMotor.getVelocity().getValue(),
        DRIVE_CONSTS.type.wheelCircumferenceM,
        DRIVE_CONSTS.model.driveRatio
    );
    _curState.angle = Rotation2d.fromRotations(
        _angleMotor.getRotorPosition().getValue() / DRIVE_CONSTS.model.steerRatio
    );

    return _curState;
  }

  public SwerveModulePosition getPosition() {
    _curPosition.distanceMeters = _driveMotor.getRotorPosition().getValueAsDouble()
        * DRIVE_CONSTS.type.wheelCircumferenceM / DRIVE_CONSTS.model.driveRatio;
    _curPosition.angle = Rotation2d.fromRotations(
        _angleMotor.getRotorPosition().getValueAsDouble() / DRIVE_CONSTS.model.steerRatio
    );

    return _curPosition;
  }

  public SwerveModulePosition getPosition(boolean refresh) {
    if (refresh) {
      /* Refresh all signals */
      _drivePosition.refresh();
      _driveVelocity.refresh();
      _steerPosition.refresh();
      _steerVelocity.refresh();
    }

    /* Now latency-compensate our signals */
    double drive_rot = BaseStatusSignal.getLatencyCompensatedValue(_drivePosition, _driveVelocity);
    double angle_rot = BaseStatusSignal.getLatencyCompensatedValue(_steerPosition, _steerVelocity);

    /*
     * Back out the drive rotations based on angle rotations due to coupling between
     * azimuth and steer
     */
    drive_rot -= angle_rot * RobotAltModes.steerDriveCouplingRatio;

    /* And push them into a SwerveModulePosition object to return */
    _curPosition.distanceMeters =
        drive_rot * DRIVE_CONSTS.type.wheelCircumferenceM / DRIVE_CONSTS.model.driveRatio;
    // _curPosition.distanceMeters = drive_rot / _driveRotationsPerMeter;

    // TODO check
    /* Angle is already in terms of steer rotations */
    _curPosition.angle = Rotation2d.fromRotations(angle_rot / DRIVE_CONSTS.model.steerRatio);
    // only if fused
    // _curPosition.angle = Rotation2d.fromRotations(angle_rot);

    return _curPosition;
  }

  private void waitForCAN() {
    if (Robot.isReal()) {
      int counter = 0;
      String identifier = "SWERVE " + _moduleNumberStr + " Check Init Status : ";
      while (!checkInitStatus()) {
        System.out.println(identifier + counter++);
      }
    } else {
    }
  }

  public void resetToAbsolute() {
    double rotation =
        (Constants.CRobot.drive.model.type.invertSteer ? 1.0 : -1.0) * getRotationAngleEncoder();
    _lastAngleDeg = rotation * 360.0;
    _absolutePosition = rotation * DRIVE_CONSTS.model.steerRatio;
    _angleMotor.setPosition(_absolutePosition);
  }

  public void configShuffleboard() {
    if (RobotAltModes.isVerboseMode) {
      ShuffleboardTab tab = Shuffleboard.getTab("SwerveModules");
      ShuffleboardLayout layout = tab.getLayout(_moduleNumberStr, BuiltInLayouts.kGrid)
                                      .withSize(1, 3)
                                      .withPosition(_moduleNumber, 0);

      _shuffleboardModuleAngle = layout.add("angleState", 0.0).withPosition(0, 0).getEntry();
      _shuffleboardModuleCANcoder = layout.add("angleCANcoder", 0.0).withPosition(0, 1).getEntry();
      _shuffleboardModuleSpeed = layout.add("speed", 0.0).withPosition(0, 2).getEntry();
    }
  }

  public void updateShuffleboard() {
    if (RobotAltModes.isVerboseMode) {
      _shuffleboardModuleAngle.setDouble(_curState.angle.getDegrees());
      _shuffleboardModuleCANcoder.setDouble(getDegreeAngleEncoder());
      _shuffleboardModuleSpeed.setDouble(_curState.speedMetersPerSecond);
    }
  }

  public void alignToZero() {
    resetToAbsolute();
    _lastAngleDeg = 0;
  }

  public boolean motorResetConfig() {
    boolean result = false;
    if (Constants.isCANEncoder && _angleEncoderCAN.hasResetOccurred()) {
      configCanEncoder();
      result = true;
    }
    if (_angleMotor.hasResetOccurred()) {
      configAngleMotor();
      result = true;
    }
    if (_driveMotor.hasResetOccurred()) {
      configDriveMotor();
      result = true;
    }
    return result;
  }

  public void configPID(double ap, double ai, double ad, double p, double i, double d) {
    if (RobotAltModes.isPIDTuningMode) {
      Constants.CRobot.drive.moduleControl.steerConf.Slot0.kP = ap;
      Constants.CRobot.drive.moduleControl.steerConf.Slot0.kI = ai;
      Constants.CRobot.drive.moduleControl.steerConf.Slot0.kD = ad;

      Constants.CRobot.drive.moduleControl.driveConf.Slot0.kP = p;
      Constants.CRobot.drive.moduleControl.driveConf.Slot0.kI = i;
      Constants.CRobot.drive.moduleControl.driveConf.Slot0.kD = d;

      _angleMotor.getConfigurator().apply(Constants.CRobot.drive.moduleControl.steerConf.Slot0);
      _driveMotor.getConfigurator().apply(Constants.CRobot.drive.moduleControl.driveConf.Slot0);
    }
  }

  public void setLastAngle(SwerveModuleState desiredState) {
    SwerveModuleState goalModuleState = optimize(desiredState, getState().angle);
    _lastAngleDeg = goalModuleState.angle.getDegrees();
  }

  public void setLastAngle(Rotation2d angle) {
    setLastAngleDeg(angle.getDegrees());
  }

  public void setLastAngleDeg(double angleDeg) {
    _lastAngleDeg = angleDeg;
  }

  public boolean isAlignedTo(SwerveModuleState goalState, double toleranceDeg) {
    return Math.abs(_angleDeg - goalState.angle.getDegrees()) < toleranceDeg;
  }

  public boolean isAlignedTo(SwerveModuleState goalState) {
    return isAlignedTo(goalState, DRIVE_CONSTS.alignmentToleranceDeg);
  }

  public void autonomousDriveMode(boolean enable) {
    _driveMotor.getConfigurator().apply(
        enable ? ModuleControl.ElectricalConf.AUTO_DRIVE_CURRENT_LIMITS_CONFIGS
               : ModuleControl.ElectricalConf.DRIVE_CURRENT_LIMITS_CONFIGS
    );
  }

  private void configAngleEncoder() {
    if (Constants.isCANEncoder) {
      initCanEncoder();
      configCanEncoder();
    } else {
      configAnalogEncoder();
    }
  }

  private void initCanEncoder() {
    _angleEncoderCAN = new CANcoder(DRIVE_CONSTS.ports.encoder[_moduleNumber], _canBusName);
  }

  private void configCanEncoder() {
    _swerveCanCoderConfig.MagnetSensor.MagnetOffset = _angleOffsetRotation;
    _angleEncoderCAN.getConfigurator().apply(_swerveCanCoderConfig);
  }

  private void configAnalogEncoder() {
    _analogInput = new AnalogInput(DRIVE_CONSTS.ports.encoder[_moduleNumber]);
    _angleEncoder = new AnalogPotentiometer(_analogInput, 1.0);
    _analogInput.setAverageBits(2);
  }

  private void configAngleMotor() {
    // _angleMotor.configFactoryDefault();
    _angleMotor.getConfigurator().apply(Constants.CRobot.drive.moduleControl.steerConf);
    _angleMotor.setInverted(Constants.CRobot.drive.model.type.invertSteer);
    _angleMotor.setNeutralMode(ModuleControl.STEER_NEUTRAL_MODE);

    resetToAbsolute();
  }

  private void configDriveMotor() {
    // _driveMotor.configFactoryDefault();
    _driveMotor.getConfigurator().apply(Constants.CRobot.drive.moduleControl.driveConf);
    _driveMotor.setInverted(Constants.CRobot.drive.model.type.invertDrive);
    _driveMotor.setNeutralMode(ModuleControl.DRIVE_NEUTRAL_MODE);
    _driveMotor.setPosition(0);
  }

  private boolean checkInitStatus() {
    // return _angleMotor.configFactoryDefault() == ErrorCode.OK;
    return true;
  }

  public void simulationInit() {
    if (!Robot.isReal())
      _driveMotorSim = _driveMotor.getSimState();
    _angleMotorSim = _angleMotor.getSimState();

    _driveWheelSim = new FlywheelSim(
        DCMotor.getFalcon500(1),
        DRIVE_CONSTS.model.driveRatio,
        0.01,
        VecBuilder.fill(2.0 * Math.PI / 2048)
    );

    _moduleAngleSim = new SingleJointedArmSim(
        DCMotor.getFalcon500(1),
        DRIVE_CONSTS.model.steerRatio,
        0.001,
        0.0,
        -Double.MAX_VALUE,
        Double.MAX_VALUE,
        false,
        0,
        VecBuilder.fill(2.0 * Math.PI / 2048)
    );
  }

  public void simulationPeriodic() {
    if (RobotAltModes.isSim) {
      // Update the model motor sim _driveWheelSim, and read its angular velocity
      _driveWheelSim.setInputVoltage(_driveMotor.get() * RobotController.getBatteryVoltage());
      _driveWheelSim.update(Constants.LOOP_PERIOD_MS);

      double driveSimOmega = _driveWheelSim.getAngularVelocityRPM();
      double driveTicksPerS =
          (_driveMotor.getInverted() ? -1 : 1) * driveSimOmega * DRIVE_CONSTS.model.driveRatio / 60;

      // Update integrated sensor sim in _driveMotor
      _driveMotorSim.setRotorVelocity(driveTicksPerS);
      _driveMotorSim.setRawRotorPosition(
          _driveMotor.getRotorPosition().getValueAsDouble() + driveTicksPerS
      );

      // Update _steeringSim single-arm simulation
      _moduleAngleSim.setInputVoltage(_angleMotor.get() * RobotController.getBatteryVoltage());
      _moduleAngleSim.update(Constants.LOOP_PERIOD_MS);

      // Update integratedSensor sim in _angleMotor
      double angleSign = _angleMotor.getInverted() ? -1 : 1;
      _angleMotorSim.setRawRotorPosition(
          angleSign
          * (_moduleAngleSim.getVelocityRadPerSec() * Constants.CTREConstants.RPM_TO_FALCON
             * DRIVE_CONSTS.model.steerRatio / (2 * Math.PI))
      );

      _angleMotorSim.setRawRotorPosition(
          angleSign * _moduleAngleSim.getAngleRads() * DRIVE_CONSTS.model.steerRatio / 60.0
      );
    }
  }

  private void initCanCoderConfigs() {
    if (Constants.isCANEncoder && _swerveCanCoderConfig == null) {
      /* Swerve CANCoder Configuration */
      _swerveCanCoderConfig = new CANcoderConfiguration();
      _swerveCanCoderConfig.MagnetSensor.AbsoluteSensorRange =
          AbsoluteSensorRangeValue.Unsigned_0To1;
      _swerveCanCoderConfig.MagnetSensor.MagnetOffset = _angleOffsetRotation;
      _swerveCanCoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
    }
  }

  public static SwerveModuleState optimize254(
      SwerveModuleState desiredState, Rotation2d currentAngle
  ) {
    // Place in closest scope
    double delta = (desiredState.angle.getDegrees() - currentAngle.getDegrees()) % 360;
    if (delta > 180.0) {
      delta += -360.0;
    } else if (delta < -180.0) {
      delta += 360.0;
    }
    double targetAngleDeg = currentAngle.getDegrees() + delta;
    double targetSpeedMPS = desiredState.speedMetersPerSecond;
    return new SwerveModuleState(targetSpeedMPS, Rotation2d.fromDegrees(targetAngleDeg));
  }

  public static SwerveModuleState optimize(
      SwerveModuleState desiredState, Rotation2d currentAngle
  ) {
    // Place in closest scope
    double delta = (desiredState.angle.getDegrees() - currentAngle.getDegrees()) % 360;
    if (delta > 180.0) {
      delta += -360.0;
    } else if (delta < -180.0) {
      delta += 360.0;
    }
    double targetAngleDeg = currentAngle.getDegrees() + delta;

    // Actual optimize
    double targetSpeedMPS = desiredState.speedMetersPerSecond;
    if (delta > 90.0) {
      targetSpeedMPS = -targetSpeedMPS;
      targetAngleDeg += -180.0;
    } else if (delta < -90.0) {
      targetSpeedMPS = -targetSpeedMPS;
      targetAngleDeg += 180.0;
    }
    return new SwerveModuleState(targetSpeedMPS, Rotation2d.fromDegrees(targetAngleDeg));
  }
}
