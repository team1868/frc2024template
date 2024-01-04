package frc.robot.constants.enums;

import com.choreo.lib.Choreo;
import com.choreo.lib.ChoreoTrajectory;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Controlboard;
import frc.robot.subsystems.Drivetrain;

public enum AutonomousRoutines {
  DEFAULT_AUTO(false, "DEFAULT", LedColors.BLOOD_RED, Commands.print("DEFAULT AUTO SAYS HI")),
  CHOREO_SQUARE(true, "Square", "Square", LedColors.OFF);

  public final boolean showInDashboard;
  public final String shuffleboardName;
  public final String trajectoryName;
  public final LedColors prepColor;
  public final boolean buildable;

  public ChoreoTrajectory trajectory;

  public Command command;
  public Command builtCommand;

  AutonomousRoutines(
      boolean show, String shuffleboardName, String trajectoryName, LedColors prepColor
  ) {
    showInDashboard = show;
    this.shuffleboardName = shuffleboardName;
    this.trajectoryName = trajectoryName;
    this.prepColor = prepColor;
    buildable = true;
  }

  AutonomousRoutines(String shuffleboardName, String trajectoryName, LedColors prepColor) {
    this(true, shuffleboardName, trajectoryName, prepColor);
  }

  AutonomousRoutines(
      boolean show, String shuffleboardName, LedColors prepColor, Command simpleCommand
  ) {
    showInDashboard = show;
    this.shuffleboardName = shuffleboardName;
    this.prepColor = prepColor;
    command = simpleCommand;
    buildable = false;

    trajectoryName = null;
  }

  public void build(Drivetrain drivetrain, Controlboard controlboard) {
    if (buildable) {
      // initialBluePose = new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(0.0));
      // initialRedPose = new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(0.0));
      trajectory = Choreo.getTrajectory(trajectoryName);

      // Create a swerve command for the robot to follow the trajectory
      command = Choreo.choreoSwerveCommand(
          trajectory, // Choreo trajectory from above
          drivetrain::getPose, // A function that returns the current field-relative pose of the
                               // robot: your wheel or vision odometry
          // TODO move and make more configurable
          new PIDController(10.0, 0.0, 0.0), // PIDController for field-relative X translation
                                             // (input: X error in meters, output: m/s).
          new PIDController(10.0, 0.0, 0.0), // PIDController for field-relative Y translation
                                             // (input: Y error in meters, output: m/s).
          new PIDController(10.0, 0.0, 0.0), // PID constants to correct for rotation error
          drivetrain::robotCentricDrive, // A function that consumes the target robot-relative
                                         // chassis speeds and commands them to the robot
          true, // Whether or not to mirror the path based on alliance (this assumes the path is
                // created for the blue alliance)
          drivetrain // The subsystem(s) to require, typically your drive subsystem (this) only
      );
    } else {
      builtCommand = command;
    }
  }

  // public Pose2d getInitialPose(Alliance alliance) {
  //   return alliance == Alliance.Blue ? initialBluePose : initialRedPose;
  // }

  public enum PathLegs {
    SUB_TRAJECTORY_1(false, "trajectoryName");

    public final boolean currentlyExists;
    public final String trajectoryName;

    PathLegs(String trajectoryName) {
      this(true, trajectoryName);
    }

    PathLegs(boolean exists, String trajectoryName) {
      this.currentlyExists = exists;
      this.trajectoryName = trajectoryName;
    }
  }
}
