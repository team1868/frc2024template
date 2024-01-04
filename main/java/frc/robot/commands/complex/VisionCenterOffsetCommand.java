package frc.robot.commands.complex;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Controlboard;
import frc.robot.subsystems.Drivetrain;

public class VisionCenterOffsetCommand extends Command {
  private final Drivetrain _drivetrain;
  private final Controlboard _controlboard;
  public VisionCenterOffsetCommand(Drivetrain drivetrain, Controlboard controlboard) {
    _drivetrain = drivetrain;
    _controlboard = controlboard;
  }

  @Override
  public void initialize() {
    _drivetrain.setSnapScoringAngle();
  }

  @Override
  public void execute() {
    // double _centerOffsetX = _drivetrain.getCenterOffsetX(ScoringLocations.SCORING_LOCATION_2);
    // if (_centerOffsetX != -1.0) {
    //   _drivetrain.fieldRelativeDrive(
    //       _controlboard.getDriveX(),
    //       _centerOffsetX < Double.MAX_VALUE
    //           ? _drivetrain.getVisionCenterOffsetController(_centerOffsetX)
    //           : _controlboard.getDriveY(),
    //       0.0
    //   );
    // }
  }
}
