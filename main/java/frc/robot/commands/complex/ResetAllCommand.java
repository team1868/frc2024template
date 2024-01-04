package frc.robot.commands.complex;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.enums.DriveModes;
import frc.robot.subsystems.Drivetrain;

public class ResetAllCommand extends Command {
  private final Drivetrain _drivetrain;

  public ResetAllCommand(Drivetrain drivetrain) {
    _drivetrain = drivetrain;
    addRequirements(drivetrain);
  }

  @Override
  public void initialize() {
    _drivetrain.drive(0.0, 0.0, 0.0, DriveModes.FIELD_RELATIVE);
  }

  @Override
  public boolean isFinished() {
    return true;
  }
}
