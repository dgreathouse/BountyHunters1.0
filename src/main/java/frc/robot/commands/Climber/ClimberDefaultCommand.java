//Copyright (c) 2020-2023 Essexville Hampton Public Schools (FRC 8517)

package frc.robot.commands.Climber;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.lib.ICommand;
import frc.robot.subsystems.ClimberSubsystem;

public class ClimberDefaultCommand extends Command  implements ICommand{
  ClimberSubsystem m_climber;

  /** Creates a new ClimberDefaultCommand. */
  public ClimberDefaultCommand(ClimberSubsystem _subsystem) {
    m_climber = _subsystem;
    addRequirements(m_climber);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
  @Override
  public void updateDashboard() {

  }
}
