//Copyright (c) 2020-2023 Essexville Hampton Public Schools (FRC 8517)

package frc.robot.commands.Lift;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.lib.ICommand;
import frc.robot.subsystems.LiftSubsystem;

public class LiftDefaultCommand extends Command  implements ICommand{
  LiftSubsystem m_lift;
  /** Creates a new LiftDefaultCommand. */
  public LiftDefaultCommand(LiftSubsystem _subsystem) {
    m_lift = _subsystem;
    addRequirements(m_lift);
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
