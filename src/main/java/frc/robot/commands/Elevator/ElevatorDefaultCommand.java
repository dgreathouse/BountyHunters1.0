//Copyright (c) 2020-2023 Essexville Hampton Public Schools (FRC 8517)

package frc.robot.commands.Elevator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;

public class ElevatorDefaultCommand extends Command {
  ElevatorSubsystem m_elevator;

  /** Creates a new ElevatorDefaultCommand. */
  public ElevatorDefaultCommand(ElevatorSubsystem _subsystem) {
    m_elevator = _subsystem;
    addRequirements(_subsystem);
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
}
