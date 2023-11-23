//Copyright (c) 2020-2023 Essexville Hampton Public Schools (FRC 8517)

package frc.robot.commands.Turret;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.TurretSubsystem;

public class TurretDefaultCommand extends Command {
  TurretSubsystem m_turret;
  /** Creates a new TurretDefaultCommand. */
  public TurretDefaultCommand(TurretSubsystem _subsystem) {
    m_turret = _subsystem;
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
