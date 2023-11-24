//Copyright (c) 2020-2023 Essexville Hampton Public Schools (FRC 8517)

package frc.robot.commands.Extension;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ExtensionSubsystem;

public class ExtensionDefaultCommand extends Command {
  ExtensionSubsystem m_extension;
  /** Creates a new ExtensionDefaultCommand. */
  public ExtensionDefaultCommand(ExtensionSubsystem _subsystem) {
    m_extension = _subsystem;
    addRequirements(m_extension);
    // Use addRequirements() here to declare subsystem dependencies.
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
