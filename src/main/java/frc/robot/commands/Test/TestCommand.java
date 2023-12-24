// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Test;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.lib.ISubsystem;

public class TestCommand extends Command {

  Subsystem m_subsystem;

  /** Creates a new TestCommand. */
  public TestCommand(Subsystem _subsystem) {
    this.setName("TestCommand");
    addRequirements((Subsystem) _subsystem);
    m_subsystem = _subsystem;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double volts = SmartDashboard.getNumber("Test Voltage", 0);
    SmartDashboard.putString("TestCommand", ((Subsystem)m_subsystem).getName());
    if (m_subsystem != null) {
      ((ISubsystem)m_subsystem).setTestVoltage(volts);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
