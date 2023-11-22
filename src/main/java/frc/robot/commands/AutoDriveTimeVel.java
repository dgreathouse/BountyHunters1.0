// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DrivetrainSubsystem;

public class AutoDriveTimeVel extends Command {
  DrivetrainSubsystem m_drivetrain;
  Timer m_timer = new Timer();
  double m_timeOut;
  double m_driveAngle;
  double m_robotAngle;
  double m_speed;
  /** Creates a new AutoDriveTimeVel. */
  public AutoDriveTimeVel(DrivetrainSubsystem _drive, double _speed, double _driveAngle, double _robotAngle, double _timeOut) {
    m_drivetrain = _drive;
    m_timeOut = _timeOut;
    m_driveAngle = _driveAngle;
    m_robotAngle = _robotAngle;
    m_speed = _speed;
    addRequirements(m_drivetrain);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    RobotContainer.m_drivetrainSubsystem.drivePolarFieldCentric(m_driveAngle, m_speed, m_robotAngle);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(m_timer.hasElapsed(m_timeOut)){
      return true;
    }
    return false;
  }
}
