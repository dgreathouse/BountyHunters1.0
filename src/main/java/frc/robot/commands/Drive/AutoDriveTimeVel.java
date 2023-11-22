// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drive;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DrivetrainSubsystem;

public class AutoDriveTimeVel extends Command {
  DrivetrainSubsystem m_drivetrain;
  Timer m_timer = new Timer();
  double m_rampTime = 1.0; // Seconds
  double m_timeOut;
  double m_driveAngle;
  double m_robotAngle;
  double m_speed;
  boolean m_rampEnalble = true;
  double m_currentSpeed = 0;

  /** 
   * * AutoDriveTimeVel
   *  <p>Drive at a set velocity, robot angle and time. A velocity ramp time of 1 second will be used if rampEnable is true.

   * @param _drive An instance of the drive subsystem
   * @param _velocity The velocity you want the robot to drive at in meters/sec
   * @param _driveAngle The Drive angle the chassis should drive at in degrees
   * @param _robotAngle The Angle the robot should face while driving in degrees
   * @param _timeOut The time to stop driving in seconds.
   * @param _rampEnable Enable the ramp of velocity at the start.
   */
  public AutoDriveTimeVel(DrivetrainSubsystem _drive, double _speed, double _driveAngle, double _robotAngle, double _timeOut, boolean _rampEnable) {
    m_drivetrain = _drive;
    m_timeOut = _timeOut;
    m_driveAngle = _driveAngle;
    m_robotAngle = _robotAngle;
    m_speed = _speed;
    m_rampEnalble = _rampEnable;
    m_currentSpeed = m_speed;
    addRequirements(m_drivetrain);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(m_rampEnalble){m_currentSpeed = m_timer.get() < m_rampTime ? m_speed / (m_rampTime/m_timer.get()) : m_speed; }
    m_drivetrain.drivePolarFieldCentric(m_driveAngle, m_currentSpeed, m_robotAngle);
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
