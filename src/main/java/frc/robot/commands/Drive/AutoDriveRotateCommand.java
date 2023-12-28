//Copyright (c) 2020-2023 Essexville Hampton Public Schools (FRC 8517)

package frc.robot.commands.Drive;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.lib.ICommand;
import frc.robot.subsystems.DrivetrainSubsystem;

public class AutoDriveRotateCommand extends Command implements ICommand{
  DrivetrainSubsystem m_drivetrain;
  Timer m_timer = new Timer();
  double m_timeOut;
  double m_robotAngle;
  Rotation2d m_rotationAngle;

  /**
   * Rotate the robot using the driveAngleFieldCentric with speeds of 0 for
   * driving.
   * 
   * @param _drive      An intstance of the drive subsystem
   * @param _robotAngle The angle in degrees to rotate to
   * @param _timeOut    A timeOut time in seconds used as safety factor to cancel
   *                    this command if angle never reached.
   */
  public AutoDriveRotateCommand(DrivetrainSubsystem _drive, double _robotAngle, double _timeOut) {
    m_drivetrain = _drive;
    m_timeOut = _timeOut;
    m_robotAngle = _robotAngle;
    addRequirements(m_drivetrain);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_timer.start();
    m_drivetrain.setLastTargetAngle(new Rotation2d(m_robotAngle));
   //m_rotationAngle = new Rotation2d(m_robotAngle);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_drivetrain.driveAngleFieldCentric(0, 0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (m_timer.hasElapsed(m_timeOut) || m_drivetrain.m_robotDrive.isTurnPIDatSetpoint()) {
      return true;
    }
    return false;
  }
  @Override
  public void updateDashboard() {
    
  }
}
