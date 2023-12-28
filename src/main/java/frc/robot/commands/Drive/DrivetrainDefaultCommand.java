//Copyright (c) 2020-2023 Essexville Hampton Public Schools (FRC 8517)

package frc.robot.commands.Drive;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.lib.ICommand;
import frc.robot.lib.k;
import frc.robot.subsystems.DrivetrainSubsystem;

public class DrivetrainDefaultCommand extends Command implements ICommand{

  private DrivetrainSubsystem m_drive;
  ChassisSpeeds m_speeds = new ChassisSpeeds();

  /** Creates a new DrivetrainDefaultCommand. */
  public DrivetrainDefaultCommand(DrivetrainSubsystem _drivetrain) {
    m_drive = _drivetrain;
    this.setName("DriveDefaultCommand");
    addRequirements(m_drive);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Set local variables to game thumbstick axis values
    double leftY = -RobotContainer.s_driverController.getLeftY();
    double leftX = -RobotContainer.s_driverController.getLeftX();
    double rightX = -RobotContainer.s_driverController.getRightX();
    double rightY = -RobotContainer.s_driverController.getRightY();

    // Limit the inputs for a deadband related to the joystick
    leftY = MathUtil.applyDeadband(leftY, 0.02, 1.0);
    leftX = MathUtil.applyDeadband(leftX, 0.02, 1.0);
    rightY = MathUtil.applyDeadband(rightY, 0.02, 1.0);
    rightX = MathUtil.applyDeadband(rightX, 0.02, 1.0);

    // Set the class variable ChassisSpeeds to the local variables in their
    // appropriate units.
    m_speeds.vxMetersPerSecond = leftY * k.DRIVE.MAX_VELOCITY_MeterPerSec;
    m_speeds.vyMetersPerSecond = leftX * k.DRIVE.MAX_VELOCITY_MeterPerSec;
    m_speeds.omegaRadiansPerSecond = rightX * k.DRIVE.MAX_ANGULAR_VELOCITY_RadianPerSec;

    // Get a new angle if the right x&y are greater than a certain point.
    if (Math.abs(rightX) > 0.8 || Math.abs(rightY) > 0.8) {
      m_drive.setLastTargetAngle(new Rotation2d(rightY, rightX));
    }

    // Call the appropriate drive mode. Selected by the driver controller Square button.
    
    switch (m_drive.getDriveMode()) {
      case FIELD_CENTRIC:
        m_drive.driveFieldCentric(m_speeds);
        break;
      case ROBOT_CENTRIC:
        m_drive.driveRobotCentric(m_speeds);
        break;
      case ANGLE_FIELD_CENTRIC:
        m_drive.driveAngleFieldCentric(m_speeds.vxMetersPerSecond, m_speeds.vyMetersPerSecond);
        break;
      default:
        break;
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

  @Override
  public void updateDashboard() {
    SmartDashboard.putNumber("ChassisSpeeds(Command) MPS(X)", m_speeds.vxMetersPerSecond);
    SmartDashboard.putNumber("ChassisSpeeds(Command) MPS(Y)", m_speeds.vyMetersPerSecond);
    SmartDashboard.putNumber("ChassisSpeeds(Command) Deg/Sec", m_speeds.omegaRadiansPerSecond * k.CONVERT.RADIANS_TO_DEGREES);
  }
}
