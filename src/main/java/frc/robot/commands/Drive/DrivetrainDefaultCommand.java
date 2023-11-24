//Copyright (c) 2020-2023 Essexville Hampton Public Schools (FRC 8517)

package frc.robot.commands.Drive;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.lib.k;
import frc.robot.subsystems.DrivetrainSubsystem;

public class DrivetrainDefaultCommand extends Command {

  public Rotation2d m_lastTargetAngle = new Rotation2d();
  private DrivetrainSubsystem m_drive;
  /** Creates a new DrivetrainDefaultCommand. */
  public DrivetrainDefaultCommand(DrivetrainSubsystem _drivetrain) {
    m_drive = _drivetrain;
    addRequirements(m_drive);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double leftY = -RobotContainer.s_driverController.getLeftY();
    double leftX = RobotContainer.s_driverController.getLeftX();
    double rightX = RobotContainer.s_driverController.getRightX();
    double rightY = -RobotContainer.s_driverController.getRightY();

    // Limit the inputs for a deadband related to the joystick
    leftY = MathUtil.applyDeadband(leftY, 0.1, 1.0);
    leftX = MathUtil.applyDeadband(leftX, 0.1, 1.0);
    rightY = MathUtil.applyDeadband(rightY, 0.1, 1.0);
    rightX = MathUtil.applyDeadband(rightX, 0.1, 1.0);

    var directions = new ChassisSpeeds();
    directions.vxMetersPerSecond = leftY * k.DRIVE.MAX_VELOCITY_MeterPerSec;
    directions.vyMetersPerSecond = leftX * k.DRIVE.MAX_VELOCITY_MeterPerSec;
    directions.omegaRadiansPerSecond = rightX * -k.DRIVE.MAX_ANGULAR_VELOCITY_RadianPerSec;
    if (Math.abs(rightX) > 0.8 || Math.abs(rightY) > 0.8) {
      m_lastTargetAngle = new Rotation2d(rightY, -rightX);
    }
    switch(m_drive.getDriveMode()){
      case FIELD_CENTRIC:
        m_drive.driveFieldCentric(directions);
      break;
      case ROBOT_CENTRIC:
        m_drive.driveRobotCentric(directions);
      break;
      case ANGLE_FIELD_CENTRIC:
        m_drive.driveAngleFieldCentric(leftY * 1, leftX * -1, m_lastTargetAngle);
      break;
      default:
      break;

    }

    // Reset the Gyro Yaw
    if (RobotContainer.s_driverController.square().getAsBoolean()) {
      m_drive.m_robotDrive.seedFieldRelative();
      // Make us target forward now to avoid jumps
      m_lastTargetAngle = new Rotation2d();
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
