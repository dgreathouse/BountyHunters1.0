// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DrivetrainSubsystem;

public class DrivetrainDefaultCommand extends Command {

  public Rotation2d m_lastTargetAngle = new Rotation2d();

  /** Creates a new DrivetrainDefaultCommand. */
  public DrivetrainDefaultCommand(DrivetrainSubsystem _drivetrain) {
    addRequirements(_drivetrain);
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

    // TODO: Do better than this deadband garbage
    if (Math.abs(leftY) < 0.1 && Math.abs(leftX) < 0.1) {
      leftY = 0;
      leftX = 0;
    }
    if (Math.abs(rightX) < 0.1 && Math.abs(rightY) < 0.1) {
      rightX = 0;
      rightY = 0;
    }
    // TODO: scale to the appropriate MPS and Rads/sec
    var directions = new ChassisSpeeds();
    directions.vxMetersPerSecond = leftY * 1;
    directions.vyMetersPerSecond = leftX * -1;
    directions.omegaRadiansPerSecond = rightX * -10;

    // TODO: Make a better detection of being in what they call FullyFieldCentric
    if (Math.abs(rightX) > 0.7 || Math.abs(rightY) > 0.7) {
      m_lastTargetAngle = new Rotation2d(rightY, -rightX);
    }
    // TODO: determine from the students what is the best approach for Fully or Field centric with rotation 
    RobotContainer.m_drivetrainSubsystem.driveFieldCentric(directions);
    //RobotContainer.m_drivetrainSubsystem.driveFullyFieldCentric(leftY * 1, leftX * -1, m_lastTargetAngle);

    

    // Reset the Gyro Yaw
    if (RobotContainer.s_driverController.square().getAsBoolean()) {
      RobotContainer.m_drivetrainSubsystem.m_robotDrive.seedFieldRelative();
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