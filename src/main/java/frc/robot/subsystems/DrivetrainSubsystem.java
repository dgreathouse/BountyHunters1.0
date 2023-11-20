// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.lib.RobotDrive;
import frc.robot.lib.SwerveDriveConstantsCreator;
import frc.robot.lib.SwerveDriveTrainConstants;
import frc.robot.lib.SwerveModuleConstants;

public class DrivetrainSubsystem extends SubsystemBase {
  public RobotDrive m_robotDrive;
  /** Creates a new DrivetrainSubsystem. */
  public DrivetrainSubsystem() {
    // TODO: Calibrate the PID values
    SwerveDriveTrainConstants drivetrain = new SwerveDriveTrainConstants().withPigeon2Id(1).withCANbusName("CANFD1")
        .withTurnKp(5);
    Slot0Configs steerGains = new Slot0Configs();
    Slot0Configs driveGains = new Slot0Configs();
    steerGains.kP = 30;
    steerGains.kD = 0.2;
    driveGains.kP = 1;
    SwerveDriveConstantsCreator m_constantsCreator = new SwerveDriveConstantsCreator(
        10, // 10:1 ratio for the drive motor
        12.8, // 12.8:1 ratio for the steer motor
        3, // 3 inch radius for the wheels
        17, // Only apply 17 stator amps to prevent slip
        steerGains, // Use the specified steer gains
        driveGains, // Use the specified drive gains
        false // CANcoder not reversed from the steer motor. For WCP Swerve X this should be
              // true.
    );
    // TODO: fix the distances for the correct size of chassis. Calibrate offsets for CANCoders. Get all CAN IDs correct. 
    // TODO: Get rid of Units and stay in metrix.
    /**
     * Note: WPI's coordinate system is X forward, Y to the left so make sure all locations are with
     * respect to this coordinate system
     * This particular drive base is 22" x 22"
     */
    SwerveModuleConstants frontRight = m_constantsCreator.createModuleConstants(
        0, 1, 0, -0.538818, Units.inchesToMeters(22.0 / 2.0), Units.inchesToMeters(-22.0 / 2.0));

    SwerveModuleConstants frontLeft = m_constantsCreator.createModuleConstants(
        2, 3, 1, -0.474609, Units.inchesToMeters(22.0 / 2.0), Units.inchesToMeters(22.0 / 2.0));
    SwerveModuleConstants back = m_constantsCreator.createModuleConstants(
        4, 5, 2, -0.928467, Units.inchesToMeters(-22.0 / 2.0), Units.inchesToMeters(-22.0 / 2.0));

    m_robotDrive = new RobotDrive(drivetrain, frontLeft, frontRight, back);

  }
  
  public void driveStopMotion(){
    m_robotDrive.driveStopMotion();
  }
  public void driveRobotCentric(ChassisSpeeds _speeds){
    m_robotDrive.driveRobotCentric(_speeds);
  }
  public void driveFieldCentric(ChassisSpeeds _speeds){
    m_robotDrive.driveFieldCentric(_speeds);
  }
  public void driveAngleFieldCentric(double _x, double _y, Rotation2d _lastTargetAngle){
    m_robotDrive.driveAngleFieldCentric(_x, _y, _lastTargetAngle);
  }
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
