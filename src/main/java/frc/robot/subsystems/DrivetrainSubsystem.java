// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.k;
import frc.robot.lib.EDriveMode;
import frc.robot.lib.RobotDrive;
import frc.robot.lib.SwerveDriveConstantsCreator;
import frc.robot.lib.SwerveDriveTrainConstants;
import frc.robot.lib.SwerveModuleConstants;

public class DrivetrainSubsystem extends SubsystemBase {
  public RobotDrive m_robotDrive;
  public EDriveMode m_driveMode = EDriveMode.FIELD_CENTRIC;
  private Notifier m_telemetry;
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
        k.DRIVE.DriveGearRatio, //  ratio for the drive motor
        k.DRIVE.SteerGearRatioToCANCoder, // ratio for the steer motor
        k.DRIVE.WheelDiameter_m, // 4 inch diameter for the wheels
        20, // Only apply 17 stator amps to prevent slip
        steerGains, // Use the specified steer gains
        driveGains, // Use the specified drive gains
        true // CANcoder not reversed from the steer motor. For WCP Swerve X this should be
              // true.
    );
    // TODO: Calibrate offsets for CANCoders. Get all CAN IDs correct. 
    
    /**
     * Note: WPI's coordinate system is X forward, Y to the left so make sure all locations are with
     * respect to this coordinate system
     * This particular drive base is 22" x 22"
     */
    SwerveModuleConstants frontRight = m_constantsCreator.createModuleConstants(
        0, 1, 0, -0.538818,k.DRIVE.WheelBaseX_m / 2.0, -k.DRIVE.WheelBaseY_m / 2.0);

    SwerveModuleConstants frontLeft = m_constantsCreator.createModuleConstants(
        2, 3, 1, -0.474609, k.DRIVE.WheelBaseX_m / 2.0, k.DRIVE.WheelBaseY_m / 2.0);
    SwerveModuleConstants back = m_constantsCreator.createModuleConstants(
        4, 5, 2, -0.928467, -k.DRIVE.WheelBaseX_m / 2.0, 0.0);

    m_robotDrive = new RobotDrive(drivetrain, frontLeft, frontRight, back);
    m_telemetry = new Notifier(this::telemetry);
    m_telemetry.startPeriodic(0.1);

  }
  public void telemetry(){
    SmartDashboard.putString(k.DRIVE.T_DriveMode, m_driveMode.toString());
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
  public void drivePolarFieldCentric(double _driveAngle, double _speed, double _robotAngle){
    double x = Math.sin(_driveAngle) * _speed;
    double y = Math.cos(_driveAngle) * _speed;
   
    driveAngleFieldCentric(x, y, new Rotation2d(Units.degreesToRadians(_robotAngle)));
  }
  public void changeDriveMode(){
    switch(m_driveMode){
      case FIELD_CENTRIC:
        m_driveMode = EDriveMode.ANGLE_FIELD_CENTRIC;
      break;
      case ANGLE_FIELD_CENTRIC:
        m_driveMode = EDriveMode.ROBOT_CENTRIC;
      break;
      case ROBOT_CENTRIC:
        m_driveMode = EDriveMode.FIELD_CENTRIC;
      break;
      default:
        m_driveMode = EDriveMode.ROBOT_CENTRIC;
      break;
    }
  }
  public EDriveMode getDriveMode(){
    return m_driveMode;
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
