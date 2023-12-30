//Copyright (c) 2020-2023 Essexville Hampton Public Schools (FRC 8517)

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.lib.EDriveMode;
import frc.robot.lib.ICommand;
import frc.robot.lib.ISubsystem;
import frc.robot.lib.k;
import frc.robot.lib.Swerve.SwerveDrive;

public class DrivetrainSubsystem extends SubsystemBase implements ISubsystem{
  public SwerveDrive m_robotDrive;
  public EDriveMode m_driveMode = EDriveMode.ANGLE_FIELD_CENTRIC;
  public Rotation2d m_lastTargetAngle = new Rotation2d();
  /** Creates a new DrivetrainSubsystem. */
  public DrivetrainSubsystem() {
     m_robotDrive = new SwerveDrive();
     
    initialize(); 
  }
  public void initialize(){
    RobotContainer.subsystems.add(this);
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

  public void driveAngleFieldCentric(double _x, double _y){
    m_robotDrive.driveAngleFieldCentric(_x, _y, m_lastTargetAngle);
  }

  public void drivePolarFieldCentric(double _driveAngle_deg, double _speed, double _robotAngle_deg){
    double x = Math.sin(Units.degreesToRadians(_driveAngle_deg)) * _speed;
    double y = Math.cos(Units.degreesToRadians(_driveAngle_deg)) * _speed;
    m_lastTargetAngle = new Rotation2d(Units.degreesToRadians(_robotAngle_deg));
    driveAngleFieldCentric(x, y);
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

  public double getRobotAngle(){
    return m_robotDrive.getRobotYaw();
  }

  public void resetYaw(){
    m_robotDrive.resetYaw();
  }

  public void setLastTargetAngle(Rotation2d _targetAngle){
    m_lastTargetAngle = _targetAngle;
  }

  public void updateDashboard(){
    SmartDashboard.putString(k.DRIVE.T_DRIVER_MODE, m_driveMode.toString());
    if(this.getCurrentCommand() != null){
      ((ICommand)this.getCurrentCommand()).updateDashboard();
      SmartDashboard.putString("DrivetrainSubsystem", this.getCurrentCommand().getName());
    }
    m_robotDrive.updateDashboard();
  }

  @Override
  public void periodic() {
    

  }
}
