//Copyright (c) 2020-2023 Essexville Hampton Public Schools (FRC 8517)

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.lib.EDriveMode;
import frc.robot.lib.ISubsystem;
import frc.robot.lib.RobotDrive;
import frc.robot.lib.k;

public class DrivetrainSubsystem extends SubsystemBase  implements ISubsystem{
  public RobotDrive m_robotDrive;
  public EDriveMode m_driveMode = EDriveMode.FIELD_CENTRIC;

  /** Creates a new DrivetrainSubsystem. */
  public DrivetrainSubsystem() {
     m_robotDrive = new RobotDrive();
    initialize(); 
  }
  public void initialize(){
    RobotContainer.subsystems.add(this);
  }
  public void updateDashboard(){
    SmartDashboard.putString(k.DRIVE.T_DRIVER_MODE, m_driveMode.toString());
    m_robotDrive.updateDashboard();
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
  public double getRobotAngle(){
    return m_robotDrive.getRobotYaw();
  }
  public void setTestVoltage(double _volts){
    
  }
  @Override
  public void periodic() {
    

  }
}
