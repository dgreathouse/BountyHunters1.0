//Copyright (c) 2020-2023 Essexville Hampton Public Schools (FRC 8517)

package frc.robot.subsystems;


import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.lib.ISubsystem;
import frc.robot.lib.k;

/**  Arm Subsystem.<p>
 * This arm utilizes a motor and gearbox to rotate a solid structure against gravity in a circular motion.
 * Voltage mode of the motor is used and not a specific control method in the TalonFX controller.
 * For the algorithms to work a arm is assumed to be at 0 degrees when horizontal.
 * A 0 degree horizontal arm is usually not what is used in robotics since the arm starts at a different location.
 * There are two ways to deal with this, offset the motor at initialization or offset it when doing this calculation.
 * This approach offsets the motor angle at intialization.
 */
public class ArmSubsystem extends SubsystemBase  implements ISubsystem{
  TalonFX m_motor;
  PIDController m_rotPid;
  VoltageOut m_VoltageOut;
  double m_currentAngle;
  
  public void updateDashboard() {

  }
  /** Creates a new ArmSubsystem. */
  public ArmSubsystem() {
    m_motor = new TalonFX(20, k.ROBOT.CANVORE_CANFD_NAME);
    m_VoltageOut = new VoltageOut(0, true, false);
    // PID values are the velocity of the motor scaled to Voltage.
    m_rotPid = new PIDController(k.ARM.ROTATION_PID_Kp, k.ARM.ROTATION_PID_Ki, k.ARM.ROTATION_PID_Kd);
    initialize();
  }
  private void initialize(){
    m_motor.setNeutralMode(NeutralModeValue.Brake);
    // Offset the motor by the distance from zero in degrees
    // Motor position is in rotations
    m_motor.setPosition(360.0 * k.ARM.OFFSET_FROM_ZERO_deg / k.ARM.GEAR_RATIO);
    RobotContainer.subsystems.add(this);

  }
  /**
   * 
   * @param _requestedAngle_Deg The angle to goto in degrees
   * @param _velocity_DegPerSec The requrested angular velocity 
   */
  public void setAngle(double _requestedAngle_Deg, double _velocity_DegPerSec){
    
    double actualAngle_Rad = Units.degreesToRadians(360.0 * m_motor.getPosition().getValueAsDouble() / k.ARM.GEAR_RATIO);
    // Calculate the position error value. This ends up being the Velocity of the arm
    double rotPID = m_rotPid.calculate(actualAngle_Rad, Units.degreesToRadians(_requestedAngle_Deg));
    // Convert the requested max velcity to RadPerSec
    double velocity_RadPerSec = Units.degreesToRadians(_velocity_DegPerSec);
    // Get the actual velocity of the motor
    double actualVelocity_RadPerSec = Units.degreesToRadians(m_motor.getVelocity().getValueAsDouble() * 360.0);

    // Clamp the velocity 
    rotPID = MathUtil.clamp(rotPID,-velocity_RadPerSec, velocity_RadPerSec);
    // Add a little boost to the velocity to overcome gravity or sticktion.
    rotPID = rotPID > 0 
             ? (k.ARM.MAX_ANGULAR_VELOCITY_RadianPerSec - actualVelocity_RadPerSec) * k.ARM.FF_Kv * (Units.degreesToRadians(_requestedAngle_Deg) - actualAngle_Rad)
             : rotPID;
    // Feedforward term is all summed up in Kg which is the amount of voltage to hold the arm at zero Degrees.
    double ff = k.ARM.FF_Kg * Math.cos(actualAngle_Rad);
    
    m_motor.setControl(m_VoltageOut.withOutput(ff + rotPID));
  }
  public void setTestVoltage(double _volts){
    m_motor.setVoltage(_volts);
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
