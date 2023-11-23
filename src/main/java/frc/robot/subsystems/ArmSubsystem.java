//Copyright (c) 2020-2023 Essexville Hampton Public Schools (FRC 8517)

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.lib.k;

/**  <b>Arm Subsystem</b>
 *  <p>
 * 
 */
public class ArmSubsystem extends SubsystemBase {
  TalonFX m_motor;

  public void updateDashboard() {

  }

  /** Creates a new ArmSubsystem. */
  public ArmSubsystem() {
    m_motor = new TalonFX(20, k.ROBOT.CANFD_NAME);
    m_motor.setNeutralMode(NeutralModeValue.Brake);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
