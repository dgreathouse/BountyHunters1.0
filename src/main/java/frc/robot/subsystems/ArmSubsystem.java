package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.lib.k;

public class ArmSubsystem extends SubsystemBase {
  TalonFX m_motor;

  public void updateDashboard() {

  }

  /** Creates a new ArmSubsystem. */
  public ArmSubsystem() {
    m_motor = new TalonFX(20, k.ROBOT.CANFD_Name);
    m_motor.setNeutralMode(NeutralModeValue.Brake);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
