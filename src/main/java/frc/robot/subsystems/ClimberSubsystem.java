//Copyright (c) 2020-2023 Essexville Hampton Public Schools (FRC 8517)

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.lib.ICommand;
import frc.robot.lib.ISubsystem;
import frc.robot.lib.k;

public class ClimberSubsystem extends SubsystemBase  implements ISubsystem{
  TalonFX m_motor;

  public void updateDashboard() {
    if(this.getCurrentCommand() != null){
      ((ICommand)this.getCurrentCommand()).updateDashboard();
      SmartDashboard.putString("ClimberSubsystem", this.getCurrentCommand().getName());
    }
  }

  /** Creates a new ClimberSubsystem. */
  public ClimberSubsystem() {
    m_motor = new TalonFX(20, k.ROBOT.CANVORE_CANFD_NAME);
    m_motor.setNeutralMode(NeutralModeValue.Brake);

    initialize();
  }
  private void initialize(){
    RobotContainer.subsystems.add(this);
  }
  // public void setTestVoltage(double _volts){
    
  // }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
