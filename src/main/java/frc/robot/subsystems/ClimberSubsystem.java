// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.lib.k;

public class ClimberSubsystem extends SubsystemBase {
  TalonFX m_motor;

  public void updateDashboard() {

  }

  /** Creates a new ClimberSubsystem. */
  public ClimberSubsystem() {
    m_motor = new TalonFX(20, k.ROBOT.CANFD_Name);
    m_motor.setNeutralMode(NeutralModeValue.Brake);


  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
