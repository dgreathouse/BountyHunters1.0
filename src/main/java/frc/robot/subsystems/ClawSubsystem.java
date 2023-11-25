// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.lib.ISubsystem;

public class ClawSubsystem extends SubsystemBase  implements ISubsystem {
  public void updateDashboard() {

  }

  /** Creates a new ClawSubsystem. */
  public ClawSubsystem() {
    initialize();
  }

  private void initialize() {
    RobotContainer.subsystems.add(this);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
