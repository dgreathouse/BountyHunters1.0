// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commandGroups.AutoDoNothing;
import frc.robot.commands.Drive.DrivetrainDefaultCommand;
import frc.robot.lib.k;
import frc.robot.subsystems.DrivetrainSubsystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;

import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  public static final DrivetrainSubsystem m_drivetrainSubsystem = new DrivetrainSubsystem();
  private final DrivetrainDefaultCommand m_drivetrainDefaultCommand = new DrivetrainDefaultCommand(m_drivetrainSubsystem);
  // TODO: Replace with CommandPS5Controller when WPILib gets it working.
  public static final CommandPS4Controller s_driverController = new CommandPS4Controller(k.OI.kDriverControllerPort);
  SendableChooser<Command> autoChooser = new SendableChooser<>();
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    m_drivetrainSubsystem.setDefaultCommand(m_drivetrainDefaultCommand);
    // Configure the trigger bindings
    configureBindings();
    autoChooser.setDefaultOption("Do Nothing", new AutoDoNothing());
    SmartDashboard.putData(autoChooser);
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   */
  private void configureBindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    //new Trigger(RobotContainer.m_drivetrainSubsystem::exampleCondition).onTrue(new ExampleCommand(m_exampleSubsystem));
    s_driverController.square().onTrue(new InstantCommand(m_drivetrainSubsystem::changeDriveMode, m_drivetrainSubsystem));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return autoChooser.getSelected();
  }
}
