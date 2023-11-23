// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commandGroups.AutoDoNothing;
import frc.robot.commands.Arm.ArmDefaultCommand;
import frc.robot.commands.Climber.ClimberDefaultCommand;
import frc.robot.commands.Drive.DrivetrainDefaultCommand;
import frc.robot.commands.Intake.IntakeDefaultCommand;
import frc.robot.commands.Lift.LiftDefaultCommand;
import frc.robot.commands.Shooter.ShooterDefaultCommand;
import frc.robot.commands.Turret.TurretDefaultCommand;
import frc.robot.lib.k;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LiftSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import edu.wpi.first.wpilibj.Notifier;
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


  public static final ArmSubsystem m_armSubsystem = new ArmSubsystem();
  private final ArmDefaultCommand m_armDefaultCommand = new ArmDefaultCommand(m_armSubsystem);

  public static final ClimberSubsystem m_climberSubsystem = new ClimberSubsystem();
  private final ClimberDefaultCommand m_climberDefaultCommand = new ClimberDefaultCommand(m_climberSubsystem);

  public static final DrivetrainSubsystem m_drivetrainSubsystem = new DrivetrainSubsystem();
  private final DrivetrainDefaultCommand m_drivetrainDefaultCommand = new DrivetrainDefaultCommand(m_drivetrainSubsystem);

  public static final IntakeSubsystem m_intakeSubsystem = new IntakeSubsystem();
  private final IntakeDefaultCommand m_intakeDefaultCommand = new IntakeDefaultCommand(m_intakeSubsystem);

  public static final LiftSubsystem m_liftSubsystem = new LiftSubsystem();
  private final LiftDefaultCommand m_liftDefaultCommand = new LiftDefaultCommand(m_liftSubsystem);

  public static final ShooterSubsystem m_shooterSubsystem = new ShooterSubsystem();
  private final ShooterDefaultCommand m_shooterDefaultCommand = new ShooterDefaultCommand(m_shooterSubsystem);

  public static final TurretSubsystem m_turretSubsystem = new TurretSubsystem();
  private final TurretDefaultCommand m_turretDefaultCommand = new TurretDefaultCommand(m_turretSubsystem);

  private Notifier m_telemetry;
  // TODO: Replace with CommandPS5Controller when WPILib gets it working.
  public static final CommandPS4Controller s_driverController = new CommandPS4Controller(k.OI.DRIVER_CONTROLLER_PORT);
  SendableChooser<Command> autoChooser = new SendableChooser<>();
  private void updateDashboard(){
    m_drivetrainSubsystem.updateDashboard();
    m_armSubsystem.updateDashboard();
    m_climberSubsystem.updateDashboard();
    m_intakeSubsystem.updateDashboard();
    m_liftSubsystem.updateDashboard();
    m_shooterSubsystem.updateDashboard();
    m_turretSubsystem.updateDashboard();
  }
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    m_armSubsystem.setDefaultCommand(m_armDefaultCommand);
    m_climberSubsystem.setDefaultCommand(m_climberDefaultCommand);
    m_drivetrainSubsystem.setDefaultCommand(m_drivetrainDefaultCommand);
    m_intakeSubsystem.setDefaultCommand(m_intakeDefaultCommand);
    m_liftSubsystem.setDefaultCommand(m_liftDefaultCommand);
    m_shooterSubsystem.setDefaultCommand(m_shooterDefaultCommand);
    m_turretSubsystem.setDefaultCommand(m_turretDefaultCommand);
    
    // Configure the trigger bindings
    configureBindings();
    autoChooser.setDefaultOption("Do Nothing", new AutoDoNothing());
    SmartDashboard.putData(autoChooser);

    m_telemetry = new Notifier(this::updateDashboard);
    m_telemetry.startPeriodic(0.1);
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
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
