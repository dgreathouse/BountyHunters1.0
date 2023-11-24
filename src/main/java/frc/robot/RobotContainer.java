//Copyright (c) 2020-2023 Essexville Hampton Public Schools (FRC 8517)

package frc.robot;

import frc.robot.commandGroups.AutoDoNothing;
import frc.robot.commands.Arm.ArmDefaultCommand;
import frc.robot.commands.Climber.ClimberDefaultCommand;
import frc.robot.commands.Drive.DrivetrainDefaultCommand;
import frc.robot.commands.Elevator.ElevatorDefaultCommand;
import frc.robot.commands.Extension.ExtensionDefaultCommand;
import frc.robot.commands.Intake.IntakeDefaultCommand;
import frc.robot.commands.Lift.LiftDefaultCommand;
import frc.robot.commands.Shooter.ShooterDefaultCommand;
import frc.robot.commands.Turret.TurretDefaultCommand;
import frc.robot.lib.GD;
import frc.robot.lib.k;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ExtensionSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LiftSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * In command-based projects, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Subsystems, commands, and trigger mappings should be defined here.
 * 
 */
public class RobotContainer {

  private static final ArmSubsystem m_armSubsystem = new ArmSubsystem();
  private final ArmDefaultCommand m_armDefaultCommand = new ArmDefaultCommand(m_armSubsystem);

  private static final ClimberSubsystem m_climberSubsystem = new ClimberSubsystem();
  private final ClimberDefaultCommand m_climberDefaultCommand = new ClimberDefaultCommand(m_climberSubsystem);

  private static final DrivetrainSubsystem m_drivetrainSubsystem = new DrivetrainSubsystem();
  private final DrivetrainDefaultCommand m_drivetrainDefaultCommand = new DrivetrainDefaultCommand(m_drivetrainSubsystem);

  private static final ElevatorSubsystem m_elevatorSubsystem = new ElevatorSubsystem();
  private final ElevatorDefaultCommand m_elevatorDefaultCommand = new ElevatorDefaultCommand(m_elevatorSubsystem);

  private static final ExtensionSubsystem m_extensionSubsystem = new ExtensionSubsystem();
  private final ExtensionDefaultCommand m_extensionDefaultCommand = new ExtensionDefaultCommand(m_extensionSubsystem);

  private static final IntakeSubsystem m_intakeSubsystem = new IntakeSubsystem();
  private final IntakeDefaultCommand m_intakeDefaultCommand = new IntakeDefaultCommand(m_intakeSubsystem);

  private static final LiftSubsystem m_liftSubsystem = new LiftSubsystem();
  private final LiftDefaultCommand m_liftDefaultCommand = new LiftDefaultCommand(m_liftSubsystem);

  private static final ShooterSubsystem m_shooterSubsystem = new ShooterSubsystem();
  private final ShooterDefaultCommand m_shooterDefaultCommand = new ShooterDefaultCommand(m_shooterSubsystem);

  private static final TurretSubsystem m_turretSubsystem = new TurretSubsystem();
  private final TurretDefaultCommand m_turretDefaultCommand = new TurretDefaultCommand(m_turretSubsystem);

  private Notifier m_telemetry;

  
  public static final CommandPS5Controller s_driverController = new CommandPS5Controller(k.OI.DRIVER_CONTROLLER_PORT);
  public static final CommandPS5Controller s_operatorController = new CommandPS5Controller(k.OI.OPERATOR_CONTROLLER_PORT);

  SendableChooser<Command> autoChooser = new SendableChooser<>();

  private void updateDashboard(){
    SmartDashboard.putString("RobotMode", GD.G_RobotMode.toString());
    m_drivetrainSubsystem.updateDashboard();
    m_armSubsystem.updateDashboard();
    m_climberSubsystem.updateDashboard();
    m_elevatorSubsystem.updateDashboard();
    m_extensionSubsystem.updateDashboard();
    m_intakeSubsystem.updateDashboard();
    m_liftSubsystem.updateDashboard();
    m_shooterSubsystem.updateDashboard();
    m_turretSubsystem.updateDashboard();
  }
  /** This is the constructor for the class. Setup of the subsystems, 
   * buttons and autonomous options should be done here */
  public RobotContainer() {
    m_armSubsystem.setDefaultCommand(m_armDefaultCommand);
    m_climberSubsystem.setDefaultCommand(m_climberDefaultCommand);
    m_drivetrainSubsystem.setDefaultCommand(m_drivetrainDefaultCommand);
    m_elevatorSubsystem.setDefaultCommand(m_elevatorDefaultCommand);
    m_extensionSubsystem.setDefaultCommand(m_extensionDefaultCommand);
    m_intakeSubsystem.setDefaultCommand(m_intakeDefaultCommand);
    m_liftSubsystem.setDefaultCommand(m_liftDefaultCommand);
    m_shooterSubsystem.setDefaultCommand(m_shooterDefaultCommand);
    m_turretSubsystem.setDefaultCommand(m_turretDefaultCommand);
    
    // Configure the trigger bindings
    configureBindings();

    // Add all autonomous command groups to the list on the Smartdashboard
    autoChooser.setDefaultOption("Do Nothing", new AutoDoNothing());
    SmartDashboard.putData(autoChooser);

    // Setup the dashboard notifier that runs at a slower rate than our main robot periodic.
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
   * @return the command to run in autonomous routine
   */
  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
