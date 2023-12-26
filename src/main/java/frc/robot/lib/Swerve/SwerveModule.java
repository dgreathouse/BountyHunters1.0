//Copyright (c) 2020-2023 Essexville Hampton Public Schools (FRC 8517)

package frc.robot.lib.Swerve;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TorqueCurrentConfigs;
import com.ctre.phoenix6.configs.VoltageConfigs;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.lib.k;


public class SwerveModule {
    private TalonFX m_driveMotor;
    private TalonFX m_steerMotor;
    private CANcoder m_cancoder;
    private String m_name;
    private StatusSignal<Double> m_drivePosition;
    private StatusSignal<Double> m_driveVelocity;
    private StatusSignal<Double> m_steerPosition;
    private StatusSignal<Double> m_steerVelocity;
    private BaseStatusSignal[] m_signals;
    private double m_driveRotationsPerMeter = 0;
    private double m_driveSetVelocity_mps = 0;
    private double m_steerSetAngle_deg = 0;
    private double m_driveActualVelocity_mps = 0;
    private double m_steerActualAngle_deg = 0;
    //TODO Determine PID and Constraints for PID in volts
    private ProfiledPIDController m_angleProPID = new ProfiledPIDController(0, 0, 0, new TrapezoidProfile.Constraints(0,0));
    //private ProfiledPIDController m_driveProPID = new ProfiledPIDController(0, 0, 0, new TrapezoidProfile.Constraints(0,0));
    //private PIDController m_anglePID = new PIDController(.125, .1, 0.0);
    // TODO Determine PID for mps to volts
    private PIDController m_drivePID = new PIDController(.125, .1, 0.0);
    // TODO Determine FF kv,ks for Volts per mps. First no load single motor test showed .11 volts per RPS
    // TODO Test by setting various voltages and measuring drive velocity to get kv. ks is the amount it takes to move the robot 
    private SimpleMotorFeedforward m_driveFF = new SimpleMotorFeedforward(0.2, .11);
    private VoltageOut m_angleVoltageOut = new VoltageOut(0.0);
    private VoltageOut m_driveVoltageOut = new VoltageOut(0.0);
    // private PositionVoltage m_angleSetter = new PositionVoltage(0);
    // //private VelocityTorqueCurrentFOC m_velocitySetter = new VelocityTorqueCurrentFOC(0);
    // private VelocityVoltage m_velocitySetter = new VelocityVoltage(0);

    private SwerveModulePosition m_internalState = new SwerveModulePosition();

    public SwerveModule(SwerveModuleConstants _constants, String _canbusName) {
        m_driveMotor = new TalonFX(_constants.m_driveMotorId, _canbusName);
        m_steerMotor = new TalonFX(_constants.m_steerMotorId, _canbusName);
        m_cancoder = new CANcoder(_constants.m_CANcoderId, _canbusName);
        m_name = _constants.m_name;
        // Configure Drive Motor
        TalonFXConfiguration talonDriveConfigs = new TalonFXConfiguration();

        talonDriveConfigs.Slot0 = _constants.m_driveMotorGains;
        
        //talonDriveConfigs.TorqueCurrent.PeakForwardTorqueCurrent = _constants.m_slipCurrent_amps;
        //talonDriveConfigs.TorqueCurrent.PeakReverseTorqueCurrent = -_constants.m_slipCurrent_amps;
        m_driveMotor.getConfigurator().apply(talonDriveConfigs);

        // Configure Steer Motor
        m_angleProPID.enableContinuousInput(-180.0, +180.0);
        TalonFXConfiguration talonSteerConfigs = new TalonFXConfiguration();
        talonSteerConfigs.Slot0 = _constants.m_steerMotorGains;
        // Modify configuration to use remote CANcoder fused
        // TODO Get rid of fused cancoder. Just rely on the motor encoder.
        //talonSteerConfigs.Feedback.FeedbackRemoteSensorID = _constants.m_CANcoderId;
        //talonSteerConfigs.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
        talonSteerConfigs.Feedback.RotorToSensorRatio = _constants.m_steerMotorGearRatio;


        // Enable continuous wrap for swerve modules
        // TODO: This means nothing since we are using voltage open loop and closing the loop in software not the motor.
        talonSteerConfigs.ClosedLoopGeneral.ContinuousWrap = true; 

        talonSteerConfigs.MotorOutput.Inverted =
                _constants.m_isSteerMotorReversed
                        ? InvertedValue.Clockwise_Positive
                        : InvertedValue.CounterClockwise_Positive;
        m_steerMotor.getConfigurator().apply(talonSteerConfigs);

        CANcoderConfiguration cancoderConfigs = new CANcoderConfiguration();
        cancoderConfigs.MagnetSensor.MagnetOffset = _constants.m_CANcoderOffset_deg;
        m_cancoder.getConfigurator().apply(cancoderConfigs);

        m_drivePosition = m_driveMotor.getPosition();
        m_driveVelocity = m_driveMotor.getVelocity();
        m_steerPosition = m_cancoder.getPosition();
        m_steerVelocity = m_cancoder.getVelocity();

        m_signals = new BaseStatusSignal[4];
        m_signals[0] = m_drivePosition;
        m_signals[1] = m_driveVelocity;
        m_signals[2] = m_steerPosition;
        m_signals[3] = m_steerVelocity;

        /* Calculate the ratio of drive motor rotation to meter on ground */
        double rotationsPerWheelRotation = _constants.m_driveMotorGearRatio;
        double metersPerWheelRotation = Math.PI * _constants.m_wheelDiameter_m;
        m_driveRotationsPerMeter = rotationsPerWheelRotation / metersPerWheelRotation;
    }

    public SwerveModulePosition getPosition(boolean _refresh) {
        if (_refresh) {
            /* Refresh all signals */
            m_drivePosition.refresh();
            m_driveVelocity.refresh();
            m_steerPosition.refresh();
            m_steerVelocity.refresh();
        }

        /* Now latency-compensate our signals */
        double drive_rot = BaseStatusSignal.getLatencyCompensatedValue(m_drivePosition, m_driveVelocity);
        double angle_rot = BaseStatusSignal.getLatencyCompensatedValue(m_steerPosition, m_steerVelocity);

        /* And push them into a SwerveModuleState object to return */
        m_internalState.distanceMeters = drive_rot / m_driveRotationsPerMeter;
        /* Angle is already in terms of steer rotations */
        m_internalState.angle = Rotation2d.fromRotations(angle_rot);

        return m_internalState;
    }

    public void apply(SwerveModuleState _state) {
        var optimized = SwerveModuleState.optimize(_state, m_internalState.angle);

        m_steerSetAngle_deg = optimized.angle.getDegrees();
        m_driveSetVelocity_mps = optimized.speedMetersPerSecond;

        m_driveActualVelocity_mps = m_driveMotor.getVelocity().getValueAsDouble() / m_driveRotationsPerMeter;
        m_steerActualAngle_deg = m_steerMotor.getPosition().getValueAsDouble() * 360.0 / k.STEER.GEAR_RATIO;
        
        // Calculate the PID value for the angle in Degrees
        double angleVolts = m_angleProPID.calculate(m_steerActualAngle_deg,m_steerSetAngle_deg);
        m_steerMotor.setControl(m_angleVoltageOut.withOutput(0));

        // Calculate the PID value of velocity in MPS
        double driveVolts = m_drivePID.calculate(m_driveActualVelocity_mps, m_driveSetVelocity_mps);
        driveVolts += m_driveFF.calculate(m_driveActualVelocity_mps);
        m_driveMotor.setControl(m_driveVoltageOut.withOutput(SmartDashboard.getNumber("Volts", 0)));
    }
    void updateDashboard(){
        SmartDashboard.putNumber(m_name+"_set_deg", m_steerSetAngle_deg);
        SmartDashboard.putNumber(m_name+"_set_mps", m_driveSetVelocity_mps);
        SmartDashboard.putNumber(m_name+"_act_deg", m_steerActualAngle_deg);
        SmartDashboard.putNumber(m_name+"_act_mps", m_driveActualVelocity_mps);
        
    }
    BaseStatusSignal[] getSignals() {
        return m_signals;
    }
}
