//Copyright (c) 2020-2023 Essexville Hampton Public Schools (FRC 8517)

package frc.robot.lib.Swerve;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TorqueCurrentConfigs;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;


public class SwerveModule {
    private TalonFX m_driveMotor;
    private TalonFX m_steerMotor;
    private CANcoder m_cancoder;

    private StatusSignal<Double> m_drivePosition;
    private StatusSignal<Double> m_driveVelocity;
    private StatusSignal<Double> m_steerPosition;
    private StatusSignal<Double> m_steerVelocity;
    private BaseStatusSignal[] m_signals;
    private double m_driveRotationsPerMeter = 0;

    private PositionVoltage m_angleSetter = new PositionVoltage(0);
    private VelocityTorqueCurrentFOC m_velocitySetter = new VelocityTorqueCurrentFOC(0);

    private SwerveModulePosition m_internalState = new SwerveModulePosition();

    public SwerveModule(SwerveModuleConstants _constants, String _canbusName) {
        m_driveMotor = new TalonFX(_constants.m_driveMotorId, _canbusName);
        m_steerMotor = new TalonFX(_constants.m_steerMotorId, _canbusName);
        m_cancoder = new CANcoder(_constants.m_CANcoderId, _canbusName);

        TalonFXConfiguration talonConfigs = new TalonFXConfiguration();

        talonConfigs.Slot0 = _constants.m_driveMotorGains;
        talonConfigs.TorqueCurrent.PeakForwardTorqueCurrent = _constants.m_slipCurrent_amps;
        talonConfigs.TorqueCurrent.PeakReverseTorqueCurrent = -_constants.m_slipCurrent_amps;
        m_driveMotor.getConfigurator().apply(talonConfigs);

        /* Undo changes for torqueCurrent */
        talonConfigs.TorqueCurrent = new TorqueCurrentConfigs();

        talonConfigs.Slot0 = _constants.m_steerMotorGains;
        // Modify configuration to use remote CANcoder fused
        talonConfigs.Feedback.FeedbackRemoteSensorID = _constants.m_CANcoderId;
        talonConfigs.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
        talonConfigs.Feedback.RotorToSensorRatio = _constants.m_steerMotorGearRatio;

        // Enable continuous wrap for swerve modules
        talonConfigs.ClosedLoopGeneral.ContinuousWrap = true; 

        talonConfigs.MotorOutput.Inverted =
                _constants.m_isSteerMotorReversed
                        ? InvertedValue.Clockwise_Positive
                        : InvertedValue.CounterClockwise_Positive;
        m_steerMotor.getConfigurator().apply(talonConfigs);

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

        double angleToSet_rot = optimized.angle.getRotations();
        m_steerMotor.setControl(m_angleSetter.withPosition(angleToSet_rot));
        double velocityToSet = optimized.speedMetersPerSecond * m_driveRotationsPerMeter;
        m_driveMotor.setControl(m_velocitySetter.withVelocity(velocityToSet));
    }

    BaseStatusSignal[] getSignals() {
        return m_signals;
    }
}
