//Copyright (c) 2020-2023 Essexville Hampton Public Schools (FRC 8517)

package frc.robot.lib.Swerve;

import com.ctre.phoenix6.configs.Slot0Configs;

public class SwerveModuleConstants {
    /** CAN ID of the drive motor */
    public int m_driveMotorId = 0;
    /** CAN ID of the steer motor */
    public int m_steerMotorId = 0;
    /** CAN ID of the CANcoder used for azimuth */
    public int m_CANcoderId = 0;
    /** Offset of the CANcoder in degrees */
    public double m_CANcoderOffset_deg = 0;
    /** Gear ratio between drive motor and wheel */
    public double m_driveMotorGearRatio = 0;
    /** Gear ratio between steer motor and CANcoder An example ratio for the SDS Mk4: 12.8 */
    public double m_steerMotorGearRatio = 0;
    /** Wheel diameter of the driving wheel in meters */
    public double m_wheelDiameter_m = 0;
    /**
     * The location of this module's wheels relative to the physical center of the robot in meters
     * along the X axis of the robot
     */
    public double m_locationX_m = 0;
    /**
     * The location of this module's wheels relative to the physical center of the robot in meters
     * along the Y axis of the robot
     */
    public double m_locationY_m = 0;

    /** The steer motor gains */
    public Slot0Configs m_steerMotorGains = new Slot0Configs();
    /** The drive motor gains */
    public Slot0Configs m_driveMotorGains = new Slot0Configs();

    /** The maximum amount of current the drive motors can apply without slippage */
    public double m_slipCurrent_amps = 400;

    /** True if the steering motor is reversed from the CANcoder */
    public boolean m_isSteerMotorReversed = false;

    public SwerveModuleConstants withDriveMotorId(int _id) {
        this.m_driveMotorId = _id;
        return this;
    }

    public SwerveModuleConstants withSteerMotorId(int _id) {
        this.m_steerMotorId = _id;
        return this;
    }

    public SwerveModuleConstants withCANcoderId(int _id) {
        this.m_CANcoderId = _id;
        return this;
    }

    public SwerveModuleConstants withCANcoderOffset(double _offset_deg) {
        this.m_CANcoderOffset_deg = _offset_deg;
        return this;
    }

    public SwerveModuleConstants withDriveMotorGearRatio(double _ratio) {
        this.m_driveMotorGearRatio = _ratio;
        return this;
    }

    public SwerveModuleConstants withSteerMotorGearRatio(double _ratio) {
        this.m_steerMotorGearRatio = _ratio;
        return this;
    }

    public SwerveModuleConstants withWheelRadius(double _diameter_m) {
        this.m_wheelDiameter_m = _diameter_m;
        return this;
    }

    public SwerveModuleConstants withLocationX(double _locationX_m) {
        this.m_locationX_m = _locationX_m;
        return this;
    }

    public SwerveModuleConstants withLocationY(double locationY_m) {
        this.m_locationY_m = locationY_m;
        return this;
    }

    public SwerveModuleConstants withSteerMotorGains(Slot0Configs _gains) {
        this.m_steerMotorGains = _gains;
        return this;
    }

    public SwerveModuleConstants withDriveMotorGains(Slot0Configs _gains) {
        this.m_driveMotorGains = _gains;
        return this;
    }

    public SwerveModuleConstants withSlipCurrent(double _slipCurrent_amps) {
        this.m_slipCurrent_amps = _slipCurrent_amps;
        return this;
    }

    public SwerveModuleConstants withSteerMotorReversed(boolean _isSteerMotorReversed) {
        this.m_isSteerMotorReversed = _isSteerMotorReversed;
        return this;
    }
}
