//Copyright (c) 2020-2023 Essexville Hampton Public Schools (FRC 8517)

package frc.robot.lib.Swerve;

public class SwerveModuleConstants {
    public String m_name ="";
    /** CAN ID of the drive motor */
    public int m_driveMotorId = 0;
    /** CAN ID of the steer motor */
    public int m_steerMotorId = 0;
    /** CAN ID of the CANcoder used for azimuth */
    public int m_CANcoderId = 0;
    /** Offset of the CANcoder in degrees */
    public double m_CANcoderOffset_deg = 0;
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

    /** True if the steering motor is reversed from the CANcoder */
    public boolean m_isSteerMotorReversed = false;
    public boolean m_isDriveMotorReversed = false;
    public SwerveModuleConstants withName(String _name){
        this.m_name = _name;
        return this;
    }
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

    public SwerveModuleConstants withLocationX(double _locationX_m) {
        this.m_locationX_m = _locationX_m;
        return this;
    }

    public SwerveModuleConstants withLocationY(double locationY_m) {
        this.m_locationY_m = locationY_m;
        return this;
    }

    public SwerveModuleConstants withSteerMotorReversed(boolean _isrMotorReversed) {
        this.m_isSteerMotorReversed = _isrMotorReversed;
        return this;
    }
      public SwerveModuleConstants withDriveMotorReversed(boolean _isMotorReversed) {
        this.m_isDriveMotorReversed = _isMotorReversed;
        return this;
    }
}
