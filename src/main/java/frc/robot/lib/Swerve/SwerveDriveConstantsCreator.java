//Copyright (c) 2020-2023 Essexville Hampton Public Schools (FRC 8517)

package frc.robot.lib.Swerve;

import com.ctre.phoenix6.configs.Slot0Configs;

public class SwerveDriveConstantsCreator {
    /** Gear ratio between drive motor and wheel */
    public double DriveMotorGearRatio;
    /** Gear ratio between steer motor and CANcoder An example ratio for the SDS Mk4: 12.8 */
    public double SteerMotorGearRatio;
    /** Wheel radius of the driving wheel in inches */
    public double WheelRadius;
    /** The maximum amount of current the drive motors can apply without slippage */
    public double SlipCurrent = 400;

    /** The steer motor gains */
    public Slot0Configs SteerMotorGains;
    /** The drive motor gains */
    public Slot0Configs DriveMotorGains;

    /** True if the steering motor is reversed from the CANcoder */
    public boolean SteerMotorReversed;
    public boolean DriveMotorReversed;
    public SwerveDriveConstantsCreator(
            double swerveModuleDriveRatio,
            double swerveModuleSteerRatio,
            double swerveModuleWheelRadius,
            boolean SteerMotorReversed,
            boolean DriveMotorReversed) {
        this.DriveMotorGearRatio = swerveModuleDriveRatio;
        this.SteerMotorGearRatio = swerveModuleSteerRatio;
        this.WheelRadius = swerveModuleWheelRadius;


        this.SteerMotorReversed = SteerMotorReversed;
        this.DriveMotorReversed = DriveMotorReversed;
    }

    public SwerveModuleConstants createModuleConstants(
            String name,
            int steerId,
            int driveId,
            int cancoderId,
            double cancoderOffset,
            double locationX,
            double locationY) {
        return new SwerveModuleConstants()
                .withName(name)
                .withSteerMotorId(steerId)
                .withDriveMotorId(driveId)
                .withCANcoderId(cancoderId)
                .withCANcoderOffset(cancoderOffset)
                .withLocationX(locationX)
                .withLocationY(locationY)
                .withDriveMotorGearRatio(DriveMotorGearRatio)
                .withSteerMotorGearRatio(SteerMotorGearRatio)
                .withWheelRadius(WheelRadius)
                .withSteerMotorReversed(SteerMotorReversed)
                .withDriveMotorReversed(DriveMotorReversed);
    }
}
