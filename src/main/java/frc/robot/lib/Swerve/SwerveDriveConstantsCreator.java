//Copyright (c) 2020-2023 Essexville Hampton Public Schools (FRC 8517)

package frc.robot.lib.Swerve;

public class SwerveDriveConstantsCreator {
    /** Gear ratio between drive motor and wheel */
    public double DriveMotorGearRatio;
    /** Gear ratio between steer motor and CANcoder An example ratio for the SDS Mk4:12.8  */
    public double SteerMotorGearRatio;
    /** Wheel radius of the driving wheel in inches */
    public double WheelRadius;
    /** True if the steering motor is reversed from the CANcoder */
    public boolean SteerMotorReversed;
    public boolean DriveMotorReversed;

    public SwerveDriveConstantsCreator(

            boolean SteerMotorReversed,
            boolean DriveMotorReversed) {
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
                .withSteerMotorReversed(SteerMotorReversed)
                .withDriveMotorReversed(DriveMotorReversed);
    }
}
