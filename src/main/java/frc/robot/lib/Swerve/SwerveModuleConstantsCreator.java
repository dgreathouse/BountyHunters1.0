//Copyright (c) 2020-2023 Essexville Hampton Public Schools (FRC 8517)

package frc.robot.lib.Swerve;

public class SwerveModuleConstantsCreator {

    /** True if the steering motor is reversed from the CANcoder */


    public SwerveModuleConstantsCreator() {

    }

    public SwerveModuleConstants createModuleConstants(
            String name,
            int steerId,
            boolean SteerMotorReversed,
            int driveId,
            boolean DriveMotorReversed,
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
