// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class k {
  public static class OI {
    public static final int kDriverControllerPort = 0;
  }
  public static class DRIVE {
    public static final String DriveMode = "DriveMode";
    public static final double DriveGearRatio = 7.85;
    public static final double WheelDiameter_m = .10287;
    public static final double WheelCircumference = Math.PI * WheelDiameter_m;
    public static final double DriveMotorMaxVelocity_RotPerMin = 6380.0;
    public static final double DriveMotorMaxVelocity_RotPerSec = DriveMotorMaxVelocity_RotPerMin / 60.0;
    public static final double DriveWheelMaxVelocity_RotPerSec = DriveMotorMaxVelocity_RotPerSec / DriveGearRatio;
    public static final double MotorPeakEfficiency_Percent = 85;
    public static final double DriveMaxVelocity_MpS = WheelCircumference * DriveWheelMaxVelocity_RotPerSec;

    public static final double WheelBaseY_m = 0.47738;
    public static final double WheelBaseX_m = 0.47851;
  }
}
