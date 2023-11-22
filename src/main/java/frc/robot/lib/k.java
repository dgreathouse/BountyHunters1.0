// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.lib;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class k {
  public static class ROBOT {
    public static final double Period = 0.2;
  }
  public static class OI {
    public static final int kDriverControllerPort = 0;
  }
  public static class DRIVE {
    public static final String T_DriveMode = "DriveMode";
    private static final double DriveMotorPinionTeeth = 10.0;
    private static final double DriveGear1Teeth = 34.0;
    private static final double DriveGear2DriveTeeth = 26.0;
    private static final double DriveGear2DrivenTeeth = 20.0;
    private static final double DriveGearBevelDriveTeeth = 15.0 ;
    private static final double DriveGearBevelDrivenTeeth = 45.0;

    public static final double DriveGearRatio = (DriveGear1Teeth/DriveMotorPinionTeeth) * (DriveGear2DrivenTeeth / DriveGear2DriveTeeth) * (DriveGearBevelDrivenTeeth / DriveGearBevelDriveTeeth);
    public static final double WheelDiameter_m = 0.10287;
    public static final double WheelCircumference = Math.PI * WheelDiameter_m;
    private static final double DriveMotorMaxVelocity_RotPerMin = 6380.0;
    private static final double DriveMotorMaxVelocity_RotPerSec = DriveMotorMaxVelocity_RotPerMin / 60.0;
    private static final double DriveWheelMaxVelocity_RotPerSec = DriveMotorMaxVelocity_RotPerSec / DriveGearRatio;
    private static final double MotorPeakEfficiency_Percent = 85.0;
    public static final double DriveMaxVelocity_MpSec = WheelCircumference * DriveWheelMaxVelocity_RotPerSec * MotorPeakEfficiency_Percent / 100.0;

    public static final double WheelBaseY_m = 0.47738;
    public static final double WheelBaseX_m = 0.47851;
    private static final double WheelBaseXYAvg_m = (WheelBaseY_m + WheelBaseX_m)/2.0;
    private static final double WheelBaseCircumference_m = Math.PI * WheelBaseXYAvg_m;
    private static final double WheelBase_MPerRad = WheelBaseCircumference_m/(2* Math.PI);
    public static final double DriveMaxAngularVelocity_RadPerSec = DriveMaxVelocity_MpSec * (1/WheelBase_MPerRad);

    private static final double SteerMotorPinionTeeth = 8.0;
    private static final double SteerMotorDrivenGearTeeth = 24.0;
    private static final double SteerGear1DriveTeeth = 14.0;
    private static final double SteerGear1DrivenTeeth = 72.0;
    private static final double SteerCANCoderGearRatio = 1.0;
    public static final double SteerGearRatio = 1/((SteerMotorPinionTeeth/SteerMotorDrivenGearTeeth)*(SteerGear1DriveTeeth/SteerGear1DrivenTeeth));
    public static final double SteerGearRatioToCANCoder = SteerGearRatio * SteerCANCoderGearRatio;

  }
}
