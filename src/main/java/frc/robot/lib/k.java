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
    public static final double PERIOD = 0.2;
    public static final String CANFD_NAME = "CANFD";
  }
  public static class OI {
    public static final int DRIVER_CONTROLLER_PORT = 0;
  }
  public static class DRIVEBASE {
    public static final double WHEEL_BASE_Y_m = 0.47738;
    public static final double WHEEL_BASE_X_m = 0.47851;
    private static final double WHEEL_BASE_XY_AVG_m = (WHEEL_BASE_Y_m + WHEEL_BASE_X_m)/2.0;
    private static final double WHEEL_BASE_CIRCUMFERENCE_m = Math.PI * WHEEL_BASE_XY_AVG_m;
    private static final double WHEEL_BASE_MeterPerRad = WHEEL_BASE_CIRCUMFERENCE_m/(2* Math.PI);
    
  }
  public static class DRIVE {
    public static final String T_DRIVER_MODE = "DriveMode";
    private static final double MOTOR_PINION_TEETH = 10.0;
    private static final double GEAR_1_TEETH = 34.0;
    private static final double GEAR_2_DRIVE_TEETH = 26.0;
    private static final double GEAR_2_DRIVEN_TEETH = 20.0;
    private static final double GEAR_BEVEL_DRIVE_TEETH = 15.0 ;
    private static final double GEAR_BEVEL_DRIVEN_TEETH = 45.0;

    public static final double GEAR_RATIO = (GEAR_1_TEETH/MOTOR_PINION_TEETH) * (GEAR_2_DRIVEN_TEETH / GEAR_2_DRIVE_TEETH) * (GEAR_BEVEL_DRIVEN_TEETH / GEAR_BEVEL_DRIVE_TEETH);
    public static final double WHEEL_DIAMETER_m = 0.10287;
    private static final double WHEEL_CIRCUMFERENCE_m = Math.PI * WHEEL_DIAMETER_m;
    private static final double MOTOR_MAX_VELOCITY_RotPerMin = 6380.0;
    private static final double MOTOR_MAX_VELOCITY_RotPerSec = MOTOR_MAX_VELOCITY_RotPerMin / 60.0;
    private static final double WHEEL_MAX_VELOCITY_RotPerSec = MOTOR_MAX_VELOCITY_RotPerSec / GEAR_RATIO;
    private static final double MOTOR_PEAK_EFFICIENCY_percent = 85.0;
    public static final double MAX_VELOCITY_MeterPerSec = WHEEL_CIRCUMFERENCE_m * WHEEL_MAX_VELOCITY_RotPerSec * MOTOR_PEAK_EFFICIENCY_percent / 100.0;
    public static final double MAX_ANGULAR_VELOCITY_RadianPerSec = MAX_VELOCITY_MeterPerSec * (1/DRIVEBASE.WHEEL_BASE_MeterPerRad);

  }
  public static class STEER {
    private static final double MOTOR_PINION_TEETH = 8.0;
    private static final double MOTOR_DRIVE_GEAR_TEETH = 24.0;
    private static final double GEAR_1_DRIVE_TEETH = 14.0;
    private static final double GEAR_1_DRIVEN_TEETH = 72.0;
    private static final double CANCODER_GEAR_RATIO = 1.0;
    private static final double GEAR_RATIO = 1/((MOTOR_PINION_TEETH/MOTOR_DRIVE_GEAR_TEETH)*(GEAR_1_DRIVE_TEETH/GEAR_1_DRIVEN_TEETH));
    public static final double GEAR_RATIO_TO_CANCODER = GEAR_RATIO * CANCODER_GEAR_RATIO;
  }
}
