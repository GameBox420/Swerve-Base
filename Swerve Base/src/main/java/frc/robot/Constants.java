// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static final class ModuleConstants {

    //Robot Geometry
    public static final double WheelDiameterMeters = Units.inchesToMeters(4.0);
    public static final double DriveMotorGearRatio = 8.14 / 1.0; // Drive ratio of 8.14 : 1
    public static final double TurningMotorGearRatio = 1.0 / (150.0 / 7.0); // Turning ratio of (150 / 7) : 1
    public static final double DriveEncoderRot2Meter = DriveMotorGearRatio * Math.PI * WheelDiameterMeters;
    public static final double TurningEncoderRot2Rad = TurningMotorGearRatio * 2 * Math.PI;
    public static final double DriveEncoderRPM2MeterPerSec = DriveEncoderRot2Meter / 60.0;
    public static final double TurningEncoderRPM2RadPerSec = TurningEncoderRot2Rad / 60.0;

    //PID Config
    public static final double TURNING_Proportional = 0.5;
    public static final double TURNING_Integral = 0.0;
    public static final double TURNING_Derivitive = 0.0;
}

  public static class SwerveSubsystemConstants {



  }
}
