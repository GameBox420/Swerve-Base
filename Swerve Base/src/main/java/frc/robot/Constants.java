// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.geometry.Translation2d;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public static final class Kinematics {
    public static final double kTrackWidth = Units.inchesToMeters(22.5); // Distance between right and left wheels
    public static final double kWheelBase = Units.inchesToMeters(22.5); // Distance between front and back wheels
    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
      new Translation2d(-kWheelBase / 2, +kTrackWidth / 2),
      new Translation2d(+kWheelBase / 2, +kTrackWidth / 2),
      new Translation2d(-kWheelBase / 2, -kTrackWidth / 2),
      new Translation2d(kWheelBase / 2, -kTrackWidth / 2));
  }


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


    // DRIVE Motor Ports
    public static final int ID_FRONT_LEFT_DRIVE = 4;
    public static final int ID_BACK_LEFT_DRIVE = 2;
    public static final int ID_FRONT_RIGHT_DRIVE = 6;
    public static final int ID_BACK_RIGHT_DRIVE = 8;

    // TURNING Motor Ports
    public static final int ID_FRONT_LEFT_TURN = 3;
    public static final int ID_BACK_LEFT_TURN = 1;
    public static final int ID_FRONT_RIGHT_TURN = 5;
    public static final int ID_BACK_RIGHT_TURN = 7;

    // CANCoder Ids
    public static final int ID_FRONT_LEFT_ENCODER_ABSOLUTE = 6;
    public static final int ID_BACK_LEFT_ENCODER_ABSOLUTE = 8;
    public static final int ID_FRONT_RIGHT_ENCODER_ABSOLUTE = 5;
    public static final int ID_BACK_RIGHT_ENCODER_ABSOLUTE = 7;

    // Invert booleans | We use MK4i modules so the turning motors are inverted
    public static final boolean REVERSED_ENCODER_TURN = true;
    public static final boolean REVERSED_ENCODER_DRIVE = false;
    public static final boolean REVERSED_ENCODER_ABSOLUTE = false;
    public static final boolean REVERSED_GYRO = true;

    // Invert Specific Motors

    public static final boolean REVERSED_FRONT_LEFT_MOTOR_DRIVE = false;
    public static final boolean REVERSED_FRONT_RIGHT_MOTOR_DRIVE = true;
    public static final boolean REVERSED_BACK_LEFT_MOTOR_DRIVE = false;
    public static final boolean REVERSED_BACK_RIGHT_MOTOR_DRIVE = true;

    // Turning encoder offsets

    /* 
     * TODO:
     * Are offsets neccesary?
     * I think if we simply copy the position of the absolute encoder to the turning encoders, it would acomplish the same thing, i think???
    */

    public static final double OFFSET_FRONT_LEFT_ENCODER_ABSOLUTE = Math.toRadians(0.0);
    public static final double OFFSET_BACK_LEFT_ENCODER_ABSOLUTE  = Math.toRadians(0.0);
    public static final double OFFSET_FRONT_RIGHT_ENCODER_ABSOLUTE= Math.toRadians(-50);
    public static final double OFFSET_BACK_RIGHT_ENCODER_ABSOLUTE = Math.toRadians(0.0);

    // Robot drive speeds
    public static final double LIMIT_HARD_SPEED_DRIVE = 3.6; // hard limit for speed of chassis
    public static final double LIMIT_SOFT_SPEED_DRIVE = 1.0; // soft limit for speed of chassis

    // Robot turning speeds
    public static final double LIMIT_SOFT_SPEED_TURN = 1 * 2*Math.PI; // soft limit for module rotation

    // Robot acceleration
    public static final double LIMIT_SOFT_ACCELERATION_SPEED = 1; // soft limit for acceleration (M/S^2)
    public static final double LIMIT_SOFT_ACCELERATION_TURN = 1;  // soft limit for acceleration (M/S^2)
  }
  public static final class OIConstants {
    public static final int kDriverControllerPort = 0;
    public static final double kDeadband = 0.05;

    // Joysticks
    public static final int kDriverYAxis = 1;
    public static final int kDriverXAxis = 0;
    public static final int kDriverRotAxis = 4;

    // Buttons
    public static final int kDriverFieldOrientedButtonId = 1;
    public static final int kDriverResetGyroButtonId = 2;
  }
}
