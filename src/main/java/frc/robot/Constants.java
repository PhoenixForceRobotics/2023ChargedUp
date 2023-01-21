// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.I2C.Port;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final class DrivebaseConstants 
    {
        public static final int WHEEL_FL_PORT = 1;
        public static final int WHEEL_FR_PORT = 2;
        public static final int WHEEL_BL_PORT = 3;
        public static final int WHEEL_BR_PORT = 4;

        public static final int FR_BUTTERFLY_FORWARD_PORT = 0;
        public static final int FR_BUTTERFLY_REVERSE_PORT = 1;
        public static final int FL_BUTTERFLY_FORWARD_PORT = 2;
        public static final int FL_BUTTERFLY_REVERSE_PORT = 3;
        public static final int BR_BUTTERFLY_FORWARD_PORT = 4;
        public static final int BR_BUTTERFLY_REVERSE_PORT = 5;
        public static final int BL_BUTTERFLY_FORWARD_PORT = 6;
        public static final int BL_BUTTERFLY_REVERSE_PORT = 7;

        public static final double GEAR_RATIO = (double) 12 / (double) 72; // output / input
        public static final double WHEEL_DIAMETER = 0.1524; // In meters (6 in wheels)
        
        public static final double MAX_MOTOR_PERCENTAGE_OUTPUT = 0.9;
        
        public static final boolean WHEEL_FL_REVERSED = true;
        public static final boolean WHEEL_FR_REVERSED = false;
        public static final boolean WHEEL_BL_REVERSED = true;
        public static final boolean WHEEL_BR_REVERSED = false;

        public static final Translation2d WHEEL_FL_LOCATION = new Translation2d(0.381, 0.381);
        public static final Translation2d WHEEL_FR_LOCATION = new Translation2d(0.381, -0.381);
        public static final Translation2d WHEEL_BL_LOCATION = new Translation2d(-0.381, 0.381);
        public static final Translation2d WHEEL_BR_LOCATION = new Translation2d(-0.381, -0.381);
        public static final Translation2d FRONT_CENTER_LOCATION = new Translation2d(0, 0.381); // Helpful for evassive manuvers


        // public static final HolonomicDriveController HOLONOMIC_DRIVE_CONTROLLER = new HolonomicDriveController(
        //     VELOCITY_PID, // PID to control error in the x direction
        //     POSITION_PID, // PID to control error in the y direction
        //     new ProfiledPIDController(1, 0, 0, // PID to controller error in angle
        //         new TrapezoidProfile.Constraints(6.28, 3.14))
        // );
        // Here, our rotation profile constraints were a max velocity
        // of 1 rotation per second and a max acceleration of 180 degrees
        // per second squared.

        public static final Pose2d STARTING_POSE = new Pose2d(
            0, 
            0, 
            new Rotation2d()
        );

        public static final Pose2d WALL = new Pose2d(
            -1,
            -1,
            new Rotation2d()
        );

        public static final double MAX_OBTAINABLE_WHEEL_VELOCITY = 6.1; // Absolute max speed of a single wheel (meters per second) *Used for desaturation*
        public static final double MAX_LINEAR_VELOCITY = 6; // Desired max speed of the chassis (meters per second)
        public static final double MAX_ANGULAR_VELOCITY = 0.5 * Math.PI; // radians per second
    }
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }
  public static final class ControllerConstants {
    public static final double AXIS_DEADZONE = 0.05;
    public static final int DPAD_UP = 0;
    public static final int DPAD_RIGHT = 90;
    public static final int DPAD_DOWN = 180;
    public static final int DPAD_LEFT = 270;

    public static final int STICK_EXPONENTIAL_CURVE = 2;
}

public static final class UtilConstants {
    public static final double FALCON_ENCODER_RESOLUTION = 2048;
    public static final int CLOSED_LOOP_SPEED_MS = 1; // in milliseconds

    public static final int POSITION_PID_SLOT = 0;
    public static final int VELOCITY_PID_SLOT = 1;
    public static final int VOLTAGE_PID_SLOT = 2;
    public static final int BLAN_PID_SLOT = 3;
    
    
  }

public static final class ShuffleboardConstants {
    public static final String DRIVEBASE_CHOOSER = "Drivebase Chooser"; // Constant to prevent *namespace mismatches* 
}

public static final class ClawConstants
{
  // TODO: Change all of these values when they are known
  public static final int CLAW_MOTOR_PORT = 8;
  public static final boolean CLAW_MOTOR_REVERSED = false;
  public static final double CLAW_GEAR_RATIO = (double) 12 / (double) 72; // output / input
  public static final double CLAW_WHEEL_DIAMETER = 0.1524; // In meters (6 in wheels)

  public static final I2C.Port COLOR_SENSOR_PORT = Port.kOnboard;
}
}
