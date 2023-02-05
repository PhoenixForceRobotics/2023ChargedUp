// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.utils.PIDValues;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static final class DrivebaseConstants {
        public static final int BUTTERFLY_FORWARD_PORT = 0;
        public static final int BUTTERFLY_REVERSE_PORT = 1;

        public static final int WHEEL_FL_PORT = 1;
        public static final int WHEEL_FR_PORT = 2;
        public static final int WHEEL_BL_PORT = 3;
        public static final int WHEEL_BR_PORT = 4;

        public static final boolean WHEEL_FL_REVERSED = true;
        public static final boolean WHEEL_FR_REVERSED = false;
        public static final boolean WHEEL_BL_REVERSED = true;
        public static final boolean WHEEL_BR_REVERSED = false;

        public static final PIDValues POSITION_PID =
                new PIDValues(4.8538, 0, 3.4846, 0.068311, 1.6687, 0.14842, 7, 4);
        public static final PIDValues VELOCITY_PID =
                new PIDValues(0.00032405, 0, 0, 0.068311, 1.6687, 0.14842, 7, 4);
        public static final PIDValues HOLONOMIC_DRIVE_PID = new PIDValues(1, 0, 0);
        public static final Translation2d WHEEL_FL_LOCATION = new Translation2d(0.345, 0.305);
        public static final Translation2d WHEEL_FR_LOCATION = new Translation2d(0.345, -0.305);
        public static final Translation2d WHEEL_BL_LOCATION = new Translation2d(-0.345, 0.305);
        public static final Translation2d WHEEL_BR_LOCATION = new Translation2d(-0.345, -0.305);
        public static final Translation2d FRONT_CENTER_LOCATION =
                new Translation2d(0.345, 0); // Helpful for evassive manuvers

        // public static final double GEAR_RATIO = (12.0 / 62.0) * (16.0 / 22.0); // output / input
        public static final double GEAR_RATIO = 12.0 / 72.0;
        // public static final double WHEEL_DIAMETER = 0.1016; // In meters (4 inch wheels)
        public static final double WHEEL_DIAMETER = 0.1524;
        public static final double WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER * Math.PI;

        public static final double MAX_OBTAINABLE_WHEEL_VELOCITY =
                UtilConstants.NEO_FREE_SPEED
                        * GEAR_RATIO
                        * WHEEL_CIRCUMFERENCE
                        / 60
                        * 0.9; // free speed of wheel (meters per second)
        public static final double MAX_LINEAR_ACCELERATION = 2; // Max acceleration
        public static final double MAX_LINEAR_VELOCITY = 3; // Desired max chassis speed

        public static final double MAX_ANGULAR_VELOCITY = 0.5 * Math.PI; // radians per second

        public static final Pose2d STARTING_POSE = new Pose2d(0, 0, new Rotation2d());

        public static final Pose2d WALL = new Pose2d(-1, -1, new Rotation2d());
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
        public static final int BLANK_PID_SLOT = 3;

        public static final double NEO_FREE_SPEED = 5820; // RPM
    }

    public static final class AutonomousConstants {
        public static final String BLUE_GRID_TO_BOTTOM_PIECE_PATH =
                "/pathplanner/generatedJSON/BlueGridToBottomPiece.wpilib.json";
    }
}
