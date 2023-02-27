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
        public static final int PNEUMATIC_HUB_CAN_ID = 0;
        public static final int BUTTERFLY_FORWARD_PORT = 0;
        public static final int BUTTERFLY_REVERSE_PORT = 0;

        public static final int WHEEL_FL_CAN_ID = 1;
        public static final int WHEEL_FR_CAN_ID = 2;
        public static final int WHEEL_BL_CAN_ID = 3;
        public static final int WHEEL_BR_CAN_ID = 4;

        public static final boolean WHEEL_FL_REVERSED = true;
        public static final boolean WHEEL_FR_REVERSED = false;
        public static final boolean WHEEL_BL_REVERSED = true;
        public static final boolean WHEEL_BR_REVERSED = false;

        public static final int PIGEON_CAN_ID = 20;

        // TODO: Recalculate all of these values
        public static final PIDValues POSITION_PID =
                new PIDValues(109.13, 0, 4.853, 0.18295, 2.8829, 0.14847, 7, 4);
        public static final PIDValues VELOCITY_PID =
                new PIDValues(0.20838, 0, 0, 0.18295, 2.8829, 0.14847, 7, 4);
        public static final PIDValues BALANCE_PID = new PIDValues(0.1, 0, 0.001, 0, 0, 0, 0.75, 0);
        public static final double BALANCE_PID_MAX_ANGLE =
                2.5; // in degrees, max angle to be LEVEL, according to rulebook
        public static final double BALANCE_PID_MAX_ANGULAR_VELOCITY = 1; // in degrees / second
        public static final PIDValues HOLONOMIC_DRIVE_PID = new PIDValues(1, 0, 0);

        // 25.25 in long (x), 23.5 in wide (y)
        public static final Translation2d WHEEL_FL_LOCATION = new Translation2d(0.320675, 0.29845);
        public static final Translation2d WHEEL_FR_LOCATION = new Translation2d(0.320675, -0.29845);
        public static final Translation2d WHEEL_BL_LOCATION = new Translation2d(-0.320675, 0.29845);
        public static final Translation2d WHEEL_BR_LOCATION =
                new Translation2d(-0.320675, -0.29845);
        public static final Translation2d FRONT_CENTER_LOCATION =
                new Translation2d(0.320675, 0); // Helpful for evassive manuvers

        public static final double GEAR_RATIO = (12.0 / 62.0) * (16.0 / 22.0); // output / input

        public static final double WHEEL_DIAMETER = 0.1016; // In meters (4 inch wheels)
        public static final double WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER * Math.PI;

        public static final double MAX_OBTAINABLE_WHEEL_VELOCITY =
                UtilConstants.NEO_FREE_SPEED
                        * GEAR_RATIO
                        * WHEEL_CIRCUMFERENCE
                        / 60
                        * 0.9; // free speed of wheel (meters per second)

        // used for slew limiters, meters per s^2
        public static final double MAX_LINEAR_ACCELERATION = 4;

        public static final double MIN_LINEAR_VELOCITY = 0.075; // Prevents undesired creep
        public static final double MAX_LINEAR_VELOCITY = 2; // meters per second

        public static final double MIN_ANGULAR_VELOCITY = Math.PI / 12; // prevents creep
        public static final double MAX_ANGULAR_VELOCITY = 0.5 * Math.PI; // radians per second
    }

    public static class OperatorConstants {
        public static final int DRIVER_CONTROLLER_PORT = 0;
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
        public static final Pose2d WALL = new Pose2d(-1, -1, new Rotation2d());
    }
}
