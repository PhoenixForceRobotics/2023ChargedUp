// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.Pair;
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
        public static final int BUTTERFLY_REVERSE_PORT = 1;

        public static final int WHEEL_FL_CAN_ID = 1;
        public static final int WHEEL_FR_CAN_ID = 2;
        public static final int WHEEL_BL_CAN_ID = 3;
        public static final int WHEEL_BR_CAN_ID = 4;
        public static final double GEAR_RATIO =
                (double) 12 / (double) 62 * (double) 16 / (double) 22; // output / input
        public static final double WHEEL_DIAMETER = 0.1016; // In meters (4 inch wheels)

        public static final boolean WHEEL_FL_REVERSED = true;
        public static final boolean WHEEL_FR_REVERSED = false;
        public static final boolean WHEEL_BL_REVERSED = true;
        public static final boolean WHEEL_BR_REVERSED = false;

        public static final int PIGEON_CAN_ID = 20;

        // TODO: Recalculate all of these values
        public static final PIDValues POSITION_PID =
                new PIDValues(133.86, 0, 9.3332, 0.13778, 2.8585, 0.41203, 7, 4);
        public static final PIDValues VELOCITY_PID =
                new PIDValues(1.5336, 0, 0, 0.13778, 2.8585, 0.41203, 7, 4);

        public static final PIDValues BALANCE_PID = new PIDValues(0.1, 0, 0.001, 0, 0, 0, 0.75, 0);
        public static final double BALANCE_PID_MAX_ANGLE =
                2.5; // in degrees, max angle to be LEVEL, according to rulebook
        public static final double BALANCE_PID_MAX_ANGULAR_VELOCITY = 1; // in degrees / second
        public static final PIDValues HOLONOMIC_DRIVE_PID = new PIDValues(1, 0, 0);

        // 24.75 in long (x), 23.5 in wide (y)
        public static final Translation2d WHEEL_FL_LOCATION = new Translation2d(0.320675, 0.29845);
        public static final Translation2d WHEEL_FR_LOCATION = new Translation2d(0.320675, -0.29845);
        public static final Translation2d WHEEL_BL_LOCATION = new Translation2d(-0.320675, 0.29845);
        public static final Translation2d WHEEL_BR_LOCATION =
                new Translation2d(-0.320675, -0.29845);
        public static final Translation2d FRONT_CENTER_LOCATION = new Translation2d(0.320675, 0);
        // Helpful for evassive manuvers

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
        public static final double MAX_LINEAR_VELOCITY = 4; // meters per second

        public static final double MAX_ANGULAR_ACCELERATION = Math.PI;

        public static final double MIN_ANGULAR_VELOCITY = Math.PI / 12; // prevents creep
        public static final double MAX_ANGULAR_VELOCITY = 0.75 * Math.PI; // radians per second
    }

    public static final class ArmConstants {

        // Constants for rotation motors (that rotate the base of the arm)
        public static final int ARM_ROTATION_MOTOR_1_PORT = 5;
        public static final int ARM_ROTATION_MOTOR_2_PORT = 6;
        public static final boolean ARM_ROTATION_MOTOR_1_REVERSED = true;
        public static final boolean ARM_ROTATION_MOTOR_2_REVERSED = true;
        public static final double ARM_ROTATION_MOTOR_GEAR_RATIO =
                12.0 / 72.0 * 20.0 / 56 * 16.0 / 32.0;
        public static final double ROTATION_MOTOR_WHEEL_DIAMETER = 0;

        // Constants for extension motors
        public static final int FIRST_STAGE_PORT = 7;
        public static final int SECOND_STAGE_PORT = 8;
        public static final boolean FIRST_STAGE_REVERSED = false;
        public static final boolean SECOND_STAGE_REVERSED = false;
        public static final double FIRST_STAGE_GEAR_RATIO = (double) 1 / (double) 12;
        public static final double SECOND_STAGE_GEAR_RATIO = (double) 1 / (double) 49;
        public static final double FIRST_STAGE_DISTANCE_PER_ROTATION = 0.20955;
        public static final double SECOND_STAGE_DISTANCE_PER_ROTATION = 0.13335;
        // 5.25 inches (second stage)
        // 8.25 inches (first stage)

        // Constants for claw rotation motors
        public static final int CLAW_ROTATION_MOTOR_1_PORT = 9;
        public static final int CLAW_ROTATION_MOTOR_2_PORT = 10;
        public static final boolean CLAW_ROTATION_MOTOR_1_REVERSED = false;
        public static final boolean CLAW_ROTATION_MOTOR_2_REVERSED = true;
        public static final double CLAW_ROTATION_MOTOR_GEAR_RATIO = (double) 1 / (double) 70;
        public static final double CLAW_ROTATION_MOTOR_WHEEL_DIAMETER = 0;

        // Numbers to caulcuate height and distance
        // PID Constants

        public static final PIDValues FIRST_STAGE_PID_VALUES =
                new PIDValues(0.12589, 0, 0, 0.29254, 6.3845, 0.21586, 0.25358, 7, 0);
        public static final PIDValues SECOND_STAGE_PID_VALUES =
                new PIDValues(0.14801, 0, 0, 0.14281, 23.875, 0.46786, 0.14937, 7, 0);
        public static final PIDValues CLAW_ROTATION_PID_VALUES =
                new PIDValues(0.15192, 0, 0, 0.15147, 0.67111, 0.039827, 0.21757, 7, 0);

        public static final PIDValues ARM_ROTATIONAL_POSITION_PID_VALUES = new PIDValues(0, 0, 0);
        public static final PIDValues FIRST_STAGE_LENGTH_PID_VALUES = new PIDValues(0, 0, 0);
        public static final PIDValues SECOND_STAGE_LENGTH_PID_VALUES = new PIDValues(0, 0, 0);

        public static final Pair<Double, Double> ROTATIONAL_SETPOINT_ERROR =
                new Pair<Double, Double>(Math.PI / 12, Math.PI / 12); // radian error, rad/s error
        public static final Pair<Double, Double> EXTENSION_SETPOINT_ERROR =
                new Pair<Double, Double>(0.025, 0.025); // meter error, m/s error

        public static final double ARM_ROTATION_STARTING_ANGLE = Math.toRadians(40);
        public static final double CLAW_STARTING_ANGLE = Math.toRadians(-130);
        // THIS ANGLE IS RELATIVE TO THE ARM!!!!!!!

        public static final double FIRST_STAGE_MIN_EXTENSION = 0.05; // 1 inch of safety
        public static final double FIRST_STAGE_MAX_EXTENSION = 0.12; // 4.6 inches to meters
        public static final double SECOND_STAGE_MIN_EXTENSION = 0.0454; // 1 inch of safety
        public static final double SECOND_STAGE_MAX_EXTENSION = 0.24; // 10 inches to meters
        public static final double NO_EXTENSION_LENGTH = 0.9398; // 37 inches to meters
        public static final double FULL_EXTENSION_LENGTH =
                NO_EXTENSION_LENGTH + FIRST_STAGE_MAX_EXTENSION + SECOND_STAGE_MAX_EXTENSION;
        public static final double HALF_EXTENSION_LENGTH = FULL_EXTENSION_LENGTH / 2;

        public static final double MIN_CLAW_ANGLE = Math.toRadians(-130); // relative to arm
        public static final double MAX_CLAW_ANGLE = Math.toRadians(135); // ]relative to arm
    }

    public static final class ClawConstants {
        public static final int CLAW_MOTOR_PORT = 11;
        public static final boolean CLAW_MOTOR_REVERSED = true;

        public static final double INTAKE_SPEED = 0.4;
        public static final double OUTPUT_SPEED = -1;
    }

    public static final class ControllerConstants {
        public static final double AXIS_DEADZONE = 0.1;
        public static final int DPAD_UP = 0;
        public static final int DPAD_RIGHT = 90;
        public static final int DPAD_DOWN = 180;
        public static final int DPAD_LEFT = 270;

        public static final int STICK_EXPONENTIAL_CURVE = 2;
    }

    public static final class AutonomousConstants {
        public static final Pose2d WALL = new Pose2d(-1, -1, new Rotation2d()); // figure this
    }

    public static final class UtilConstants {
        public static final double FALCON_ENCODER_RESOLUTION = 2048;
        public static final int CLOSED_LOOP_SPEED_MS = 1; // in milliseconds

        public static final int STICK_EXPONENTIAL_CURVE = 2;

        public static final double NEO_FREE_SPEED = 5820; // RPM

        public static final int OI_NUM_BUTTONS = 21;
    }
}
