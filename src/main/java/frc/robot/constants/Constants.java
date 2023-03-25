// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.constants;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.utils.PIDValues;
import frc.robot.utils.vision.UnitConverter;

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
        public static final int PNEUMATIC_HUB_CAN_ID = 30;
        public static final int BUTTERFLY_FORWARD_PORT = 0;

        public static final int WHEEL_FL_CAN_ID = 1;
        public static final int WHEEL_FR_CAN_ID = 2;
        public static final int WHEEL_BL_CAN_ID = 3;
        public static final int WHEEL_BR_CAN_ID = 4;

        public static final double GEAR_RATIO = (12.0 / 62.0) * (16.0 / 22.0); // output / input

        public static final double WHEEL_DIAMETER = 0.1016; // In meters (4 inch wheels)
        public static final double WHEEL_CIRCUMFERENCE =
            WHEEL_DIAMETER * Math.PI;

        public static final double MAX_MOTOR_PERCENTAGE_OUTPUT = 0.9;

        public static final boolean WHEEL_FL_REVERSED = true;
        public static final boolean WHEEL_FR_REVERSED = false;
        public static final boolean WHEEL_BL_REVERSED = true;
        public static final boolean WHEEL_BR_REVERSED = false;

        public static final int PIGEON_CAN_ID = 20;

        // TODO: Recalculate all of these values
        public static final PIDValues POSITION_PID = new PIDValues(
            4.8538,
            0,
            3.4846,
            0.068311,
            1.6687,
            0.14842,
            7,
            4
        );
        public static final PIDValues VELOCITY_PID = new PIDValues(
            0.00032405,
            0,
            0,
            0.068311,
            1.6687,
            0.14842,
            7,
            4
        );
        public static final PIDValues BALANCE_PID = new PIDValues(
            0.05,
            0.001,
            0.0001,
            0,
            0,
            0,
            0.75,
            0
        );
        public static final double BALANCE_PID_MAX_ANGLE = 2.5; // in degrees, max angle to be LEVEL according to rulebook
        public static final double BALANCE_PID_MAX_ANGULAR_VELOCITY = 1; // in degrees / second
        public static final PIDValues HOLONOMIC_DRIVE_PID = new PIDValues(
            1,
            0,
            0
        );

        // 25.25 in long (x), 23.5 in wide (y)
        public static final Translation2d WHEEL_FL_LOCATION = new Translation2d(
            0.320675,
            0.29845
        );
        public static final Translation2d WHEEL_FR_LOCATION = new Translation2d(
            0.320675,
            -0.29845
        );
        public static final Translation2d WHEEL_BL_LOCATION = new Translation2d(
            -0.320675,
            0.29845
        );
        public static final Translation2d WHEEL_BR_LOCATION = new Translation2d(
            -0.320675,
            -0.29845
        );
        public static final Translation2d FRONT_CENTER_LOCATION = new Translation2d(
            0.320675,
            0
        ); // Helpful for evassive manuvers

        public static final Pose2d STARTING_POSE = new Pose2d(
            0,
            0,
            new Rotation2d()
        );

        public static final Pose2d WALL = new Pose2d(-1, -1, new Rotation2d());

        public static final double MAX_OBTAINABLE_WHEEL_VELOCITY =
            UtilConstants.NEO_FREE_SPEED *
            GEAR_RATIO *
            WHEEL_CIRCUMFERENCE /
            60 *
            0.9; // free speed of wheel (meters per second)
        public static final double MIN_LINEAR_VELOCITY = 0.075; // Prevents undesired creep
        public static final double MAX_LINEAR_VELOCITY = 3.5; // meters per second

        public static final double MIN_ANGULAR_VELOCITY = Math.PI / 12; // prevents creep
        public static final double MAX_ANGULAR_VELOCITY = 1.5 * Math.PI; // radians per second

        // used for slew limiters, meters per s^2
        public static final double MAX_LINEAR_ACCELERATION = 6;
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

    public static final class AutonomousConstants {
        public static final Pose2d WALL = new Pose2d(-1, -1, new Rotation2d());
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

    public static final class ShuffleboardConstants {
        public static final String DRIVEBASE_CHOOSER = "Drivebase Chooser"; // Constant to prevent *namespace mismatches*
    }

    // Constants relevant to vision processing in general
    public static final class VisionConstants {

        public static final class CameraNames { // The names of cameras on the coprocessor side
            public static final String CAM_TAG = "Camera_F_Tag";
            public static final String CAM_BLOB_1 = "Camera_F_ColorBlob";
        }

        public static final class StrafePIDValues {
            public static final PIDValues PID_X_VALUES = new PIDValues(1, 0, 1);
            public static final PIDValues PID_Y_VALUES = new PIDValues(1, 0, 1);
            public static final PIDValues PID_THETA_VALUES = new PIDValues(
                1,
                0,
                1
            );

            public static final double PID_POSITION_TOLERANCE = 2; // 2 inches tolerance bay bee
            public static final double PID_ANGLE_TOLERANCE = 2; // also 2 degrees
        }

        // Constants relevant to physical attributes of the camera
        public static final class CameraSpecConstants {
            public static final int[] CAM_TAG_RES = { 800, 600 }; // Tag camera resolution TODO: verify value
            public static final int[] CAM_BLOB_CONE_RES = { 1920, 1080 }; // Blob camera resolution TODO: give actual value
            public static final int[] CAM_BLOB_CUBE_RES = { 1920, 1080 }; // Blob camera resolution TODO: give actual value

            public static final Transform3d ROBOT_TO_CAM_TAG = new Transform3d();
        }

        public static final class ProcessingConstants { // Constants relevant to the vision pipeline
            /*
             * The size of MedianFilters for processing apriltags.
             * Essentially: how much of a delay there will be, in robot ticks.
             */
            public static final int MEDIAN_FILTER_SIZE_TAG = 5;

            /*
             * How many bad robot ticks can enter the buffer before the pose guess is considered useless.
             * Remember: this is robot ticks, NOT camera frames.
             */
            // intentional integer division
            public static final int MAX_BAD_FRAME_TOLERANCE_TAG =
                MEDIAN_FILTER_SIZE_TAG / 2;

            // Position and angle of the robot relative to the AprilTag for the primary map
            // Remember: THIS IS IN INCHES (1 meter away)
            // TODO: Does this need to additionally compensate for positional difference between
            // blob detection and apriltag detection cameras? If so, add a way to do that
            public static final Pose2d PRIMARY_MAP_TAKEN_FROM = new Pose2d(
                UnitConverter.MetersToInches(1),
                0,
                new Rotation2d(0)
            );
            public static final double[][][] PRIMARY_MAP = { // Primary vision map
                {
                    { -0.309375, -0.5675925925925926 },
                    { -0.246875, -0.5675925925925926 },
                    { -0.246875, -0.4342592592592593 },
                    { -0.309375, -0.4342592592592593 },
                },
                {
                    { -0.17083333333333334, -0.5194444444444444 },
                    { 0.12395833333333334, -0.5194444444444444 },
                    { 0.12395833333333334, -0.39537037037037037 },
                    { -0.17083333333333334, -0.39537037037037037 },
                },
                {
                    { 0.221875, -0.5675925925925926 },
                    { 0.284375, -0.5675925925925926 },
                    { 0.284375, -0.4342592592592593 },
                    { 0.221875, -0.4342592592592593 },
                },
                {
                    { -0.39895833333333336, -0.3435185185185185 },
                    { -0.33541666666666664, -0.3435185185185185 },
                    { -0.33541666666666664, -0.16388888888888886 },
                    { -0.39895833333333336, -0.16388888888888886 },
                },
                {
                    { -0.19895833333333335, -0.3212962962962963 },
                    { 0.16666666666666666, -0.3212962962962963 },
                    { 0.16666666666666666, -0.14537037037037037 },
                    { -0.19895833333333335, -0.14537037037037037 },
                },
                {
                    { 0.3104166666666667, -0.3435185185185185 },
                    { 0.37395833333333334, -0.3435185185185185 },
                    { 0.37395833333333334, -0.16388888888888886 },
                    { 0.3104166666666667, -0.16388888888888886 },
                },
                {
                    { -0.06041666666666667, -0.10462962962962963 },
                    { 0.060416666666666674, -0.10462962962962963 },
                    { 0.060416666666666674, 0.10462962962962963 },
                    { -0.06041666666666667, 0.10462962962962963 },
                },
                {
                    { -0.778125, 0.28981481481481486 },
                    { -0.278125, 0.28981481481481486 },
                    { -0.278125, 0.8453703703703703 },
                    { -0.778125, 0.8453703703703703 },
                },
                {
                    { -0.20833333333333334, 0.28240740740740744 },
                    { 0.21250000000000002, 0.28240740740740744 },
                    { 0.21250000000000002, 0.8472222222222223 },
                    { -0.20833333333333334, 0.8472222222222223 },
                },
                {
                    { 0.253125, 0.28981481481481486 },
                    { 0.753125, 0.28981481481481486 },
                    { 0.753125, 0.8453703703703703 },
                    { 0.253125, 0.8453703703703703 },
                },
            };
        }
    }
}