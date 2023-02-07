// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.constants;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.util.UnitConverter;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static final class VisionConstants { //Constants relevant to vision processing in general
        public static final class CameraNames { //The names of cameras on the coprocessor side
            public static final String CAM_TAG = "Camera_F_Tag";
            public static final String CAM_BLOB = "Camera_F_ColorBlob";
        }
        public static final class CameraSpecConstants { //Constants relevant to physical attributes of the camera
            public static final int CAM_RES_X = 1920;   //Horizontal camera resolution
            public static final int CAM_RES_Y = 1080;   //Vertical camera resolution
        }
        public static final class ProcessingConstants { //Constants relevant to the vision pipeline
            //Position and angle of the robot relative to the AprilTag for the primary map
            //Remember: THIS IS IN INCHES (1 meter away)
            //TODO: Does this need to additionally compensate for positional difference between blob detection and apriltag detection cameras? If so, add a way to do that
            public static final Pose2d PRIMARY_MAP_TAKEN_FROM = new Pose2d(UnitConverter.MetersToInches(1), 0, new Rotation2d(0)); 
            public static final double[][][] PRIMARY_MAP =  { //Primary vision map
                {
                    {-0.309375, -0.5675925925925926},
                    {-0.246875, -0.5675925925925926},
                    {-0.246875, -0.4342592592592593},
                    {-0.309375, -0.4342592592592593},
                },
                {
                    {-0.17083333333333334, -0.5194444444444444},
                    {0.12395833333333334, -0.5194444444444444},
                    {0.12395833333333334, -0.39537037037037037},
                    {-0.17083333333333334, -0.39537037037037037},
                },
                {
                    {0.221875, -0.5675925925925926},
                    {0.284375, -0.5675925925925926},
                    {0.284375, -0.4342592592592593},
                    {0.221875, -0.4342592592592593},
                },
                {
                    {-0.39895833333333336, -0.3435185185185185},
                    {-0.33541666666666664, -0.3435185185185185},
                    {-0.33541666666666664, -0.16388888888888886},
                    {-0.39895833333333336, -0.16388888888888886},
                },
                {
                    {-0.19895833333333335, -0.3212962962962963},
                    {0.16666666666666666, -0.3212962962962963},
                    {0.16666666666666666, -0.14537037037037037},
                    {-0.19895833333333335, -0.14537037037037037},
                },
                {
                    {0.3104166666666667, -0.3435185185185185},
                    {0.37395833333333334, -0.3435185185185185},
                    {0.37395833333333334, -0.16388888888888886},
                    {0.3104166666666667, -0.16388888888888886},
                },
                {
                    {-0.06041666666666667, -0.10462962962962963},
                    {0.060416666666666674, -0.10462962962962963},
                    {0.060416666666666674, 0.10462962962962963},
                    {-0.06041666666666667, 0.10462962962962963},
                },
                {
                    {-0.778125, 0.28981481481481486},
                    {-0.278125, 0.28981481481481486},
                    {-0.278125, 0.8453703703703703},
                    {-0.778125, 0.8453703703703703},
                },
                {
                    {-0.20833333333333334, 0.28240740740740744},
                    {0.21250000000000002, 0.28240740740740744},
                    {0.21250000000000002, 0.8472222222222223},
                    {-0.20833333333333334, 0.8472222222222223},
                },
                {
                    {0.253125, 0.28981481481481486},
                    {0.753125, 0.28981481481481486},
                    {0.753125, 0.8453703703703703},
                    {0.253125, 0.8453703703703703},
                },
            };
        }
    }
}
