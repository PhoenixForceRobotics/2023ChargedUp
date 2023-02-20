package frc.robot.utils.drivebase;

import edu.wpi.first.math.kinematics.MecanumDriveOdometry;

//TODO: convert to actual useful class
//TODO: replace MecanumDriveOdometry in drivebase with this so that it can be poked by vision

public class PositionContainer {
    private MecanumDriveOdometry odometry;

    public PositionContainer {
        odometry =
                new MecanumDriveOdometry(
                        kinematics,
                        gyro.getRotation2d(),
                        currentWheelPositions,
                        DrivebaseConstants.STARTING_POSE);
    }
    odometry.update(gyro.getRotation2d(), currentWheelPositions);
}
/**
     * Sets the position of the robot to a particular position and rotation relative to the field
     *
     * @param poseMeters The position on the field that your robot is at.
     */
    public void resetPosition(Pose2d poseMeters) {
        odometry.resetPosition(gyro.getRotation2d(), currentWheelPositions, poseMeters);
    }

    public Pose2d getPose() {
        return odometry.getPoseMeters();
    }
