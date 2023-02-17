package frc.robot.subsystems.vision;

import org.apache.commons.lang3.ObjectUtils.Null;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.imgproc.Imgproc;
import java.lang.Math;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;

import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.networktables.*;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;
import frc.robot.constants.Constants.VisionConstants.ProcessingConstants;
import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.constants.FieldConstants;
import frc.robot.constants.Constants.VisionConstants;

import java.util.ArrayList;
import java.util.Optional;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;


public class TagProcessing extends SubsystemBase{

    public PhotonCamera cameraTag;
    public PhotonCamera cameraCube;
    public PhotonCamera cameraCone;
	public PhotonPoseEstimator photonPoseEstimator;

    private MedianFilter positionFilter_X; 
    private MedianFilter positionFilter_Y;
    private MedianFilter positionFilter_THETA;
    private Pose3d mostRecentPoseGuess; //most recent guess; prone to instability
    private Pose3d mostAccuratePoseGuess; //the most recent ACCURATE pose guess; created from median filters

    private RegionCluster cluster;

    public TagProcessing() {
        this.cameraTag = new PhotonCamera(Constants.VisionConstants.CameraNames.CAM_TAG);

        // Set up a test arena of two apriltags at the center of each driver station set
		final AprilTag tag18 =
            new AprilTag(
                18,
                new Pose3d(
                    new Pose2d(
                        FieldConstants.FIELD_LENGTH,
                        FieldConstants.FIELD_WIDTH / 2.0,
                        Rotation2d.fromDegrees(180))));
        final AprilTag tag01 =
            new AprilTag(
                01,
                new Pose3d(new Pose2d(0.0, FieldConstants.FIELD_WIDTH / 2.0, Rotation2d.fromDegrees(0.0))));
        ArrayList<AprilTag> atList = new ArrayList<AprilTag>();
        atList.add(tag18);
        atList.add(tag01);

        // TODO - once 2023 happens, replace this with just loading the 2023 field arrangement
        AprilTagFieldLayout atfl =
            new AprilTagFieldLayout(atList, FieldConstants.FIELD_LENGTH, FieldConstants.FIELD_WIDTH);

        // Forward Camera
        cameraTag =
            new PhotonCamera(VisionConstants.CameraNames.CAM_TAG); // Change the name of your camera here to whatever it is in the
        // PhotonVision UI.

        // Create pose estimator
        photonPoseEstimator =
            new PhotonPoseEstimator(atfl, PoseStrategy.CLOSEST_TO_REFERENCE_POSE, cameraTag, VisionConstants.robotToCam);
    }

    //this is for the default head-on case for a single substation and assumes:
    // - a perfect camera with no lens fuckery
    // - camera is at the exact height of the apriltag
    //TODO: add extrappolation for other angles
    //TODO also: write a version that works OUTSIDE of magical christmas land

    public void UpdateRegions(int tagID) {
        //Update PCS region coordinates of blob hunting regions
        //TODO: this is hardcoded; add accounting for skew
        
    }

    public void update() {
        UpdateRegions(0);
        updateGlobalPose();
    }

    public void updateGlobalPose() {
        null
        positionFilter_X.calculate(0);
        positionFilter_Y.calculate(0);
        positionFilter_THETA.calculate(0);
    }
    
}
