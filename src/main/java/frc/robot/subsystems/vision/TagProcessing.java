package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;
import frc.robot.constants.Constants.VisionConstants;
import frc.robot.constants.FieldConstants;
import java.util.ArrayList;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class TagProcessing extends SubsystemBase {

    public PhotonCamera cameraTag;
    public PhotonCamera cameraCube;
    public PhotonCamera cameraCone;
    public PhotonPoseEstimator photonPoseEstimator;

    private MedianFilter positionFilter_X;
    private MedianFilter positionFilter_Y;
    private MedianFilter positionFilter_THETA;
    public Pose3d mostRecentPoseGuess; // most recent guess; prone to instability
    public Pose3d mostAccuratePoseGuess; // the most recent ACCURATE pose guess; created from median
    // filters

    // private RegionCluster cluster;

    /*
     * Creates a new tag processing instance. Defines a camera, a test tag field, a pose estimator, median filters, and pose guesses.
     */
    public TagProcessing() {
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
                        new Pose3d(
                                new Pose2d(
                                        0.0,
                                        FieldConstants.FIELD_WIDTH / 2.0,
                                        Rotation2d.fromDegrees(0.0))));
        ArrayList<AprilTag> atList = new ArrayList<AprilTag>();
        atList.add(tag18);
        atList.add(tag01);

        // TODO - once 2023 happens, replace this with just loading the 2023 field arrangement
        AprilTagFieldLayout atfl =
                new AprilTagFieldLayout(
                        atList, FieldConstants.FIELD_LENGTH, FieldConstants.FIELD_WIDTH);

        this.cameraTag = new PhotonCamera(Constants.VisionConstants.CameraNames.CAM_TAG);

        // Create pose estimator
        photonPoseEstimator =
                new PhotonPoseEstimator(
                        atfl,
                        PoseStrategy.CLOSEST_TO_REFERENCE_POSE,
                        cameraTag,
                        VisionConstants.CameraSpecConstants.ROBOT_TO_CAM_TAG);

        // Init median filters
        positionFilter_X =
                new MedianFilter(
                        Constants.VisionConstants.ProcessingConstants.MEDIAN_FILTER_SIZE_TAG);
        positionFilter_Y =
                new MedianFilter(
                        Constants.VisionConstants.ProcessingConstants.MEDIAN_FILTER_SIZE_TAG);
        positionFilter_THETA =
                new MedianFilter(
                        Constants.VisionConstants.ProcessingConstants.MEDIAN_FILTER_SIZE_TAG);

        // h
        mostRecentPoseGuess = new Pose3d();
        mostAccuratePoseGuess = new Pose3d();
    }

    // this is for the default head-on case for a single substation and assumes:
    // - a perfect camera with no lens fuckery
    // - camera is at the exact height of the apriltag
    // TODO: add extrappolation for other angles
    // TODO also: write a version that works OUTSIDE of magical christmas land

    public void updateRegions(int tagID) {
        // Update PCS region coordinates of blob hunting regions
        // TODO: this is hardcoded; add accounting for skew
        // TODO write this in general really
    }

    public void update() {
        updateRegions(0);
        updateGlobalPose();
    }

    public void updateGlobalPose() {
        // TODO write this out properly; should pull photonvision tag data and feed it into median
        // filters
        // TODO add timestamping; also add wrapper getter function that ensures
        PhotonPipelineResult cool = cameraTag.getLatestResult();
        if (cool.hasTargets()) {
            System.out.println("target found");
            PhotonTrackedTarget target = cool.getBestTarget();
            Transform3d targetPos = target.getBestCameraToTarget();
            mostRecentPoseGuess =
                    new Pose3d(
                            new Pose2d(
                                    targetPos.getX(),
                                    targetPos.getY(),
                                    Rotation2d.fromRadians(
                                            targetPos
                                                    .getRotation()
                                                    .getZ() // Z is yaw (which is apparently the
                                            // relevant angle for robot heading)
                                            )));
            mostAccuratePoseGuess =
                    new Pose3d(
                            new Pose2d(
                                    positionFilter_X.calculate(targetPos.getX()),
                                    positionFilter_Y.calculate(targetPos.getY()),
                                    Rotation2d.fromRadians(
                                            positionFilter_THETA.calculate(
                                                    targetPos.getRotation().getZ()))));
        }
    }

    @Override
    public void periodic() {
        update();
        //System.out.println("screaming into the void");
    }
}
