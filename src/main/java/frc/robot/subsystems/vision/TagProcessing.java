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
import frc.robot.utils.vision.VisionMath;
import java.util.ArrayList;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

// words cannot express how much i despise gradle automatically adding a space to the start of
// comments
public class TagProcessing extends SubsystemBase {

    public PhotonCamera cameraTag;
    public PhotonCamera cameraCube;
    public PhotonCamera cameraCone;
    public PhotonPoseEstimator photonPoseEstimator;

    private MedianFilter positionFilter_X;
    private MedianFilter positionFilter_Y;
    private MedianFilter positionFilter_THETA;
    public Pose2d mostRecentPoseGuess; // most recent guess; prone to instability
    public Pose2d mostAccuratePoseGuess; // the most recent ACCURATE pose guess; created from median

    public int bufferedWithTargetFrames; // how many frames of buffer of target data we have to work
    // with
    public boolean isBuffered; // if the frame buffer is full

    private ArrayList<AprilTag> tagList;
    private AprilTagFieldLayout protoFieldLayout;

    /*
     * Creates a new tag processing instance. Defines a camera, a test tag field, a pose estimator, median filters, and pose guesses.
     */
    public TagProcessing() {
        this.cameraTag = new PhotonCamera(Constants.VisionConstants.CameraNames.CAM_TAG);
        initField();
        initEstimator();
        initFilters();
    }

    private void initField() {
        //Stolen from photonvision docs
        tagList = new ArrayList<AprilTag>();
        tagList.add(new AprilTag(
                18,
                new Pose3d(
                        new Pose2d(
                                FieldConstants.FIELD_LENGTH,
                                FieldConstants.FIELD_WIDTH / 2.0,
                                Rotation2d.fromDegrees(180)))));
        tagList.add(new AprilTag(
                01,
                new Pose3d(
                        new Pose2d(
                                0.0,
                                FieldConstants.FIELD_WIDTH / 2.0,
                                Rotation2d.fromDegrees(0.0)))));

        protoFieldLayout =
                new AprilTagFieldLayout(
                        tagList, FieldConstants.FIELD_LENGTH, FieldConstants.FIELD_WIDTH);
    }

    private void initEstimator() {
        // Create pose estimator
        photonPoseEstimator =
                new PhotonPoseEstimator(
                        protoFieldLayout,
                        PoseStrategy.CLOSEST_TO_REFERENCE_POSE,
                        cameraTag,
                        VisionConstants.CameraSpecConstants.ROBOT_TO_CAM_TAG);
    }

    private void initFilters() {
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
        mostRecentPoseGuess = new Pose2d();
        mostAccuratePoseGuess = new Pose2d();

        bufferedWithTargetFrames = 0;
    }

    // this is for the default head-on case for a single substation and assumes:
    // - a perfect camera with no lens fuckery
    // - camera is at the exact height of the apriltag
    // TODO: add extrappolation for other angles

    public void updateRegions(int tagID) {
        // Update PCS region coordinates of blob hunting regions
        // TODO: this is hardcoded; add accounting for skew
        // TODO write this in general really

        //TODO invest in an actual project management system beyond spamming todo
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
            PhotonTrackedTarget target = cool.getBestTarget();
            if (target.getPoseAmbiguity() <= .2) { //ensure target is unambiguous
                Transform3d targetPos = target.getBestCameraToTarget();
                mostRecentPoseGuess =
                        new Pose2d(
                                targetPos.getX(),
                                targetPos.getY(),
                                Rotation2d.fromRadians(
                                        targetPos
                                                .getRotation()
                                                .getZ() // Z is yaw (which is apparently the
                                        // relevant angle for robot heading)
                                        ));
                mostAccuratePoseGuess =
                        new Pose2d(
                                positionFilter_X.calculate(targetPos.getX()),
                                positionFilter_Y.calculate(targetPos.getY()),
                                Rotation2d.fromRadians(
                                        positionFilter_THETA.calculate(
                                                targetPos.getRotation().getZ())));

                incrementBufferedFrames();
            } else {
                System.out.println("too much pose ambiguity");
                decrementBufferedFrames();
            }
        } else {
            decrementBufferedFrames();
        }
    }

    public void incrementBufferedFrames() {
        bufferedWithTargetFrames =
                        VisionMath.clamp(
                                bufferedWithTargetFrames + 1,
                                0,
                                VisionConstants.ProcessingConstants.MEDIAN_FILTER_SIZE_TAG);
    }

    public void decrementBufferedFrames() {
        bufferedWithTargetFrames =
                    VisionMath.clamp(
                            bufferedWithTargetFrames - 1,
                            0,
                            VisionConstants.ProcessingConstants.MEDIAN_FILTER_SIZE_TAG);
    }
    /*
     * Returns whether or not the frames buffer is full (i.e. if the most accurate pose guess is trustable).
     */
    public boolean checkIfBuffered() {
        return bufferedWithTargetFrames
                >= VisionConstants.ProcessingConstants.MAX_BAD_FRAME_TOLERANCE_TAG;
    }

    /*
     * Returns the best pose guess. User is expected to ensure that the pose value is properly buffered using checkIfBuffered()
     */
    public Pose2d getBestPoseGuess() {
        return mostAccuratePoseGuess;
    }

    /*
     * Returns the most recent pose guess. User is expected to ensure that the pose value is properly buffered using checkIfBuffered()
     */
    public Pose2d getRecentPoseGuess() {
        return mostRecentPoseGuess;
    }
}
