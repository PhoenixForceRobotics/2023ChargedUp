package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;
import frc.robot.constants.Constants.VisionConstants;
import frc.robot.utils.exceptions.AllianceNotSetException;
import frc.robot.utils.vision.VisionMath;
import java.io.IOException;
import java.util.Optional;
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
    // with at present
    public boolean isBuffered; // if the frame buffer is full

    private AprilTagFieldLayout fieldLayout;

    /*
     * Creates a new tag processing instance. Defines a camera, a test tag field, a pose estimator, median filters, and pose guesses.
     */
    public TagProcessing() throws IOException {
        this.cameraTag = new PhotonCamera(Constants.VisionConstants.CameraNames.CAM_TAG);
        this.initField();
        this.initEstimator();
        this.initFilters();
    }

    private void initField() throws IOException {
        // we do a little AAAAAAAAAAAAAAAAAAAAAAA
        this.fieldLayout = AprilTagFields.k2023ChargedUp.loadAprilTagLayoutField();
        switch (DriverStation.getAlliance()) {
            case Red:
                this.fieldLayout.setOrigin(OriginPosition.kRedAllianceWallRightSide);
                break;
            case Blue:
                this.fieldLayout.setOrigin(OriginPosition.kBlueAllianceWallRightSide);
                break;
            default:
                throw new AllianceNotSetException();
        }
    }

    private void initEstimator() {
        // Create pose estimator
        this.photonPoseEstimator =
                new PhotonPoseEstimator(
                        this.fieldLayout,
                        PoseStrategy.CLOSEST_TO_REFERENCE_POSE,
                        this.cameraTag,
                        VisionConstants.CameraSpecConstants.ROBOT_TO_CAM_TAG);
    }

    private void initFilters() {
        // Init median filters
        this.positionFilter_X =
                new MedianFilter(
                        Constants.VisionConstants.ProcessingConstants.MEDIAN_FILTER_SIZE_TAG);
        this.positionFilter_Y =
                new MedianFilter(
                        Constants.VisionConstants.ProcessingConstants.MEDIAN_FILTER_SIZE_TAG);
        this.positionFilter_THETA =
                new MedianFilter(
                        Constants.VisionConstants.ProcessingConstants.MEDIAN_FILTER_SIZE_TAG);

        // h
        this.mostRecentPoseGuess = new Pose2d();
        this.mostAccuratePoseGuess = new Pose2d();

        this.bufferedWithTargetFrames = 0;
    }

    // this is for the default head-on case for a single substation and assumes:
    // - a perfect camera with no lens fuckery
    // - camera is at the exact height of the apriltag
    // TODO: add extrappolation for other angles

    public void updateRegions(int tagID) {
        // Update PCS region coordinates of blob hunting regions
        // TODO: this is hardcoded; add accounting for skew
        // TODO write this in general really

        // TODO invest in an actual project management system beyond spamming todo
    }

    public void update() {
        this.updateRegions(0);
        this.updateGlobalPose();
    }

    public void updateGlobalPose() {
        // TODO add wrapper getter function that ensures
        PhotonPipelineResult cool = this.cameraTag.getLatestResult();
        if (cool.hasTargets()) {
            PhotonTrackedTarget target = cool.getBestTarget();
            if (target.getPoseAmbiguity() <= .2) { // ensure target is unambiguous
                Transform3d targetPos = target.getBestCameraToTarget();
                this.mostRecentPoseGuess =
                        new Pose2d(
                                targetPos.getX(),
                                targetPos.getY(),
                                Rotation2d.fromRadians(
                                        targetPos
                                                .getRotation()
                                                .getZ() // Z is yaw (which is apparently the
                                        // relevant angle for robot heading)
                                        ));
                this.mostAccuratePoseGuess =
                        new Pose2d(
                                this.positionFilter_X.calculate(targetPos.getX()),
                                this.positionFilter_Y.calculate(targetPos.getY()),
                                Rotation2d.fromRadians(
                                        this.positionFilter_THETA.calculate(
                                                targetPos.getRotation().getZ())));

                this.incrementBufferedFrames();
            } else {
                System.out.println("too much pose ambiguity");
                this.decrementBufferedFrames();
            }
        } else {
            this.decrementBufferedFrames();
        }
    }

    public void incrementBufferedFrames() {
        this.bufferedWithTargetFrames =
                VisionMath.clamp(
                        this.bufferedWithTargetFrames + 1,
                        0,
                        VisionConstants.ProcessingConstants.MEDIAN_FILTER_SIZE_TAG);
    }

    public void decrementBufferedFrames() {
        this.bufferedWithTargetFrames =
                VisionMath.clamp(
                        this.bufferedWithTargetFrames - 1,
                        0,
                        VisionConstants.ProcessingConstants.MEDIAN_FILTER_SIZE_TAG);
    }

    /*
     * Returns whether or not the frames buffer is full (i.e. if the most accurate pose guess is trustable).
     */
    public boolean checkIfBuffered() {
        return (this.bufferedWithTargetFrames
                >= VisionConstants.ProcessingConstants.MAX_BAD_FRAME_TOLERANCE_TAG);
    }

    /*
     * Returns an Optional<Pose2d> containing the best pose guess, if it's properly buffered.
     */
    public Optional<Pose2d> getBestPoseGuess() {
        return this.checkIfBuffered() ? Optional.of(this.mostAccuratePoseGuess) : Optional.empty();
    }

    /*
     * Returns the best pose guess without checking buffering.
     * Mostly to rewrite as little as is possible.
     */
    public Pose2d sudoGetBestPoseGuess() {
        return this.mostAccuratePoseGuess;
    }

    /*
     * Returns the most recent pose guess. User is expected to ensure that the pose value is properly buffered using checkIfBuffered()
     */
    public Pose2d getRecentPoseGuess() {
        return this.mostRecentPoseGuess;
    }
}
