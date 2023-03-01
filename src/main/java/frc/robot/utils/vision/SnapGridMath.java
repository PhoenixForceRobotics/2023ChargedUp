package frc.robot.utils.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.constants.FieldConstants;
import frc.robot.constants.FieldConstants.SnapGrid;
import frc.robot.utils.exceptions.AllianceNotSetException;
import frc.robot.utils.exceptions.NoValidSnapPointsException;
import org.opencv.core.Point;

// TODO: who the fuck cares

// Ease of use class. Snaps inputs to grid positions
public class SnapGridMath {
    // grid to snap to
    public Point position;

    public static double[] GRIDSNAP_CHECK_BOUNDARIES;

    /*
     * Returns what index a given position should snap to, a -1 if it should snap to the substation, or a -2 if all are too far.
     *
     * @param alliance What alliance the robot is on. Used to compensate for field asymmetry.
     * @param position Current robot position as a Translation2d.
     * @param maxSnapDistance How far is considered too far (as the crow flies) before it's rejected as a snap point.
     */
    // TODO: enum return values (or container)
    public static int snapToGrid(Alliance alliance, Translation2d position, double maxSnapDistance)
            throws AllianceNotSetException {
        // basic algorithm: find the shortest distance of the robot to each point, then at the end
        // check if it's within the max distance allowed
        // this allows for Fast:tm: because the implementation does it per frame because I'm an
        // awful programmer and should never have been given the position of programming lead
        int bestYDistance;
        double bestYDistanceValue;

        double xPosition = position.getX(); // store value for performance
        double yPosition = position.getY(); // store value for performance

        // check distance to substation snappoints; if too far, ignore them
        double substationXDistance = Math.abs(xPosition - SnapGrid.SUBSTATION_SNAP_X);
        double gridXDistance = Math.abs(xPosition - SnapGrid.GRID_SNAP_X);
        if (gridXDistance > substationXDistance) { // if substation is closer
            // this could probably be a ternary operator sequence but frankly I don't give a shit
            // this is good enough

            // The snapping rules for substations are much stricter, just because the odds of
            // accidentally drifting into a protected zone must be made 0 at all costs

            // Check that Y value is within bounds while accounting for field asymmetry
            // https://docs.wpilib.org/en/stable/docs/software/advanced-controls/geometry/coordinate-systems.html
            // Remember: Y+ is up, so Red has its substation starting at the end of the grid
            // and Blue has its substation starting at 0
            double lowerBound, upperBound;
            switch (alliance) {
                case Red:
                    lowerBound = SnapGrid.TOTAL_GRID_LENGTH;
                    upperBound = FieldConstants.FIELD_LENGTH;
                    break;
                case Blue:
                    lowerBound = 0;
                    upperBound = SnapGrid.TOTAL_SUBSTATION_LENGTH;
                    break;
                default:
                    throw new AllianceNotSetException();
            }
            // Now actually do position checks
            if (yPosition >= lowerBound
                    && yPosition <= upperBound) { // If within Y bounds of substation
                // Only need to check X distance if it's already within the substation boundaries;
                // we're trying to just force it to a line, really
                return (substationXDistance < maxSnapDistance) ? -1 : -2;
            } else { // If not within Y bounds of substation
                // Remember: it's already determined the substation is closer, so no need to check
                // the grid
                return -2;
            }
        } else { // if grid is closer, proceed with algorithm
            // assuming things are working correctly, these should be overwritten
            bestYDistance = -1;
            bestYDistanceValue = 9999;
        }

        // Account for field asymmetry
        // https://docs.wpilib.org/en/stable/docs/software/advanced-controls/geometry/coordinate-systems.html
        // Remember: Y+ is up, so Red has its grid starting at 0
        // and Blue has its grid starting at the end of the substation
        double teamYOffset;
        switch (alliance) {
            case Red:
                teamYOffset = 0;
                break;
            case Blue:
                teamYOffset = SnapGrid.TOTAL_SUBSTATION_LENGTH;
                break;
            default:
                throw new AllianceNotSetException();
        }

        // actual distance check
        for (int i = 0; i < SnapGrid.GRID_SNAP_Y.length; i++) {
            double yDifference = Math.abs(yPosition - (SnapGrid.GRID_SNAP_Y[i] + teamYOffset));
            if (yDifference < bestYDistanceValue) {
                bestYDistance = i;
                bestYDistanceValue = yDifference;
            }
        }

        // only calculate pythagorean distance at the end for PLUS 1% SPEEEEEEEED
        return (((Math.pow(bestYDistanceValue, 2) + Math.pow(gridXDistance, 2)))
                        < Math.pow(maxSnapDistance, 2))
                ? bestYDistance
                : -2;
    }

    /*
     * Returns what index a given position should snap to, a -1 if it should snap to the substation, or a -2 if all are further than the distance defined in constants.
     * @param alliance What alliance the robot is on. Used to compensate for field asymmetry.
     * @param position Current robot position as a Translation2d.
     */
    public static int snapToGrid(Alliance alliance, Translation2d position)
            throws AllianceNotSetException {
        return snapToGrid(alliance, position, SnapGrid.DEFAULT_SNAP_MAX_DISTANCE);
    }

    /*
     * Returns what index a given position should snap to, a -1 if it should snap to the substation, or a -2 if all are further than the distance defined in constants.
     * @param alliance What alliance the robot is on. Used to compensate for field asymmetry.
     * @param position Current robot position as a Pose2d.
     */
    public static int snapToGrid(Alliance alliance, Pose2d position)
            throws AllianceNotSetException {
        return snapToGrid(alliance, position.getTranslation(), SnapGrid.DEFAULT_SNAP_MAX_DISTANCE);
    }

    /*
     * Returns a Pose2D containing the field-relative position and angle the robot should be facing, given which alliance it's part of and where it is currently.
     */
    public static Pose2d getSnapPositionFromPosition(Alliance alliance, Pose2d positionPose)
            throws AllianceNotSetException, NoValidSnapPointsException {
        Pose2d targetPos;

        int snapIndex = SnapGridMath.snapToGrid(alliance, positionPose);

        if (snapIndex == -2) {
            throw new NoValidSnapPointsException();
        }

        // -2 already got filtered out from this clause
        if (snapIndex > -1) { // if within the grid
            double teamYOffset;
            switch (alliance) {
                case Red:
                    teamYOffset = 0;
                    break;
                case Blue:
                    teamYOffset = SnapGrid.TOTAL_SUBSTATION_LENGTH;
                    break;
                default:
                    throw new AllianceNotSetException();
            }
            targetPos =
                    new Pose2d(
                            SnapGrid.GRID_SNAP_X,
                            SnapGrid.GRID_SNAP_Y[snapIndex] + teamYOffset,
                            new Rotation2d(0));
        } else { // if snapping to substation
            // remember, the substation snap line is only a valid snap if the robot is already
            // within its Y bounds; as such, Y for the target pose is just passthrough (no Y offset
            // necessary)
            targetPos =
                    new Pose2d(
                            SnapGrid.SUBSTATION_SNAP_X,
                            positionPose.getY(),
                            new Rotation2d(Math.PI));
        }

        return targetPos;
    }

    public static Pose2d getSnapPositionFromIndex(Alliance alliance, Pose2d positionPose, int index)
            throws AllianceNotSetException, NoValidSnapPointsException {
        Pose2d targetPos;

        if (index == -2) {
            throw new NoValidSnapPointsException();
        }

        // -2 already got filtered out from this clause
        if (index > -1) { // if within the grid
            double teamYOffset;
            switch (alliance) {
                case Red:
                    teamYOffset = 0;
                    break;
                case Blue:
                    teamYOffset = SnapGrid.TOTAL_SUBSTATION_LENGTH;
                    break;
                default:
                    throw new AllianceNotSetException();
            }
            targetPos =
                    new Pose2d(
                            SnapGrid.GRID_SNAP_X,
                            SnapGrid.GRID_SNAP_Y[index] + teamYOffset,
                            new Rotation2d(0));
        } else { // if snapping to substation
            // remember, the substation snap line is only a valid snap if the robot is already
            // within its Y bounds; as such, Y for the target pose is just passthrough (no Y offset
            // necessary)
            targetPos =
                    new Pose2d(
                            SnapGrid.SUBSTATION_SNAP_X,
                            positionPose.getY(),
                            new Rotation2d(Math.PI));
        }

        return targetPos;
    }
}
