package frc.robot.constants;

/*import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;*/
import frc.robot.utils.vision.UnitConverter;
// import java.util.Arrays;

// The following is all in inches
// I'm so sorry
public final class FieldConstants {

    /*
     * Positions that the robot should snap to. In inches. Suffer.
     * TODO: this is relative to the end of the substation line;
     * TODO: THIS IS ONLY VALID FOR RED AND REQUIRES AN ADDITIONAL OFFSET FOR BLUE
     * TODO: IF YOU IGNORE ALL OF THESE TODOS YOU HAVE NOBODY BUT YOURSELF TO BLAME
     */
    public static final class SnapGrid {
        public static final double DEFAULT_SNAP_MAX_DISTANCE =
                54; // how far away before gridsnap gives up (inches)

        public static final double GRID_SNAP_X = 527;
        /*
         * The actual midpoints of each individual grid slot.
         * THESE ARE MIDPOINTS. NOT DIVIDERS OR ENDPOINTS.
         */
        public static final double[] GRID_SNAP_Y = { // TODO: actual values
            16, 34.375, 52.875, 71.25,
        };

        /*
         * Length of the grid.
         */
        public static final double TOTAL_GRID_LENGTH = 216;

        /*
         * Length of the substation.
         */
        public static final double TOTAL_SUBSTATION_LENGTH =
                FieldConstants.FIELD_LENGTH - SnapGrid.TOTAL_GRID_LENGTH;

        public static final double GRID_OFFSET_BLUE = UnitConverter.FeetToInches(8) + 3;

        public static final double SUBSTATION_SNAP_X = UnitConverter.FeetToInches(52);
    }

    /*
     * Blue-alliance aligned april tags.
     */
    public static final class TagFieldPositions {
        // The following is used from FRC 6328 Mechanical Advantage under the MIT license
        // because frankly open source is way easier than spending hours typing things from the
        // rules
        // Original can be found at:
        // https://github.com/Mechanical-Advantage/RobotCode2023/blob/245956d9635309737d78d7f915cfd6d1b94167a1/src/main/java/org/littletonrobotics/frc2023/FieldConstants.java

        // Woe be to whoever has to maintain this garbage

        // photonvision works in meters so now this is all in meters
        // GOD FUCKING DAMMIT I WASTED ALL MY TIME ON THIS
        /*public static final AprilTag[] TAG_LOCATIONS = {
            new AprilTag(
                1,
                new Pose3d(
                    Units.inchesToMeters(610.77),
                    Units.inchesToMeters(42.19),
                    Units.inchesToMeters(18.22),
                    new Rotation3d(0.0, 0.0, Math.PI)
                )
            ),
            new AprilTag(
                2,
                new Pose3d(
                    Units.inchesToMeters(610.77),
                    Units.inchesToMeters(108.19),
                    Units.inchesToMeters(18.22),
                    new Rotation3d(0.0, 0.0, Math.PI)
                )
            ),
            new AprilTag(
                3,
                new Pose3d(
                    Units.inchesToMeters(610.77),
                    Units.inchesToMeters(174.19), // FIRST's diagram has a typo (it says 147.19)
                    Units.inchesToMeters(18.22),
                    new Rotation3d(0.0, 0.0, Math.PI)
                )
            ),
            new AprilTag(
                4,
                new Pose3d(
                    Units.inchesToMeters(636.96),
                    Units.inchesToMeters(265.74),
                    Units.inchesToMeters(27.38),
                    new Rotation3d(0.0, 0.0, Math.PI)
                )
            ),
            new AprilTag(
                5,
                new Pose3d(
                    Units.inchesToMeters(14.25),
                    Units.inchesToMeters(265.74),
                    Units.inchesToMeters(27.38),
                    new Rotation3d()
                )
            ),
            new AprilTag(
                6,
                new Pose3d(
                    Units.inchesToMeters(40.45),
                    Units.inchesToMeters(174.19), // FIRST's diagram has a typo (it says 147.19)
                    Units.inchesToMeters(18.22),
                    new Rotation3d()
                )
            ),
            new AprilTag(
                7,
                new Pose3d(
                    Units.inchesToMeters(40.45),
                    Units.inchesToMeters(108.19),
                    Units.inchesToMeters(18.22),
                    new Rotation3d()
                )
            ),
            new AprilTag(
                8,
                new Pose3d(
                    Units.inchesToMeters(40.45),
                    Units.inchesToMeters(42.19),
                    Units.inchesToMeters(18.22),
                    new Rotation3d()
                )
            ),
        };

        public static final AprilTagFieldLayout TAG_GAME_FIELD = new AprilTagFieldLayout(
            Arrays.asList(TAG_LOCATIONS),
            FieldConstants.FIELD_LENGTH,
            FieldConstants.FIELD_WIDTH
        );*/
    }

    public static final double FIELD_LENGTH = UnitConverter.FeetToInches(26) + 3.5;
    public static final double FIELD_WIDTH = UnitConverter.FeetToInches(54) + 3.25;
}
