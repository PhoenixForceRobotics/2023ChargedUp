package frc.robot.constants;

import frc.robot.utils.vision.UnitConverter;

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

        public static final double GRID_SNAP_X = 527; // TODO: actual values
        /*
         * The actual midpoints of each individual grid slot.
         * THESE ARE MIDPOINTS. NOT DIVIDERS OR ENDPOINTS.
         */
        public static final double[] GRID_SNAP_Y = { // TODO: actual values
            16, 34.375, 52.875, 71.25
        };

        /*
         * Length of the grid.
         */
        public static final double TOTAL_GRID_LENGTH = 216;

        /*
         * Length of the substation.
         */
        public static final double TOTAL_SUBSTATION_LENGTH = FieldConstants.FIELD_LENGTH - TOTAL_GRID_LENGTH;

        public static final double GRID_OFFSET_BLUE = UnitConverter.FeetToInches(8) + 3;

        public static final double SUBSTATION_SNAP_X = UnitConverter.FeetToInches(52);
    }

    public static final double FIELD_LENGTH = UnitConverter.FeetToInches(26) + 3.5;
    public static final double FIELD_WIDTH = UnitConverter.FeetToInches(54) + 3.25;
}
