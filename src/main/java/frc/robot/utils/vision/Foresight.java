package frc.robot.utils.vision;

import frc.robot.utils.vision.Foresight.SlotPriority; //java isn't mad about cyclical imports for interfaces, apparently

/**
 * Interface for minmax evaluation lambda functions
 */
@FunctionalInterface
interface EvaluationFunction {
    SlotPriority Execute(
        int peak,
        int slotX,
        int slotY,
        int currentScore,
        boolean coopertition,
        int robotSlotX,
        int robotSlotY
    );
}

//I regret my life choices immensely
//Woe to whomever has to read through this

/**
 * Class for optimizing piece placement. Also contains a handy journal of my descent into madness why not
 * TODO: write in functions for actually pulling best slots
 */
public class Foresight {

    enum SlotPriority {
        FILLED(-1),
        HIGHEST(4),
        HIGH(3),
        MEDIUM(2),
        LOW(1),
        UNPRIORITIZED(0);

        public final int value;

        private SlotPriority(int value) {
            this.value = value;
        }

        public static SlotPriority fromValue(int valueToFind) {
            if (valueToFind < 0) {
                return UNPRIORITIZED;
            }
            for (SlotPriority slotPriority : SlotPriority.values()) {
                if (slotPriority.value == valueToFind) {
                    return slotPriority;
                }
            }
            return null;
        }
    }

    enum PlacementStrategy {
        MAXIMIZE_SCORE(0),
        MINIMIZE_MOVEMENT(1),
        MINIMIZE_TIME(2);

        public final int index;

        private PlacementStrategy(int index) {
            this.index = index;
        }

        public int getIndex() {
            return this.index;
        }
    }

    private static boolean[][] gridSlots;

    private static SlotPriority[][][] gridSlotPriorities; //0: highest scoring (obvious), 1: least positional movement (collision avoidance), 2: least positional/arm movement (cycle times)

    public static void initGrid() {
        Foresight.gridSlots = new boolean[9][3];
        Foresight.gridSlotPriorities = new SlotPriority[3][9][3];
    }

    /**
     * Overwrites grid.
     * I can make these comments as bad as I want and nobody can stop me muahahahhahahahhahaha
     */
    public static boolean[][] updateGrid(boolean[][] newGridSlots) {
        Foresight.gridSlots = newGridSlots;
        return Foresight.gridSlots;
    }

    /**
     * Pokes value into grid at given coordinates.
     * GCS (grid coordinate system and def not gender confirmation surgery) is as follows:
     * (0, 0) is at the bottom left hybrid node; (8, 2) is at the top right cone node
     */
    public static boolean[][] updateGrid(int x, int y, boolean newData) {
        Foresight.gridSlots[x][y] = newData;
        return Foresight.gridSlots;
    }

    /**
     * Return the current predicted grid state. (does not return predictions)
     */
    public static boolean[][] getGrid() {
        return Foresight.gridSlots;
    }

    /**
     * Return the grid of prioritized slots for a given strategy. (returns predictions)
     */
    public static SlotPriority[][] getSlotPriorities(
        PlacementStrategy strategy
    ) {
        int which = strategy.getIndex();
        return Foresight.gridSlotPriorities[which];
    }

    /**
     * Picks a slot to target for placement based on a given strategy.
     */
    public static int[] getTargetSlot(PlacementStrategy strategy) {
        int[] target = new int[2];

        

        return target;
    }

    /**
     * generic evaluator just so I don't forget to separately copy and paste however many god forsaken copies of this I'm gonna need once it inevitably breaks
     */
    private static SlotPriority[][] calculateSlotPriorities(
        EvaluationFunction eval,
        int robotSlotX,
        int robotSlotY
    ) {
        //TODO: optimize this
        SlotPriority[][] computedPriorities = new SlotPriority[9][3];
        int[][] computedScores = Foresight.calculateAdjacency();

        int peak = Foresight.calculatePeak(computedScores);

        boolean coopertitionFilled = Foresight.isCooperitionBonusFulfilled();

        for (int slotY = 0; slotY < 3; slotY++) {
            for (int slotX = 0; slotX < 9; slotX++) {
                computedPriorities[slotX][slotY] =
                    eval.Execute(
                        peak,
                        slotX,
                        slotY,
                        computedScores[slotX][slotY],
                        coopertitionFilled,
                        robotSlotX,
                        robotSlotY
                    );
            }
        }

        return computedPriorities;
    }
    
    /**
     * the
     * @param eval the
     */
    private static SlotPriority[][] calculateSlotPriorities(EvaluationFunction eval) {
        return Foresight.calculateSlotPriorities(eval, -1, -1);
    }

    /**
     * Return optimized calculations for specific strategy.
     */
    public static SlotPriority[][] calculateSlotPriorities(
        PlacementStrategy strategy,
        int currentGridX,
        int currentGridY
    ) {
        int which;
        switch (strategy) {
            case MAXIMIZE_SCORE:
                Foresight.calculateSlotPrioritiesMaxScore();
                which = 0;
                break;
            case MINIMIZE_MOVEMENT:
                Foresight.calculateSlotPrioritiesMinMovement(currentGridX);
                which = 1;
                break;
            case MINIMIZE_TIME:
                Foresight.calculateSlotPrioritiesMinTime(
                    currentGridX,
                    currentGridY
                );
                which = 2;
                break;
            default:
                Foresight.calculateSlotPrioritiesMaxScore();
                which = 0;
        }
        return Foresight.gridSlotPriorities[which];
    }

    /**
     * Force updates all score strategies. To be used with driver station widget.
     */
    public static void calculateSlotPriorities(
        int currentGridX,
        int currentGridY
    ) {
        Foresight.gridSlotPriorities[0] =
            Foresight.calculateSlotPrioritiesMaxScore();
        Foresight.gridSlotPriorities[1] =
            Foresight.calculateSlotPrioritiesMinMovement(currentGridX);
        Foresight.gridSlotPriorities[2] =
            Foresight.calculateSlotPrioritiesMinTime(
                currentGridX,
                currentGridY
            );
    }

    /**
     * Try to make each placement as impactful as possible. We don't care if we hit people or have slower cycles, we want to win on quality by any means necessary.
     */
    private static SlotPriority[][] calculateSlotPrioritiesMaxScore() {
        //lambdas keep it clean... somehow
        EvaluationFunction eval = (int peak, int slotX, int slotY, int score, boolean coopertition, int robotSlotX, int robotSlotY) -> {
            SlotPriority out;
            if (Foresight.gridSlots[slotX][slotY]) { //ignore basically everything if it's already filled
                out = SlotPriority.FILLED;
            } else { //score reduction
                int scoreReduction = 3 - slotY; //value is lower the further down you go
                if (score == peak) { //peak adjacency values get assigned
                    if (coopertition) {
                        if (!Foresight.isSlotXInCoopGrid(slotX)) { //prioritize center
                            scoreReduction += 1;
                        }
                    }
                }
                out = SlotPriority.fromValue(4 - scoreReduction);
            }

            return out;
        };

        return Foresight.calculateSlotPriorities(eval);
    }

    /**
     * Try to move the robot as little as possible. We don't care if we move the arm a lot, we just don't want to hit people.
     */
    private static SlotPriority[][] calculateSlotPrioritiesMinMovement(
        int currentGridX
    ) {
        //I'm not repasting my earlier comments literally just scroll up most of this is copy pasted
        EvaluationFunction eval = (int peak, int slotX, int slotY, int score, boolean coopertition, int robotSlotX, int robotSlotY) -> {
            SlotPriority out;
            if (Foresight.gridSlots[slotX][slotY]) {
                out = SlotPriority.FILLED;
            } else { //score reduction
                int scoreReduction = 3 - slotY;
                if (score == peak) {
                    if (coopertition) {
                        if (!Foresight.isSlotXInCoopGrid(slotX)) {
                            scoreReduction += 1;
                        }
                    }
                }
                //penalize being far from the robot
                scoreReduction += Math.abs(slotX - robotSlotX);

                out = SlotPriority.fromValue(4 - scoreReduction);
            }

            return out;
        };

        return Foresight.calculateSlotPriorities(eval, currentGridX, -1);
    }

    /**
     * Try to move both robot and arm as little as possible while still scoring as much as possible. We want to drop cycle time as low as it'll go and win on quantity.
     */
    private static SlotPriority[][] calculateSlotPrioritiesMinTime(
        int currentGridX,
        int currentGridY
    ) {
        EvaluationFunction eval = (int peak, int slotX, int slotY, int score, boolean coopertition, int robotSlotX, int robotSlotY) -> {
            SlotPriority out;
            if (Foresight.gridSlots[slotX][slotY]) {
                out = SlotPriority.FILLED;
            } else { //score reduction
                int scoreReduction = 3 - slotY;
                if (score == peak) {
                    if (coopertition) {
                        if (!Foresight.isSlotXInCoopGrid(slotX)) {
                            scoreReduction += 1;
                        }
                    }
                }
                //penalize being far from the robot
                scoreReduction += Math.abs(slotX - robotSlotX);
                scoreReduction += Math.abs(slotY - robotSlotY);

                out = SlotPriority.fromValue(4 - scoreReduction);
            }

            return out;
        };

        return Foresight.calculateSlotPriorities(
            eval,
            currentGridX,
            currentGridY
        );
    }

    /**
     * Returns the adjacency of a row; i.e. for each slot, how much would placing a piece here contribute to a link?.
     */
    private static int[] calculateAdjacency(int row) {
        //TODO: optimize this

        int[] output = new int[9];
        for (int slot = 0; slot < 9; slot++) {
            output[slot] = 0;
            //basic adjacency check
            for (int offset = -2; offset < 3; offset++) {
                if (VisionMath.clamp(slot + offset, 0, 8) == slot + offset) { //ensure checked slot exists
                    if (slot + offset != slot) { //ensure we're not counting a slot towards its own adjacency
                        if (Foresight.gridSlots[row][slot + offset]) { //actually check slot
                            output[slot]++;
                        }
                    }
                }
            }
        }
        //adjacency link resetter (to avoid the algorithm trying to count pieces for multiple links)
        for (int slot = 0; slot < 7; slot++) {
            if (
                Foresight.gridSlots[row][slot] &&
                Foresight.gridSlots[row][slot + 1] &&
                Foresight.gridSlots[row][slot + 2]
            ) {
                //reduce the adjacency of all those slots by one link's worth
                output[slot] -= 2;
                output[slot + 1] -= 2;
                output[slot + 2] -= 2;
                slot += 3; //ensure we don't decrement stuff unnecessarily (e.g. a 4 in a row cutting the adjacency of the center 2 twice)
            }
        }

        return output;
    }

    /**
     * Returns the adjacency of the grid; for each slot within the grid, how much would placing a piece here contribute to a link?
     */
    private static int[][] calculateAdjacency() {
        int[][] output = new int[9][3];
        for (int i = 0; i < 3; i++) {
            int[] row = Foresight.calculateAdjacency(i);
            //TODO: this function is a time bomb for future bugs
            for (int rowSlot = 0; rowSlot < 9; rowSlot++) {
                output[rowSlot][i] = row[rowSlot];
            }
        }

        return output;
    }

    /**
     * Calculates the highest present value in a 2d integer array.
     */
    private static int calculatePeak(int[][] scores) {
        int peak = 0;
        for (int slotX = 0; slotX < 9; slotX++) {
            for (int slotY = 0; slotY < 3; slotY++) {
                if (scores[slotX][slotY] > peak) {
                    peak = scores[slotX][slotY];
                }
            }
        }
        return peak;
    }

    /**
     * Returns how many pieces are in the center 3x3 region.
     */
    private static int calculatePiecesInCoopGrid() {
        int output = 0;
        for (int x = 3; x < 6; x++) {
            for (int y = 0; y < 3; y++) {
                output += Foresight.gridSlots[x][y] ? 1 : 0; //ternary operators are proof that god is dead
            }
        }
        return output;
    }

    /**
     * Returns whether or not our alliance has at least 3 pieces in the center grid.
     */
    private static boolean isCooperitionBonusFulfilled() {
        return Foresight.calculatePiecesInCoopGrid() >= 3;
    }

    /**
     * Returns if a piece is in the center 3x3 region.
     */
    private static boolean isSlotXInCoopGrid(int slotX) {
        return 2 < slotX && slotX < 6;
    }
}
