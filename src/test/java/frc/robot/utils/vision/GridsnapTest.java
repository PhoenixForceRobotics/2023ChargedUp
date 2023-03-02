package frc.robot.utils.vision;

import static org.junit.jupiter.api.Assertions.*;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.constants.FieldConstants.SnapGrid;
import org.junit.jupiter.api.Test;

public class GridsnapTest {

    /*
     * Tests 1 inch away on the left side of the snap point closest to the left.
     */
    @Test
    void testOneInchX_BarelyLeftY_OfLeftmostSnapPoint() {
        int snapIndex = SnapGridMath.snapToGrid(
            Alliance.Red,
            new Translation2d(526, 15)
        );
        assertEquals(0, snapIndex);
    }

    /*
     * Tests 1 inch away on the right side of the snap point closest to the left.
     */
    @Test
    void testOneInchX_BarelyRightY_OfLeftmostSnapPoint() {
        int snapIndex = SnapGridMath.snapToGrid(
            Alliance.Red,
            new Translation2d(526, 17)
        );
        assertEquals(0, snapIndex);
    }

    /*
     * Tests 16 inches away on the left side of the snap point closest to the left.
     */
    @Test
    void testOneInchX_ExtremelyLeftY_OfLeftmostSnapPoint() {
        int snapIndex = SnapGridMath.snapToGrid(
            Alliance.Red,
            new Translation2d(526, 1)
        );
        assertEquals(0, snapIndex);
    }

    /*
     * Tests 9.1875 inches away on the right side of the snap point closest to the left.
     */
    @Test
    void testOneInchX_ExtremelyRightY_OfLeftmostSnapPoint() {
        int snapIndex = SnapGridMath.snapToGrid(
            Alliance.Red,
            new Translation2d(526, 25)
        );
        assertEquals(0, snapIndex);
    }

    /*
     * Tests 1 inch away on the left side of the snap point closest to the right (currently present in constants).
     * TODO: if this test ever gets reused later, be sure to actually update the point being tested; as it stands, this is
     */
    @Test
    void testOneInchX_BarelyLeftY_OfRightmostSnapPoint() {
        int snapIndex = SnapGridMath.snapToGrid(
            Alliance.Red,
            new Translation2d(526, 70.25)
        );
        assertEquals(3, snapIndex);
    }

    /*
     * Tests 1 inch away on the left side of the snap point closest to the right (currently present in constants).
     * TODO: if this test ever gets reused later, be sure to actually update the point being tested; as it stands, this is
     */
    @Test
    void testOneInchX_BarelyRightY_OfRightmostSnapPoint() {
        int snapIndex = SnapGridMath.snapToGrid(
            Alliance.Red,
            new Translation2d(526, 72.25)
        );
        assertEquals(3, snapIndex);
    }

    // This is basically just a broad test for literally everything; it's not particularly
    // informative on its own without copious amounts of breakpoints
    // TODO: test substation snap
    // TODO: test snap for blue
    @Test
    void completeGridTestBattery() {
        // increasing alteration
        for (
            int alterationX = 1;
            alterationX <= SnapGrid.DEFAULT_SNAP_MAX_DISTANCE - 1;
            alterationX++
        ) {
            // each grid snap index
            for (int index = 0; index < SnapGrid.GRID_SNAP_Y.length; index++) {
                // increasing alteration
                for (int alterationY = 1; alterationY <= 4; alterationY++) {
                    assertEquals(
                        index,
                        SnapGridMath.snapToGrid(
                            Alliance.Red,
                            new Translation2d(
                                SnapGrid.GRID_SNAP_X - alterationX,
                                SnapGrid.GRID_SNAP_Y[index] + alterationY
                            )
                        )
                    );
                    assertEquals(
                        index,
                        SnapGridMath.snapToGrid(
                            Alliance.Red,
                            new Translation2d(
                                SnapGrid.GRID_SNAP_X - alterationX,
                                SnapGrid.GRID_SNAP_Y[index] - alterationY
                            )
                        )
                    );
                }
            }
        }
    }
}
