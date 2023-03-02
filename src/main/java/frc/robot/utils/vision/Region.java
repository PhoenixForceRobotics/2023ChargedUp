package frc.robot.utils.vision;

import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.imgproc.Imgproc;

/**
 * Region defines a quadrilateral bounding box, and provides functions to check if a point falls
 * within it. To be used for vision processing; specifically, if the centroid of a blob (a game
 * piece) falls within it (where a piece would be if it were scored).
 */
public class Region {
    Point[] bounds; // quadrilateral is defined 0 -> 1 -> 2 -> 3 -> 0

    /**
     * Constructs a region from an array of points.
     *
     * @param bounds The points bounding the quadrilateral. Points should be ordered as follows:
     *     top-left, top-right, bottom-right, bottom-left (clockwise from top-left).
     */
    public Region(Point[] bounds) {
        this.UpdateBounds(bounds);
    }

    /** Constructs an empty region with values for each point of (0,0). */
    public Region() {
        this.bounds = new Point[4];
    }

    /** Overwrites the boundaries of the current region. */
    public void UpdateBounds(Point[] bounds) {
        this.bounds = bounds;
    }

    /**
     * Checks if a given point falls within this region.
     *
     * @param point The point to test.
     */
    public boolean PointInRegion(Point point) {
        // the Java version of pointPolygonTest returns a double no matter what
        // pain
        double dist = Imgproc.pointPolygonTest(
            new MatOfPoint2f(bounds),
            point,
            false
        );
        return Math.signum(dist) == 1.0;
    }
}
