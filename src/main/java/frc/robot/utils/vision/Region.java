package frc.robot.subsystems.vision;

import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.imgproc.Imgproc;

public class Region {
    //because mats are stinky
    Point[] bounds; //quadrilateral is defined 0 -> 1 -> 2 -> 3 -> 0

    public Region(Point[] bounds) {
        this.UpdateBounds(bounds);
    }

    public Region() {
        this.bounds = new Point[4];
    }

    public void UpdateBounds(Point[] bounds) {
        this.bounds = bounds;
    }

    public boolean PointInRegion(Point point) {
        //the Java version of pointPolygonTest returns a double no matter what
        //pain
        double dist = Imgproc.pointPolygonTest(new MatOfPoint2f(bounds), point, false);
        return Math.signum(dist) == 1.0;
    }
}
