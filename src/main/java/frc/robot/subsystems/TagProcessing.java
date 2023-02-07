package frc.robot.subsystems;

import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.imgproc.Imgproc;
import java.lang.Math;

import org.photonvision.PhotonCamera;

import edu.wpi.first.networktables.*;


public class TagProcessing {
    public class Region {
        //because mats are stinky
        Point[] bounds; //quadrilateral is defined 0 -> 1 -> 2 -> 3 -> 0

        public Region(Point[] bounds) {
            this.UpdateBounds(bounds);
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

    private PhotonCamera cam_AprilTag;
    private Region[] regions;

    //this is for the default head-on case for a single substation and assumes:
    // - a perfect camera with no lens fuckery
    // - camera is at the exact height of the apriltag
    //TODO: add extrappolation for other angles
    //TODO also: write a version that works OUTSIDE of magical christmas land

    public void UpdateRegions(int tagID) {
        //TODO: this is hardcoded; add accounting for skew
        
    }

    public void Update() {
        UpdateRegions(0);
    }
    
    public Region[] GetRegions() {
        return regions;
    }

}
