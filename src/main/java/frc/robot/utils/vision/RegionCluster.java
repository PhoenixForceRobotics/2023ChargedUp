package frc.robot.utils.vision;

import frc.robot.constants.Constants.VisionConstants.ProcessingConstants;
import frc.robot.utils.exceptions.MapWrongSizeException;
import org.opencv.core.Point;

/**
 * A grid of regions, with functions to skew it. Details on functionality can be found in the 2023
 * engineering notebook. Used for vision processing.
 */
public class RegionCluster {
    Region[] cluster;
    Region[] skewedCluster;
    Region tagRegion;

    /**
     * Creates a RegionCluster from an array of regions (seeing as that's all this class is a
     * wrapper for). Regions are numbered as follows:<br></br>[0][1][2]<br></br>[3][4][5]<br></br>&emsp;&nbsp;[6] (the aprilTag
     * itself)<br></br>[7][8][9]
     *
     * @param newCluster
     * @exception MapWrongSizeException Thrown if the given map is incorrectly sized (i.e. not size
     *     10, indices 0-9).
     */
    public RegionCluster(Region[] newCluster) throws MapWrongSizeException {
        // Create new RegionCluster from an array of regions; if the map isn't the right size, throw
        // an exception
        cluster = newCluster.clone();
        if (cluster.length == 10) {
            tagRegion = cluster[6];
        } else {
            throw new MapWrongSizeException();
        }
        skewedCluster = newCluster.clone();
    }

    public RegionCluster() throws MapWrongSizeException {
        // If not given arguments, a cluster will be produced from the primary map
        cluster = new Region[10];
        for (int regionIndex = 0;
                regionIndex < ProcessingConstants.PRIMARY_MAP.length;
                regionIndex++) {
            double[][] regionPoints = ProcessingConstants.PRIMARY_MAP[regionIndex];
            Point[] points = {
                new Point(regionPoints[0]),
                new Point(regionPoints[1]),
                new Point(regionPoints[2]),
                new Point(regionPoints[3])
            };
            cluster[regionIndex] = new Region(points.clone());
        }
        if (cluster.length == 10) {
            tagRegion = cluster[6];
        } else {
            throw new MapWrongSizeException();
        }
    }

    public void updateToAccountForSkew(double x, double y, double angle) {}
}
