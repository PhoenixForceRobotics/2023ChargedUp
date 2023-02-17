package frc.robot.subsystems.vision;

import org.opencv.core.Point;

import frc.robot.constants.Constants.VisionConstants.ProcessingConstants;

//A collection of regions
public class RegionCluster {
    Region[] cluster;
    Region[] skewedCluster;
    Region tagRegion;

    public RegionCluster(Region[] newCluster) throws MapWrongSizeException {
        //Create new RegionCluster from an array of regions; if the map isn't the right size, throw an exception
        cluster = newCluster.clone();
        if (cluster.length == 10) {
            tagRegion = cluster[6];
        } else {
            throw new MapWrongSizeException();
        }
        skewedCluster = newCluster.clone();
    }

    public RegionCluster() throws MapWrongSizeException {
        //If not given arguments, a cluster will be produced from the primary map
        cluster = new Region[10];
        for(int regionIndex = 0; regionIndex < ProcessingConstants.PRIMARY_MAP.length; regionIndex++) {
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

    public void updateToAccountForSkew(double x, double y, double angle) {
        
    }
}