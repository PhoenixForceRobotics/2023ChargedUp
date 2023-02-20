package frc.robot.utils.vision;

import edu.wpi.first.math.geometry.Pose2d;

public class UnitConverter {
    public static double FeetToInches(double distFeet) {
        return distFeet * 12;
    }

    public static double InchesToFeet(double distInches) {
        return distInches / 12;
    }

    public static double MetersToInches(double distMeters) {
        return distMeters * 39.3701;
    }

    public static Pose2d MetersToInches(Pose2d poseMeters) {
        return new Pose2d(
            poseMeters.getX() * 39.3701,
            poseMeters.getY() * 39.3701,
            poseMeters.getRotation()
            );
    }
}
