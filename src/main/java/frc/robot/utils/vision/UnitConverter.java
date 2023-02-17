package frc.robot.utils.vision;

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
}
