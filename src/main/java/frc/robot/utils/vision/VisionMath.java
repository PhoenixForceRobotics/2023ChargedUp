package frc.robot.utils.vision;

import frc.robot.constants.Constants;

/*
 * Utility class for any pure computation functions necessitated by robot functions related to vision processing.
 */
public class VisionMath {

    /*
     * Clamps a value onto a given range.
     * Apparently for some god forsaken reason neither the base library nor Math have built in functions for this.
     */
    public static int clamp(int value, int min, int max) {
        return Math.max(min, Math.min(max, value));
    }

    /*
     * Converts PCS to ACS. See WPIlib documentation on vision for details.
     */
    public static double[] PCStoACS(int x, int y) {
        double halfResX =
            Constants.VisionConstants.CameraSpecConstants.CAM_TAG_RES[0] / 2;
        double halfResY =
            Constants.VisionConstants.CameraSpecConstants.CAM_TAG_RES[1] / 2;

        double ACSX = (x - halfResX) / halfResX;
        double ACSY = (y - halfResY) / halfResY;
        double[] ACSCOORDS = { ACSX, ACSY };

        return ACSCOORDS; // thing from FRC
    }
}
