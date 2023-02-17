package frc.robot.utils.vision;

import frc.robot.constants.Constants;

public class PolyMath {
    //TODO: delete if not necessary
    public static double[] PCStoACS(int x, int y) {
        double halfResX = Constants.VisionConstants.CameraSpecConstants.CAM_RES_X / 2;
        double halfResY = Constants.VisionConstants.CameraSpecConstants.CAM_RES_Y / 2;

        double ACSX = (x - halfResX) / halfResX;
        double ACSY = (y - halfResY) / halfResY;
        double[] ACSCOORDS = {ACSX, ACSY};

        return ACSCOORDS; //thing from FRC
    }
}