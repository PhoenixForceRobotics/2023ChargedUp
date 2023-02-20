package frc.robot.utils.motors;

import frc.robot.utils.PIDValues;

public class FalconConfigFields {
    private PIDValues[] falconPIDs;
    private double maxOutput;

    public FalconConfigFields(PIDValues[] falconPIDs, double maxOutput) {
        this.falconPIDs = falconPIDs;
        this.maxOutput = maxOutput;
    }

    public FalconConfigFields(PIDValues falconPID, double maxOutput) {
        this.falconPIDs = new PIDValues[] {falconPID};
        this.maxOutput = maxOutput;
    }

    public PIDValues[] getFalconPIDs() {
        return falconPIDs;
    }

    public double getMaxOutput() {
        return maxOutput;
    }
}
