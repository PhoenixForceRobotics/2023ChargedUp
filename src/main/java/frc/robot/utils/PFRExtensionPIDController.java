package frc.robot.utils;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;

public class PFRExtensionPIDController extends PIDController {
    private final double MAX_CONTROL_EFFORT;
    private final ElevatorFeedforward FEED_FORWARD;

    public PFRExtensionPIDController(PIDValues pidValues) {
        super(pidValues.P, pidValues.I, pidValues.D);
        MAX_CONTROL_EFFORT = pidValues.MAX_CONTROL_EFFORT;
        FEED_FORWARD =
                new ElevatorFeedforward(
                        pidValues.STATIC_F,
                        pidValues.G,
                        pidValues.VELOCITY_F,
                        pidValues.ACCELERATION_F);
    }

    /**
     * Combines both the feedforward and the feedback (PID) loops
     *
     * @param positionMeasurement - in METERS
     * @param velocityMeasurement - in METERS PER SECOND
     * @param velocitySetpoint - METERS PER SECOND
     * @return desired voltage
     */
    public double filteredCalculate(double velocityMeasurement, double velocitySetpoint) {
        double pidOutputVolts = calculate(velocityMeasurement, velocitySetpoint);
        double feedForwardVolts = FEED_FORWARD.calculate(velocitySetpoint);
        return MathUtil.clamp(pidOutputVolts, -MAX_CONTROL_EFFORT, MAX_CONTROL_EFFORT)
                + feedForwardVolts;
    }
}
