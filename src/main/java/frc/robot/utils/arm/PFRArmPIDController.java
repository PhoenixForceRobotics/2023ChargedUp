package frc.robot.utils.arm;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import frc.robot.utils.PIDValues;

public class PFRArmPIDController extends PIDController {
    private final double MAX_CONTROL_EFFORT;
    private final ArmFeedforward FEED_FORWARD;

    public PFRArmPIDController(PIDValues pidValues) {
        super(pidValues.P, pidValues.I, pidValues.D);
        MAX_CONTROL_EFFORT = pidValues.MAX_CONTROL_EFFORT;
        FEED_FORWARD =
                new ArmFeedforward(
                        pidValues.STATIC_F, pidValues.VELOCITY_F, pidValues.ACCELERATION_F);
    }

    /**
     * Combines both the feedforward and the feedback (PID) loops
     *
     * @param positionMeasurement - in RADIANS
     * @param velocityMeasurement - in RADIANS PER SECOND
     * @param velocitySetpoint - in RADIANS PER SECOND
     * @return desired voltage
     */
    public double filteredCalculate(
            double positionMeasurement, double velocityMeasurement, double velocitySetpoint) {
        double pidOutputVolts = calculate(velocityMeasurement, velocitySetpoint);
        double feedForwardVolts = FEED_FORWARD.calculate(positionMeasurement, velocitySetpoint);
        return MathUtil.clamp(pidOutputVolts, -MAX_CONTROL_EFFORT, MAX_CONTROL_EFFORT)
                + feedForwardVolts;
    }
}
