package frc.robot.utils;


import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ArmFeedforward;


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


    public double filteredCalculate(double positionMeasurement, double velocityMeasurement, double velocitySetpoint) {
        double pidOutput = calculate(velocityMeasurement, velocitySetpoint);
        double feedForwardVolts = FEED_FORWARD.calculate(positionMeasurement, velocitySetpoint);
        return MathUtil.clamp(pidOutput, -MAX_CONTROL_EFFORT, MAX_CONTROL_EFFORT) + feedForwardVolts;
    }
}