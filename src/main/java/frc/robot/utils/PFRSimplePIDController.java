package frc.robot.utils;


import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;


public class PFRSimplePIDController extends PIDController {
    private final double MAX_CONTROL_EFFORT;
    private final SimpleMotorFeedforward FEED_FORWARD;


    public PFRSimplePIDController(PIDValues pidValues) {
        super(pidValues.P, pidValues.I, pidValues.D);
        MAX_CONTROL_EFFORT = pidValues.MAX_CONTROL_EFFORT;
        FEED_FORWARD =
                new SimpleMotorFeedforward(
                        pidValues.STATIC_F, pidValues.VELOCITY_F, pidValues.ACCELERATION_F);
    }


    public double filteredCalculate(double measurement, double setpoint) {
        double rawOutput = calculate(measurement, setpoint);
        return MathUtil.clamp(rawOutput, -MAX_CONTROL_EFFORT, MAX_CONTROL_EFFORT)
                + FEED_FORWARD.calculate(getSetpoint());
    }


    public double filteredCalculate(double measurement) {
        double rawOutput = calculate(measurement);
        return MathUtil.clamp(rawOutput, -MAX_CONTROL_EFFORT, MAX_CONTROL_EFFORT)
                + FEED_FORWARD.calculate(getSetpoint());
    }
}