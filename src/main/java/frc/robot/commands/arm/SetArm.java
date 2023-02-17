package frc.robot.commands.arm;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.Arm;

public class SetArm extends CommandBase {
    private final Arm arm;
    private final double targetLength;
    private final double targetAngle;
    private PIDController lengthPID;
    private PIDController anglePID;
    
    public SetArm(Arm arm, double distanceFromBumperMeters, double distanceFromGroundMeters)
    {
        this.arm = arm;

        double xDistanceToFulcrum = distanceFromBumperMeters + ArmConstants.DISTANCE_BUMPER_TO_FULCRUM;
        double yDistanceToFulcrum = distanceFromGroundMeters + ArmConstants.DISTANCE_GROUND_TO_FULCRUM; 
        this.targetLength = Math.sqrt(Math.pow(xDistanceToFulcrum, 2) + Math.pow(yDistanceToFulcrum, 2));
        this.targetAngle = Math.atan(xDistanceToFulcrum / yDistanceToFulcrum);

        anglePID = new PIDController(ArmConstants.ROTATION_PID_P, ArmConstants.ROTATION_PID_I, ArmConstants.ROTATION_PID_D);
        lengthPID = new PIDController(ArmConstants.EXTENSION_PID_P, ArmConstants.EXTENSION_PID_I, ArmConstants.EXTENSION_PID_D);
    }

    @Override
    public void initialize()
    {
        anglePID.setSetpoint(targetAngle);
        lengthPID.setSetpoint(targetLength);
    }

    @Override
    public void execute()
    {
        double lengthOutput = MathUtil.clamp(lengthPID.calculate(arm.getExtensionLength()), -0.9, 0.9);
        arm.setExtensionMetersPerSecond(lengthOutput);

        double angleOutput = MathUtil.clamp(lengthPID.calculate(arm.getRotationAngle()), -0.9, 0.9);
        arm.setRotationRadiansPerSecond(angleOutput);
    }

    @Override
    public boolean isFinished()
    {
        return anglePID.atSetpoint() && lengthPID.atSetpoint();
    }

    @Override
    public void end(boolean interrupted)
    {
        arm.setRotationRadiansPerSecond(0);
        arm.setExtensionMetersPerSecond(0);
    }
}
