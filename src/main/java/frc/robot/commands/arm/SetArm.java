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
    private final double targetClawAngle;
    private PIDController lengthPID;
    private PIDController anglePID;
    private PIDController clawAnglePID;
    
    public SetArm(Arm arm, double distanceFromBumperMeters, double distanceFromGroundMeters, double targetClawAngle)
    {
        this.arm = arm;

        double xDistanceToFulcrum = distanceFromBumperMeters + ArmConstants.DISTANCE_BUMPER_TO_FULCRUM;
        double yDistanceToFulcrum = distanceFromGroundMeters + ArmConstants.DISTANCE_GROUND_TO_FULCRUM; 
        this.targetLength = Math.sqrt(Math.pow(xDistanceToFulcrum, 2) + Math.pow(yDistanceToFulcrum, 2));
        this.targetAngle = Math.atan(xDistanceToFulcrum / yDistanceToFulcrum);

        anglePID = new PIDController(ArmConstants.ROTATION_PID_P, ArmConstants.ROTATION_PID_I, ArmConstants.ROTATION_PID_D);
        lengthPID = new PIDController(ArmConstants.EXTENSION_PID_P, ArmConstants.EXTENSION_PID_I, ArmConstants.EXTENSION_PID_D);
        clawAnglePID = new PIDController(ArmConstants.CLAW_ROTATION_PID_P, ArmConstants.CLAW_ROTATION_PID_I, ArmConstants.CLAW_ROTATION_PID_D);

        this.targetClawAngle = targetClawAngle;
    }

    @Override
    public void initialize()
    {
        anglePID.setSetpoint(targetAngle);
        lengthPID.setSetpoint(targetLength);
        clawAnglePID.setSetpoint(targetClawAngle);
    }

    @Override
    public void execute()
    {
        double lengthOutput = MathUtil.clamp(lengthPID.calculate(arm.getExtensionLength()), -0.9, 0.9);
        arm.setExtensionMetersPerSecond(lengthOutput);

        double angleOutput = MathUtil.clamp(anglePID.calculate(arm.getRotationAngle()), -0.9, 0.9);
        arm.setRotationRadiansPerSecond(angleOutput);

        double clawAngleOutput = MathUtil.clamp(clawAnglePID.calculate(arm.getClawRotationAngle()), -0.9, 0.9);
        arm.setClawRotationRadiansPerSecond(clawAngleOutput);
    }

    @Override
    public boolean isFinished()
    {
        return anglePID.atSetpoint() && lengthPID.atSetpoint() && clawAnglePID.atSetpoint();
    }

    @Override
    public void end(boolean interrupted)
    {
        arm.setRotationRadiansPerSecond(0);
        arm.setExtensionMetersPerSecond(0);
        arm.setClawRotationRadiansPerSecond(0);
    }
}
