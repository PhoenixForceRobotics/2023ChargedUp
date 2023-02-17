package frc.robot.commands.arm;

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
        this.targetAngle = Math.atan(xDistanceToFulcrum / yDistanceToFulcrum); //  
    }
}
