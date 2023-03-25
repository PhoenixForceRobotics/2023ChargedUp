package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ControllerConstants;
import frc.robot.subsystems.Arm;
import frc.robot.utils.PFRController;

public class SetArmVelocities extends CommandBase {
    private final Arm arm;
    private final PFRController operatorController, secondaryController;

    public SetArmVelocities(Arm arm, PFRController operatorController, PFRController secondaryController) {
        this.arm = arm;
        this.operatorController = operatorController;
        this.secondaryController = secondaryController;
        addRequirements(arm);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        
        // STANDARD CONTROL
        double extensionSpeed = operatorController.getLeftYSquared() * -0.35;
        arm.setExtensionMetersPerSecond(extensionSpeed, extensionSpeed);

        if(operatorController.getPOV() == ControllerConstants.DPAD_UP)
        {
            arm.setClawRelativeRadiansPerSecond(Math.PI / 2);
        } 
        else if(operatorController.getPOV() == ControllerConstants.DPAD_DOWN)
        {
            arm.setClawRelativeRadiansPerSecond(-Math.PI / 2);
        }
        else
        {
            arm.setClawRelativeRadiansPerSecond(0);
        }

        arm.setRotationMotor(operatorController.getRightYSquared());
        
        // OVERRIDE CONTROLS
        if(secondaryController.getLeftBumper())
        {
            arm.resetFirstStageEncoder(0);
        }
        if(secondaryController.getRightBumper())
        {
            arm.resetSecondStageEncoder(0);
        }

        if(secondaryController.getPOV() == ControllerConstants.DPAD_UP)
        {
            arm.setClawRelativeRadiansPerSecond(Math.PI);
        } 
        else if(secondaryController.getPOV() == ControllerConstants.DPAD_DOWN)
        {
            arm.setClawRelativeRadiansPerSecond(-Math.PI);
        }
        else
        {
            arm.setClawRelativeRadiansPerSecond(0);
        }
        arm.setFirstStageMetersPerSecond(-secondaryController.getLeftYSquared() * 0.35);
        arm.setSecondStageMetersPerSecond(-secondaryController.getRightYSquared() * 0.35);
        arm.setRotationMotor(-secondaryController.getLeftXSquared() * 0.7);
    
        
    }
}
