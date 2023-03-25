package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.constants.Constants.ControllerConstants;
import frc.robot.subsystems.Arm;
import frc.robot.utils.PFRController;

public class SetArmVelocities extends CommandBase {
    private final Arm arm;
    private final PFRController operatorController, secondaryController;

    public SetArmVelocities(
            Arm arm, PFRController operatorController, PFRController secondaryController) {
        this.arm = arm;
        this.operatorController = operatorController;
        this.secondaryController = secondaryController;
        this.addRequirements(arm);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {

        // STANDARD CONTROL
        double extensionSpeed = this.operatorController.getLeftYSquared() * -0.35;
        this.arm.setExtensionMetersPerSecond(extensionSpeed, extensionSpeed);

        if (this.operatorController.getPOV() == ControllerConstants.DPAD_UP) {
            this.arm.setClawRelativeRadiansPerSecond(Math.PI / 2);
        } else if (this.operatorController.getPOV() == ControllerConstants.DPAD_DOWN) {
            this.arm.setClawRelativeRadiansPerSecond(-Math.PI / 2);
        } else {
            this.arm.setClawRelativeRadiansPerSecond(0);
        }

        this.arm.setRotationMotor(this.operatorController.getRightYSquared());

        // OVERRIDE CONTROLS
        if (this.secondaryController.getLeftBumper()) {
            this.arm.resetFirstStageEncoder(0);
        }
        if (this.secondaryController.getRightBumper()) {
            this.arm.resetSecondStageEncoder(0);
        }

        if (this.secondaryController.getPOV() == ControllerConstants.DPAD_UP) {
            this.arm.setClawRelativeRadiansPerSecond(Math.PI);
        } else if (this.secondaryController.getPOV() == ControllerConstants.DPAD_DOWN) {
            this.arm.setClawRelativeRadiansPerSecond(-Math.PI);
        } else {
            this.arm.setClawRelativeRadiansPerSecond(0);
        }
        this.arm.setFirstStageMetersPerSecond(-this.secondaryController.getLeftYSquared() * 0.35);
        this.arm.setSecondStageMetersPerSecond(-this.secondaryController.getRightYSquared() * 0.35);
        this.arm.setRotationMotor(-this.secondaryController.getLeftXSquared() * 0.7);
    }
}
