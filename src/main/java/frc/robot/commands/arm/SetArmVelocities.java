package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;
import frc.robot.utils.PFRController;

public class SetArmVelocities extends CommandBase {
    private final Arm arm;
    private final PFRController operatorController;

    public SetArmVelocities(Arm arm, PFRController operatorController) {
        this.arm = arm;
        this.operatorController = operatorController;
    }

    @Override
    public void initialize() {
        arm.setClawIndependentlyControlled(true);
    }

    @Override
    public void execute() {
        arm.setClawIndependentlyControlled(true);
        arm.setFirstStageMetersPerSecond(-operatorController.getLeftYSquared() * 0.5);
        arm.setSecondStageMetersPerSecond(-operatorController.getRightYSquared() * 0.5);
        arm.setRotationMotor(operatorController.getLeftXSquared() * 0.5);
        arm.setClawRelativeRadiansPerSecond(operatorController.getRightXSquared() * Math.PI);
        System.out.println(operatorController.getRightX());
    }
}
