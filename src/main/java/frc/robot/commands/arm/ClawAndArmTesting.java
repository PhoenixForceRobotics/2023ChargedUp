package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Claw;
import frc.robot.utils.PFRController;

public class ClawAndArmTesting extends CommandBase {
    private Arm arm;
    private Claw claw;
    private PFRController operatorController;

    public ClawAndArmTesting(Arm arm, Claw claw, PFRController operatorController) {
        this.arm = arm;
        this.claw = claw;
        this.operatorController = operatorController;
    }

    @Override
    public void execute() {
        arm.setRotationMotor(-operatorController.getLeftX());
        arm.setFirstStageMotor(-operatorController.getLeftY());
        arm.setSecondStageMotor(-operatorController.getRightY());
        arm.setWristMotor(operatorController.getRightX());
        claw.setMotor(operatorController.getAButton() ?  0.5 : 0);

        System.out.println ("Left: " + -operatorController.getLeftY() + ", Right: " + -operatorController.getRightY());
    }
}
