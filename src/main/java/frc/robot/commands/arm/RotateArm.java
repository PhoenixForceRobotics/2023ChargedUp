package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;
import frc.robot.utils.PFRController;

public class RotateArm extends CommandBase {
    private Arm arm;
    private PFRController operatorController;

    /**
     * This command is used in tele-op to allow the operator to rotate the arm to reach game pieces
     *
     * @param arm - the arm that is going to be rotated
     * @param operatorController - the controller used to receive input from the operator
     */
    public RotateArm(Arm arm, PFRController operatorController) {
        this.arm = arm;
        this.operatorController = operatorController;
    }

    @Override
    public void initialize() {
        arm.setRotationMotors(0);
    }

    @Override
    public void execute() {
        arm.setRotationMotors(operatorController.getLeftYSquared());
    }

    @Override
    public void end(boolean interrupted) {
        arm.setRotationMotors(0);
    }
}
