package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;
import frc.robot.utils.PFRController;

public class ExtendArm extends CommandBase {
    private Arm arm;
    private PFRController operatorController;

    /**
     * This command can extend and retract the arm using the two extension motors on the arm
     *
     * @param arm - the arm that will be extended/retracted
     * @param operatorController - the controller that will be referenced for input
     */
    public ExtendArm(Arm arm, PFRController operatorController) {
        this.arm = arm;
        this.operatorController = operatorController;
    }

    @Override
    public void initialize() {
        arm.setExtensionMotors(0);
    }

    @Override
    public void execute() {
        arm.setExtensionMotors(
                operatorController
                        .getRightYSquared()); // Sets the speed of the extension motors to the input
        // from the right stick of the operator controller
    }

    @Override
    public void end(boolean interrupted) {
        arm.setExtensionMotors(0);
    }
}
