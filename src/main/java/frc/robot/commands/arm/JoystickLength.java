package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;
import frc.robot.utils.PFRController;

public class JoystickLength extends CommandBase {
    private Arm arm;
    private PFRController operatorController;

    /**
     * Extends the arm according to joystick input
     *
     * @param arm - the arm that will be extended
     * @param operatorController - the controller that will be referenced for joystick input
     */
    public JoystickLength(Arm arm, PFRController operatorController) {
        this.arm = arm;
        this.operatorController = operatorController;
    }

    @Override
    public void initialize() {
        arm.setFirstStageMetersPerSecond(0); // Stops the motors from moving
        arm.setSecondStageMetersPerSecond(0);
    }

    @Override
    public void execute() {
        arm.setFirstStageMetersPerSecond(operatorController.getLeftYSquared() * 0.1);
        arm.setSecondStageMetersPerSecond(operatorController.getLeftYSquared() * 0.1);
    }

    @Override
    public void end(boolean interrupted) {
        arm.setFirstStageMetersPerSecond(0); // Stops the motors from moving
        arm.setSecondStageMetersPerSecond(0);
    }
}
