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
        arm.setExtensionMotor1(0); // Stops the motors from moving
        arm.setExtensionMotor2(0);
    }

    @Override
    public void execute() {
        arm.setExtensionMotor1(operatorController.getLeftYSquared() * 0.1);
        arm.setExtensionMotor2(operatorController.getLeftYSquared() * 0.1);
    }

    @Override
    public void end(boolean interrupted) {
        arm.setExtensionMotor1(0); // Stops the motors from moving
        arm.setExtensionMotor2(0);
    }
}
