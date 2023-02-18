package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;
import frc.robot.utils.PFRController;

public class JoystickAngle extends CommandBase {
    private Arm arm;
    private PFRController operatorController;

    /**
     * Rotates the arm according to joystick input in radians per second
     *
     * @param arm - the arm that will be rotated
     * @param operatorController - the controller that will be referenced for this command
     */
    public JoystickAngle(Arm arm, PFRController operatorController) {
        this.arm = arm;
        this.operatorController = operatorController;
    }

    @Override
    public void initialize() {
        arm.setRotationRadiansPerSecond(
                0); // Resets the radians per second to zero, essentially stopping any movement
    }

    @Override
    public void execute() {
        arm.setRotationRadiansPerSecond(
                operatorController
                        .getLeftYSquared()); // Sets the radians per second to the squared input
        // from the controller
    }

    @Override
    public void end(boolean interrupted) {
        arm.setRotationRadiansPerSecond(
                0); // Resets the radians per second to zero, essentially stopping any movement
    }
}
