package frc.robot.commands.claw;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Claw.BeamBreakStatus;
import frc.robot.utils.PFRController;

public class TeleopClaw extends CommandBase {
    private Claw claw;
    private PFRController operatorController;

    /**
     * Moves the claw motors using joystick input from the operator controller
     * @param claw - the claw that will be moved
     * @param operatorController - the controller that will be referenced for joystick input
     */
    public TeleopClaw(Claw claw, PFRController operatorController) {
        this.claw = claw;
        this.operatorController = operatorController;
    }

    @Override
    public void initialize() {
        claw.setMotor(0); // Stops any movement of the motor
    }

    @Override
    public void execute() {
        if (!claw.hasPiece())
        {
            claw.setMotor(operatorController.getRightYSquared() * 0.5); // Maps the movement of the claw motors to the right joystick of the operator controller
        }

        else
        {
            claw.setMotor(0);
        }
    }

    @Override
    public void end(boolean interrupted)
    {
        claw.setMotor(0); // Stops any movement of the motor
    }
}
