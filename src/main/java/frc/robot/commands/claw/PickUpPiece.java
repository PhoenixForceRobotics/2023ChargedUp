package frc.robot.commands.claw;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ClawConstants;
import frc.robot.subsystems.Claw;

public class PickUpPiece extends CommandBase {
    private Claw claw; // Claw that this command is controlling
    private boolean
            isPickingUpCube; // Determines whether the command is set to picking up a cube or a
    // cone, as the motor direction differs between them

    /**
     * This command picks up the specified piece using the claw subsystem
     *
     * @param claw - claw that this command is controlling
     * @param isPickingUpCube - determines whether the command is set to picking up a cube or a
     *     cone, as the motor direction differs between them
     */
    public PickUpPiece(Claw claw, boolean isPickingUpCube) {
        this.claw = claw;
        this.isPickingUpCube = isPickingUpCube;
    }

    @Override
    public void initialize() {
        claw.setMotor(0); // As soon as this command is called the motor speed is set to 0
    }

    @Override
    public void execute() {
        if (isPickingUpCube) {
            // If we are picking up the cube, then we use a positive motor speed
            claw.setMotor(ClawConstants.CLAW_MOTOR_SPEED);
        } else {
            // Otherwise, we use a negative motor speed to spin the motor the opposite direction
            claw.setMotor(-ClawConstants.CLAW_MOTOR_SPEED);
        }
    }

    @Override
    public boolean isFinished() {
        return claw.hasPiece(); // TODO: Check with Raf or Chloe if this is the right condition
    }

    @Override
    public void end(boolean interrupted) {}
}
