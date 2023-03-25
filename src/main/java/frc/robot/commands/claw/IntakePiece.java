package frc.robot.commands.claw;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.constants.Constants.ClawConstants;
import frc.robot.subsystems.Claw;

public class IntakePiece extends CommandBase {
    private Claw claw; // Claw that this command is controlling

    /**
     * This command picks up the specified piece using the claw subsystem
     *
     * @param claw - claw that this command is controlling
     * @param isPickingUpCube - determines whether the command is set to picking up a cube or a
     *     cone, as the motor direction differs between them
     */
    public IntakePiece(Claw claw) {
        this.claw = claw;
    }

    @Override
    public void initialize() {
        this.claw.setMotor(ClawConstants.INTAKE_SPEED);
    }

    @Override
    public void end(boolean interrupted) {
        this.claw.setMotor(0);
    }
}
