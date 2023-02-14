package frc.robot.commands.claw;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Claw;

public class ClawIntakeSequence extends SequentialCommandGroup {

    /**
     * This command combines the PickUpPiece and TurnOffClaw commands to first pick up the piece,
     * then let the motors spin for a set amount of time, then turn off the motors
     *
     * @param claw - the claw that this command is controlling
     * @param isPickingUpCube - determines whether the command is set to picking up a cube or a
     *     cone, as the motor direction differs between them
     */
    public ClawIntakeSequence(Claw claw, boolean isPickingUpCube) {
        super(
                new PickUpPiece(claw, isPickingUpCube) // Picks up the piece on the field
                );
    }
}
