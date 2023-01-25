package frc.robot.commands.claw;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Claw;

public class TurnOffClaw extends InstantCommand {

    private Claw claw; // Claw that this command is telling to stop

    /**
     * Turns off and stops the claw motors
     * @param claw - Claw that this command is telling to stop
     */
    public TurnOffClaw(Claw claw) {
        this.claw = claw;
    }

    @Override
    public void initialize() {
        claw.setMotor(0);
    }
}
