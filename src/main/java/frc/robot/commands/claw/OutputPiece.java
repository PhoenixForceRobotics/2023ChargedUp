package frc.robot.commands.claw;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.constants.Constants.ClawConstants;
import frc.robot.subsystems.Claw;

public class OutputPiece extends CommandBase {
    private Claw claw;

    public OutputPiece(Claw claw) {
        this.claw = claw;
    }

    @Override
    public void initialize() {
        this.claw.setMotor(ClawConstants.OUTPUT_SPEED);
    }

    @Override
    public void execute() {}

    @Override
    public void end(boolean interrupted) {
        this.claw.setMotor(0);
    }
}
