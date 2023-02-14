package frc.robot.commands.claw;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Claw;
import frc.robot.utils.PFRController;

public class TeleopClaw extends CommandBase {
    private Claw claw;
    private PFRController operatorController;

    public TeleopClaw(Claw claw, PFRController operatorController) {
        this.claw = claw;
        this.operatorController = operatorController;
    }

    @Override
    public void initialize() {
        claw.setMotor(0);
    }

    @Override
    public void execute() {
        claw.setMotor(operatorController.getRightYSquared());
    }
}
