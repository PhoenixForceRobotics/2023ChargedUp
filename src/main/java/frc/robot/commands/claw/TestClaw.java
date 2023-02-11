package frc.robot.commands.claw;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Claw;
import frc.robot.utils.PFRController;

public class TestClaw extends CommandBase {
    private Claw claw;
    private PFRController operatorController;

    public TestClaw(Claw claw, PFRController operatorController) {
        this.claw = claw;
        this.operatorController = operatorController;
    }

    @Override
    public void initialize() {
        claw.setMotor(0);
    }

    @Override
    public void execute() {
        claw.setMotor(operatorController.getLeftYSquared());
    }

    @Override
    public void end(boolean interrupted) {
        claw.setMotor(0);
    }
}
