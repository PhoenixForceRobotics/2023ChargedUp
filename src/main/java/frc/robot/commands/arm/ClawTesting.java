package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Claw;
import frc.robot.utils.PFRController;

public class ClawTesting extends CommandBase {
    private Arm arm;
    private PFRController operatorController;

    public ClawTesting(Arm arm, PFRController operatorController, Claw claw) {
        this.arm = arm;
        this.operatorController = operatorController;
    }

    @Override
    public void initialize() {
        arm.setClawRotationRadiansPerSecond(0);
    }

    @Override
    public void execute() {
        arm.setClawRotationRadiansPerSecond(operatorController.getRightYSquared() * 2 * Math.PI);
        System.out.println("Absolute Angle: " + arm.getClawRotationAbsoluteAngleInDegrees());
    }

    @Override
    public void end(boolean interrupted) {
        arm.setClawRotationRadiansPerSecond(0);
    }
}
