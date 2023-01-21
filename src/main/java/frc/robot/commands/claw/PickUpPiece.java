package frc.robot.commands.claw;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ClawConstants;
import frc.robot.subsystems.Claw;

public class PickUpPiece extends CommandBase {
    private Claw claw;
    private boolean isPickingUpCube;

    public PickUpPiece(Claw claw, boolean isPickingUpCube) {
        this.claw = claw;
        this.isPickingUpCube = isPickingUpCube;
    }

    @Override
    public void initialize() {
        claw.setMotor(0);
    }

    @Override
    public void execute() {
        if (isPickingUpCube) {
            claw.setMotor(ClawConstants.CLAW_MOTOR_SPEED);
        } else {
            claw.setMotor(-ClawConstants.CLAW_MOTOR_SPEED);
        }
    }

    @Override
    public boolean isFinished() {
        return claw.hasPiece();
    }

    @Override
    public void end(boolean interrupted) {
        claw.setMotor(0);
    }
}
