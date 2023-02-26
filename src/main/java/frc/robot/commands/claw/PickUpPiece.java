package frc.robot.commands.claw;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ClawConstants;
import frc.robot.subsystems.Claw;

public class PickUpPiece extends CommandBase {
    private Claw claw; // Claw that this command is controlling
    private boolean
            isPickingUpCube; // Determines whether the command is set to picking up a cube or a
    // cone, as the motor direction differs between them
    private Timer timer;

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
        timer = new Timer();
    }

    @Override
    public void initialize() {
        if (isPickingUpCube) {
            // If we are picking up the cube, then we use a positive motor speed
            claw.setMotor(ClawConstants.CLAW_MOTOR_SPEED);
        } else {
            // Otherwise, we use a negative motor speed to spin the motor the opposite direction
            claw.setMotor(-ClawConstants.CLAW_MOTOR_SPEED);
        }
        timer.reset();
        timer.stop();
    }

    @Override
    public void execute() {
        if (claw.hasPiece()) {
            timer.start();
        }
    }

    @Override
    public boolean isFinished() {
        if (isPickingUpCube) {
            return timer.get() >= ClawConstants.CUBE_TIMER_DELAY_LENGTH;
        } else {
            return timer.get() >= ClawConstants.CONE_TIMER_DELAY_LENGTH;
        }
    }

    @Override
    public void end(boolean interrupted) {
        claw.setMotor(0);
        timer.stop();
        timer.reset();
    }
}
