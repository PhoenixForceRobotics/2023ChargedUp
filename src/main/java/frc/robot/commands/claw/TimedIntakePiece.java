package frc.robot.commands.claw;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.constants.Constants.ClawConstants;
import frc.robot.subsystems.Claw;

public class TimedIntakePiece extends CommandBase {
    public final Claw claw;
    public final Timer timer;
    public final double seconds;

    public TimedIntakePiece(Claw claw, double seconds) {
        this.claw = claw;
        this.timer = new Timer();
        this.seconds = seconds;
    }

    @Override
    public void initialize() {
        this.timer.restart();
        this.claw.setMotor(ClawConstants.INTAKE_SPEED);
    }

    @Override
    public boolean isFinished() {
        return this.timer.get() >= this.seconds;
    }

    @Override
    public void end(boolean interrupted) {
        this.claw.setMotor(0);
    }
}
