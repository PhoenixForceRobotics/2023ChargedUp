package frc.robot.commands.claw;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ClawConstants;
import frc.robot.subsystems.Claw;

public class TimedOutputPiece extends CommandBase{
    public final Claw claw;
    public final Timer timer;
    public final double seconds;
    public TimedOutputPiece(Claw claw, double seconds)
    {
        this.claw = claw;
        timer = new Timer();
        this.seconds = seconds;
    }

    @Override
    public void initialize() {
        timer.restart();
        claw.setMotor(ClawConstants.OUTPUT_SPEED);
    }
    @Override
    public boolean isFinished() {
        return timer.get() >= seconds;
    }

    @Override
    public void end(boolean interrupted) {
        claw.setMotor(0);
    }
}
