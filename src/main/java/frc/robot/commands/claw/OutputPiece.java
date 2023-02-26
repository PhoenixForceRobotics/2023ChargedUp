package frc.robot.commands.claw;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ClawConstants;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Claw.BeamBreakStatus;
import frc.robot.utils.PFRController;

public class OutputPiece extends CommandBase {
    private Claw claw;
    private PFRController operatorController;

    public OutputPiece(Claw claw, PFRController operatorController)
    {
        this.claw = claw;
        this.operatorController = operatorController;
    }

    @Override
    public void initialize()
    {
        claw.setMotor(0);
    }

    @Override
    public void execute()
    {
        if (claw.getBreakStatus() == BeamBreakStatus.CUBE)
        {
            claw.setMotor(ClawConstants.CLAW_MOTOR_SPEED);
        }

        else if (claw.getBreakStatus() == BeamBreakStatus.CONE)
        {
            claw.setMotor(-ClawConstants.CLAW_MOTOR_SPEED);
        }
    }

    @Override
    public void end(boolean interrupted)
    {
        claw.setMotor(0);
    }
}
