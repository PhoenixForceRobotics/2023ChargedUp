package frc.robot.commands.claw;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Claw;
import frc.robot.utils.PFRController;

public class MoveClaw extends CommandBase
{
    private Claw claw;
    private PFRController driverController;
    private boolean isPickingUpCube;

    public MoveClaw(Claw claw, PFRController driverController, boolean isPickingUpCube)
    {
        this.claw = claw;
        this.driverController = driverController;
        this.isPickingUpCube = isPickingUpCube;
    }

    @Override
    public void initialize()
    {
        claw.setMotor(0);
    }

    @Override
    public void execute()
    {
        if (isPickingUpCube)
        {
            claw.setMotor(1);
        }
        else
        {
            claw.setMotor(-1);
        }
    }

    @Override
    public boolean isFinished()
    {
        return claw.getBreakBeamSensor().get() || driverController.getAButtonReleased();
    }

    @Override
    public void end(boolean interrupted)
    {
        claw.setMotor(0);
    }
}
