package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;

public class SetAngle extends CommandBase {
    private double desiredAngle;
    private Arm arm;

    public SetAngle(Arm arm, double desiredAngle)
    {
        this.arm = arm;
        this.desiredAngle = desiredAngle;
    }

    @Override
    public void initialize()
    {
        arm.getRotationPid().setSetpoint(desiredAngle);
    }

    @Override
    public void execute()
    {
        double error = desiredAngle - arm.getRotationAngle();
    }

    @Override
    public boolean isFinished()
    {
        return arm.getRotationPid().atSetpoint();
    }

    @Override
    public void end(boolean interrupted)
    {

    }
}
