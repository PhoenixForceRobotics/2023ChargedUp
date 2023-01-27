package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Elevator;

public class MoveArm extends CommandBase 
{

    private Elevator elevator;
    private double speed;


    public MoveArm(Elevator elevator, double speed)
    {
        this.elevator = elevator;
    }


    @Override
    public void initialize() 
    {
        speed = 0;
    }

    @Override
    public void execute()
    {
        
    }
}