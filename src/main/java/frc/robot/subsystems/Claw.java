package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.Motor;

public class Claw extends SubsystemBase
{
    private Motor motor;
    private DigitalInput cubeSensor;
    private DigitalInput coneSensor;

    public enum BeamBreakStatus
    {
      NONE,
      CUBE,
      CONE,
      BOTH;
    }

    public Claw(Motor motor, DigitalInput cubeSensor, DigitalInput coneSensor)
    {
        this.motor = motor;
        this.cubeSensor = cubeSensor;
        this.coneSensor = coneSensor;
    }

    public void setMotor(double speed)
    {
        motor.set(speed);
    }

    public BeamBreakStatus getBreakStatus()
    {
        if (cubeSensor.get() && coneSensor.get())
        {
            return BeamBreakStatus.BOTH;
        }

        else if (cubeSensor.get())
        {
            return BeamBreakStatus.CUBE;
        }

        else if (coneSensor.get())
        {
            return BeamBreakStatus.CONE;
        }

        else
        {
            return BeamBreakStatus.NONE;
        }
    }
 
    public boolean hasPiece()
    {
        return cubeSensor.get() || coneSensor.get();
    }

    public DigitalInput getCubeSensor() 
    {
        return cubeSensor;
    }

    public DigitalInput getConeSensor()
    {
        return coneSensor;
    }

    public Motor getMotor() 
    {
        return motor;
    }

    @Override
    public void periodic()
    {

    }
}
    