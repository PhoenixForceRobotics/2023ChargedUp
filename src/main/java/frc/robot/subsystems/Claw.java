package frc.robot.subsystems;

import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClawConstants;
import frc.robot.utils.Motor;

public class Claw extends SubsystemBase
{
    private Motor motor;
    private ColorSensorV3 colorSensor;
    private DigitalInput breakBeamSensor;

    public Claw()
    {
        motor = new Motor(ClawConstants.CLAW_MOTOR_PORT, ClawConstants.CLAW_MOTOR_REVERSED, ClawConstants.CLAW_GEAR_RATIO, ClawConstants.CLAW_WHEEL_DIAMETER);
        colorSensor = new ColorSensorV3(ClawConstants.COLOR_SENSOR_PORT);
        breakBeamSensor = new DigitalInput(0);
    }

    public void setMotor(double speed)
    {
        motor.set(speed);
    }

    public ColorSensorV3 getColorSensor() 
    {
        return colorSensor;
    }

    public DigitalInput getBreakBeamSensor() 
    {
        return breakBeamSensor;
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
