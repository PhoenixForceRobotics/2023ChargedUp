package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.Motor;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.utils.SparkMotorGroup;

public class Elevator extends SubsystemBase
{
    private Motor motor1;
    private Motor motor2;
    private Motor motor3;
    private SparkMotorGroup motorGroup;
    
    public Elevator()
    {
        motor1 = new Motor(ElevatorConstants.WHEEL_1_PORT, ElevatorConstants.WHEEL_1_REVERSED, ElevatorConstants.ELEVATOR_GEAR_RATIO, ElevatorConstants.ELEVATOR_WHEEL_DIAMETER);
        motor2 = new Motor(ElevatorConstants.WHEEL_2_PORT, ElevatorConstants.WHEEL_2_REVERSED, ElevatorConstants.ELEVATOR_GEAR_RATIO, ElevatorConstants.ELEVATOR_WHEEL_DIAMETER);
        motor3 = new Motor(ElevatorConstants.WHEEL_3_PORT, ElevatorConstants.WHEEL_3_REVERSED, ElevatorConstants.ELEVATOR_GEAR_RATIO, ElevatorConstants.ELEVATOR_WHEEL_DIAMETER);
        motorGroup = new SparkMotorGroup(true, motor1, motor2, motor3);
    }

    public void setMotors(double speed)
    {
        motorGroup.set(speed);
    }
}
