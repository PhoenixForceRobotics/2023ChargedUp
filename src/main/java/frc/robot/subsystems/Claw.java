package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClawConstants;

public class Claw extends SubsystemBase {
    private CANSparkMax motor; // References the motor of the claw

    /**
     * This class references the claw subsystem. It is able to pick up game pieces with an intake
     * and it has methods for setting the speed of the motors. This particular overload sets the
     * motor to a new object using Constants.java.
     */
    public Claw() {
        motor = new CANSparkMax(ClawConstants.CLAW_MOTOR_PORT, MotorType.kBrushed);
    }

    /**
     * Sets the speed of the claw motor
     *
     * @param speed - the speed that the motor is set to (percentage in decimals from -1 to 1)
     */
    public void setMotor(double speed) {
        motor.set(speed);
    }
}
