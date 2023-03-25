package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClawConstants;
import frc.robot.utils.Motor;

public class Claw extends SubsystemBase {
    private Motor motor; // References the motor of the claw

    /**
     * This class references the claw subsystem. It is able to pick up game pieces with an intake
     * and it has methods for setting the speed of the motors. This particular overload sets the
     * motor to a new object using Constants.java.
     */
    public Claw() {
        motor = new Motor(ClawConstants.CLAW_MOTOR_PORT, ClawConstants.CLAW_MOTOR_REVERSED);
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
