package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClawConstants;
import frc.robot.utils.Motor;

public class Claw extends SubsystemBase {
    private CANSparkMax motor; // References the motor of the claw
    private DigitalInput cubeSensor; // Beam break sensor to detect if there is a cube
    private DigitalInput coneSensor; // Beam break sensor to detect if there is a cone

    // Stores the different states of the beam break sensors
    public enum BeamBreakStatus {
        NONE, // If there is no game piece in the claw
        CUBE, // If there is a cube in the claw
        CONE, // If there is a cone in the claw
        BOTH; // If there is somehow both a cube and a cone in the claw
    }

    /**
     * This class references the claw subsystem. It is able to pick up game pieces with an intake
     * and it has methods for setting the speed of the motors and checking the status of the beam
     * break sensors that check what game piece is currently in the claw if applicable.
     *
     * @param motor - references the motor of the claw
     * @param cubeSensor - a sensor that detects if there is a cube in the claw (two states, true or
     *     false)
     * @param coneSensor - a sensor that detects if there is a cone in the claw (two states, true or
     *     false)
     */
    public Claw(Motor motor, DigitalInput cubeSensor, DigitalInput coneSensor) {
        this.motor = motor;
        this.cubeSensor = cubeSensor;
        this.coneSensor = coneSensor;
    }

    /**
     * This class references the claw subsystem. It is able to pick up game pieces with an intake
     * and it has methods for setting the speed of the motors and checking the status of the beam
     * break sensors that check what game piece is currently in the claw if applicable. This
     * particular overload sets the motor and sensors to new objects using values from
     * Constants.java.
     */
    public Claw() {
        motor =
                new CANSparkMax(ClawConstants.CLAW_MOTOR_PORT, MotorType.kBrushed);
        cubeSensor = new DigitalInput(ClawConstants.CUBE_SENSOR_CHANNEL);
        coneSensor = new DigitalInput(ClawConstants.CONE_SENSOR_CHANNEL);
    }

    /**
     * Sets the speed of the claw motor
     *
     * @param speed - the speed that the motor is set to (percentage in decimals from -1 to 1)
     */
    public void setMotor(double speed) {
        motor.set(speed);
    }

    /**
     * Gets the status of the beam break sensors (what piece is currently in the claw)
     *
     * @return An enum representing which piece is in the claw currently, if there is any (BOTH:
     *     both pieces, CUBE: only cube piece, CONE: only cone piece, NONE: no pieces)
     */
    public BeamBreakStatus getBreakStatus() {
        if (!cubeSensor.get() && !coneSensor.get()) {
            return BeamBreakStatus.BOTH;
        } else if (!cubeSensor.get()) {
            return BeamBreakStatus.CUBE;
        } else if (!coneSensor.get()) {
            return BeamBreakStatus.CONE;
        } else {
            return BeamBreakStatus.NONE;
        }
    }

    /**
     * Checks if the claw has any piece
     *
     * @return A boolean that is true if there is a piece, false if not
     */
    public boolean hasPiece() {
        return cubeSensor.get() != coneSensor.get();
    }

    /**
     * Getter for cube sensor object
     *
     * @return Cube sensor for current claw object
     */
    public DigitalInput getCubeSensor() {
        return cubeSensor;
    }

    /**
     * Getter for cone sensor object
     *
     * @return Cone sensor for current claw object
     */
    public DigitalInput getConeSensor() {
        return coneSensor;
    }

    @Override
    public void periodic() {}
}
