package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.utils.Motor;
import frc.robot.utils.SparkMotorGroup;

public class Arm extends SubsystemBase {
    // Rotation motors
    // private Motor rotationMotor1;
    // private Motor rotationMotor2;
    // private SparkMotorGroup rotationMotors;

    // Extension motors
    // private Motor extensionMotor1;
    // private Motor extensionMotor2;
    // private SparkMotorGroup extensionMotors;

    // Motors for rotating claw independently to keep it level when arm has rotated
    private Motor clawRotationMotor1;
    private Motor clawRotationMotor2;
    private SparkMotorGroup clawRotationMotors;

    private double clawRotationError;
    private double clawRotationSetpoint;

    // // Rotation PID
    // private PIDController rotationPid;

    // // Extension PID
    // private PIDController extensionPid;

    // Shuffleboard Stuff
    private ShuffleboardTab armTab;
    private GenericEntry shuffleboardClawRotationSetpoint;
    private GenericEntry shuffleboardClawRotationError;

    private ArmFeedforward clawRotationFeedforward;
    private PIDController clawRotationPid;

    /**
     * The arm that picks up game pieces from the floor through the use of the intake. It can rotate
     * -180 to 180 degrees and can extend a certain distance.
     */
    public Arm() {

        clawRotationFeedforward = new ArmFeedforward(ArmConstants.CLAW_S_VOLTS, ArmConstants.CLAW_G, ArmConstants.CLAW_V_VOLTS_SECONDS_PER_METER, ArmConstants.CLAW_A_VOLTS_SECONDS_SQUARED_PER_METER);
        // Define rotational motors
        // rotationMotor1 =
        //         new Motor(
        //                 ArmConstants.ROTATION_MOTOR_1_PORT,
        //                 ArmConstants.ROTATION_MOTOR_1_REVERSED,
        //                 ArmConstants.ROTATION_MOTOR_GEAR_RATIO,
        //                 ArmConstants.ROTATION_MOTOR_WHEEL_DIAMETER);
        // rotationMotor2 =
        //         new Motor(
        //                 ArmConstants.ROTATION_MOTOR_2_PORT,
        //                 ArmConstants.ROTATION_MOTOR_2_REVERSED,
        //                 ArmConstants.ROTATION_MOTOR_GEAR_RATIO,
        //                 ArmConstants.ROTATION_MOTOR_WHEEL_DIAMETER);
        // rotationMotors = new SparkMotorGroup(false, rotationMotor1, rotationMotor2);

        // // Define extension motors
        // extensionMotor1 =
        //         new Motor(
        //                 ArmConstants.EXTENSION_MOTOR_1_PORT,
        //                 ArmConstants.EXTENSION_MOTOR_1_REVERSED,
        //                 ArmConstants.EXTENSION_MOTOR_GEAR_RATIO,
        //                 ArmConstants.EXTENSION_MOTOR_WHEEL_DIAMETER);
        // extensionMotor2 =
        //         new Motor(
        //                 ArmConstants.EXTENSION_MOTOR_2_PORT,
        //                 ArmConstants.EXTENSION_MOTOR_2_REVERSED,
        //                 ArmConstants.EXTENSION_MOTOR_GEAR_RATIO,
        //                 ArmConstants.EXTENSION_MOTOR_WHEEL_DIAMETER);
        // extensionMotors = new SparkMotorGroup(false, extensionMotor1, extensionMotor2);

        // Define claw rotational motors
        clawRotationMotor1 =
                new Motor(
                        ArmConstants.CLAW_ROTATION_MOTOR_1_PORT,
                        ArmConstants.CLAW_ROTATION_MOTOR_1_REVERSED,
                        ArmConstants.CLAW_ROTATION_MOTOR_GEAR_RATIO,
                        ArmConstants.CLAW_ROTATION_MOTOR_WHEEL_DIAMETER);
        clawRotationMotor2 =
                new Motor(
                        ArmConstants.CLAW_ROTATION_MOTOR_2_PORT,
                        ArmConstants.CLAW_ROTATION_MOTOR_2_REVERSED,
                        ArmConstants.CLAW_ROTATION_MOTOR_GEAR_RATIO,
                        ArmConstants.CLAW_ROTATION_MOTOR_WHEEL_DIAMETER);
        clawRotationMotors = new SparkMotorGroup(true, clawRotationMotor1, clawRotationMotor2);
        clawRotationPid = new PIDController(ArmConstants.CLAW_ROTATION_PID_P, 0, ArmConstants.CLAW_ROTATION_PID_D);

        // Defines shuffleboard tab and entries
        armTab = Shuffleboard.getTab("Arm");
        shuffleboardClawRotationError = armTab.add("Claw Rotation Error", 0).getEntry();
        shuffleboardClawRotationSetpoint = armTab.add("Claw Rotation Setpoint", 0).getEntry();
        // armTab.add("Extension PID Controller", extensionPid);
        // armTab.add("Rotation PID Controller", rotationPid);
    }

    @Override
    public void periodic() {
        clawRotationSetpoint = 180 - clawRotationSetpoint;
        clawRotationError = clawRotationSetpoint - getClawRotationAbsoluteAngle();

        shuffleboardClawRotationError.setDouble(clawRotationError);
        shuffleboardClawRotationSetpoint.setDouble(clawRotationSetpoint);
    }

    // /**
    //  * Sets the angular velocity of the rotation motors in radians per second
    //  *
    //  * @param angularVelocity - velocity of the rotation motors in radians per second
    //  */
    // public void setRotationRadiansPerSecond(double angularVelocity) {
    //     double voltage =
    //             ArmConstants.ARM_ROTATION_FEED_FORWARD.calculate(
    //                             Math.toRadians(getRotationAngle()), getRotationRadiansPerSecond())
    //                     + rotationPid.calculate(getRotationRadiansPerSecond());
    //     rotationMotors.setVoltage(voltage);
    // }

    // /**
    //  * Sets the velocity for the extension motors
    //  *
    //  * @param velocity - velocity in meters per second you want to set it to
    //  */
    // public void setExtensionMetersPerSecond(double velocity) {
    //     double voltage =
    //             ArmConstants.ARM_EXTENSION_FEED_FORWARD.calculate(
    //                             getExtensionLength(), getExtensionMetersPerSecond())
    //                     + extensionPid.calculate(getExtensionMetersPerSecond());
    //     extensionMotors.setVoltage(voltage);
    // }

    /**
     * Sets the angular velocity of the claw rotations motors in radians per second
     *
     * @param angularVelocity - velocity of the claw rotation motors in radians per second
     */
    public void setClawRotationRadiansPerSecond(double angularVelocity) {
        double voltage =
                clawRotationFeedforward.calculate(
                        Math.toRadians(getClawRotationAbsoluteAngle()),
                        angularVelocity) 
                + MathUtil.clamp(clawRotationPid.calculate(getClawRotationRadiansPerSecond(), angularVelocity), -7, 7);
        clawRotationMotors.setVoltage(-voltage);
        System.out.println("Ouput Voltage: " + voltage);
        System.out.println("Desired Angular Velocity: " + angularVelocity);
    }

    // /**
    //  * Gets the velocity for the extension motors
    //  *
    //  * @return the velocity of the motors in meters per second
    //  */
    // public double getExtensionMetersPerSecond() {
    //     return getExtensionEncoder().getPosition()
    //             * ArmConstants.EXTENSION_MOTOR_GEAR_RATIO
    //             * ArmConstants.EXTENSION_MOTOR_WHEEL_DIAMETER
    //             * Math.PI
    //             / 60;
    // }

    // /**
    //  * Gets the rotational velocity of the arm in radians per second
    //  *
    //  * @return the rotational velocity of the arm in radians per second
    //  */
    // public double getRotationRadiansPerSecond() {
    //     return rotationMotor1.getRPM() / 60 * Math.PI * 2;
    // }

    /**
     * Gets the rotational velocity of the claw in radians per second
     *
     * @return the rotational velocity of the claw rotation motors in radians per second
     */
    public double getClawRotationRadiansPerSecond() {
        System.out.println("Radians Per Second: " + clawRotationMotor1.getRPM() / 60 * Math.PI * 2);
        return clawRotationMotor1.getRPM() / 60 * Math.PI * 2;
    }

    // /**
    //  * Gets the angle of rotation of the arm in degrees
    //  *
    //  * @return the angle of the arm in degrees
    //  */
    // public double getRotationAngle() {
    //     return rotationMotor1.getRotations() * 360;
    // }

    // /**
    //  * Gets the length of the extension arm from the center of rotation
    //  *
    //  * @return the length in meters of the arm from the center of rotation
    //  */
    // public double getExtensionLength() {
    //     return extensionMotor1.getMeters();
    // }

    /**
     * Gets the angle of the rotation of the claw in degrees
     *
     * @return the rotation of the claw in degrees
     */
    public double getClawRotationRelativeAngle() {
        return clawRotationMotor1.getRotations() * 360;
    }

    // /**
    //  * Gets the encoder for the rotation motors
    //  *
    //  * @return an encoder for the rotation motors
    //  */
    // public RelativeEncoder getRotationEncoder() {
    //     return rotationMotors.getEncoder();
    // }

    // /**
    //  * Gets the encoder for the extension motors
    //  *
    //  * @return an encoder for the extension motors
    //  */
    // public RelativeEncoder getExtensionEncoder() {
    //     return extensionMotors.getEncoder();
    // }

    /**
     * Gets the encoder for the claw rotation motors
     *
     * @return an encoder for the claw rotation motors
     */
    public RelativeEncoder getClawRotationEncoder() {
        return clawRotationMotors.getEncoder();
    }

    // /**
    //  * Gets the pid controller for rotation
    //  *
    //  * @return the pid controller object for rotation
    //  */
    // public PIDController getRotationPid() {
    //     return rotationPid;
    // }

    // /**
    //  * Gets the pid controller for extension
    //  *
    //  * @return the pid controller object for extension
    //  */
    // public PIDController getExtensionPid() {
    //     return extensionPid;
    // }

    // /**
    //  * Gets the amount of meters traveled by the rotation motors
    //  *
    //  * @return the amount of meters traveled by the rotation motors
    //  */
    // public double getRotationMeters() {
    //     return getRotationEncoder().getPosition()
    //             * ArmConstants.ROTATION_MOTOR_GEAR_RATIO
    //             * ArmConstants.ROTATION_MOTOR_WHEEL_DIAMETER
    //             * Math.PI;
    // }

    // /**
    //  * Gets the amount of meters traveled by the extension motors
    //  *
    //  * @return the amount of meters traveled by the extension motors
    //  */
    // public double getExtensionMeters() {
    //     return getExtensionEncoder().getPosition()
    //             * ArmConstants.EXTENSION_MOTOR_GEAR_RATIO
    //             * ArmConstants.EXTENSION_MOTOR_WHEEL_DIAMETER
    //             * Math.PI;
    // }

    /**
     * Gets the amount of meters traveled by the claw rotation motors
     *
     * @return the amount of meters traveled by the claw rotation motors
     */
    public double getClawRotationMeters() {
        return getClawRotationEncoder().getPosition()
                * ArmConstants.CLAW_ROTATION_MOTOR_GEAR_RATIO
                * ArmConstants.CLAW_ROTATION_MOTOR_WHEEL_DIAMETER
                * Math.PI;
    }

    /**
     * Gets the error for the claw rotation
     *
     * @return the difference between the current claw angle and the desired angle (which is always
     *     level)
     */
    public double getClawRotationError() {
        return clawRotationError;
    }

    /**
     * Gets the setpoint for the rotation of the claw
     *
     * @return the amount of degrees the claw has to rotate in order to reach level postion
     */
    public double getClawRotationSetpoint() {
        return clawRotationSetpoint;
    }

    /**
     * Gets the angle of the claw taking into account the angle at which the claw originally started
     * at
     *
     * @return the angle of the claw gotten from the encoders plus the starting the angle
     */
    public double getClawRotationAbsoluteAngle() {
        return getClawRotationRelativeAngle() + ArmConstants.CLAW_STARTING_ANGLE;
    }
}
