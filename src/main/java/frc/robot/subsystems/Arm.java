package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.utils.Motor;
import frc.robot.utils.SparkMotorGroup;

public class Arm extends SubsystemBase {
    // Rotation motors
    private Motor rotationMotor1;
    private Motor rotationMotor2;
    private SparkMotorGroup rotationMotors;

    // Extension motors
    private Motor extensionMotor1;
    private Motor extensionMotor2;
    private SparkMotorGroup extensionMotors;

    // Rotation PID
    private PIDController rotationPid;

    // Extension PID
    private PIDController extensionPid;

    /**
     * The arm that picks up game pieces from the floor through the use of the intake. It can rotate
     * -180 to 180 degrees and can extend a certain distance.
     */
    public Arm() {
        // Define rotational motors
        rotationMotor1 =
                new Motor(
                        ArmConstants.ROTATION_MOTOR_1_PORT,
                        ArmConstants.ROTATION_MOTOR_1_REVERSED,
                        ArmConstants.ARM_MOTOR_GEAR_RATIO,
                        ArmConstants.ARM_MOTOR_WHEEL_DIAMETER);
        rotationMotor2 =
                new Motor(
                        ArmConstants.ROTATION_MOTOR_2_PORT,
                        ArmConstants.ROTATION_MOTOR_2_REVERSED,
                        ArmConstants.ARM_MOTOR_GEAR_RATIO,
                        ArmConstants.ARM_MOTOR_WHEEL_DIAMETER);
        rotationMotors = new SparkMotorGroup(false, rotationMotor1, rotationMotor2);

        // Define extension motors
        extensionMotor1 =
                new Motor(
                        ArmConstants.EXTENSION_MOTOR_1_PORT,
                        ArmConstants.EXTENSION_MOTOR_1_REVERSED,
                        ArmConstants.ARM_MOTOR_GEAR_RATIO,
                        ArmConstants.ARM_MOTOR_WHEEL_DIAMETER);
        extensionMotor2 =
                new Motor(
                        ArmConstants.EXTENSION_MOTOR_2_PORT,
                        ArmConstants.EXTENSION_MOTOR_2_REVERSED,
                        ArmConstants.ARM_MOTOR_GEAR_RATIO,
                        ArmConstants.ARM_MOTOR_WHEEL_DIAMETER);
        extensionMotors = new SparkMotorGroup(false, extensionMotor1, extensionMotor2);

        // Defines pid controllers
        rotationPid =
                new PIDController(
                        ArmConstants.ROTATION_PID_P,
                        ArmConstants.ROTATION_PID_I,
                        ArmConstants.ROTATION_PID_D);
        extensionPid =
                new PIDController(
                        ArmConstants.EXTENSION_PID_P,
                        ArmConstants.EXTENSION_PID_I,
                        ArmConstants.EXTENSION_PID_D);
    }

    public void setRotationMetersPerSecond(double velocity)
    {
        double voltage = ArmConstants.ARM_FEED_FORWARD.calculate(velocity) + rotationPid.calculate(getRotationMetersPerSecond());
        rotationMotors.setVoltage(voltage);
    }

    public void setExtensionMetersPerSecond(double velocity)
    {
        double voltage = ArmConstants.ARM_FEED_FORWARD.calculate(velocity) + extensionPid.calculate(getExtensionMetersPerSecond());
        extensionMotors.setVoltage(voltage);
    }

    public double getExtensionMetersPerSecond()
    {
        return getExtensionEncoder().getPosition() * ArmConstants.ARM_MOTOR_GEAR_RATIO * ArmConstants.ARM_MOTOR_WHEEL_DIAMETER * Math.PI / 60;
    }

    public double getRotationMetersPerSecond()
    {
        return getRotationEncoder().getPosition() * ArmConstants.ARM_MOTOR_GEAR_RATIO * ArmConstants.ARM_MOTOR_WHEEL_DIAMETER * Math.PI / 60;
    }

    public double getRotationAngle()
    {
        return rotationMotor1.getRotations() * 360;
    }

    public double getExtensionLength()
    {
        return extensionMotor1.getMeters();
    }

    public RelativeEncoder getRotationEncoder()
    {
        return rotationMotors.getEncoder();
    }

    public RelativeEncoder getExtensionEncoder()
    {
        return extensionMotors.getEncoder();
    }

    public PIDController getRotationPid()
    {
        return rotationPid;
    }

    public PIDController getExtensionPid()
    {
        return extensionPid;
    }

    public double getRotationMeters()
    {
        return getRotationEncoder().getPosition() * ArmConstants.ARM_MOTOR_GEAR_RATIO * ArmConstants.ARM_MOTOR_WHEEL_DIAMETER * Math.PI;
    }

    public double getExtensionMeters()
    {
        return getExtensionEncoder().getPosition() * ArmConstants.ARM_MOTOR_GEAR_RATIO * ArmConstants.ARM_MOTOR_WHEEL_DIAMETER * Math.PI;
    }
}
