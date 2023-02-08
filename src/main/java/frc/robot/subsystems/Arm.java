package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.utils.Motor;

public class Arm extends SubsystemBase {
    // Rotation motors
    private Motor rotationMotor1;
    private Motor rotationMotor2;

    // Extension motors
    private Motor extensionMotor1;
    private Motor extensionMotor2;

    // Rotation PID
    private PIDController rotationPid;

    // Extension PID
    private PIDController extensionPid;

    // Shuffleboard Display
    private final ShuffleboardTab armTab; // Tab that will be used for PID tracking
    private final GenericEntry currentAngle, // tracks current angle of arm
            extensionPidP, // tracks the proportional PID value of the extension pid controller
            extensionPidI, // tracks the integral PID value of the extension pid controller
            extensionPidD, // tracks the derivative PID value of the extension pid controller
            rotationPidP, // tracks the proportional PID value of the rotation pid controller
            rotationPidI, // tracks the integral PID value of the rotation pid controller
            rotationPidD; // tracks the derivative PID value of the rotation pid controller

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

        // Defines shuffleboard entries and tab
        armTab = Shuffleboard.getTab("Arm");
        currentAngle = armTab.add("Current Angle", 0).getEntry();
        extensionPidP = armTab.add("Extension PID P", 0).getEntry();
        extensionPidI = armTab.add("Extension PID I", 0).getEntry();
        extensionPidD = armTab.add("Extension PID D", 0).getEntry();
        rotationPidP = armTab.add("Rotation PID P", 0).getEntry();
        rotationPidI = armTab.add("Rotation PID I", 0).getEntry();
        rotationPidD = armTab.add("Rotation PID D", 0).getEntry();
    }

    @Override
    public void periodic() {
        // TODO: Set current angle shuffleboard value to current angle of arm
        // currentAngle.setDouble(currentAngle);
        extensionPidP.setDouble(ArmConstants.EXTENSION_PID_P);
        extensionPidI.setDouble(ArmConstants.EXTENSION_PID_I);
        extensionPidD.setDouble(ArmConstants.EXTENSION_PID_D);
        rotationPidP.setDouble(ArmConstants.ROTATION_PID_P);
        rotationPidI.setDouble(ArmConstants.ROTATION_PID_I);
        rotationPidD.setDouble(ArmConstants.ROTATION_PID_D);
    }
}
