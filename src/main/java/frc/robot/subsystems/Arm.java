package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.utils.Motor;
import frc.robot.utils.PFRArmPIDController;
import frc.robot.utils.PFRExtensionPIDController;

public class Arm extends SubsystemBase {
    // Rotation motors + controller
    private Motor armRotationLeader;
    private Motor armRotationFollower;
    private PFRArmPIDController armRotationControllerNoExtension;
    private PFRArmPIDController armRotationControllerHalfExtension;
    private PFRArmPIDController armRotationControllerFullExtension;

    // Extension motors + controller
    private Motor firstStageExtensionMotor; // Fullsized Neo (bottom side)
    private Motor secondStageExtensionMotor; // Neo 550 (top side)
    private PFRExtensionPIDController
            firstStageExtensionController0Degrees; // PID + feedforward for full neo, 0 deg
    private PFRExtensionPIDController
            firstStageExtensionController45Degrees; // PID + feedforward for full neo, 45 deg
    private PFRExtensionPIDController
            firstStageExtensionController90Degrees; // PID + feedforward for full neo, 90 deg

    private PFRExtensionPIDController
            secondStageExtensionContoller0Degrees; // PID + feedforward for neo 550, 0 deg
    private PFRExtensionPIDController
            secondStageExtensionContoller45Degrees; // PID + feedforward for neo 550, 45 deg
    private PFRExtensionPIDController
            secondStageExtensionContoller90Degrees; // PID + feedforward for neo 550, 90 deg

    // Motors + controller for rotating claw independently to keep it level when arm has rotated
    private Motor clawRotationLeader;
    private Motor clawRotationFollower;
    private PFRArmPIDController clawRotationController;

    // Changing these variables changes how the arm moves
    // Change these variables through the "setters"
    private double desiredArmRadiansPerSecond = 0;
    private double desiredFirstStageMetersPerSecond = 0;
    private double desiredSecondStageMetersPerSecond = 0; 

    // Changing this variable ONLY changes IF it is not in independent control
    private double desiredClawRadiansPerSecond = 0; // relative to the arm!
    private boolean isClawIndependentlyControlled = false;

    /**
     * The arm moves the intake It can rotate -180 to 180 degrees and can extend a certain distance.
     */
    public Arm() {

        // Define rotational motors
        armRotationLeader =
                new Motor(
                        ArmConstants.ARM_ROTATION_MOTOR_1_PORT,
                        ArmConstants.ARM_ROTATION_MOTOR_1_REVERSED,
                        ArmConstants.ARM_ROTATION_MOTOR_GEAR_RATIO,
                        ArmConstants.ROTATION_MOTOR_WHEEL_DIAMETER);
        armRotationFollower =
                new Motor(
                        ArmConstants.ARM_ROTATION_MOTOR_2_PORT,
                        ArmConstants.ARM_ROTATION_MOTOR_2_REVERSED,
                        ArmConstants.ARM_ROTATION_MOTOR_GEAR_RATIO,
                        ArmConstants.ROTATION_MOTOR_WHEEL_DIAMETER);

        // set follower motor to follow leader
        armRotationFollower.follow(armRotationLeader);

        // create controllers for arm rotation
        armRotationControllerNoExtension =
                new PFRArmPIDController(ArmConstants.ARM_ROTATION_NO_EXTENSION_PID_VALUES);
        armRotationControllerHalfExtension =
                new PFRArmPIDController(ArmConstants.ARM_ROTATION_HALF_EXTENSION_PID_VALUES);
        armRotationControllerFullExtension =
                new PFRArmPIDController(ArmConstants.ARM_ROTATION_FULL_EXTENSION_PID_VALUES);

        // Define extension motors
        firstStageExtensionMotor =
                new Motor(
                        ArmConstants.FIRST_STAGE_PORT,
                        ArmConstants.FIRST_STAGE_REVERSED,
                        ArmConstants.FIRST_STAGE_GEAR_RATIO,
                        ArmConstants.FIRST_STAGE_WHEEL_DIAMETER);
        secondStageExtensionMotor =
                new Motor(
                        ArmConstants.SECOND_STAGE_PORT,
                        ArmConstants.SECOND_STAGE_REVERSED,
                        ArmConstants.SECOND_STAGE_GEAR_RATIO,
                        ArmConstants.SECOND_STAGE_WHEEL_DIAMETER);

        firstStageExtensionController0Degrees =
                new PFRExtensionPIDController(ArmConstants.FIRST_STAGE_0_DEGREES_PID_VALUES);
        firstStageExtensionController45Degrees =
                new PFRExtensionPIDController(ArmConstants.FIRST_STAGE_45_DEGREES_PID_VALUES);
        firstStageExtensionController90Degrees =
                new PFRExtensionPIDController(ArmConstants.FIRST_STAGE_90_DEGREES_PID_VALUES);

        secondStageExtensionContoller0Degrees =
                new PFRExtensionPIDController(ArmConstants.SECOND_STAGE_0_DEGREES_PID_VALUES);
        secondStageExtensionContoller45Degrees =
                new PFRExtensionPIDController(ArmConstants.SECOND_STAGE_45_DEGREES_PID_VALUES);
        secondStageExtensionContoller90Degrees =
                new PFRExtensionPIDController(ArmConstants.SECOND_STAGE_90_DEGREES_PID_VALUES);

        // Define claw rotational motors
        clawRotationLeader =
                new Motor(
                        ArmConstants.CLAW_ROTATION_MOTOR_1_PORT,
                        ArmConstants.CLAW_ROTATION_MOTOR_1_REVERSED,
                        ArmConstants.CLAW_ROTATION_MOTOR_GEAR_RATIO,
                        ArmConstants.CLAW_ROTATION_MOTOR_WHEEL_DIAMETER);
        clawRotationFollower =
                new Motor(
                        ArmConstants.CLAW_ROTATION_MOTOR_2_PORT,
                        ArmConstants.CLAW_ROTATION_MOTOR_2_REVERSED,
                        ArmConstants.CLAW_ROTATION_MOTOR_GEAR_RATIO,
                        ArmConstants.CLAW_ROTATION_MOTOR_WHEEL_DIAMETER);

        // make follwer follw the leader, BUT INVERTED (due to how it works)
        clawRotationFollower.follow(clawRotationLeader, true);

        // Claw rotation controller
        clawRotationController = new PFRArmPIDController(ArmConstants.CLAW_ROTATION_PID_VALUES);
    }

    @Override
    public void periodic() {
        if (getArmRotationRadians() <= ArmConstants.MIN_ARM_ANGLE) {
            desiredArmRadiansPerSecond = Math.max(0, desiredArmRadiansPerSecond);
        } else if (ArmConstants.MAX_ARM_ANGLE <= getArmRotationRadians()) {
            desiredArmRadiansPerSecond = Math.min(desiredArmRadiansPerSecond, 0);
        }

        if (ArmConstants.FIRST_STAGE_MIN_EXTENSION >= getFirstStageMeters()) {
            desiredFirstStageMetersPerSecond = Math.max(0, desiredFirstStageMetersPerSecond);
        } else if (ArmConstants.FIRST_STAGE_MAX_EXTENSION <= getFirstStageMeters()) {
            desiredFirstStageMetersPerSecond = Math.min(desiredFirstStageMetersPerSecond, 0);
        }
        if (ArmConstants.SECOND_STAGE_MIN_EXTENSION >= getSecondStageMeters()) {
            desiredSecondStageMetersPerSecond = Math.max(0, desiredSecondStageMetersPerSecond);
        } else if (ArmConstants.SECOND_STAGE_MAX_EXTENSION <= getSecondStageMeters()) {
            desiredSecondStageMetersPerSecond = Math.min(desiredSecondStageMetersPerSecond, 0);
        }

        if (ArmConstants.MIN_CLAW_ANGLE >= getClawRelativeAngleRadians()) {
            desiredClawRadiansPerSecond = Math.min(0, desiredClawRadiansPerSecond);
        } else if (ArmConstants.MAX_CLAW_ANGLE <= getClawRelativeAngleRadians()) {
            desiredClawRadiansPerSecond = Math.max(desiredClawRadiansPerSecond, 0);
        }
        // TODO: Re-enable these once we have feedforwards

        // Choose which arm rotation feedforward would be best
        double armRotationVoltageNoExtension =
                armRotationControllerNoExtension.filteredCalculate(
                        getArmRotationRadians(),
                        getArmRotationRadiansPerSecond(),
                        desiredArmRadiansPerSecond);
        double armRotationVoltageHalfExtension =
                armRotationControllerHalfExtension.filteredCalculate(
                        getArmRotationRadians(),
                        getArmRotationRadiansPerSecond(),
                        desiredArmRadiansPerSecond);
        double armRotationVoltageFullExtension =
                armRotationControllerFullExtension.filteredCalculate(
                        getArmRotationRadians(),
                        getArmRotationRadiansPerSecond(),
                        desiredArmRadiansPerSecond);

        double armLength = getFullExtensionMeters();
        if (armLength <= ArmConstants.LOOKUP_TABLE_BOUNDARY_1) {
            armRotationLeader.setVoltage(armRotationVoltageNoExtension);
        } else if (ArmConstants.LOOKUP_TABLE_BOUNDARY_1 <= armLength
                && armLength <= ArmConstants.LOOKUP_TABLE_BOUNDARY_2) {
            armRotationLeader.setVoltage(armRotationVoltageHalfExtension);
        } else if (ArmConstants.LOOKUP_TABLE_BOUNDARY_2 <= armLength) {
            armRotationLeader.setVoltage(armRotationVoltageFullExtension);
        } else {
            System.out.println("Ruh Roh!!!!!");
        }
        // Choose which arm extension feedforward would be best
        double firstStageVoltage0Degree =
                firstStageExtensionController0Degrees.filteredCalculate(
                        getFirstStageMeters(),
                        getFirstStageMetersPerSecond(),
                        desiredFirstStageMetersPerSecond);
        double firstStageVoltage45Degree =
                firstStageExtensionController45Degrees.filteredCalculate(
                        getFirstStageMeters(),
                        getFirstStageMetersPerSecond(),
                        desiredFirstStageMetersPerSecond);
        double firstStageVoltage90Degree =
                firstStageExtensionController90Degrees.filteredCalculate(
                        getFirstStageMeters(),
                        getFirstStageMetersPerSecond(),
                        desiredFirstStageMetersPerSecond);

        double secondStageVoltage0Degree =
                secondStageExtensionContoller0Degrees.filteredCalculate(
                        getSecondStageMeters(),
                        getSecondStageMetersPerSecond(),
                        desiredSecondStageMetersPerSecond);
        double secondStageVoltage45Degree =
                secondStageExtensionContoller45Degrees.filteredCalculate(
                        getSecondStageMeters(),
                        getSecondStageMetersPerSecond(),
                        desiredSecondStageMetersPerSecond);
        double secondStageVoltage90Degree =
                secondStageExtensionContoller90Degrees.filteredCalculate(
                        getSecondStageMeters(),
                        getSecondStageMetersPerSecond(),
                        desiredSecondStageMetersPerSecond);
        ;

        double radiansOverPI =
                getArmRotationRadians()
                        / Math.PI; // Makes it easier to do stuff if we don't worry about PI
        if ((-1 / 8 <= radiansOverPI && radiansOverPI <= 1 / 8)
                || (7 / 8 <= radiansOverPI && radiansOverPI <= 9 / 8)) {
            firstStageExtensionMotor.setVoltage(firstStageVoltage0Degree);
            secondStageExtensionMotor.setVoltage(secondStageVoltage0Degree);

        } else if ((-3 / 8 <= radiansOverPI && radiansOverPI <= -1 / 8)
                || (1 / 8 <= radiansOverPI && radiansOverPI <= 3 / 8)
                || (5 / 8 <= radiansOverPI && radiansOverPI <= 7 / 8)) {

            firstStageExtensionMotor.setVoltage(firstStageVoltage45Degree);
            secondStageExtensionMotor.setVoltage(secondStageVoltage45Degree);
        } else if ((Math.PI * 3 / 8 <= radiansOverPI && radiansOverPI <= 5 / 8)) {
            firstStageExtensionMotor.setVoltage(firstStageVoltage90Degree);
            secondStageExtensionMotor.setVoltage(secondStageVoltage90Degree);
        } else {
            System.out.println("RUH ROH!");
        }
        
        // Output velocity is relative to the arm
        double clawMotorVoltage; 
        if(isClawIndependentlyControlled)
        {
            clawMotorVoltage = clawRotationController.filteredCalculate(getClawAbsoluteAngleRadians(), getClawRelativeRadiansPerSecond(), -desiredArmRadiansPerSecond);
        }
        else
        {
            clawMotorVoltage = clawRotationController.filteredCalculate(getClawAbsoluteAngleRadians(), getClawRelativeRadiansPerSecond(), desiredClawRadiansPerSecond);
        }

        armRotationLeader.setVoltage(clawMotorVoltage);
        SmartDashboard.putNumber("Arm Rotation (deg)", Math.toDegrees(getArmRotationRadians()));
        SmartDashboard.putNumber("First Stage (m)", Math.toDegrees(getFirstStageMeters()));
        SmartDashboard.putNumber("Second Stage (m)", Math.toDegrees(getSecondStageMeters()));
        SmartDashboard.putNumber(
                "Claw-Arm Rotation (deg)", Math.toDegrees(getClawRelativeAngleRadians()));
        SmartDashboard.putNumber(
                "Claw Absolute (deg)", Math.toDegrees(getClawAbsoluteAngleRadians()));
    }

    /**
     * Sets the angular velocity of the rotation motors in radians per second
     *
     * @param angularVelocity - velocity of the rotation motors in radians per second
     */
    public void setArmRotationRadiansPerSecond(double angularVelocity) {
        desiredArmRadiansPerSecond = angularVelocity;
    }

    /**
     * Sets the velocity for the extension motors
     *
     * @param velocity - velocity in meters per second you want to set it to
     */
    public void setFirstStageMetersPerSecond(double velocity) {
        desiredFirstStageMetersPerSecond = velocity;
    }

    public void setSecondStageMetersPerSecond(double velocity) {
        desiredSecondStageMetersPerSecond = velocity;
    }

    public void setExtensionMetersPerSecond(double firstStageVelocity, double secondStageVelocity) {
        setFirstStageMetersPerSecond(firstStageVelocity);
        setSecondStageMetersPerSecond(secondStageVelocity);
    }

    /**
     * Sets the angular velocity of the claw in radians per second RELATIVE TO ARM
     *
     * @param angularVelocity - angular velocity of the claw rotation motors in radians per second
     */
    public void setClawRelativeRadiansPerSecond(double angularVelocity) {
        desiredClawRadiansPerSecond = angularVelocity;
    }
    /**
     * Sets the angular velocity of the claw in radians per second RELATIVE TO GROUND
     *
     * @param angularVelocity - angular velocity of the claw relative to gorund in radians per
     *     second
     */
    public void setClawAbsoluteRadiansPerSecond(double angularVelocity) {
        desiredClawRadiansPerSecond = angularVelocity - getArmRotationRadiansPerSecond();
    }

    public void setClawIndependentlyControlled(boolean isClawIndependentlyControlled) {
        this.isClawIndependentlyControlled = isClawIndependentlyControlled;
    }

    /**
     * Gets the arm angle relative to level to ground
     *
     * @return radians (CCW+, 🔄 positive, when )
     */
    public double getArmRotationRadians() {
        return armRotationLeader.getRotations() * 2 * Math.PI
                + ArmConstants.ARM_ROTATION_STARTING_ANGLE;
    }

    /**
     * Gets the arm angular velocity
     *
     * @return radians per second (CCW+, 🔄 positive, when FACING PORT SIDE)
     */
    public double getArmRotationRadiansPerSecond() {
        return armRotationLeader.getRPM() / 60 * 2 * Math.PI;
    }

    public double getFirstStageMeters() {
        return firstStageExtensionMotor.getRotations()
                * ArmConstants.FIRST_STAGE_DISTANCE_PER_ROTATION;
    }

    public double getSecondStageMeters() {
        return secondStageExtensionMotor.getRotations()
                * ArmConstants.SECOND_STAGE_DISTANCE_PER_ROTATION;
    }

    /**
     * Gets the length of the extension arm from the center of rotation to tip
     *
     * @return the length in meters of the arm from the center of rotation to tip
     */
    public double getFullExtensionMeters() {
        return getFirstStageMeters() + getSecondStageMeters() + ArmConstants.NO_EXTENSION_LENGTH;
    }

    public double getFirstStageMetersPerSecond() {
        return firstStageExtensionMotor.getMetersPerSecond();
    }

    public double getSecondStageMetersPerSecond() {
        return secondStageExtensionMotor.getMetersPerSecond();
    }

    public double getFullExtensionMetersPerSecond() {
        return getFirstStageMetersPerSecond() + getSecondStageMeters();
    }

    /**
     * Gets the claw offset angle relative to parralel with arm
     *
     * @return radians (CCW+, 🔄 positive, when viewed from STARBOARD SIDE)
     */
    public double getClawRelativeAngleRadians() {
        return clawRotationLeader.getRotations() * Math.PI * 2;
    }

    /**
     * Gets the claw angle relative to level to ground
     *
     * @return radians (CCW+, 🔄 positive, when viewed from STARBOARD SIDE)
     */
    public double getClawAbsoluteAngleRadians() {
        return getClawAbsoluteAngleRadians() + getArmRotationRadians();
    }

    /**
     * Gets the rotational velocity of the claw in radians per second RELATIVE TO ARM
     *
     * @return radians per second (CCW+, 🔄 positive, when viewed from STARBOARD SIDE)
     */
    public double getClawRelativeRadiansPerSecond() {
        return clawRotationLeader.getRPM() / 60 * 2 * Math.PI;
    }

    /**
     * Gets the rotational velocity of the claw in radians per second RELATIVE TO GROUND
     *
     * @return radians per second (CCW+, 🔄 positive, when viewed from STARBOARD SIDE)
     */
    public double getClawAbsoluteRadiansPerSecond() {
        return getClawRelativeRadiansPerSecond() + getClawRelativeRadiansPerSecond();
    }

    // ***STUFF FOR TESTING*** //
    // TODO: REMOVE THESE EVENTUALLY
    public void setRotationMotor(double percentage) {
        armRotationLeader.set(percentage);
    }

    public void setFirstStageMotor(double percentage) {
        firstStageExtensionMotor.set(percentage);
    }

    public void setSecondStageMotor(double percentage) {
        secondStageExtensionMotor.set(percentage);
    }

    public void setWristMotor(double percentage) {
        clawRotationLeader.set(percentage);
    }
}
