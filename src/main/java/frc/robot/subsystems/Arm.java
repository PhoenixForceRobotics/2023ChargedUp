package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants.ArmConstants;
import frc.robot.utils.arm.PFRArmPIDController;
import frc.robot.utils.arm.PFRExtensionPIDController;
import frc.robot.utils.motors.Motor;

public class Arm extends SubsystemBase {
    // Rotation motors + controller
    private Motor armRotationLeader;
    private Motor armRotationFollower;

    // Extension motors + controller
    private Motor firstStageExtensionMotor; // Fullsized Neo (bottom side)
    private Motor secondStageExtensionMotor; // Neo 550 (top side)
    private PFRExtensionPIDController
            firstStageExtensionController; // Extension once it's all the way down
    private PFRExtensionPIDController
            secondStageExtensionController; // Extension once it's all the way down

    // Motors + controller for rotating claw independently to keep it level when arm has rotated
    private Motor clawRotationLeader;
    private Motor clawRotationFollower; // TODO: Re'add this once we get the motor installed
    private PFRArmPIDController clawRotationController;

    // Changing these variables changes how the arm moves
    // Change these variables through the "setters"
    private double desiredFirstStageMetersPerSecond = 0;
    private double desiredSecondStageMetersPerSecond = 0;

    private double desiredClawRadiansPerSecond = 0; // relative to the arm!

    /**
     * The arm moves the intake It can rotate -180 to 180 degrees and can extend a certain distance.
     */
    public Arm() {

        // Define rotational motors
        this.armRotationLeader =
                new Motor(
                        ArmConstants.ARM_ROTATION_MOTOR_1_PORT,
                        ArmConstants.ARM_ROTATION_MOTOR_1_REVERSED,
                        ArmConstants.ARM_ROTATION_MOTOR_GEAR_RATIO,
                        ArmConstants.ROTATION_MOTOR_WHEEL_DIAMETER);
        this.armRotationFollower =
                new Motor(
                        ArmConstants.ARM_ROTATION_MOTOR_2_PORT,
                        ArmConstants.ARM_ROTATION_MOTOR_2_REVERSED,
                        ArmConstants.ARM_ROTATION_MOTOR_GEAR_RATIO,
                        ArmConstants.ROTATION_MOTOR_WHEEL_DIAMETER);

        // set follower motor to follow leader
        this.armRotationFollower.follow(this.armRotationLeader);

        // Define extension motors
        this.firstStageExtensionMotor =
                new Motor(
                        ArmConstants.FIRST_STAGE_PORT,
                        ArmConstants.FIRST_STAGE_REVERSED,
                        ArmConstants.FIRST_STAGE_GEAR_RATIO,
                        ArmConstants.FIRST_STAGE_DISTANCE_PER_ROTATION / Math.PI);
        this.secondStageExtensionMotor =
                new Motor(
                        ArmConstants.SECOND_STAGE_PORT,
                        ArmConstants.SECOND_STAGE_REVERSED,
                        ArmConstants.SECOND_STAGE_GEAR_RATIO,
                        ArmConstants.SECOND_STAGE_DISTANCE_PER_ROTATION / Math.PI);

        this.firstStageExtensionController =
                new PFRExtensionPIDController(ArmConstants.FIRST_STAGE_PID_VALUES);
        this.secondStageExtensionController =
                new PFRExtensionPIDController(ArmConstants.SECOND_STAGE_PID_VALUES);

        // Define claw rotational motors
        this.clawRotationLeader =
                new Motor(
                        ArmConstants.CLAW_ROTATION_MOTOR_1_PORT,
                        ArmConstants.CLAW_ROTATION_MOTOR_1_REVERSED,
                        ArmConstants.CLAW_ROTATION_MOTOR_GEAR_RATIO,
                        ArmConstants.CLAW_ROTATION_MOTOR_WHEEL_DIAMETER);
        this.clawRotationFollower =
                new Motor(
                        ArmConstants.CLAW_ROTATION_MOTOR_2_PORT,
                        ArmConstants.CLAW_ROTATION_MOTOR_2_REVERSED,
                        ArmConstants.CLAW_ROTATION_MOTOR_GEAR_RATIO,
                        ArmConstants.CLAW_ROTATION_MOTOR_WHEEL_DIAMETER);

        // make follwer follow the leader, BUT INVERTED (due to how it works)
        this.clawRotationFollower.follow(this.clawRotationLeader, true);

        // Claw rotation controller
        this.clawRotationController =
                new PFRArmPIDController(ArmConstants.CLAW_ROTATION_PID_VALUES);
    }

    @Override
    public void periodic() {
        if (ArmConstants.FIRST_STAGE_MIN_EXTENSION >= this.getFirstStageMeters()) {
            this.desiredFirstStageMetersPerSecond =
                    Math.max(0, this.desiredFirstStageMetersPerSecond);
        } else if (ArmConstants.FIRST_STAGE_MAX_EXTENSION <= this.getFirstStageMeters()) {
            this.desiredFirstStageMetersPerSecond =
                    Math.min(this.desiredFirstStageMetersPerSecond, 0);
        }
        if (ArmConstants.SECOND_STAGE_MIN_EXTENSION >= this.getSecondStageMeters()) {
            this.desiredSecondStageMetersPerSecond =
                    Math.max(0, this.desiredSecondStageMetersPerSecond);
        } else if (ArmConstants.SECOND_STAGE_MAX_EXTENSION <= this.getSecondStageMeters()) {
            this.desiredSecondStageMetersPerSecond =
                    Math.min(this.desiredSecondStageMetersPerSecond, 0);
        }
        if (ArmConstants.MIN_CLAW_ANGLE >= this.getClawRelativeAngleRadians()) {
            this.desiredClawRadiansPerSecond = Math.max(0, this.desiredClawRadiansPerSecond);
        } else if (ArmConstants.MAX_CLAW_ANGLE <= this.getClawRelativeAngleRadians()) {
            this.desiredClawRadiansPerSecond = Math.min(this.desiredClawRadiansPerSecond, 0);
        }

        double firstStageVoltage =
                this.firstStageExtensionController.filteredCalculate(
                        this.getFirstStageMetersPerSecond(), this.desiredFirstStageMetersPerSecond);
        double secondStageVoltage =
                this.secondStageExtensionController.filteredCalculate(
                        this.getSecondStageMetersPerSecond(),
                        this.desiredSecondStageMetersPerSecond);

        this.firstStageExtensionMotor.setVoltage(firstStageVoltage);
        this.secondStageExtensionMotor.setVoltage(secondStageVoltage);

        double clawMotorVoltage =
                this.clawRotationController.filteredCalculate(
                        this.getClawAbsoluteAngleRadians(),
                        this.getClawRelativeRadiansPerSecond(),
                        this.desiredClawRadiansPerSecond);
        this.clawRotationLeader.setVoltage(clawMotorVoltage);

        SmartDashboard.putNumber("Desired Claw Velocity (rad/s)", this.desiredClawRadiansPerSecond);
        SmartDashboard.putNumber(
                "Arm Rotation (deg)", Math.toDegrees(this.getArmRotationRadians()));
        SmartDashboard.putNumber("First Stage (m)", this.getFirstStageMeters());
        SmartDashboard.putNumber("Second Stage (m)", this.getSecondStageMeters());
        SmartDashboard.putNumber(
                "Claw-Arm Rotation (deg)", Math.toDegrees(this.getClawRelativeAngleRadians()));
        SmartDashboard.putNumber(
                "Claw Absolute (deg)", Math.toDegrees(this.getClawAbsoluteAngleRadians()));
        SmartDashboard.putNumber("Claw Voltage", clawMotorVoltage);
        SmartDashboard.putNumber("Velocity", this.getClawRelativeRadiansPerSecond());
    }

    public void resetArmEncoder() {
        this.armRotationLeader.getEncoder().setPosition(0);
    }
    /**
     * Sets the velocity for the extension motors
     *
     * @param velocity - velocity in meters per second you want to set it to
     */
    public void setFirstStageMetersPerSecond(double velocity) {
        this.desiredFirstStageMetersPerSecond = velocity;
    }

    public void setSecondStageMetersPerSecond(double velocity) {
        this.desiredSecondStageMetersPerSecond = velocity;
    }

    public void setExtensionMetersPerSecond(double firstStageVelocity, double secondStageVelocity) {
        this.setFirstStageMetersPerSecond(firstStageVelocity);
        this.setSecondStageMetersPerSecond(secondStageVelocity);
    }

    /**
     * Sets the angular velocity of the claw in radians per second RELATIVE TO ARM
     *
     * @param angularVelocity - angular velocity of the claw rotation motors in radians per second
     */
    public void setClawRelativeRadiansPerSecond(double angularVelocity) {
        this.desiredClawRadiansPerSecond = angularVelocity;
    }
    /**
     * Sets the angular velocity of the claw in radians per second RELATIVE TO GROUND
     *
     * @param angularVelocity - angular velocity of the claw relative to gorund in radians per
     *     second
     */
    public void setClawAbsoluteRadiansPerSecond(double angularVelocity) {
        this.desiredClawRadiansPerSecond = angularVelocity - this.getArmRotationRadiansPerSecond();
    }

    /**
     * Gets the arm angle relative to level to ground
     *
     * @return radians (CCW+, 🔄 positive, when FACING PORT SIDE)
     */
    public double getArmRotationRadians() {
        return this.armRotationLeader.getRotations() * 2 * Math.PI
                + ArmConstants.ARM_ROTATION_STARTING_ANGLE;
    }

    /**
     * Gets the arm angular velocity
     *
     * @return radians per second (CCW+, 🔄 positive, when FACING PORT SIDE)
     */
    public double getArmRotationRadiansPerSecond() {
        return this.armRotationLeader.getRPM() / 60 * 2 * Math.PI;
    }

    public double getFirstStageMeters() {
        return this.firstStageExtensionMotor.getMeters();
    }

    public double getSecondStageMeters() {
        return this.secondStageExtensionMotor.getMeters();
    }

    /**
     * Gets the length of the extension arm from the center of rotation to tip
     *
     * @return the length in meters of the arm from the center of rotation to tip
     */
    public double getFullExtensionMeters() {
        return this.getFirstStageMeters()
                + this.getSecondStageMeters()
                + ArmConstants.NO_EXTENSION_LENGTH;
    }

    public double getFirstStageMetersPerSecond() {
        return this.firstStageExtensionMotor.getMetersPerSecond();
    }

    public double getSecondStageMetersPerSecond() {
        return this.secondStageExtensionMotor.getMetersPerSecond();
    }

    public double getFullExtensionMetersPerSecond() {
        return this.getFirstStageMetersPerSecond() + this.getSecondStageMeters();
    }

    public void resetFirstStageEncoder(double meters) {
        this.firstStageExtensionMotor
                .getEncoder()
                .setPosition(
                        meters
                                * ArmConstants.FIRST_STAGE_GEAR_RATIO
                                * ArmConstants.FIRST_STAGE_DISTANCE_PER_ROTATION);
    }

    public void resetSecondStageEncoder(double meters) {
        this.secondStageExtensionMotor
                .getEncoder()
                .setPosition(
                        meters
                                * ArmConstants.FIRST_STAGE_GEAR_RATIO
                                * ArmConstants.FIRST_STAGE_DISTANCE_PER_ROTATION);
    }

    /**
     * Gets the claw offset angle relative to parralel with arm
     *
     * @return radians (CCW+, 🔄 positive, when viewed from STARBOARD SIDE)
     */
    public double getClawRelativeAngleRadians() {
        return this.clawRotationLeader.getRotations() * Math.PI * 2
                + ArmConstants.CLAW_STARTING_ANGLE;
    }

    /**
     * Gets the claw angle relative to level to ground
     *
     * @return radians (CCW+, 🔄 positive, when viewed from STARBOARD SIDE)
     */
    public double getClawAbsoluteAngleRadians() {
        return this.getClawRelativeAngleRadians()
                + this.getArmRotationRadians(); // ARM IS NOT ALWAYS AT 42 DEGREES NOW
    }

    /**
     * Gets the rotational velocity of the claw in radians per second RELATIVE TO ARM
     *
     * @return radians per second (CCW+, 🔄 positive, when viewed from STARBOARD SIDE)
     */
    public double getClawRelativeRadiansPerSecond() {
        return this.clawRotationLeader.getRPM() / 60 * 2 * Math.PI;
    }

    /**
     * Gets the rotational velocity of the claw in radians per second RELATIVE TO GROUND
     *
     * @return radians per second (CCW+, 🔄 positive, when viewed from STARBOARD SIDE)
     */
    public double getClawAbsoluteRadiansPerSecond() {
        return this.getClawRelativeRadiansPerSecond() + this.getClawRelativeRadiansPerSecond();
    }

    // ***STUFF FOR TESTING*** //
    public void setRotationMotor(double percentage) {
        this.armRotationLeader.set(percentage);
    }
}
