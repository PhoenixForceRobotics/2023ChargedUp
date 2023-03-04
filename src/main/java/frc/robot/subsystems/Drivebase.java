package frc.robot.subsystems;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.sensors.Pigeon2;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.MecanumDriveKinematics;
import edu.wpi.first.math.kinematics.MecanumDriveOdometry;
import edu.wpi.first.math.kinematics.MecanumDriveWheelPositions;
import edu.wpi.first.math.kinematics.MecanumDriveWheelSpeeds;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DrivebaseConstants;
import frc.robot.utils.Motor;

public class Drivebase extends SubsystemBase {

    public enum CenterOfRotation {
        FL_WHEEL(DrivebaseConstants.WHEEL_FL_LOCATION),
        FR_WHEEL(DrivebaseConstants.WHEEL_FR_LOCATION),
        BL_WHEEL(DrivebaseConstants.WHEEL_BL_LOCATION),
        BR_WHEEL(DrivebaseConstants.WHEEL_BR_LOCATION),
        FRONT_CENTER(DrivebaseConstants.FRONT_CENTER_LOCATION),
        CENTER(new Translation2d());

        private Translation2d location;

        private CenterOfRotation(Translation2d location) {
            this.location = location;
        }

        /**
         * Where the center of rotation is relative to dead center of the robot
         *
         * @return Translation2d of the position
         */
        public Translation2d get() {
            return location;
        }
    }

    private Motor flWheel, frWheel, blWheel, brWheel;

    private PneumaticHub pneumaticHub;
    private DoubleSolenoid butterflyPistons;

    // TODO: Re-add this once we install the pigeon
    private Pigeon2 gyro;

    private MecanumDriveWheelPositions
            currentWheelPositions; // the distance each wheel has travelled
    private MecanumDriveWheelSpeeds
            currentWheelSpeeds; // the velocities of each wheel (not just "speed" :/)
    private MecanumDriveWheelSpeeds
            desiredWheelSpeeds; // the *velocities* being sent to pid controllers
    private ChassisSpeeds currentChassisSpeeds; // the VELOCITIES of the robot relative to the robot
    private ChassisSpeeds
            desiredChassisSpeeds; // The ***VELOCITY*** we want to the set the robot to
    // (WPI needs to work on their language and correct terminology)

    // Everything we use to track the robot's location and behavior
    private MecanumDriveKinematics kinematics;
    private MecanumDriveOdometry odometry;

    private CenterOfRotation centerOfRotation; // Where the mecanum drive will rotate around

    private boolean isMeccanum = true; // whether the drivebase is in meccanum or differential mode

    private Field2d fieldWidget; // Enables visual status of robot on the field

    private final ShuffleboardTab drivebaseTab; // The shuffleboard tab we are using for TELEMETRY
    private final GenericEntry currentXVelocityEntry,
            currentYVelocityEntry,
            currentRotationalVelocityEntry;

    public Drivebase() {
        // creates the components on the drivebase
        flWheel =
                new Motor(
                        DrivebaseConstants.WHEEL_FL_CAN_ID,
                        DrivebaseConstants.WHEEL_FL_REVERSED,
                        DrivebaseConstants.GEAR_RATIO,
                        DrivebaseConstants.WHEEL_DIAMETER,
                        DrivebaseConstants.POSITION_PID,
                        DrivebaseConstants.VELOCITY_PID);
        frWheel =
                new Motor(
                        DrivebaseConstants.WHEEL_FR_CAN_ID,
                        DrivebaseConstants.WHEEL_FR_REVERSED,
                        DrivebaseConstants.GEAR_RATIO,
                        DrivebaseConstants.WHEEL_DIAMETER,
                        DrivebaseConstants.POSITION_PID,
                        DrivebaseConstants.VELOCITY_PID);
        blWheel =
                new Motor(
                        DrivebaseConstants.WHEEL_BL_CAN_ID,
                        DrivebaseConstants.WHEEL_BL_REVERSED,
                        DrivebaseConstants.GEAR_RATIO,
                        DrivebaseConstants.WHEEL_DIAMETER,
                        DrivebaseConstants.POSITION_PID,
                        DrivebaseConstants.VELOCITY_PID);
        brWheel =
                new Motor(
                        DrivebaseConstants.WHEEL_BR_CAN_ID,
                        DrivebaseConstants.WHEEL_BR_REVERSED,
                        DrivebaseConstants.GEAR_RATIO,
                        DrivebaseConstants.WHEEL_DIAMETER,
                        DrivebaseConstants.POSITION_PID,
                        DrivebaseConstants.VELOCITY_PID);

        pneumaticHub = new PneumaticHub();
        pneumaticHub.enableCompressorAnalog(50, 120);
        butterflyPistons =
                pneumaticHub.makeDoubleSolenoid(
                        DrivebaseConstants.BUTTERFLY_FORWARD_PORT,
                        DrivebaseConstants.BUTTERFLY_REVERSE_PORT);

        // inertialMeasurementUnit = new Pigeon2(0);
        gyro = new Pigeon2(20);

        // Sets the current wheel positions
        currentWheelPositions =
                new MecanumDriveWheelPositions(
                        flWheel.getMeters(),
                        frWheel.getMeters(),
                        blWheel.getMeters(),
                        brWheel.getMeters());

        currentWheelSpeeds =
                new MecanumDriveWheelSpeeds(); // the velocities of each wheel (not just "speed" :/)
        desiredWheelSpeeds =
                new MecanumDriveWheelSpeeds(); // the *velocities* being sent to pid controllers
        currentChassisSpeeds =
                new ChassisSpeeds(); // the VELOCITIES of the robot relative to the robot
        desiredChassisSpeeds =
                new ChassisSpeeds(); // The ***VELOCITY*** we want to the set the robot to
        // (WPI needs to work on their language and correct terminology)

        // Creates the kinematics
        kinematics =
                new MecanumDriveKinematics(
                        DrivebaseConstants.WHEEL_FL_LOCATION,
                        DrivebaseConstants.WHEEL_FR_LOCATION,
                        DrivebaseConstants.WHEEL_BL_LOCATION,
                        DrivebaseConstants.WHEEL_BR_LOCATION);

        // Creates the odometry (SET POSE BEFORE AUTO STARTS)
        odometry = new MecanumDriveOdometry(kinematics, getRotation2d(), currentWheelPositions);

        centerOfRotation = CenterOfRotation.CENTER; // used to have custom CoR for holonomic control

        // Allows the robot to be seen on the field
        fieldWidget = new Field2d();
        fieldWidget.setRobotPose(getPose());

        // Non-essential telemetry for troubleshooting
        drivebaseTab = Shuffleboard.getTab("Drivebase");

        currentXVelocityEntry = drivebaseTab.add("Current X Velocity", 0).getEntry();
        currentYVelocityEntry = drivebaseTab.add("Current Y Velocity", 0).getEntry();
        currentRotationalVelocityEntry =
                drivebaseTab.add("Current Rotational Velocity", 0).getEntry();
    }

    @Override
    public void periodic() {
        // Ensure butterfly modules are in the right spot

        if (isMeccanum && getButterflyPistonsValue() != Value.kForward) {
            setButterflyPistons(Value.kForward);
        } else if (!isMeccanum && getButterflyPistonsValue() != Value.kReverse) {
            setButterflyPistons(Value.kReverse);
        }

        // Causes the math to work like standard differential drive
        if (!isMeccanum) {
            desiredChassisSpeeds.vyMetersPerSecond = 0; // Zero's out the y component
            centerOfRotation = CenterOfRotation.CENTER;
        }

        currentWheelPositions =
                new MecanumDriveWheelPositions(
                        flWheel.getMeters(),
                        frWheel.getMeters(),
                        blWheel.getMeters(),
                        brWheel.getMeters());

        currentWheelSpeeds =
                new MecanumDriveWheelSpeeds(
                        flWheel.getMetersPerSecond(),
                        frWheel.getMetersPerSecond(),
                        blWheel.getMetersPerSecond(),
                        brWheel.getMetersPerSecond());

        // convert wheel speeds to chassis speeds (relative to robot)
        currentChassisSpeeds = kinematics.toChassisSpeeds(currentWheelSpeeds);

        // Update then set the (estimated) pose from odometry (not very accurate, reset odometry
        // often, whether with known locations or with APRIL TAGS!!!!!)
        odometry.update(getRotation2d(), currentWheelPositions);

        // Updates the velocities sent to each wheel's PID
        desiredWheelSpeeds = kinematics.toWheelSpeeds(desiredChassisSpeeds, centerOfRotation.get());

        // Scales the values to prevent values from being too high
        desiredWheelSpeeds.desaturate(DrivebaseConstants.MAX_OBTAINABLE_WHEEL_VELOCITY);

        // Calculate voltages for wheels using feedforward and PID
        // Set the output of motors
        flWheel.setMetersPerSecond(desiredWheelSpeeds.frontLeftMetersPerSecond);
        frWheel.setMetersPerSecond(desiredWheelSpeeds.frontRightMetersPerSecond);
        blWheel.setMetersPerSecond(desiredWheelSpeeds.rearLeftMetersPerSecond);
        brWheel.setMetersPerSecond(desiredWheelSpeeds.rearRightMetersPerSecond);

        // Puts the new pose on the main tab
        fieldWidget.setRobotPose(getPose());
        SmartDashboard.putData("Robot Pose", fieldWidget);

        // Publishes the data to the telemetry tab
        currentXVelocityEntry.setDouble(currentChassisSpeeds.vxMetersPerSecond);
        currentYVelocityEntry.setDouble(currentChassisSpeeds.vyMetersPerSecond);
        currentRotationalVelocityEntry.setDouble(
                currentChassisSpeeds.omegaRadiansPerSecond * 180 / Math.PI);
    }

    // ---- SETTING CHASSIS SPEEDS ----//

    /**
     * Sets the speeds RELATIVE TO THE ROBOT with a {@link ChassisSpeeds}
     *
     * @param desiredChassisSpeeds (⬆️ Positive), (⬅️ Positive), (🔄 Positive)
     */
    public void setChassisSpeeds(ChassisSpeeds desiredChassisSpeeds) {
        this.desiredChassisSpeeds = desiredChassisSpeeds;
    }

    /**
     * Sets the speeds RELATIVE TO THE ROBOT with seperate components
     *
     * @param vx velocity forwards (⬆️ Positive)
     * @param vy velocity to the left (horizontal) (⬅️ Positive)
     * @param theta velocity counter-clockwise (🔄 Positive)
     */
    public void setChassisSpeeds(double vx, double vy, double theta) {
        desiredChassisSpeeds = new ChassisSpeeds(vx, vy, theta);
    }

    /**
     * Sets the speeds RELATIVE TO THE FIELD with a {@link ChassisSpeeds}
     *
     * @param desiredChassisSpeeds (⬆️ Positive), (⬅️ Positive), (🔄 Positive)
     */
    public void setFieldRelativeChassisSpeeds(ChassisSpeeds desiredChassisSpeeds) {
        setFieldRelativeChassisSpeeds(
                desiredChassisSpeeds.vxMetersPerSecond,
                desiredChassisSpeeds.vyMetersPerSecond,
                desiredChassisSpeeds.omegaRadiansPerSecond);
    }
    /**
     * Sets the speeds RELATIVE TO THE FIELD with seperate components
     *
     * @param vx velocity toward enemy alliance (⬆️ Positive)
     * @param vy velocity to the left (horizontal) (⬅️ Positive)
     * @param theta velocity counter-clockwise (🔄 Positive)
     */
    public void setFieldRelativeChassisSpeeds(double vx, double vy, double theta) {
        desiredChassisSpeeds =
                ChassisSpeeds.fromFieldRelativeSpeeds(vx, vy, theta, getRotation2d());
    }

    /** Stops the robot */
    public void stop() {
        desiredChassisSpeeds = new ChassisSpeeds();
    }

    // ---- BUTTERFLY MODULE FUNCTION ----//

    /**
     * @return {@link Value} whether the piston is forward or reverse
     */
    public Value getButterflyPistonsValue() {
        return butterflyPistons.get();
    }

    /**
     * Set the whether the butterfly pistons should be out or not
     *
     * @param value to go forward or reverse
     */
    public void setButterflyPistons(Value value) {
        butterflyPistons.set(value);
    }

    public boolean isMeccanum() {
        return isMeccanum;
    }

    public void setMeccanum(boolean isMeccanum) {
        this.isMeccanum = isMeccanum;
    }

    // ---- KINEMATIC STUFF -----//

    /**
     * Sets where the robot will rotate
     *
     * @return the enum value that contains position
     */
    public CenterOfRotation getCenterOfRotation() {
        return centerOfRotation;
    }

    /** Set custom rotation for holonimic control (extra control, v funky) */
    public void setCenterOfRotation(CenterOfRotation centerOfRotation) {
        this.centerOfRotation = centerOfRotation;
    }

    /**
     * Returns the position of the robot on the field
     *
     * @return The pose2d of the robot (in meters)
     */
    public Pose2d getPose() {
        return odometry.getPoseMeters();
    }

    /**
     * Sets the position of the robot to a particular position and rotation relative to the field
     *
     * @param poseMeters The position on the field that your robot is at.
     */
    public void resetPose(Pose2d poseMeters) {
        odometry.resetPosition(getRotation2d(), currentWheelPositions, poseMeters);
    }

    /**
     * Get the angle the robot is facing (Counter-Clockwise POSITIVE)
     *
     * @return current heading (CCW+)
     */
    public double getHeading() {
        return gyro.getYaw();
    }

    /**
     * Get the angle the robot is facing (Counter-Clockwise POSITIVE)
     *
     * @return current heading in the form of a {@link Rotation2d}
     */
    public Rotation2d getRotation2d() {
        return new Rotation2d(Math.toRadians(gyro.getYaw()));
    }

    /**
     * Get the angle of elevation (pitch) of the robot
     *
     * @return pitch (in degrees) (Positive for upwards, Negative for downwards)
     */
    public double getPitch() {
        return 0;
        // return inertialMeasurementUnit.getPitch();
    }

    // We shouldn't be changing the yaw on the IMU, rather changing it within the pose estimation :D

    /**
     * Resets the yaw to the desired value
     *
     * @param yawDegrees
     * @return {@link ErrorCode} for setting the yaw
     */
    // public ErrorCode setYaw(double yawDegrees) {
    //     // return inertialMeasurementUnit.setYaw(yawDegrees);
    // }

    /**
     * Resets the yaw to ZERO (0)
     *
     * @return {@link ErrorCode} for setting the yaw
     */
    // public ErrorCode resetYaw() {
    //     return setYaw(0);
    // }

    // ---- SHUFFLEBOARD STUFF ----//

    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);
    }
}
