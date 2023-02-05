package frc.robot.subsystems;

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
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
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

    private DoubleSolenoid butterflyPistons;

    private Pigeon2 inertialMeasurementUnit;
    // private ADXRS450_Gyro gyro;

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

    private MecanumDriveKinematics
            kinematics; // Everything we use to track the robot's location and behavior
    private MecanumDriveOdometry odometry;

    private CenterOfRotation centerOfRotation; // Where the mecanum drive will rotate around

    private boolean isMeccanum = true; // whether the drivebase is in meccanum or differential mode

    private final ShuffleboardTab drivebaseTab; // The shuffleboard tab we are using for TELEMETRY
    private final GenericEntry currentXVelocityEntry,
            currentYVelocityEntry,
            currentRotationalVelocityEntry; // entries for telemetry

    public Drivebase() {
        // creates the components on the drivebase
        flWheel =
                new Motor(
                        DrivebaseConstants.WHEEL_FL_PORT,
                        DrivebaseConstants.WHEEL_FL_REVERSED,
                        DrivebaseConstants.GEAR_RATIO,
                        DrivebaseConstants.WHEEL_DIAMETER,
                        DrivebaseConstants.POSITION_PID,
                        DrivebaseConstants.VELOCITY_PID);
        frWheel =
                new Motor(
                        DrivebaseConstants.WHEEL_FR_PORT,
                        DrivebaseConstants.WHEEL_FR_REVERSED,
                        DrivebaseConstants.GEAR_RATIO,
                        DrivebaseConstants.WHEEL_DIAMETER,
                        DrivebaseConstants.POSITION_PID,
                        DrivebaseConstants.VELOCITY_PID);
        blWheel =
                new Motor(
                        DrivebaseConstants.WHEEL_BL_PORT,
                        DrivebaseConstants.WHEEL_BL_REVERSED,
                        DrivebaseConstants.GEAR_RATIO,
                        DrivebaseConstants.WHEEL_DIAMETER,
                        DrivebaseConstants.POSITION_PID,
                        DrivebaseConstants.VELOCITY_PID);
        brWheel =
                new Motor(
                        DrivebaseConstants.WHEEL_BR_PORT,
                        DrivebaseConstants.WHEEL_BR_REVERSED,
                        DrivebaseConstants.GEAR_RATIO,
                        DrivebaseConstants.WHEEL_DIAMETER,
                        DrivebaseConstants.POSITION_PID,
                        DrivebaseConstants.VELOCITY_PID);

        butterflyPistons =
                new DoubleSolenoid(
                        PneumaticsModuleType.REVPH,
                        DrivebaseConstants.BUTTERFLY_FORWARD_PORT,
                        DrivebaseConstants.BUTTERFLY_REVERSE_PORT);

        inertialMeasurementUnit = new Pigeon2(20);
        // gyro = new ADXRS450_Gyro();

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
        odometry =
                new MecanumDriveOdometry(
                        kinematics, getRotation2d(), currentWheelPositions, new Pose2d());

        centerOfRotation = CenterOfRotation.CENTER; // used to have custom CoR for holonomic control

        drivebaseTab = Shuffleboard.getTab("Drivebase");

        currentXVelocityEntry = drivebaseTab.add("Current X Velocity", 0).getEntry();
        currentYVelocityEntry = drivebaseTab.add("Current Y Velocity", 0).getEntry();
        currentRotationalVelocityEntry =
                drivebaseTab.add("Current Rotational Velocity", 0).getEntry();
    }

    @Override
    public void periodic() {
        // Ensure butterfly modules are in the right spot

        if (isMeccanum && butterflyPistons.get() != Value.kForward) {
            butterflyPistons.set(Value.kForward);
        } else if (!isMeccanum && butterflyPistons.get() != Value.kReverse) {
            butterflyPistons.set(Value.kReverse);
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
        // often)
        odometry.update(getRotation2d(), currentWheelPositions);

        // Updates the velocities sent to each wheel's PID
        desiredWheelSpeeds = kinematics.toWheelSpeeds(desiredChassisSpeeds, centerOfRotation.get());

        // Scales the values to prevent values from being too high (100%, velocity -> output of
        // motor)
        desiredWheelSpeeds.desaturate(DrivebaseConstants.MAX_OBTAINABLE_WHEEL_VELOCITY);

        // Calculate voltages for wheels using feedforward

        // Set the output of motors
        flWheel.setMetersPerSecond(desiredWheelSpeeds.frontLeftMetersPerSecond);
        frWheel.setMetersPerSecond(desiredWheelSpeeds.frontRightMetersPerSecond);
        blWheel.setMetersPerSecond(desiredWheelSpeeds.rearLeftMetersPerSecond);
        brWheel.setMetersPerSecond(desiredWheelSpeeds.rearRightMetersPerSecond);

        // Publishes the data to the Shuffleboard Tab
        currentXVelocityEntry.setDouble(currentChassisSpeeds.vxMetersPerSecond);
        currentYVelocityEntry.setDouble(currentChassisSpeeds.vyMetersPerSecond);
        currentRotationalVelocityEntry.setDouble(
                currentChassisSpeeds.omegaRadiansPerSecond * 180 / Math.PI);
    }

    public void setChassisSpeeds(ChassisSpeeds desiredChassisSpeeds) {
        this.desiredChassisSpeeds = desiredChassisSpeeds;
    }

    public void setChassisSpeeds(double vx, double vy, double theta) {
        desiredChassisSpeeds = new ChassisSpeeds(vx, vy, theta);
    }

    public void setFieldRelativeChassisSpeeds(ChassisSpeeds desiredChassisSpeeds) {
        setFieldRelativeChassisSpeeds(
                desiredChassisSpeeds.vxMetersPerSecond,
                desiredChassisSpeeds.vyMetersPerSecond,
                desiredChassisSpeeds.omegaRadiansPerSecond);
    }

    public void setFieldRelativeChassisSpeeds(double vx, double vy, double theta) {
        desiredChassisSpeeds =
                ChassisSpeeds.fromFieldRelativeSpeeds(vx, vy, theta, getRotation2d());
    }

    public void setCenterOfRotation(CenterOfRotation centerOfRotation) {
        this.centerOfRotation = centerOfRotation;
    }

    public void setButterflyPistons(Value value) {
        butterflyPistons.set(value);
    }

    public void toggleButterflyModules() {
        butterflyPistons.toggle();
    }

    public void setMeccanum(boolean isMeccanum) {
        this.isMeccanum = isMeccanum;
    }

    /**
     * Sets the "proportational" variable for the PID in each motor
     *
     * @param kp what to set each motor's "proportional" parameter to
     */
    public void setP(double kp) {
        flWheel.setVelocityP(kp);
        frWheel.setVelocityP(kp);
        blWheel.setVelocityP(kp);
        brWheel.setVelocityP(kp);
    }

    /**
     * Sets the "derivative" variable (slope at one point) for the PID in each motor
     *
     * @param kd what to set each motor's "derivative" parameter to
     */
    public void setD(double kd) {
        flWheel.setVelocityD(kd);
        frWheel.setVelocityD(kd);
        blWheel.setVelocityD(kd);
        brWheel.setVelocityD(kd);
    }

    /**
     * Sets the position of the robot to a particular position and rotation relative to the field
     *
     * @param poseMeters The position on the field that your robot is at.
     */
    public void resetPosition(Pose2d poseMeters) {
        odometry.resetPosition(getRotation2d(), currentWheelPositions, poseMeters);
    }

    /** Stops the robot */
    public void stop() {
        desiredChassisSpeeds = new ChassisSpeeds();
    }

    /**
     * Sets where the robot will rotate
     *
     * @return the enum value that contains position
     */
    public CenterOfRotation getCenterOfRotation() {
        return centerOfRotation;
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
     * @return difference in angle since last reset
     */
    public double getHeading() {
        // return inertialMeasurementUnit.getYaw();
        return inertialMeasurementUnit.getYaw();
    }

    public Rotation2d getRotation2d() {
        return Rotation2d.fromDegrees(getHeading());
    }

    public Motor getFlWheel() {
        return flWheel;
    }

    public Motor getFrWheel() {
        return frWheel;
    }

    public Motor getBlWheel() {
        return blWheel;
    }

    public Motor getBrWheel() {
        return brWheel;
    }

    public MecanumDriveKinematics getKinematics() {
        return kinematics;
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);
    }
}
