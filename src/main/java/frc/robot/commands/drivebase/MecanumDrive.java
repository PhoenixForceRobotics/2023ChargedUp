package frc.robot.commands.drivebase;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DrivebaseConstants;
import frc.robot.subsystems.Drivebase;
import frc.robot.utils.PFRController;

public class MecanumDrive extends CommandBase {

    public enum FrameOfReference {
        ROBOT,
        FIELD;
    }

    private final Drivebase drivebase;
    private final PFRController driverController;

    // Caps the acceleration of the robot
    private final SlewRateLimiter vxLimiter, vyLimiter;

    // Indicates whether to go robot or field relative
    private FrameOfReference frameOfReference;

    public MecanumDrive(Drivebase drivebase, PFRController driverController) {
        this.drivebase = drivebase;
        this.driverController = driverController;
        frameOfReference = FrameOfReference.ROBOT;
        addRequirements(drivebase);

        vxLimiter = new SlewRateLimiter(DrivebaseConstants.MAX_LINEAR_ACCELERATION);
        vyLimiter = new SlewRateLimiter(DrivebaseConstants.MAX_LINEAR_ACCELERATION);
    }

    @Override
    public void initialize() {
        drivebase.setMeccanum(true);
        drivebase.setButterflyPistons(Value.kReverse);
        vxLimiter.reset(0);
        vyLimiter.reset(0);
    }

    @Override
    public void execute() {

        if (driverController.getLeftBumperPressed()) {
            // allows us to toggle frame of reference when button pressed
            if (frameOfReference == FrameOfReference.ROBOT) {
                frameOfReference = FrameOfReference.FIELD;
            } else {
                frameOfReference = FrameOfReference.ROBOT;
            }
        }

        double xVelocity =
                vxLimiter.calculate(
                        -driverController.getLeftYSquared()
                                * DrivebaseConstants.MAX_LINEAR_VELOCITY);
        double yVelocity =
                vyLimiter.calculate(
                        -driverController.getLeftXSquared()
                                * DrivebaseConstants.MAX_LINEAR_VELOCITY);
        double angularVelocity =
                -driverController.getRightXSquared() * DrivebaseConstants.MAX_ANGULAR_VELOCITY;

        // Adds deadzones to velocities(to prevent unwanted drifting)
        xVelocity =
                MathUtil.clamp(
                        xVelocity,
                        DrivebaseConstants.MIN_LINEAR_VELOCITY,
                        DrivebaseConstants.MAX_LINEAR_VELOCITY);
        yVelocity =
                MathUtil.clamp(
                        yVelocity,
                        DrivebaseConstants.MIN_LINEAR_VELOCITY,
                        DrivebaseConstants.MAX_LINEAR_VELOCITY);
        angularVelocity =
                MathUtil.clamp(
                        angularVelocity,
                        DrivebaseConstants.MIN_ANGULAR_VELOCITY,
                        DrivebaseConstants.MAX_ANGULAR_VELOCITY);

        // sets ChassisSpeeds according to indicated frame of reference
        if (frameOfReference == FrameOfReference.ROBOT) {
            drivebase.setChassisSpeeds(xVelocity, yVelocity, angularVelocity);
        } else { // frame of reference must be field-relative
            drivebase.setFieldRelativeChassisSpeeds(xVelocity, yVelocity, angularVelocity);
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        drivebase.stop();
    }
}
