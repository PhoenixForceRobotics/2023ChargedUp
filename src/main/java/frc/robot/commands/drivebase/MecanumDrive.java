package frc.robot.commands.drivebase;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.constants.Constants.DrivebaseConstants;
import frc.robot.subsystems.Drivebase;
import frc.robot.utils.PFRController;

public class MecanumDrive extends CommandBase {

    public enum FrameOfReference {
        ROBOT,
        FIELD,
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
        this.frameOfReference = FrameOfReference.ROBOT;
        this.addRequirements(drivebase);

        this.vxLimiter = new SlewRateLimiter(DrivebaseConstants.MAX_LINEAR_ACCELERATION);
        this.vyLimiter = new SlewRateLimiter(DrivebaseConstants.MAX_LINEAR_ACCELERATION);
    }

    @Override
    public void initialize() {
        this.drivebase.setMeccanum(true);
        this.drivebase.setButterflyPistons(Value.kReverse);
        this.vxLimiter.reset(0);
        this.vyLimiter.reset(0);
    }

    @Override
    public void execute() {

        if (this.driverController.getAButton()) {
            // allows us to toggle frame of reference when button pressed
            if (this.frameOfReference == FrameOfReference.ROBOT) {
                this.frameOfReference = FrameOfReference.FIELD;
            } else {
                this.frameOfReference = FrameOfReference.ROBOT;
            }
        }

        double xVelocity =
                this.vxLimiter.calculate(
                        -this.driverController.getLeftYSquared()
                                * DrivebaseConstants.MAX_LINEAR_VELOCITY);
        double yVelocity =
                this.vyLimiter.calculate(
                        -this.driverController.getLeftXSquared()
                                * DrivebaseConstants.MAX_LINEAR_VELOCITY);
        double angularVelocity =
                -this.driverController.getRightXSquared() * DrivebaseConstants.MAX_ANGULAR_VELOCITY;

        // Adds deadzones to velocities(to prevent unwanted drifting)
        xVelocity =
                MathUtil.applyDeadband(
                        xVelocity,
                        DrivebaseConstants.MIN_LINEAR_VELOCITY,
                        DrivebaseConstants.MAX_LINEAR_VELOCITY);
        yVelocity =
                MathUtil.applyDeadband(
                        yVelocity,
                        DrivebaseConstants.MIN_LINEAR_VELOCITY,
                        DrivebaseConstants.MAX_LINEAR_VELOCITY);
        angularVelocity =
                MathUtil.applyDeadband(
                        angularVelocity,
                        DrivebaseConstants.MIN_ANGULAR_VELOCITY,
                        DrivebaseConstants.MAX_ANGULAR_VELOCITY);

        // sets ChassisSpeeds according to indicated frame of reference
        if (this.frameOfReference == FrameOfReference.ROBOT) {
            this.drivebase.setChassisSpeeds(xVelocity, yVelocity, angularVelocity);
        } else { // frame of reference must be field-relative
            this.drivebase.setFieldRelativeChassisSpeeds(xVelocity, yVelocity, angularVelocity);
        }
        SmartDashboard.putBoolean(
                "Field Oriented", this.frameOfReference == FrameOfReference.FIELD);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        this.drivebase.stop();
    }
}
