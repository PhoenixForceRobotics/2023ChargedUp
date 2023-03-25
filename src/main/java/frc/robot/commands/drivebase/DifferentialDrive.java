package frc.robot.commands.drivebase;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.constants.Constants.DrivebaseConstants;
import frc.robot.subsystems.Drivebase;
import frc.robot.utils.PFRController;

public class DifferentialDrive extends CommandBase {
    private final Drivebase drivebase;
    private final PFRController driverController;

    // Caps the acceleration of the robot
    private final SlewRateLimiter vxLimiter;

    public DifferentialDrive(Drivebase drivebase, PFRController driverController) {
        this.drivebase = drivebase;
        this.driverController = driverController;
        this.addRequirements(drivebase);
        this.vxLimiter = new SlewRateLimiter(DrivebaseConstants.MAX_LINEAR_ACCELERATION);
    }

    @Override
    public void initialize() {
        this.drivebase.setMeccanum(false);
        this.drivebase.setButterflyPistons(Value.kReverse);
        this.vxLimiter.reset(0);
    }

    @Override
    public void execute() {
        double xVelocity =
                this.vxLimiter.calculate(
                        -this.driverController.getLeftYSquared()
                                * DrivebaseConstants.MAX_LINEAR_VELOCITY);
        double yVelocity = 0; // acts as DifferentialDrive
        double angularVelocity =
                -this.driverController.getRightXSquared() * DrivebaseConstants.MAX_ANGULAR_VELOCITY;

        // Adds deadzones to velocities(to prevent unwanted drifting)
        xVelocity =
                MathUtil.applyDeadband(
                        xVelocity,
                        DrivebaseConstants.MIN_LINEAR_VELOCITY,
                        DrivebaseConstants.MAX_LINEAR_VELOCITY);
        angularVelocity =
                MathUtil.applyDeadband(
                        angularVelocity,
                        DrivebaseConstants.MIN_ANGULAR_VELOCITY,
                        DrivebaseConstants.MAX_ANGULAR_VELOCITY);
        this.drivebase.setChassisSpeeds(xVelocity, yVelocity, angularVelocity);
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
