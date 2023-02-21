package frc.robot.commands.drivebase;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DrivebaseConstants;
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
        addRequirements(drivebase);
        vxLimiter = new SlewRateLimiter(DrivebaseConstants.MAX_LINEAR_ACCELERATION);
    }

    @Override
    public void initialize() {
        drivebase.setMeccanum(false);
        drivebase.setButterflyPistons(Value.kReverse);
        vxLimiter.reset(0);
    }

    @Override
    public void execute() {

        double xVelocity =
                vxLimiter.calculate(
                        driverController.getLeftYSquared()
                                * DrivebaseConstants.MAX_LINEAR_VELOCITY);
        double yVelocity = 0;
        double angularVelocity =
                driverController.getRightXSquared() * DrivebaseConstants.MAX_ANGULAR_VELOCITY;

        // zeros the any velocity if under minimum velocity (to prevent drifting)
        xVelocity = xVelocity < DrivebaseConstants.MIN_LINEAR_VELOCITY ? 0 : xVelocity;
        angularVelocity =
                angularVelocity < DrivebaseConstants.MIN_LINEAR_VELOCITY ? 0 : angularVelocity;

        drivebase.setChassisSpeeds(xVelocity, yVelocity, angularVelocity);
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
