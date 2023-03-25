package frc.robot.commands.drivebase.autonomous;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.constants.Constants.DrivebaseConstants;
import frc.robot.subsystems.Drivebase;
import frc.robot.utils.pid.PFRPIDController;

public class BalanceOnChargeStation extends CommandBase {
    private Drivebase drivebase;
    private PFRPIDController pidController;

    public BalanceOnChargeStation(Drivebase drivebase) {
        this.drivebase = drivebase;
        this.pidController = new PFRPIDController(DrivebaseConstants.BALANCE_PID);
        this.pidController.setSetpoint(0);
        this.pidController.setTolerance(
                DrivebaseConstants.BALANCE_PID_MAX_ANGLE,
                DrivebaseConstants.BALANCE_PID_MAX_ANGULAR_VELOCITY);
    }

    @Override
    public void initialize() {
        // Assume we are already somwhere on the charge station, and perpendicular to the panel (so
        // we don't need to change angle)
        // Also assuming that we move
        this.pidController.reset();
    }

    @Override
    public void execute() {
        double xVelocity =
                -this.pidController.calculate(
                        this.drivebase.getPitch()); // must invert due to nature of measurement
        this.drivebase.setChassisSpeeds(xVelocity, 0, 0);
    }

    @Override
    public boolean isFinished() {
        return this.pidController.atSetpoint();
    }

    @Override
    public void end(boolean interrupted) {
        this.drivebase.stop();
    }
}
