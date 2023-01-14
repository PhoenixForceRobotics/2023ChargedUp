package frc.robot.commands.drivebase;

import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivebase;
import frc.robot.utils.PFRController;

public class DifferentialDrive extends CommandBase 
{
    private final Drivebase drivebase;
    private final PFRController driverController;

    public DifferentialDrive(Drivebase drivebase, PFRController driverController)
    {
        this.drivebase = drivebase;
        this.driverController = driverController;
        addRequirements(drivebase);
    }

    @Override
    public void initialize() {
        drivebase.setMeccanum(false);
        drivebase.setButterflyModules(Value.kReverse);
    }
    

    @Override
    public void execute() {
        
        
        
        double xVelocity = driverController.getLeftYSquared();
        double yVelocity = 0; 
        double angularVelocity = driverController.getRightXSquared();

        drivebase.setChassisSpeeds(
            xVelocity,
            yVelocity,
            angularVelocity
        );
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