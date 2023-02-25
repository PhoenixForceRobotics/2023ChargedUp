package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;
import frc.robot.utils.PFRController;

public class ClawTesting extends CommandBase {
    private Arm arm;
    private PFRController operatorController;

    public ClawTesting(Arm arm, PFRController operatorController)
    {
        this.arm = arm;
        this.operatorController = operatorController;
    }

    @Override
    public void initialize()
    { 
        arm.setClawRotationRadiansPerSecond(0);
    }

    @Override
    public void execute()
    {
        double armOutput = operatorController.getLeftYSquared() * 0.5;
        arm.setClawRotationRadiansPerSecond(armOutput);
        if (operatorController.getAButton())
        {
            arm.setTestingIntakeMotors(1);
        }

        else if (operatorController.getBButton())
        {
            arm.setTestingIntakeMotors(-1);
        }
        else
        {
            arm.setTestingIntakeMotors(0);
        }
       
    }

    @Override
    public void end(boolean interrupted)
    {
        arm.setClawRotationRadiansPerSecond(0);
        
    }
}
