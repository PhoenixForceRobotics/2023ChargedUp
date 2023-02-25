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
        arm.setRotationRadiansPerSecond(0);
    }

    @Override
    public void execute()
    {
        arm.setRotationRadiansPerSecond(operatorController.getLeftYSquared() * 0.3);
        if (operatorController.getAButton())
        {
            arm.setTestingIntakeMotors(0.3);
        }

        else if (operatorController.getBButton())
        {
            arm.setTestingIntakeMotors(-0.3);
        }
    }

    @Override
    public void end(boolean interrupted)
    {
        arm.setRotationRadiansPerSecond(0);
        arm.setTestingIntakeMotors(0);
    }
}
