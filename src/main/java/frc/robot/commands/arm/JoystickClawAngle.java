package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;
import frc.robot.utils.PFRController;

public class JoystickClawAngle extends CommandBase {
    private Arm arm;
    private PFRController operatorController;
    
    public JoystickClawAngle(Arm arm, PFRController operatorController)
    {
        this.arm = arm;
        this.operatorController = operatorController;
    }

    @Override
    public void initialize()
    {
        arm.setClawRotation(0);
    }

    @Override
    public void execute()
    {
        arm.setClawRotation(operatorController.getLeftYSquared());
    }

    @Override
    public void end(boolean interrupted)
    {
        arm.setClawRotation(0);
    }
}
