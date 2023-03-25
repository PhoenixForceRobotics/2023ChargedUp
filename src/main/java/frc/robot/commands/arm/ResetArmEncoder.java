package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Arm;

public class ResetArmEncoder extends InstantCommand {
    private final Arm arm; 
    public ResetArmEncoder(Arm arm)
    {
        this.arm = arm;
    }

    @Override
    public void initialize() {
        arm.resetArmEncoder();
    }
}
