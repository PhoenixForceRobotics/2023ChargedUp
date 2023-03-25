package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.constants.Constants.ArmConstants;
import frc.robot.subsystems.Arm;

public class StowArm extends CommandBase {
    private final Arm arm;

    public StowArm(Arm arm) {
        this.arm = arm;
    }

    @Override
    public void initialize() {
        this.arm.setExtensionMetersPerSecond(-0.25, -0.25);
    }

    @Override
    public void execute() {
        this.arm.setExtensionMetersPerSecond(-0.25, -0.25);
        System.out.println("TESTING");
    }

    @Override
    public boolean isFinished() {
        return this.arm.getFirstStageMeters() <= ArmConstants.FIRST_STAGE_MIN_EXTENSION
                && this.arm.getSecondStageMeters() <= ArmConstants.SECOND_STAGE_MIN_EXTENSION;
    }

    @Override
    public void end(boolean interrupted) {
        this.arm.setExtensionMetersPerSecond(0, 0);
    }
}
