package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;

public class SetArmLengthBangBang extends CommandBase {
    private final Arm arm;
    private final double firstStageLength;
    private final double secondStageLength;
    private boolean isFirstStageReversed, isSecondStageReversed;
    private boolean isFirstStageFinished, isSecondStageFinished;

    public SetArmLengthBangBang(Arm arm, double firstStageLength, double secondStageLength) {
        this.arm = arm;
        this.firstStageLength = firstStageLength;
        this.secondStageLength = secondStageLength;
        addRequirements(arm);
    }

    @Override
    public void initialize() {
        isFirstStageReversed = arm.getFirstStageMeters() > firstStageLength;
        arm.setFirstStageMetersPerSecond(isFirstStageReversed ? -0.25 : 0.25);

        isSecondStageReversed = arm.getSecondStageMeters() > secondStageLength;
        arm.setSecondStageMetersPerSecond(isSecondStageReversed ? -0.25 : 0.25);
        isFirstStageFinished = false;
        isSecondStageFinished = false;
    }

    @Override
    public void execute() {
        isFirstStageFinished =
                isFirstStageFinished
                        || (isFirstStageReversed && arm.getFirstStageMeters() <= firstStageLength)
                        || (!isFirstStageReversed && arm.getFirstStageMeters() >= firstStageLength);
        isSecondStageFinished =
                isSecondStageFinished
                        || (isSecondStageReversed
                                && arm.getSecondStageMeters() <= secondStageLength)
                        || (!isSecondStageReversed
                                && arm.getSecondStageMeters() >= firstStageLength);

        if (isFirstStageFinished) {
            arm.setFirstStageMetersPerSecond(0);
        } else {
            arm.setFirstStageMetersPerSecond(isFirstStageReversed ? -0.25 : 0.25);
        }

        if (isSecondStageFinished) {
            arm.setSecondStageMetersPerSecond(0);
        } else {
            arm.setSecondStageMetersPerSecond(isSecondStageReversed ? -0.25 : 0.25);
        }
        System.out.println(isFirstStageFinished + ", " + isSecondStageFinished);
    }

    @Override
    public boolean isFinished() {
        return isFirstStageFinished && isSecondStageFinished;
    }

    @Override
    public void end(boolean interrupted) {
        arm.setExtensionMetersPerSecond(0, 0);
        System.out.println("Is Finished");
    }
}
