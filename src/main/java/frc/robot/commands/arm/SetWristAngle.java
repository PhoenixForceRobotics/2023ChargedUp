package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;

public class SetWristAngle extends CommandBase {
    private final Arm arm;
    private final double relativeAngleRadians;
    private boolean isClawReversed;
    public SetWristAngle(Arm arm, double relativeAngleRadians)
    {
        this.arm = arm;       
        this.relativeAngleRadians = relativeAngleRadians;
    }

    @Override
    public void initialize() {
        isClawReversed = arm.getClawRelativeAngleRadians() > relativeAngleRadians;
        arm.setClawRelativeRadiansPerSecond(isClawReversed ? -Math.PI : Math.PI);
        
    }

    @Override
    public void execute() {
        arm.setClawRelativeRadiansPerSecond(isClawReversed ? -Math.PI : Math.PI);

    }
    @Override
    public boolean isFinished() {
        return (isClawReversed && arm.getClawRelativeAngleRadians() <= relativeAngleRadians) || (!isClawReversed && arm.getClawRelativeAngleRadians() >= relativeAngleRadians);
    }
    
    @Override
    public void end(boolean interrupted) {
        arm.setClawRelativeRadiansPerSecond(0);
    }

}
