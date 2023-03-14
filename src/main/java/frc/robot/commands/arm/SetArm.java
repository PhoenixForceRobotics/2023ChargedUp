package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.Arm;

public class SetArm extends ParallelCommandGroup {

    public SetArm(Arm arm, double angle, double length) {
        super(new SetAngle(arm, angle), new SetLength(arm, length));
    }
}
