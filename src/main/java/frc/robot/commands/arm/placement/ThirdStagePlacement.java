package frc.robot.commands.arm.placement;

import frc.robot.commands.arm.SetArmLengthBangBang;
import frc.robot.commands.arm.SetWristAngle;
import frc.robot.commands.arm.StowArm;
import frc.robot.commands.claw.TimedOutputPiece;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Claw;

public class ThirdStagePlacement extends PlacementSequence {
    public ThirdStagePlacement(Arm arm, Claw claw) {
        super(
                // TODO: FIGURE OUT THESE VALUES
                new SetWristAngle(arm, Math.toRadians(45)),
                new SetArmLengthBangBang(arm, 0.12, 0.22),
                new SetWristAngle(arm, Math.toRadians(45)),
                new TimedOutputPiece(claw, 1),
                new SetWristAngle(arm, Math.toRadians(90)),
                new StowArm(arm));
    }
}
