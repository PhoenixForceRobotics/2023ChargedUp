package frc.robot.commands.arm.placement;

import frc.robot.commands.arm.SetArmLengthBangBang;
import frc.robot.commands.arm.SetWristAngle;
import frc.robot.commands.arm.StowArm;
import frc.robot.commands.claw.TimedOutputPiece;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Claw;

public class SecondStagePlacement extends PlacementSequence {
    public SecondStagePlacement(Arm arm, Claw claw) {
        super(
                // TODO: FIGURE OUT THESE VALUES
                new SetWristAngle(arm, Math.toRadians(45)),
                new SetArmLengthBangBang(arm, 0.07, 0.08),
                new SetWristAngle(arm, Math.toRadians(-40)),
                new TimedOutputPiece(claw, 1),
                new SetWristAngle(arm, Math.toRadians(45)),
                new StowArm(arm));
    }
}
