package frc.robot.commands.arm.placement;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.commands.arm.SetWristAngle;
import frc.robot.commands.arm.StowArm;
import frc.robot.commands.claw.TimedOutputPiece;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Claw;

public class FirstStagePlacement extends PlacementSequence {
    public FirstStagePlacement(Arm arm, Claw claw) {
        super(
                // TODO: FIGURE OUT THESE VALUES
                new SetWristAngle(arm, Math.toRadians(45)),
                new InstantCommand(), // Don't extend
                new SetWristAngle(arm, Math.toRadians(-40)),
                new TimedOutputPiece(claw, 0.5),
                new SetWristAngle(arm, Math.toRadians(45)),
                new StowArm(arm));
    }
}
