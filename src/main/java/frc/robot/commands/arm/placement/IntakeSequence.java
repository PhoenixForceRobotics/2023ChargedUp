package frc.robot.commands.arm.placement;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.arm.SetArmLengthBangBang;
import frc.robot.commands.arm.SetWristAngle;
import frc.robot.commands.arm.StowArm;
import frc.robot.commands.claw.IntakePiece;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Claw;

public class IntakeSequence extends SequentialCommandGroup{
    public IntakeSequence(Arm arm, Claw claw)
    {
        super(
            // Rotate wrist backward
            new SetWristAngle(arm, Math.toRadians(50)),
            // Extend arm
            new SetArmLengthBangBang(arm, 0.12, 0.22),
            // Start up intake
            // Rotate Downwards and stop intake when done
            Commands.race(new IntakePiece(claw), new SetWristAngle(arm, Math.toRadians(-40))),
            // Stow Arm
            new StowArm(arm)
        );
        addRequirements(arm, claw);
    }
}
