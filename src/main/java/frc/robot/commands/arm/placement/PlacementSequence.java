package frc.robot.commands.arm.placement;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class PlacementSequence extends SequentialCommandGroup {

    public PlacementSequence(
        Command rotateClawBack,
        Command extendArm, 
        Command rotateClaw, 
        Command outtakePiece, 
        Command rotateClawForwards, 
        Command unextendArm)
    {
        super(
            rotateClawBack,
            // Extend Arm
            extendArm,
            // Rotate Claw 
            rotateClaw,
            // Outtake Piece
            outtakePiece,
            // Rotate Claw Back
            rotateClawForwards,
            // Unextend Arm
            unextendArm
        );
    }

    
}
