package frc.robot.commands.drivebase.autonomous;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Drivebase;

public class ExampleAutonomousRoutine extends SequentialCommandGroup {

    public ExampleAutonomousRoutine(Drivebase drivebase) {
        super(
                // Place piece,
                PathPlannerCommandFactory.fromJSON(
                        drivebase, "MiddleGridToBottomPiece", true, true),
                // Turn on the arm,
                PathPlannerCommandFactory.fromJSON(
                        drivebase, "BottomPieceToMiddleGrid", false, true),
                // Place Piece,
                PathPlannerCommandFactory.fromJSON(
                        drivebase, "MiddleGridToChargeStation", false, true)
                // Autobalance code
                );
    }
}
