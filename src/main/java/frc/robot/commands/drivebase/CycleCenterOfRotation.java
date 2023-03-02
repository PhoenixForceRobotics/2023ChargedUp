package frc.robot.commands.drivebase;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Drivebase;
import frc.robot.subsystems.Drivebase.CenterOfRotation;

public class CycleCenterOfRotation extends InstantCommand {

    // Signifies whether to go up or down in the list of CenterOfRotation
    public enum Direction {
        UP,
        DOWN,
    }

    private Drivebase drivebase;
    private Direction direction;

    public CycleCenterOfRotation(Drivebase drivebase, Direction direction) {
        this.drivebase = drivebase;
        this.direction = direction;
    }

    @Override
    public void initialize() {
        CenterOfRotation currentCenterOfRotation = drivebase.getCenterOfRotation();
        int lengthOfEnum = CenterOfRotation.values().length;
        int index; // the position in the CenterOfRotation enum to use

        if (direction == Direction.UP) {
            // Get next highest CenterOfRotation (or wraps around)
            index = (currentCenterOfRotation.ordinal() + 1) % lengthOfEnum;
        } else if (direction == Direction.DOWN) {
            // Get next lowest CenterOfRotation (or wraps around)
            index =
                (currentCenterOfRotation.ordinal() - 1 + lengthOfEnum) %
                lengthOfEnum;
        } else {
            // Something went wrong, just go to the original
            index = currentCenterOfRotation.ordinal();
        }

        // sets the new given CenterOfRotation
        drivebase.setCenterOfRotation(CenterOfRotation.values()[index]);
    }
}
