package frc.robot.utils.exceptions;

public class NoValidSnapPointsException extends RuntimeException {

    public NoValidSnapPointsException() {
        super(
            "getSnapPoints() called without first verifying a point is in range; use snapToGrid() first to ensure a valid snap point exists"
        );
    }
}
