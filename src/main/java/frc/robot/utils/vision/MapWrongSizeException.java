package frc.robot.subsystems.vision;

public class MapWrongSizeException extends RuntimeException {
    public MapWrongSizeException() {
        super("Provided primary tag map incorrect size; should be array of Region size 10, indices 0-9");
    }
}