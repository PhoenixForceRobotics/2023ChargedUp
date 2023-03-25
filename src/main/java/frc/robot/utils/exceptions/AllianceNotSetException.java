package frc.robot.utils.exceptions;

public class AllianceNotSetException extends RuntimeException {

    public AllianceNotSetException() {
        super(
                "Alliance not set; expected DriverStationAlliance.Red or DriverStationAlliance.Blue, received something else");
    }
}
