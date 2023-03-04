package frc.robot.utils;

import com.revrobotics.CANSparkMax;

public class Motor extends CANSparkMax {
    private double gearRatio;
    private double wheelDiameter;
    private PFRPIDController positionPID;
    private PFRPIDController velocityPID;

    public Motor(
            int port,
            boolean reversed,
            double gearRatio,
            double wheelDiameter,
            PIDValues positionPID,
            PIDValues velocityPID) {
        super(port, MotorType.kBrushless);

        this.gearRatio = gearRatio;
        this.wheelDiameter = wheelDiameter;
        this.positionPID = new PFRPIDController(positionPID);
        this.velocityPID = new PFRPIDController(velocityPID);

        setInverted(reversed);
    }

    public Motor(int port, boolean reversed, double gearRatio, double wheelDiameter) {
        this(
                port,
                reversed,
                gearRatio,
                wheelDiameter,
                new PIDValues(0, 0, 0),
                new PIDValues(0, 0, 0));
    }

    public Motor(int port, boolean reversed) {
        this(port, reversed, 1, 1);
    }

    public void setMetersPerSecond(double metersPerSecond) {
        double output = velocityPID.filteredCalculate(getMetersPerSecond(), metersPerSecond);
        setVoltage(output);
    }

    public void setMeters(double meters) {

        double output = positionPID.filteredCalculate(getMeters(), meters);
        setVoltage(output);
    }

    public void setVelocityP(double kp) {
        velocityPID.setP(kp);
    }

    public void setVelocityI(double ki) {
        velocityPID.setP(ki);
    }

    public void setVelocityD(double kd) {
        velocityPID.setD(kd);
    }

    public double getVelocityP() {
        return velocityPID.getP();
    }

    public double getVelocityI() {
        return velocityPID.getI();
    }

    public double getVelocityD() {
        return velocityPID.getD();
    }

    public double getRotations() {
        return getEncoder().getPosition() * gearRatio;
    }

    public double getMeters() {
        return getRotations() * wheelDiameter * Math.PI;
    }

    public double getRPM() {
        return getEncoder().getVelocity() * gearRatio;
    }

    public double getMetersPerSecond() {
        return getRPM() * wheelDiameter * Math.PI / 60;
    }

    public double getGearRatio() {
        return gearRatio;
    }

    public double getwheelDiameter() {
        return wheelDiameter;
    }

    public boolean isReversed() {
        return getInverted();
    }
}
