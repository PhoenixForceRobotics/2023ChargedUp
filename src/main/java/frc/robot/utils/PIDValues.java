package frc.robot.utils;

public class PIDValues {
    public final double P,
            I,
            D,
            STATIC_F,
            VELOCITY_F,
            ACCELERATION_F,
            G,
            MAX_CONTROL_EFFORT,
            MAX_VELOCITY_ERROR;

    public PIDValues(
            double P,
            double I,
            double D,
            double STATIC_F,
            double VELOCITY_F,
            double ACCELERATION_F,
            double G,
            double MAX_CONTROL_EFFORT,
            double MAX_VELOCITY_ERROR) {
        this.P = P;
        this.I = I;
        this.D = D;
        this.STATIC_F = STATIC_F;
        this.VELOCITY_F = VELOCITY_F;
        this.ACCELERATION_F = ACCELERATION_F;
        this.G = G;
        this.MAX_CONTROL_EFFORT = MAX_CONTROL_EFFORT;
        this.MAX_VELOCITY_ERROR = MAX_VELOCITY_ERROR;
    }

    public PIDValues(
            double P,
            double I,
            double D,
            double STATIC_F,
            double VELOCITY_F,
            double ACCELERATION_F,
            double G,
            double MAX_CONTROL_EFFORT,
            double MAX_VELOCITY_ERROR) {
        this.P = P;
        this.I = I;
        this.D = D;
        this.STATIC_F = STATIC_F;
        this.VELOCITY_F = VELOCITY_F;
        this.ACCELERATION_F = ACCELERATION_F;
        this.G = G;
        this.MAX_CONTROL_EFFORT = MAX_CONTROL_EFFORT;
        this.MAX_VELOCITY_ERROR = MAX_VELOCITY_ERROR;
    }

    public PIDValues(
            double P,
            double I,
            double D,
            double STATIC_F,
            double VELOCITY_F,
            double ACCELERATION_F,
            double MAX_CONTROL_EFFORT,
            double MAX_VELOCITY_ERROR) {
        this.P = P;
        this.I = I;
        this.D = D;
        this.STATIC_F = STATIC_F;
        this.VELOCITY_F = VELOCITY_F;
        this.ACCELERATION_F = ACCELERATION_F;
        this.G = 0;
        this.MAX_CONTROL_EFFORT = MAX_CONTROL_EFFORT;
        this.MAX_VELOCITY_ERROR = MAX_VELOCITY_ERROR;
    }

    public PIDValues(
        double P,
        double I,
        double D,
        double STATIC_F,
        double VELOCITY_F,
        double ACCELERATION_F
    ) {
        this(P, I, D, STATIC_F, VELOCITY_F, ACCELERATION_F, 0, 0);
    }

    public PIDValues(double P, double I, double D) {
        this(P, I, D, 0.0, 0.0, 0.0);
    }
}
