package frc.robot.utils.coordinateSystems;

import edu.wpi.first.math.Pair;

public class CoordinateMath {
    public static class PolarCoordinates extends Pair<Double, Double> {

        public PolarCoordinates(double radialCoordinate, double angularCoordinate) {
            super(radialCoordinate, angularCoordinate);
        }

        public double getRadialCoordinate() {
            return getFirst();
        }

        public double getAngularCoordinate() {
            return getSecond();
        }
    }

    public static class PolarVelocities extends Pair<Double, Double> {
        public PolarVelocities(double radialVelocity, double angularVelocity) {
            super(radialVelocity, angularVelocity);
        }

        public double getRadialVelocity() {
            return getFirst();
        }

        public double getAngularVelocity() {
            return getSecond();
        }
    }

    public static class CartesianCoordinates extends Pair<Double, Double> {

        public CartesianCoordinates(Double x, Double y) {
            super(x, y);
        }

        public double getXCoordinate() {
            return getFirst();
        }

        public double getYCoordinate() {
            return getSecond();
        }
    }

    public static class CartesianVelocities extends Pair<Double, Double> {
        public CartesianVelocities(double xVelocity, double yVelocity) {
            super(xVelocity, yVelocity);
        }

        public double getXVelocity() {
            return getFirst();
        }

        public double getYVelocity() {
            return getSecond();
        }
    }

    public static PolarCoordinates cartesianToPolarCoordinates(
            CartesianCoordinates cartesianCoordinates) {
        double radialCoordinate =
                Math.sqrt(
                        Math.pow(cartesianCoordinates.getXCoordinate(), 2)
                                + Math.pow(cartesianCoordinates.getYCoordinate(), 2));
        double angularCoordinate =
                Math.atan2(
                        cartesianCoordinates.getYCoordinate(),
                        cartesianCoordinates.getXCoordinate());

        return new PolarCoordinates(radialCoordinate, angularCoordinate);
    }

    public static CartesianCoordinates polarToCartesianCoordinates(
            PolarCoordinates polarCoordinates) {
        double x =
                polarCoordinates.getRadialCoordinate()
                        * Math.cos(polarCoordinates.getAngularCoordinate());
        double y =
                polarCoordinates.getRadialCoordinate()
                        * Math.sin(polarCoordinates.getAngularCoordinate());

        return new CartesianCoordinates(x, y);
    }

    public static PolarVelocities cartesianVelocitiesToPolarVelocities(
            PolarCoordinates polarCoordinates, CartesianVelocities cartesianVelocities) {
        return cartesianVelocitiesToPolarVelocities(
                polarToCartesianCoordinates(polarCoordinates), cartesianVelocities);
    }

    public static PolarVelocities cartesianVelocitiesToPolarVelocities(
            CartesianCoordinates cartesianCoordinates, CartesianVelocities cartesianVelocities) {
        // shorter values to make code readable
        double x = cartesianCoordinates.getXCoordinate();
        double y = cartesianCoordinates.getYCoordinate();
        double dx = cartesianVelocities.getXVelocity();
        double dy = cartesianVelocities.getYVelocity();

        // Use the above to calculate the velocities

        // (x * dx/dt + y * dy/dt ) / (sqrt(x^2 + y^2))
        double radialVelocity = (x * dx + y * dy) / Math.sqrt(Math.pow(x, 2) + Math.pow(y, 2));

        // (x * dy/dt - dx/dt * y) / (x^2 + y^2)
        double angularVelocity = (x * dy - dx * y) / (Math.pow(x, 2) + Math.pow(y, 2));

        return new PolarVelocities(radialVelocity, angularVelocity);
    }
}
