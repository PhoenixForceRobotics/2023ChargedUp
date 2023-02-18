package frc.robot.subsystems;

import static org.junit.jupiter.api.Assertions.assertTrue;
import static org.mockito.Mockito.mock;

import edu.wpi.first.math.controller.PIDController;
import frc.robot.utils.Motor;
import frc.robot.utils.SparkMotorGroup;
import org.junit.jupiter.api.Test;
import org.mockito.Mockito;

public class ArmTest {
    Motor rotationMotor1 = mock(Motor.class);
    Motor rotationMotor2 = mock(Motor.class);
    SparkMotorGroup rotationMotors = mock(SparkMotorGroup.class);

    Motor extensionMotor1 = mock(Motor.class);
    Motor extensionMotor2 = mock(Motor.class);
    SparkMotorGroup extensionMotors = mock(SparkMotorGroup.class);

    PIDController rotationPID = mock(PIDController.class);
    PIDController extensionPID = mock(PIDController.class);

    Arm arm =
            new Arm(
                    rotationMotor1,
                    rotationMotor2,
                    extensionMotor1,
                    extensionMotor2,
                    extensionMotors,
                    rotationMotors,
                    rotationPID,
                    extensionPID);

    @Test
    public void testForRotationForward() {
        Mockito.when(rotationMotor1.getRPM()).thenReturn(0.1);
        assertTrue(arm.getRotationRadiansPerSecond() > 0);
    }

    @Test
    public void testForRotationBackward() {
        Mockito.when(rotationMotor1.getRPM()).thenReturn(-0.1);
        assertTrue(arm.getRotationRadiansPerSecond() < 0);
    }

    @Test
    public void testForAnyRotationMovement() {
        Mockito.when(rotationMotor1.getRPM()).thenReturn(0.1);
        assertTrue(arm.getRotationRadiansPerSecond() != 0);
    }

    @Test
    public void testForExtensionForward() {
        assertTrue(arm.getExtensionMetersPerSecond() > 0);
    }

    @Test
    public void testForExtensionBackward() {
        arm.setExtensionMetersPerSecond(-1);
        assertTrue(arm.getExtensionMetersPerSecond() < 0);
    }

    @Test
    public void testForAnyExtensionMovement() {
        arm.setExtensionMetersPerSecond(1);
        assertTrue(arm.getExtensionMetersPerSecond() != 0);
    }
}
