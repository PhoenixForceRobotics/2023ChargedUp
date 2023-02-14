package frc.robot.subsystems;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.mockito.Mockito.mock;

import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.utils.Motor;
import org.junit.jupiter.api.Test;
import org.mockito.Mockito;

public class ClawTest {
    Motor wheel = mock(Motor.class); // Creates a mockup version of the motor class
    // Creates a mockup version of the DigitalInput class
    DigitalInput coneSensor = mock(DigitalInput.class);
    DigitalInput cubeSensor = mock(DigitalInput.class);
    Claw claw = new Claw(wheel, cubeSensor, coneSensor); // Creates a mockup version of the claw class that uses the components above

    // Tests to see if there is a cone in the claw
    @Test
    void testForCone() {
        Mockito.when(coneSensor.get()).thenReturn(true);
        assertEquals(Claw.BeamBreakStatus.CONE, claw.getBreakStatus());
    }

    // Tests to see if there is a cube in the claw
    @Test
    void testForCube() {
        Mockito.when(cubeSensor.get()).thenReturn(true);
        assertEquals(Claw.BeamBreakStatus.CUBE, claw.getBreakStatus());
    }

    // Tests to see if both pieces are in the claw
    @Test
    void testForBothPieces() {
        Mockito.when(cubeSensor.get()).thenReturn(true);
        Mockito.when(coneSensor.get()).thenReturn(true);
        assertEquals(Claw.BeamBreakStatus.BOTH, claw.getBreakStatus());
    }

    // Tests to see if there are no pieces
    @Test
    void testForNoPieces() {
        Mockito.when(cubeSensor.get()).thenReturn(false);
        Mockito.when(coneSensor.get()).thenReturn(false);
        assertEquals(Claw.BeamBreakStatus.NONE, claw.getBreakStatus());
    }

    // Tests to see if there is any piece in the claw
    @Test
    void testForAnyPiece() {
        Mockito.when(cubeSensor.get()).thenReturn(true);
        Mockito.when(coneSensor.get()).thenReturn(false);
        assertEquals(true, claw.hasPiece());
    }

    // Tests to see if there is a motor
    @Test
    void testForMotor() {
        assertEquals(wheel, claw.getMotor());
    }
}
