package frc.robot.subsystems;

import static org.junit.jupiter.api.Assertions.assertEquals;
import org.junit.jupiter.api.Test;
import org.mockito.Mockito;
import frc.robot.utils.Motor;
import edu.wpi.first.wpilibj.DigitalInput;
import static org.mockito.Mockito.mock;

public class ClawTest 
{
    Motor wheel = mock(Motor.class);
    DigitalInput coneSensor = mock(DigitalInput.class);
    DigitalInput cubeSensor = mock(DigitalInput.class);
    Claw claw = new Claw(wheel, cubeSensor, coneSensor);

    @Test
    void testForCone()
    {
        Mockito.when(coneSensor.get()).thenReturn(true);
        assertEquals(Claw.BeamBreakStatus.CONE, claw.getBreakStatus());
    }

    @Test
    void testForCube()
    {
        Mockito.when(cubeSensor.get()).thenReturn(true);
        assertEquals(Claw.BeamBreakStatus.CUBE, claw.getBreakStatus());
    }

    @Test
    void testForBothPieces()
    {
        Mockito.when(cubeSensor.get()).thenReturn(true);
        Mockito.when(coneSensor.get()).thenReturn(true);
        assertEquals(Claw.BeamBreakStatus.BOTH, claw.getBreakStatus());
    }

    @Test
    void testForNoPieces()
    {
        Mockito.when(cubeSensor.get()).thenReturn(false);
        Mockito.when(coneSensor.get()).thenReturn(false);
        assertEquals(Claw.BeamBreakStatus.NONE, claw.getBreakStatus());
    }

    @Test
    void testForAnyPiece()
    {
        Mockito.when(cubeSensor.get()).thenReturn(true);
        Mockito.when(coneSensor.get()).thenReturn(true);
        assertEquals(true, claw.hasPiece());
    }
}
