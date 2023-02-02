package frc.robot.subsystems;

import static org.mockito.Mockito.mock;
import static org.mockito.Mockito.times;
import static org.mockito.Mockito.verify;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import frc.robot.utils.Motor;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.mockito.Mockito;

public class DriveBaseTest {
    Drivebase driveBase;
    Motor flWheel = mock(Motor.class);
    Motor frWheel = mock(Motor.class);
    Motor blWheel = mock(Motor.class);
    Motor brWheel = mock(Motor.class);
    DoubleSolenoid flPiston = mock(DoubleSolenoid.class);
    DoubleSolenoid frPiston = mock(DoubleSolenoid.class);
    DoubleSolenoid blPiston = mock(DoubleSolenoid.class);
    DoubleSolenoid brPiston = mock(DoubleSolenoid.class);
    Gyro gyro = mock(Gyro.class);

    @BeforeEach // this method will run before each test
    void setup() {

        // The robot starting angle for testing is at 0.0
        Mockito.when(gyro.getRotation2d()).thenReturn(new Rotation2d(0.0));

        // The motor starting speed is 0
        Mockito.when(flWheel.getMeters()).thenReturn(0.0d);
        Mockito.when(frWheel.getMeters()).thenReturn(0.0d);
        Mockito.when(blWheel.getMeters()).thenReturn(0.0d);
        Mockito.when(brWheel.getMeters()).thenReturn(0.0d);

        driveBase =
                new Drivebase(
                        flWheel, frWheel, blWheel, brWheel, flPiston, frPiston, blPiston, brPiston,
                        gyro);
    }

    @Test
    public void testSetChassisSpeeds() {
        driveBase.setChassisSpeeds(1.0, 1.0, 1.0);
        driveBase.periodic();

        verify(flWheel, times(1)).set(0.12156640181611805d);
        verify(frWheel, times(1)).set(0.9);
        verify(blWheel, times(1)).set(0.12156640181611805d);
        verify(brWheel, times(1)).set(0.9);
    }

    @Test
    public void testStop() {
        driveBase.setChassisSpeeds(1.0, 1.0, 1.0);
        driveBase.periodic();
        verify(flWheel, times(1)).set(0.12156640181611805d);
        verify(frWheel, times(1)).set(0.9);
        verify(blWheel, times(1)).set(0.12156640181611805d);
        verify(brWheel, times(1)).set(0.9);

        driveBase.stop();
        driveBase.periodic();
        verify(flWheel, times(1)).set(0.0d);
        verify(frWheel, times(1)).set(0.0d);
        verify(blWheel, times(1)).set(0.0d);
        verify(brWheel, times(1)).set(0.0d);
    }

    @Test
    public void testGetInitialMotorSpeed() {
        driveBase.periodic();

        verify(flWheel, times(1)).set(0.0d);
        verify(frWheel, times(1)).set(0.0d);
        verify(blWheel, times(1)).set(0.0d);
        verify(brWheel, times(1)).set(0.0d);
    }
}
