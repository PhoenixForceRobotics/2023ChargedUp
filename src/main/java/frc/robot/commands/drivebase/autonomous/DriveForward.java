package frc.robot.commands.drivebase.autonomous;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivebase;

public class DriveForward extends CommandBase {
    private final Drivebase drivebase;

    public DriveForward(Drivebase drivebase) {
        this.drivebase = drivebase;
    }

    @Override
    public void initialize() {
        drivebase.setChassisSpeeds(2, 0, 0);
    }
}
