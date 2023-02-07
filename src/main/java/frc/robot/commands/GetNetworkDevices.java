package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.TagProcessing;

public class GetNetworkDevices extends CommandBase {
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final TagProcessing TagProcessor;
    public GetNetworkDevices(TagProcessing subsystem) {
        this.TagProcessor = subsystem;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(subsystem);
    }
}
