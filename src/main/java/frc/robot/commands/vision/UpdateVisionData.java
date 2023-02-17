package frc.robot.commands.vision;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.vision.TagProcessing;

public class UpdateVisionData extends CommandBase {
    TagProcessing tagProcessor;

    public UpdateVisionData(TagProcessing newTagProcessor) {
        this.tagProcessor = newTagProcessor;
        addRequirements(newTagProcessor);
    }

    @Override
    public void initialize() {

    }
}
