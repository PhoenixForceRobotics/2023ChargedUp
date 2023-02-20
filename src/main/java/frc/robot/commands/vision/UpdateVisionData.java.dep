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
        System.out.println("h");
    }

    @Override
    public void execute() {
        this.tagProcessor.update();
        System.out.println("UPDATED BY COMMAND");
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {}
}
