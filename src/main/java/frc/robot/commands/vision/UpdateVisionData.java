package frc.robot.commands.vision;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivebase;
import frc.robot.subsystems.vision.TagProcessing;
import frc.robot.utils.vision.UnitConverter;

public class UpdateVisionData extends CommandBase {
    TagProcessing tagProcessor;
    Drivebase drivebase;

    public UpdateVisionData(TagProcessing newTagProcessor, Drivebase newDrivebase) {
        this.tagProcessor = newTagProcessor;
        this.drivebase = newDrivebase;
        addRequirements(newTagProcessor);
    }

    @Override
    public void initialize() {
        System.out.println("Vision processing position injector initialized");
        System.out.println("Welcome back, pilot. Focus. Plan. Attack.");
    }

    @Override
    public void execute() {
        this.tagProcessor.update();
        // TODO: upgrade to proper dedicated fusion-based pose estimator using kalman filtering
        // as it stands this literally just injects any values it gets which is kinda cringe
        // method detailed at
        // https://github.com/PhotonVision/photonvision/blob/master/photonlib-java-examples/simposeest/src/main/java/frc/robot/DrivetrainPoseEstimator.java
        if (tagProcessor.checkIfBuffered()) {
            System.out.println(tagProcessor.getBestPoseGuess());
            this.drivebase.resetPose(
                    UnitConverter.MetersToInches(tagProcessor.sudoGetBestPoseGuess()));
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {}
}
