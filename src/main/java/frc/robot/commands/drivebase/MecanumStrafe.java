package frc.robot.commands.drivebase;

import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.constants.Constants.ControllerConstants;
import frc.robot.subsystems.Drivebase;
import frc.robot.utils.PFRController;

public class MecanumStrafe extends CommandBase {

    public enum FrameOfReference {
        ROBOT,
        FIELD;
    }

    private final Drivebase drivebase;
    private final PFRController driverController;
    private FrameOfReference frameOfReference;

    public MecanumStrafe(Drivebase drivebase, PFRController driverController) {
        this.drivebase = drivebase;
        this.driverController = driverController;
        frameOfReference = FrameOfReference.ROBOT;
        addRequirements(drivebase);
    }

    @Override
    public void initialize() {
        drivebase.setMeccanum(true);
        drivebase.setButterflyModules(Value.kForward);
    }

    @Override
    public void execute() {

        if (driverController.getLeftBumperPressed()) {
            // allows us to toggle frame of reference when button pressed
            if (frameOfReference == FrameOfReference.ROBOT) {
                frameOfReference = FrameOfReference.FIELD;
            } else {
                frameOfReference = FrameOfReference.ROBOT;
            }
        }

        // now for the funny (pain)
        int xIntent =
                (driverController.getPOV() == ControllerConstants.DPAD_LEFT ? 1 : 0)
                        - (driverController.getPOV() == ControllerConstants.DPAD_RIGHT ? 1 : 0);
        xIntent *= .1; // damper

        // TODO: add snap to grid in order to actually make it helpful for drivers
        // TODO: make useful by having it automatically align parallel with the apriltag

        if (frameOfReference == FrameOfReference.ROBOT) {
            drivebase.setChassisSpeeds(xIntent, 0, 0);
        } else {
            drivebase.setFieldRelativeChassisSpeeds(xIntent, 0, 0);
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        drivebase.stop();
    }
}
