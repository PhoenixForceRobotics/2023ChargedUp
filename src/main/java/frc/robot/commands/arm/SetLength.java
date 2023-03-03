package frc.robot.commands.arm;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.Arm;
import frc.robot.utils.PFRSimplePIDController;

public class SetLength extends CommandBase {
    private Arm arm;

    private PIDController firstStageController;
    private PIDController secondStageController;

    /**
     * Sets the length of the arm to the desired length specificed in meters. NOTE: The length of
     * the arm is measured from the center of rotation to its edge
     *
     * @param arm - the arm that will be extended or contracted based on the desired length
     * @param desiredLength - the length that the arm will be set to in meters (RELATIVE TO CENTER OF ROTATION)
     */
    public SetLength(Arm arm, double desiredLength) {
        this.arm = arm;

        firstStageController = new PFRSimplePIDController(ArmConstants.FIRST_STAGE_PID_VALUES);
        firstStageController.setTolerance(ArmConstants.ROTATIONAL_SETPOINT_ERROR.getFirst(), ArmConstants.ROTATIONAL_SETPOINT_ERROR.getSecond());

        secondStageController = new PFRSimplePIDController(ArmConstants.SECOND_STAGE_PID_VALUES);
        firstStageController.setTolerance(ArmConstants.ROTATIONAL_SETPOINT_ERROR.getFirst(), ArmConstants.ROTATIONAL_SETPOINT_ERROR.getSecond());

        double proportionOfTotalLength = (desiredLength - ArmConstants.EXTENSION_STARTING_LENGTH) /  (ArmConstants.FIRST_STAGE_MAX_EXTENSION + ArmConstants.SECOND_STAGE_MAX_EXTENSION);
        double desiredFirstStageLength = proportionOfTotalLength * ArmConstants.FIRST_STAGE_MAX_EXTENSION;
        double desiredSecondStageLength = proportionOfTotalLength * ArmConstants.SECOND_STAGE_MAX_EXTENSION;

        firstStageController.setSetpoint(desiredFirstStageLength);
        secondStageController.setSetpoint(desiredSecondStageLength);
    }

    @Override
    public void initialize() {
        arm.setExtensionMetersPerSecond(0, 0);
    }

    @Override
    public void execute() {
        double firstStageOutput = 0;
        double secondStageOutput = 0;
        if(!firstStageController.atSetpoint())
        {
            firstStageOutput = firstStageController.calculate(arm.getFirstStageMeters());
        }

        if(!secondStageController.atSetpoint())
        {
            secondStageOutput = secondStageController.calculate(arm.getSecondStageMeters());
        }

        arm.setExtensionMetersPerSecond(firstStageOutput, secondStageOutput);

    }

    @Override
    public boolean isFinished() {
        return (firstStageController.atSetpoint() && secondStageController.atSetpoint()); // Checks if the arm is at the desired length
    }

    @Override
    public void end(boolean interrupted) {
        arm.setExtensionMetersPerSecond(
                0, 0); 
    }
}
