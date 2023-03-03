package frc.robot.commands.arm;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.Arm;
import frc.robot.utils.PFRSimplePIDController;

public class SetAngle extends CommandBase {
    private Arm arm;
    private double desiredAngle;

    private PIDController pidController;

    /**
     * Sets the angle (in degrees) of the arm to the desired angle inputted
     *
     * @param arm - the arm that will be rotated
     * @param desiredAngle - the angle you want the arm to be set to
     */
    public SetAngle(Arm arm, double desiredAngle) {
        this.arm = arm;
        this.desiredAngle = desiredAngle;
        pidController =
                new PFRSimplePIDController(ArmConstants.ARM_ROTATION_PID_VALUES); 
        // This will keep the motors from overshooting/undershooting

    }

    @Override
    public void initialize() {
        pidController.setSetpoint(
                desiredAngle); // Sets the setpoint (how the pid controller will determine if it is
        // done with this command) to the desired angle
    }

    @Override
    public void execute() {
        // Calculates the output that needs to be sent in radians per second to the motors by
        // using the pid controller to determine how fast it needs to speed up or slow down
        double output = pidController.calculate(arm.getArmRadians());
        arm.setArmRotationRadiansPerSecond(output);
    }

    @Override
    public boolean isFinished() {
        return pidController
                .atSetpoint(); // Checks if the pid controller notices that it is at its setpoint
    }

    @Override
    public void end(boolean interrupted) {
        arm.setArmRotationRadiansPerSecond(
                0); // stops motors at the end
    }
}
