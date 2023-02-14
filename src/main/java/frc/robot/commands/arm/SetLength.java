package frc.robot.commands.arm;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.Arm;

public class SetLength extends CommandBase {
    private Arm arm;
    private double desiredLength; // Length is measured from center of rotation

    private PIDController pidController;

    private ShuffleboardTab extensionTab;
    private GenericEntry lengthError;
    private GenericEntry currentLength;

    /**
     * Sets the length of the arm to the desired length specificed in meters.
     * NOTE: The length of the arm is measured from the center of rotation to its edge
     * @param arm - the arm that will be extended or contracted based on the desired length
     * @param desiredLength - the length that the arm will be set to in meters
     */
    public SetLength(Arm arm, double desiredLength)
    {
        this.arm = arm;
        this.desiredLength = desiredLength;

        pidController = new PIDController(ArmConstants.EXTENSION_PID_P, ArmConstants.EXTENSION_PID_I, ArmConstants.EXTENSION_PID_D); // This will keep the motors from overshooting/undershooting

        extensionTab = Shuffleboard.getTab("Extension"); // This tab will be used to keep track of any data that needs to be displayed in the shuffleboard
        lengthError = extensionTab.add("Length Error", 0).getEntry(); // Displays the error (the difference between the desired length and the current length)
        currentLength = extensionTab.add("Current Length", 0).getEntry(); // Displays the current legnth of the arm relative to 
        extensionTab.add("PID Controller", pidController); // Displays all of the pid controllers values
    }

    @Override
    public void initialize()
    {
        pidController.setSetpoint(desiredLength); // Sets the goal length the arm wants to be at, in other words the desired length
    }

    @Override
    public void execute()
    {
        currentLength.setDouble(arm.getExtensionLength()); // Updates the shuffleboard value so that we know the length of the arm relative to the center of rotation at all times

        // Calculates the error by subtracting the current length from the desired length and setting the shuffleboard value to that
        double error = desiredLength - arm.getExtensionLength();
        lengthError.setDouble(error);

        // Calculates the output that needs to be sent in meters per second to the motors by using the pid controller to determine how fast it needs to speed up or slow down
        double output = MathUtil.clamp(pidController.calculate(arm.getExtensionLength()), -0.9, 0.9);
        arm.setExtensionMetersPerSecond(output);
    }

    @Override
    public boolean isFinished()
    {
        return pidController.atSetpoint(); // Checks if the arm is at the desired length
    }

    @Override
    public void end(boolean interrupted)
    {
        arm.setExtensionMetersPerSecond(0); // Stops the motors so that there is no unnecessary movement after the command is finished
    }
}
