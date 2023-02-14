package frc.robot.commands.arm;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.Arm;

public class SetAngle extends CommandBase {
    private Arm arm;
    private double desiredAngle;

    private PIDController pidController;

    private ShuffleboardTab rotationTab;
    private GenericEntry angleError;
    private GenericEntry currentAngle;

    /**
     * Sets the angle (in degrees) of the arm to the desired angle inputted
     * @param arm - the arm that will be rotated
     * @param desiredAngle - the angle you want the arm to be set to
     */
    public SetAngle(Arm arm, double desiredAngle)
    {
        this.arm = arm;
        this.desiredAngle = desiredAngle;

        pidController = new PIDController(ArmConstants.ROTATION_PID_P, ArmConstants.ROTATION_PID_I, ArmConstants.ROTATION_PID_D); // This will keep the motors from overshooting/undershooting

        rotationTab = Shuffleboard.getTab("Rotation"); // This tab will be used for any data that needs to be displayed on the shuffleboard
        angleError = rotationTab.add("Angle Error", 0).getEntry(); // Displays the error (the difference between the desired angle and the current angle)
        currentAngle = rotationTab.add("Current Angle", 0).getEntry(); // Displays the current angle of the arm
        rotationTab.add("PID Controller", pidController); // Displays the pid controller values on the shuffleboard
    }

    @Override
    public void initialize()
    {
        pidController.setSetpoint(desiredAngle); // Sets the setpoint (how the pid controller will determine if it is done with this command) to the desired angle
    }

    @Override
    public void execute()
    {
        // Calculates the output that needs to be sent in radians per second to the motors by using the pid controller to determine how fast it needs to speed up or slow down
        double output = MathUtil.clamp(pidController.calculate(arm.getRotationAngle()), -0.9, 0.9);
        arm.setRotationRadiansPerSecond(output);
        
        // Calculates the error of the arm and sets the shuffleboard data to that value to be displayed
        double error = desiredAngle - arm.getRotationAngle();
        angleError.setDouble(error);
        currentAngle.setDouble(arm.getRotationAngle());
        
    }

    @Override
    public boolean isFinished()
    {
        return pidController.atSetpoint(); // Checks if the pid controller notices that it is at its setpoint
    }

    @Override
    public void end(boolean interrupted) {
        arm.setRotationRadiansPerSecond(0); // Once the command is done, it will stop the motors so that we won't have any excess movement
    }
}
