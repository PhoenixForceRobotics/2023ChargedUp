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

    public SetLength(Arm arm, double desiredLength)
    {
        this.arm = arm;
        this.desiredLength = desiredLength;

        pidController = new PIDController(ArmConstants.EXTENSION_PID_P, ArmConstants.EXTENSION_PID_I, ArmConstants.EXTENSION_PID_D);

        extensionTab = Shuffleboard.getTab("Extension");
        lengthError = extensionTab.add("Length Error", 0).getEntry();
        currentLength = extensionTab.add("Current Length", 0).getEntry();
        extensionTab.add("PID Controller", pidController);
    }

    @Override
    public void initialize()
    {
        pidController.setSetpoint(desiredLength);
    }

    @Override
    public void execute()
    {
        currentLength.setDouble(arm.getExtensionLength());

        double error = desiredLength - arm.getExtensionLength();
        lengthError.setDouble(error);

        double output = MathUtil.clamp(pidController.calculate(arm.getExtensionLength()), -0.9, 0.9);
        arm.setExtensionMetersPerSecond(output);
    }

    @Override
    public boolean isFinished()
    {
        return pidController.atSetpoint();
    }
}
